/*
 *	An Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor.
 *
 *  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 *	The code in this library is based off the manufacturer datasheet and reading of this initial piece of work for ESPHome https://github.com/rain931215/ESPHome-LD2410.
 *
 *	https://github.com/ncmreynolds/ld2410
 *
 *	Released under LGPL-2.1 see https://github.com/ncmreynolds/ld2410/LICENSE for full license
 *
 */
#ifndef ld2410_cpp
#define ld2410_cpp
#include "ld2410.h"
#include "ld2410_frame.h"
ld2410::ld2410()	//Constructor function
{
}

ld2410::~ld2410()	//Destructor function
{
#if defined(ESP32)
	if (cmd_mutex_ != nullptr) {
		vSemaphoreDelete(cmd_mutex_);
		cmd_mutex_ = nullptr;
	}
#endif
}

void ld2410::add_to_buffer(uint8_t byte) {
    // Inserisce il byte nel buffer circolare
    circular_buffer[buffer_head] = byte;
    buffer_head = (buffer_head + 1) % LD2410_BUFFER_SIZE;

    // Gestione del caso in cui il buffer si riempia
    if (buffer_head == buffer_tail) {
        buffer_tail = (buffer_tail + 1) % LD2410_BUFFER_SIZE;  // Sovrascrive i dati più vecchi
    }
}

// Funzione per leggere il buffer
bool ld2410::read_from_buffer(uint8_t &byte) {
    if (buffer_head == buffer_tail) {
        return false;  // Buffer vuoto
    } else {
        byte = circular_buffer[buffer_tail];
        buffer_tail = (buffer_tail + 1) % LD2410_BUFFER_SIZE;
        return true;
    }
}

bool ld2410::begin(Stream &radarStream, bool waitForRadar) {
    radar_uart_ = &radarStream;
#if defined(ESP32)
    if (cmd_mutex_ == nullptr) {
        cmd_mutex_ = xSemaphoreCreateMutex();
    }
#endif
    
    if (debug_uart_ != nullptr) {
        debug_uart_->println(F("ld2410 started"));
    }
    
    if (!waitForRadar) {
        return true;
    }
    
    // Prova a leggere la versione firmware
    if (debug_uart_ != nullptr) {
        debug_uart_->print(F("\nLD2410 firmware: "));
    }
    
    uint32_t start_time = millis();
    bool firmware_received = false;
    
    while (millis() - start_time < 1000) { // timeout di 1 secondo
        if (requestFirmwareVersion()) {
            firmware_received = true;
            break;
        }
        yield();
    }
    
    if (firmware_received) {
        if (debug_uart_ != nullptr) {
            debug_uart_->print(F(" v"));
            debug_uart_->print(firmware_major_version);
            debug_uart_->print('.');
            debug_uart_->print(firmware_minor_version);
            debug_uart_->print('.');
            debug_uart_->print(firmware_bugfix_version);
        }
        return true;
    }
    
    if (debug_uart_ != nullptr) {
        debug_uart_->print(F("no response"));
    }
    return false;
}

void ld2410::debug(Stream &terminalStream)
{
	debug_uart_ = &terminalStream;		//Set the stream used for the terminal
	#if defined(ESP8266)
	if(&terminalStream == &Serial)
	{
		  debug_uart_->write(17);			//Send an XON to stop the hung terminal after reset on ESP8266
	}
	#endif
}

bool ld2410::isConnected()
{
	if(millis() - radar_uart_last_packet_ < radar_uart_timeout)	//Use the last reading
	{
		return true;
	}
	if(read_frame_())	//Try and read a frame if the current reading is too old
	{
		return true;
	}
	return false;
}

bool ld2410::read() {
    bool new_data = false;
    // Leggi tutti i dati disponibili dal buffer UART
    while (radar_uart_->available()) {
        add_to_buffer(radar_uart_->read());
        new_data = true;
    }

    // Prova a leggere e processare un frame
    bool frame_processed = read_frame_();
    
    // Restituisce true se sono stati letti nuovi dati o se un frame è stato processato
    return new_data || frame_processed;
}


#if defined(ESP32)
void ld2410::taskFunction(void* param) {
    ld2410* sensor = static_cast<ld2410*>(param);
    for (;;) {
        // Legge i dati dalla UART e li aggiunge al buffer circolare
        bool new_data = false;
        while (sensor->radar_uart_->available()) {
            sensor->add_to_buffer(sensor->radar_uart_->read());
            new_data = true;
        }

        // Se ci sono nuovi dati, tenta di processare un frame
        if (new_data) {
            bool frame_processed = sensor->read_frame_();
        }
        
        // Delay per evitare il sovraccarico del task
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Avvia il task FreeRTOS che legge in continuo dalla UART del radar.
// Ritorna true se il task è stato creato con successo. Se un task era già
// attivo viene ignorata la nuova richiesta e si ritorna true.
bool ld2410::autoReadTask(uint32_t stack, UBaseType_t priority, BaseType_t core) {
    if (taskHandle_ != nullptr) {
        return true;
    }
    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunction,
        "LD2410Task",
        stack,
        this,
        priority,
        &taskHandle_,
        core
    );
    if (result != pdPASS) {
        taskHandle_ = nullptr;
        return false;
    }
    return true;
}

// Ferma il task se in esecuzione.
void ld2410::stopAutoReadTask() {
    if (taskHandle_ != nullptr) {
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
}
#endif

// Reports whether autoReadTask() is currently running. Always false on
// platforms without the FreeRTOS task path, so consumer code can branch on
// runtime mode without compile-time #ifs.
bool ld2410::isAutoReadTaskRunning() {
#if defined(ESP32)
    return taskHandle_ != nullptr;
#else
    return false;
#endif
}

// Acquire the per-instance command mutex (ESP32). Returns true if the
// lock was obtained within timeout_ms or if no mutex exists yet (begin()
// not called — single-thread assumption). Mirrors the always-false
// stub pattern of isAutoReadTaskRunning() so the .h declaration is
// universal and consumer code can call request*/set* without #if guards.
bool ld2410::lock_command_(uint32_t timeout_ms) {
#if defined(ESP32)
	if (cmd_mutex_ == nullptr) {
		return true;
	}
	return xSemaphoreTake(cmd_mutex_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
#else
	(void)timeout_ms;
	return true;
#endif
}

// Release the per-instance command mutex (ESP32). No-op on other
// platforms and on ESP32 if the mutex was never created.
void ld2410::unlock_command_() {
#if defined(ESP32)
	if (cmd_mutex_ == nullptr) return;
	xSemaphoreGive(cmd_mutex_);
#endif
}


bool ld2410::presenceDetected()
{
	return target_type_ != 0;
}

bool ld2410::stationaryTargetDetected()
{
	if((target_type_ & 0x02) && stationary_target_distance_ > 0 && stationary_target_energy_ > 0)
	{
		return true;
	}
	return false;
}

uint16_t ld2410::stationaryTargetDistance()
{
	//if(stationary_target_energy_ > 0)
	{
		return stationary_target_distance_;
	}
	//return 0;
}

uint8_t ld2410::stationaryTargetEnergy()
{
	//if(stationary_target_distance_ > 0)
	{
		return stationary_target_energy_;
	}
	//return 0;
}

bool ld2410::movingTargetDetected()
{
	if((target_type_ & 0x01) && moving_target_distance_ > 0 && moving_target_energy_ > 0)
	{
		return true;
	}
	return false;
}

uint16_t ld2410::movingTargetDistance()
{
	//if(moving_target_energy_ > 0)
	{
		return moving_target_distance_;
	}
	//return 0;
}

uint8_t ld2410::movingTargetEnergy() {
    if (moving_target_energy_ > 100) {
        return 100;  // Limita a 100 se il valore è superiore
    }
    return moving_target_energy_;  // Restituisci il valore se è già compreso tra 0 e 100
}

uint16_t ld2410::detectionDistance() {
    return detection_distance_;
}

uint8_t ld2410::movingEnergyAtGate(uint8_t gate) {
    if (gate >= LD2410_GATE_COUNT) {
        return 0;
    }
    return engineering_motion_energy_[gate];
}

uint8_t ld2410::stationaryEnergyAtGate(uint8_t gate) {
    if (gate >= LD2410_GATE_COUNT) {
        return 0;
    }
    return engineering_stationary_energy_[gate];
}

bool ld2410::engineeringRetrieved() {
    return engineering_data_received_;
}


bool ld2410::check_frame_end_() {
    const uint8_t *head = ack_frame_ ? LD2410_CMD_FRAME_HEAD : LD2410_DATA_FRAME_HEAD;
    const uint8_t *tail = ack_frame_ ? LD2410_CMD_FRAME_TAIL : LD2410_DATA_FRAME_TAIL;
    return (radar_data_frame_[0] == head[0] &&
            radar_data_frame_[1] == head[1] &&
            radar_data_frame_[2] == head[2] &&
            radar_data_frame_[3] == head[3] &&
            radar_data_frame_[radar_data_frame_position_ - 4] == tail[0] &&
            radar_data_frame_[radar_data_frame_position_ - 3] == tail[1] &&
            radar_data_frame_[radar_data_frame_position_ - 2] == tail[2] &&
            radar_data_frame_[radar_data_frame_position_ - 1] == tail[3]);
}

void ld2410::print_frame_()
{
	if(debug_uart_ != nullptr)
	{
		if(ack_frame_ == true)
		{
			debug_uart_->print(F("\nCmnd : "));
		}
		else
		{
			debug_uart_->print(F("\nData : "));
		}
		for(uint8_t i = 0; i < radar_data_frame_position_ ; i ++)
		{
			if(radar_data_frame_[i] < 0x10)
			{
				debug_uart_->print('0');
			}
			debug_uart_->print(radar_data_frame_[i],HEX);
			debug_uart_->print(' ');
		}
	}
}

// Length-driven frame parser.
//
// Layout (HLK-LD2410C protocol V1.00 §2.3):
//   [0..3]  4-byte magic header  (F4 F3 F2 F1 for data, FD FC FB FA for cmd)
//   [4..5]  intra-frame data length, little-endian (uint16)
//   [6..N]  intra-frame data, exactly `intra_len` bytes
//   [N+1..N+4] 4-byte footer    (F8 F7 F6 F5 for data, 04 03 02 01 for cmd)
// Total frame length = intra_len + 10.
//
// The previous parser (a) committed to a frame after a single 0xF4/0xFD byte
// and (b) tested the footer pattern after every byte from position 8 onwards.
// Both broke alignment under any kind of byte loss or when intra-frame data
// happened to contain bytes resembling the footer (e.g. a stationary distance
// of 244 cm has 0xF4 as its low byte). This version validates all four header
// bytes before committing, captures the length once, accumulates exactly that
// many bytes, then validates the footer and parses. On any mid-frame
// inconsistency it resyncs, and if the offending byte is itself a candidate
// header start it is reused as the new position 0 so we don't lose a header.
bool ld2410::read_frame_() {
    uint8_t byte_read;
    while (read_from_buffer(byte_read)) {
        const uint8_t pos = radar_data_frame_position_;

        // Stage A: locate the magic header (positions 0..3).
        if (pos == 0) {
            if (byte_read == LD2410_DATA_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = false;
            } else if (byte_read == LD2410_CMD_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = true;
            }
            // else: drop byte, keep scanning
            continue;
        }

        if (pos < 4) {
            const uint8_t expected = (ack_frame_ ? LD2410_CMD_FRAME_HEAD : LD2410_DATA_FRAME_HEAD)[pos];
            if (byte_read == expected) {
                radar_data_frame_[pos] = byte_read;
                radar_data_frame_position_ = pos + 1;
            } else if (byte_read == LD2410_DATA_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = false;
            } else if (byte_read == LD2410_CMD_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = true;
            } else {
                radar_data_frame_position_ = 0;
            }
            continue;
        }

        // Stage B: capture the length field, then the body.
        radar_data_frame_[pos] = byte_read;
        radar_data_frame_position_ = pos + 1;

        if (pos == 5) {
            const uint16_t intra = (uint16_t)radar_data_frame_[4]
                                 | ((uint16_t)radar_data_frame_[5] << 8);
            if (intra == 0 || intra + 10 > LD2410_MAX_FRAME_LENGTH) {
                radar_data_frame_position_ = 0;
            }
            continue;
        }

        if (pos < 6) continue;

        const uint16_t intra = (uint16_t)radar_data_frame_[4]
                             | ((uint16_t)radar_data_frame_[5] << 8);
        const uint16_t total = intra + 10;
        if (radar_data_frame_position_ < total) continue;

        // Frame fully received. Validate footer once at the known position.
        const bool footer_ok = check_frame_end_();
        const bool was_ack   = ack_frame_;
        bool ok = false;
        if (footer_ok) {
            ok = was_ack ? parse_command_frame_() : parse_data_frame_();
        }
        radar_data_frame_position_ = 0;
        if (ok) return true;
    }
    return false;
}

bool ld2410::parse_data_frame_() {
    uint16_t intra_frame_data_length = radar_data_frame_[4] | (radar_data_frame_[5] << 8);

    // Frame total = header(4) + length(2) + intra-frame + footer(4) = intra + 10
    if (radar_data_frame_position_ != intra_frame_data_length + 10) {
        return false;
    }

    uint8_t data_type = radar_data_frame_[6];

#if defined(LD2410_VARIANT_S)
    // S protocol exposes only the "standard" data type (0x01) on this code
    // path. Auto-threshold-progress (0x03) and minimal frame (0x6E…0x62)
    // are handled by step 10b/c. The S standard frame layout is fixed at
    // 70 bytes intra (HLK-LD2410S §2.1 Table 2-1):
    //   1 B data type + 1 B target state + 2 B object distance
    //   + 2 B reserved + 64 B per-gate energy.
    // Note: S does NOT use the 0xAA / 0x55 / 0x00 intra-frame markers
    // that base/C use, so we only validate length here.
    if (data_type != LD2410_DATA_TYPE_STANDARD || intra_frame_data_length != 70) {
        return false;
    }
#else
    // base/C: 0x01 engineering frame (basic info + per-gate appended) or
    // 0x02 basic frame. Both use 0xAA / 0x55 / 0x00 intra-frame markers
    // (HLK Table 9 / Table 10).
    if (data_type != LD2410_DATA_TYPE_ENGINEERING && data_type != LD2410_DATA_TYPE_BASIC) {
        return false;
    }
    uint16_t tail_index = intra_frame_data_length + 4;        // expects LD2410_DATA_INTRA_TAIL  (0x55)
    uint16_t cal_index  = intra_frame_data_length + 5;        // expects LD2410_DATA_INTRA_CHECK (0x00)
    if (radar_data_frame_[7]          != LD2410_DATA_INTRA_HEAD ||
        radar_data_frame_[tail_index] != LD2410_DATA_INTRA_TAIL ||
        radar_data_frame_[cal_index]  != LD2410_DATA_INTRA_CHECK) {
        return false;
    }
#endif

    // Critical section: on ESP32 dual-core the task pinned to core 0 may
    // update these fields while the user loop on core 1 reads them. The
    // portMUX prevents inconsistent state between fields of a single frame
    // and acts as a memory barrier for the getters.
#if defined(ESP32)
    portENTER_CRITICAL(&data_mux_);
#endif

#if defined(LD2410_VARIANT_S)
    // S standard frame field decode (offsets are full-frame indices).
    target_type_        = radar_data_frame_[7];
    detection_distance_ = (uint16_t)radar_data_frame_[8] | ((uint16_t)radar_data_frame_[9] << 8);
    // base/C-only fields not present on S — clear them so getters return
    // consistent zeros instead of stale base/C-shaped values.
    moving_target_distance_     = 0;
    moving_target_energy_       = 0;
    stationary_target_distance_ = 0;
    stationary_target_energy_   = 0;

    // Per-gate energy block: 64 bytes starting at offset 12 (after data
    // type + target state + object distance + reserved bits).
    //
    // LAYOUT ASSUMPTION — UNVERIFIED ON HARDWARE:
    // The S V1.00 protocol document specifies "64 bytes" but does not
    // detail the per-gate breakdown. We assume the same convention as
    // base/C (1 byte per energy value), giving 16 motion + 16 stationary
    // = 32 bytes consumed; the remaining 32 bytes of the documented
    // 64-byte block are left unread pending HW verification. See the
    // STATUS block at the top of src/ld2410_variants/ld2410_s.h.
    for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
        engineering_motion_energy_[g]     = radar_data_frame_[12 + g];
        engineering_stationary_energy_[g] = radar_data_frame_[12 + LD2410_GATE_COUNT + g];
    }
    engineering_data_received_ = true;
#else
    // base/C basic-info layout (HLK Table 12 — full-frame indices):
    //   [8]    target state
    //   [9-10] moving target distance (cm, LE)
    //   [11]   moving target energy
    //   [12-13] stationary target distance (cm, LE)
    //   [14]   stationary target energy
    //   [15-16] detection distance (cm, LE)
    target_type_                = radar_data_frame_[8];
    moving_target_distance_     = *(uint16_t*)(&radar_data_frame_[9]);
    moving_target_energy_       = radar_data_frame_[11];
    stationary_target_distance_ = *(uint16_t*)(&radar_data_frame_[12]);
    stationary_target_energy_   = radar_data_frame_[14];
    detection_distance_         = *(uint16_t*)(&radar_data_frame_[15]);

    // Engineering frame extras (HLK Table 14):
    //   [17]    max moving gate N (redundant with max_moving_gate from 0x61 ACK)
    //   [18]    max stationary gate N
    //   [19..27] motion energies for gate 0..8
    //   [28..36] stationary energies for gate 0..8
    //   then M reserved bytes + intra tail/check (already validated above).
    if (data_type == LD2410_DATA_TYPE_ENGINEERING && intra_frame_data_length >= 33) {
        for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
            engineering_motion_energy_[g]     = radar_data_frame_[19 + g];
            engineering_stationary_energy_[g] = radar_data_frame_[28 + g];
        }
        engineering_data_received_ = true;
    }
#endif

    last_valid_frame_length = radar_data_frame_position_;
    radar_uart_last_packet_ = millis();
#if defined(ESP32)
    portEXIT_CRITICAL(&data_mux_);
#endif
    return true;
}


// ---------------------------------------------------------------------------
// Command synchronization helpers.
//
// begin_command_() is called by every request*/set* function right before
// writing the command bytes to the UART. It bumps cmd_seq_ and resets the
// expected-ACK opcode under data_mux_, and -- when no autoReadTask is
// running -- discards any stale frame state (circular buffer, in-progress
// parser state, UART RX FIFO bytes) from a previous command that may have
// timed out. When the task is running it must NOT touch UART or buffer
// state because the task owns them; the seq bump alone is enough to
// invalidate stale ACKs.
//
// wait_for_ack_() drives the wait loop. Two modes:
//   - no task running: drains UART hardware into the circular buffer and
//     calls read_frame_() ourselves until either a matching ACK is parsed
//     or the timeout expires.
//   - task running:    leaves UART alone; polls cmd_ack_seq_ while letting
//     the task fill the buffer and parse frames in its own context.
// In both modes the matching condition is identical:
//   cmd_ack_seq_ == cmd_seq_ AND latest_ack_ == expected_op
// returning latest_command_success_ on match, false on timeout.
// ---------------------------------------------------------------------------
void ld2410::begin_command_(uint8_t expected_op)
{
	bool task_running = false;
#if defined(ESP32)
	task_running = (taskHandle_ != nullptr);
	portENTER_CRITICAL(&data_mux_);
#endif
	expected_ack_opcode_ = expected_op;
	cmd_seq_++;
	if (!task_running) {
		// Discard any half-parsed frame and drop the circular buffer contents
		// so a stale ACK left over from a previous timeout cannot be matched
		// by the new command.
		buffer_tail = buffer_head;
		radar_data_frame_position_ = 0;
	}
#if defined(ESP32)
	portEXIT_CRITICAL(&data_mux_);
#endif
	if (!task_running) {
		// Drain in-flight UART bytes from the previous command's timeout.
		while (radar_uart_->available()) {
			radar_uart_->read();
		}
	}
}

bool ld2410::wait_for_ack_(uint8_t expected_op, uint32_t timeout_ms)
{
	uint32_t start = millis();
	bool task_running = false;
#if defined(ESP32)
	task_running = (taskHandle_ != nullptr);
#endif
	while (millis() - start < timeout_ms) {
		if (!task_running) {
			// Drive UART -> circular buffer -> parser ourselves
			while (radar_uart_->available()) {
				add_to_buffer(radar_uart_->read());
			}
			read_frame_();
		}

		bool got_ack = false;
		bool ok = false;
#if defined(ESP32)
		portENTER_CRITICAL(&data_mux_);
#endif
		if (cmd_ack_seq_ == cmd_seq_ && latest_ack_ == expected_op) {
			got_ack = true;
			ok = latest_command_success_;
		}
#if defined(ESP32)
		portEXIT_CRITICAL(&data_mux_);
#endif
		if (got_ack) {
			return ok;
		}

#if defined(ESP32)
		if (task_running) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
#endif
		yield();
	}
	return false;
}

bool ld2410::parse_command_frame_()
{
	uint16_t intra_frame_data_length_ = radar_data_frame_[4] + (radar_data_frame_[5] << 8);
	#ifdef LD2410_DEBUG_COMMANDS
	if(debug_uart_ != nullptr)
	{
		print_frame_();
		debug_uart_->print(F("\nACK frame payload: "));
		debug_uart_->print(intra_frame_data_length_);
		debug_uart_->print(F(" bytes"));
	}
	#endif
	// Atomic update of (latest_ack_, latest_command_success_, cmd_ack_seq_).
	// wait_for_ack_() reads this triplet to decide if the current command's
	// ACK has landed. cmd_ack_seq_ mirrors cmd_seq_ only on opcode match,
	// preventing false positives from stale ACKs of previously-timed-out commands.
	uint8_t this_ack = radar_data_frame_[6];
	bool this_success = (radar_data_frame_[8] == 0x00 && radar_data_frame_[9] == 0x00);
#if defined(ESP32)
	portENTER_CRITICAL(&data_mux_);
#endif
	latest_ack_ = this_ack;
	latest_command_success_ = this_success;
	if (this_ack == expected_ack_opcode_) {
		cmd_ack_seq_ = cmd_seq_;
	}
#if defined(ESP32)
	portEXIT_CRITICAL(&data_mux_);
#endif
	if(intra_frame_data_length_ == 8 && latest_ack_ == LD2410_OP_ENABLE_CFG)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for entering configuration mode: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_END_CFG)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for leaving configuration mode: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#ifdef LD2410_HAS_MAX_VALUES
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_SET_MAX_VALUES)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for setting max values: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#endif
#ifdef LD2410_HAS_READ_PARAMS
	else if(intra_frame_data_length_ == 28 && latest_ack_ == LD2410_OP_READ_PARAMS)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for current configuration: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			max_gate = radar_data_frame_[11];
			max_moving_gate = radar_data_frame_[12];
			max_stationary_gate = radar_data_frame_[13];
			motion_sensitivity[0] = radar_data_frame_[14];
			motion_sensitivity[1] = radar_data_frame_[15];
			motion_sensitivity[2] = radar_data_frame_[16];
			motion_sensitivity[3] = radar_data_frame_[17];
			motion_sensitivity[4] = radar_data_frame_[18];
			motion_sensitivity[5] = radar_data_frame_[19];
			motion_sensitivity[6] = radar_data_frame_[20];
			motion_sensitivity[7] = radar_data_frame_[21];
			motion_sensitivity[8] = radar_data_frame_[22];
			stationary_sensitivity[0] = radar_data_frame_[23];
			stationary_sensitivity[1] = radar_data_frame_[24];
			stationary_sensitivity[2] = radar_data_frame_[25];
			stationary_sensitivity[3] = radar_data_frame_[26];
			stationary_sensitivity[4] = radar_data_frame_[27];
			stationary_sensitivity[5] = radar_data_frame_[28];
			stationary_sensitivity[6] = radar_data_frame_[29];
			stationary_sensitivity[7] = radar_data_frame_[30];
			stationary_sensitivity[8] = radar_data_frame_[31];
			sensor_idle_time = radar_data_frame_[32];
			sensor_idle_time += (radar_data_frame_[33] << 8);
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nMax gate distance: "));
				debug_uart_->print(max_gate);
				debug_uart_->print(F("\nMax motion detecting gate distance: "));
				debug_uart_->print(max_moving_gate);
				debug_uart_->print(F("\nMax stationary detecting gate distance: "));
				debug_uart_->print(max_stationary_gate);
				debug_uart_->print(F("\nSensitivity per gate"));
				for(uint8_t i = 0; i < 9; i++)
				{
					debug_uart_->print(F("\nGate "));
					debug_uart_->print(i);
					debug_uart_->print(F(" ("));
					debug_uart_->print(i * 0.75);
					debug_uart_->print('-');
					debug_uart_->print((i+1) * 0.75);
					debug_uart_->print(F(" metres) Motion: "));
					debug_uart_->print(motion_sensitivity[i]);
					debug_uart_->print(F(" Stationary: "));
					debug_uart_->print(stationary_sensitivity[i]);

				}
				debug_uart_->print(F("\nSensor idle timeout: "));
				debug_uart_->print(sensor_idle_time);
				debug_uart_->print('s');
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#endif
#ifdef LD2410_HAS_GATE_SENSITIVITY
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_GATE_SENSITIVITY)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for setting sensitivity values: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#endif
#if defined(LD2410_VARIANT_S)
	// HLK-LD2410S §2.2.2 — FW version ACK has 8-byte intra-frame data
	// directly containing cmd-word + major + minor + patch (each 2 B LE);
	// there is NO status field, so latest_command_success_ (computed at
	// the top of this function from bytes [8-9] — which here are the
	// major version) cannot be trusted for this opcode. Accept any
	// well-framed ACK at the documented length.
	else if(intra_frame_data_length_ == 8 && latest_ack_ == LD2410_OP_FIRMWARE_VERSION)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for firmware version (S): "));
		}
		#endif
		// firmware_major/minor_version are uint8_t (base/C semantics);
		// store the LE low byte of each S 16-bit field. Typical S
		// firmwares fit in 8 bits (e.g. major=0x0001, minor=0x0000).
		firmware_major_version = radar_data_frame_[8];
		firmware_minor_version = radar_data_frame_[10];
		// firmware_bugfix_version is uint32_t — store the full 16-bit
		// S patch value zero-extended.
		firmware_bugfix_version = radar_data_frame_[12] | (radar_data_frame_[13] << 8);
		radar_uart_last_packet_ = millis();
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("OK"));
		}
		#endif
		return true;
	}
#else
	// HLK-LD2410C §2.2.8 — FW version ACK is 12-byte intra-frame data:
	// cmd-word + status + firmware-type + 2-byte major.minor + 4-byte bugfix.
	// (Note: byte [13] holds major, byte [12] holds minor — the protocol
	// document calls this "2 bytes major version number" displayed as Vmajor.minor.)
	else if(intra_frame_data_length_ == 12 && latest_ack_ == LD2410_OP_FIRMWARE_VERSION)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for firmware version: "));
		}
		#endif
		if(latest_command_success_)
		{
			firmware_major_version = radar_data_frame_[13];
			firmware_minor_version = radar_data_frame_[12];
			firmware_bugfix_version = radar_data_frame_[14];
			firmware_bugfix_version += radar_data_frame_[15]<<8;
			firmware_bugfix_version += radar_data_frame_[16]<<16;
			firmware_bugfix_version += radar_data_frame_[17]<<24;
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#endif
#ifdef LD2410_HAS_FACTORY_RESET
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_FACTORY_RESET)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for factory reset: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#endif
#ifdef LD2410_HAS_RESTART
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_RESTART)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for restart: "));
		}
		#endif
		if(latest_command_success_)
		{
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
			}
			#endif
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("failed"));
			}
			return false;
		}
	}
#endif
	else
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nUnknown ACK"));
		}
		#endif
	}

    if (latest_command_success_) {
        last_valid_frame_length = radar_data_frame_position_;  // Aggiungi questa linea
        radar_uart_last_packet_ = millis();
        return true;
    }
	return false;
}



void ld2410::send_command_preamble_()
{
	ld2410_write_cmd_frame_head(radar_uart_);
}

void ld2410::send_command_postamble_()
{
	ld2410_write_cmd_frame_tail(radar_uart_);
}

bool ld2410::enter_configuration_mode_()
{
	begin_command_(LD2410_OP_ENABLE_CFG);
	send_command_preamble_();
	radar_uart_->write((byte) 0x04);	//Command is four bytes long
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) LD2410_OP_ENABLE_CFG);	//Request enter command mode
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) 0x01);
	radar_uart_->write((byte) 0x00);
	send_command_postamble_();
	return wait_for_ack_(LD2410_OP_ENABLE_CFG, radar_uart_command_timeout_);
}

bool ld2410::leave_configuration_mode_()
{
	begin_command_(LD2410_OP_END_CFG);
	send_command_preamble_();
	radar_uart_->write((byte) 0x02);	//Command is two bytes long
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) LD2410_OP_END_CFG);	//Request leave command mode
	radar_uart_->write((byte) 0x00);
	send_command_postamble_();
	return wait_for_ack_(LD2410_OP_END_CFG, radar_uart_command_timeout_);
}

#ifdef LD2410_HAS_ENGINEERING_MODE
bool ld2410::requestStartEngineeringMode()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	// Per protocol §2.4.1, every config command must be issued inside an
	// enter/leave configuration window — otherwise the radar silently
	// rejects it. The other request*/set* helpers all wrap; these two
	// engineering-mode helpers were originally missing the wrap, which
	// caused the command to time out without an ACK.
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_START_ENGINEERING);
		send_command_preamble_();
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) LD2410_OP_START_ENGINEERING);	//Request enter engineering mode
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_START_ENGINEERING, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_ENGINEERING_MODE
bool ld2410::requestEndEngineeringMode()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_END_ENGINEERING);
		send_command_preamble_();
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) LD2410_OP_END_ENGINEERING);	//Request leave engineering mode
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_END_ENGINEERING, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_READ_PARAMS
bool ld2410::requestCurrentConfiguration()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_READ_PARAMS);
		send_command_preamble_();
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) LD2410_OP_READ_PARAMS);	//Request current configuration
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_READ_PARAMS, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

bool ld2410::requestFirmwareVersion()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_FIRMWARE_VERSION);
		send_command_preamble_();
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) LD2410_OP_FIRMWARE_VERSION);	//Request firmware version
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_FIRMWARE_VERSION, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}



#ifdef LD2410_HAS_RESTART
bool ld2410::requestRestart()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_RESTART);
		send_command_preamble_();
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) LD2410_OP_RESTART);	//Request restart
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_RESTART, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		if (ok) {
			// After ACK 0xA3 the radar reboots and emits ~500-800ms of garbage
			// or silence on its UART. If autoReadTask is running it would
			// happily feed those bytes to parse_data_frame_, occasionally
			// matching a 0xF4/0xFD frame start and clobbering the field cache
			// with synthetic data. Suspend the task across the reboot window,
			// drain the UART RX FIFO and the circular buffer twice, then
			// resume.
#if defined(ESP32)
			TaskHandle_t suspended = nullptr;
			portENTER_CRITICAL(&data_mux_);
			suspended = taskHandle_;
			portEXIT_CRITICAL(&data_mux_);
			if (suspended != nullptr) {
				vTaskSuspend(suspended);
			}
#endif
			while (radar_uart_->available()) radar_uart_->read();
			delay(800);
			while (radar_uart_->available()) radar_uart_->read();
#if defined(ESP32)
			portENTER_CRITICAL(&data_mux_);
			buffer_tail = buffer_head;
			radar_data_frame_position_ = 0;
			portEXIT_CRITICAL(&data_mux_);
			if (suspended != nullptr) {
				vTaskResume(suspended);
			}
#else
			buffer_tail = buffer_head;
			radar_data_frame_position_ = 0;
#endif
		}
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_FACTORY_RESET
bool ld2410::requestFactoryReset()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_FACTORY_RESET);
		send_command_preamble_();
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) LD2410_OP_FACTORY_RESET);	//Request factory reset
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_FACTORY_RESET, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_MAX_VALUES
bool ld2410::setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_SET_MAX_VALUES);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0014);                              // intra-frame data length (20 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_SET_MAX_VALUES);            // command word (LE)
		ld2410_write_le16(radar_uart_, LD2410_PARAM_MAX_MOVING);
		ld2410_write_le32(radar_uart_, moving);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_MAX_STATIONARY);
		ld2410_write_le32(radar_uart_, stationary);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_UNMANNED_DELAY);
		ld2410_write_le32(radar_uart_, inactivityTimer);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_SET_MAX_VALUES, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_GATE_SENSITIVITY
bool ld2410::setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_GATE_SENSITIVITY);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0014);                              // intra-frame data length (20 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_GATE_SENSITIVITY);          // command word (LE)
		ld2410_write_le16(radar_uart_, LD2410_PARAM_GATE);
		ld2410_write_le32(radar_uart_, gate);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_MOTION_SENS);
		ld2410_write_le32(radar_uart_, moving);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_STATIONARY_SENS);
		ld2410_write_le32(radar_uart_, stationary);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_GATE_SENSITIVITY, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

FrameData ld2410::getFrameData() const {
    // Usa last_valid_frame_length come lunghezza iniziale
    uint16_t frame_length = last_valid_frame_length;

    // Verifica dell'header
    if (radar_data_frame_[0] != 0xF4 || 
        radar_data_frame_[1] != 0xF3 || 
        radar_data_frame_[2] != 0xF2 || 
        radar_data_frame_[3] != 0xF1) {
        // Header non valido
        return {nullptr, 0};
    }

    // Verifica la lunghezza del frame dai byte 4 e 5
    uint16_t reported_length = radar_data_frame_[4] | (radar_data_frame_[5] << 8);
    reported_length += 10;  // Aggiungi 10 per header e footer

    // Usa la lunghezza minore tra quella riportata e last_valid_frame_length
    frame_length = (reported_length < frame_length) ? reported_length : frame_length;

    // Verifica che la lunghezza del frame sia valida
    if (frame_length > LD2410_MAX_FRAME_LENGTH || frame_length < 10) {
        // Lunghezza del frame non valida
        return {nullptr, 0};
    }

    // Verifica del footer
    if (radar_data_frame_[frame_length - 4] != 0xF8 || 
        radar_data_frame_[frame_length - 3] != 0xF7 || 
        radar_data_frame_[frame_length - 2] != 0xF6 || 
        radar_data_frame_[frame_length - 1] != 0xF5) {
        // Footer non valido
        return {nullptr, 0};
    }

    // Se tutti i controlli sono passati, restituisci i dati validi
    return {radar_data_frame_, frame_length};
}
#endif
