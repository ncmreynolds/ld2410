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
#include <string.h>
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

// LD2410_BUFFER_SIZE is 4 * LD2410_MAX_FRAME_LENGTH — 256 on base/C
// (power of 2, gcc folds % to AND) and 384 on S (NOT a power of 2, so
// % becomes a real division on AVR/Cortex-M0/ESP8266 and a Xtensa DIVU
// instruction on ESP32). Replacing the modulo with a single equality
// check + reset is identity-functional for any size and avoids the
// division on the variants where it is not free. Increment is always
// by 1 between checks so the simple ++/== form is sufficient (no need
// for "if (head >= SIZE) head -= SIZE").
void ld2410::add_to_buffer(uint8_t byte) {
    circular_buffer[buffer_head] = byte;
    if (++buffer_head == LD2410_BUFFER_SIZE) buffer_head = 0;

    if (buffer_head == buffer_tail) {
        // Buffer full: overwrite the oldest byte by advancing tail.
        if (++buffer_tail == LD2410_BUFFER_SIZE) buffer_tail = 0;
    }
}

bool ld2410::read_from_buffer(uint8_t &byte) {
    if (buffer_head == buffer_tail) {
        return false;
    }
    byte = circular_buffer[buffer_tail];
    if (++buffer_tail == LD2410_BUFFER_SIZE) buffer_tail = 0;
    return true;
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

// Atomic snapshot of engineering_motion_energy_[]. On ESP32 the copy is
// performed under portENTER_CRITICAL(data_mux_) so the LD2410_GATE_COUNT
// bytes belong to ONE frame — guaranteed not to interleave with the
// concurrent writes done by parse_data_frame_() running inside
// autoReadTask. On other platforms the lock degenerates to a plain memcpy.
void ld2410::snapshotEngineeringMotionEnergies(uint8_t out[LD2410_GATE_COUNT]) const {
#if defined(ESP32)
    portENTER_CRITICAL(&data_mux_);
#endif
    memcpy(out, engineering_motion_energy_, LD2410_GATE_COUNT);
#if defined(ESP32)
    portEXIT_CRITICAL(&data_mux_);
#endif
}

void ld2410::snapshotEngineeringStationaryEnergies(uint8_t out[LD2410_GATE_COUNT]) const {
#if defined(ESP32)
    portENTER_CRITICAL(&data_mux_);
#endif
    memcpy(out, engineering_stationary_energy_, LD2410_GATE_COUNT);
#if defined(ESP32)
    portEXIT_CRITICAL(&data_mux_);
#endif
}

void ld2410::snapshotTargetState(LD2410TargetState& out) const {
#if defined(ESP32)
    portENTER_CRITICAL(&data_mux_);
#endif
    out.target_type         = target_type_;
    out.moving_distance     = moving_target_distance_;
    out.moving_energy       = moving_target_energy_;
    out.stationary_distance = stationary_target_distance_;
    out.stationary_energy   = stationary_target_energy_;
    out.detection_distance  = detection_distance_;
#if defined(ESP32)
    portEXIT_CRITICAL(&data_mux_);
#endif
}

#ifdef LD2410_HAS_AUTO_THRESHOLD
uint16_t ld2410::autoThresholdProgress() { return auto_threshold_progress_; }
bool     ld2410::autoThresholdReceived() { return auto_threshold_received_; }
#endif


// Validate the 4-byte header magic at the start of the buffer. Stage A of
// read_frame_() already validates these byte-by-byte during accumulation, so
// this is a defensive post-hoc check (e.g. against a hypothetical buffer
// corruption between accumulation and frame finalisation).
bool ld2410::check_frame_start_() {
    const uint8_t *head = ack_frame_ ? LD2410_CMD_FRAME_HEAD : LD2410_DATA_FRAME_HEAD;
    return memcmp(radar_data_frame_, head, 4) == 0;
}

// Validate the 4-byte trailer magic at the end of the assembled frame.
bool ld2410::check_frame_end_() {
    const uint8_t *tail = ack_frame_ ? LD2410_CMD_FRAME_TAIL : LD2410_DATA_FRAME_TAIL;
    return memcmp(&radar_data_frame_[radar_data_frame_position_ - 4], tail, 4) == 0;
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

        // Stage A: locate the magic header at pos == 0.
        // Three possible heads:
        //   F4 → standard data frame (4-byte header, intra+length+body+4-byte tail)
        //   FD → command frame (4-byte header)
        //   6E → S minimal frame (1-byte header, S-only, fixed 5-byte total)
        if (pos == 0) {
            if (byte_read == LD2410_DATA_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = false;
#if defined(LD2410_VARIANT_S)
                minimal_frame_ = false;
#endif
            } else if (byte_read == LD2410_CMD_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = true;
#if defined(LD2410_VARIANT_S)
                minimal_frame_ = false;
#endif
#if defined(LD2410_VARIANT_S)
            } else if (byte_read == LD2410_MINIMAL_FRAME_HEAD) {
                // S-only minimal frame (HLK-LD2410S §2.1 Table 2-1).
                // Fixed 5 bytes total: 6E + state + dist_lo + dist_hi + 62.
                // Activated by the user via setOutputMode(0x7A).
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = false;
                minimal_frame_ = true;
#endif
            }
            // else: drop byte, keep scanning
            continue;
        }

#if defined(LD2410_VARIANT_S)
        // Minimal-frame fast path: 5 bytes total.
        // [0]=0x6E (validated above), [1]=state, [2-3]=dist LE, [4] must be 0x62.
        if (minimal_frame_) {
            radar_data_frame_[pos] = byte_read;
            radar_data_frame_position_ = pos + 1;
            if (pos == 4) {
                const bool ok = (byte_read == LD2410_MINIMAL_FRAME_TAIL) && parse_minimal_frame_();
                radar_data_frame_position_ = 0;
                minimal_frame_ = false;
                if (ok) return true;
            }
            continue;
        }
#endif

        if (pos < 4) {
            const uint8_t expected = (ack_frame_ ? LD2410_CMD_FRAME_HEAD : LD2410_DATA_FRAME_HEAD)[pos];
            if (byte_read == expected) {
                radar_data_frame_[pos] = byte_read;
                radar_data_frame_position_ = pos + 1;
            } else if (byte_read == LD2410_DATA_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = false;
#if defined(LD2410_VARIANT_S)
                minimal_frame_ = false;
#endif
            } else if (byte_read == LD2410_CMD_FRAME_HEAD[0]) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = true;
#if defined(LD2410_VARIANT_S)
                minimal_frame_ = false;
            } else if (byte_read == LD2410_MINIMAL_FRAME_HEAD) {
                // Resync into a minimal-frame start byte mid-header.
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                ack_frame_ = false;
                minimal_frame_ = true;
#endif
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

        // Frame fully received. Validate envelope (header + footer) at known positions.
        const bool envelope_ok = check_frame_start_() && check_frame_end_();
        const bool was_ack   = ack_frame_;
        bool ok = false;
        if (envelope_ok) {
            ok = was_ack ? parse_command_frame_() : parse_data_frame_();
        }
        radar_data_frame_position_ = 0;
        if (ok) return true;
    }
    return false;
}

#if defined(LD2410_VARIANT_S)
// Decode the 5-byte minimal frame (HLK-LD2410S §2.1 Table 2-1):
//   [0] = 0x6E (validated by read_frame_'s Stage A)
//   [1] = target state (0/1 = no one, 2/3 = present)
//   [2..3] = object distance (cm, LE)
//   [4] = 0x62 (validated by caller before invocation)
//
// The minimal frame carries no per-gate data, so engineering arrays are
// NOT cleared here — the user keeps whatever was last reported via a
// standard frame. base/C-only fields are zeroed for consistency with the
// standard-frame S decode path (see parse_data_frame_).
bool ld2410::parse_minimal_frame_() {
#if defined(ESP32)
    portENTER_CRITICAL(&data_mux_);
#endif
    target_type_        = radar_data_frame_[1];
    detection_distance_ = (uint16_t)radar_data_frame_[2] | ((uint16_t)radar_data_frame_[3] << 8);
    moving_target_distance_     = 0;
    moving_target_energy_       = 0;
    stationary_target_distance_ = 0;
    stationary_target_energy_   = 0;
    last_valid_frame_length     = 5;
    radar_uart_last_packet_     = millis();
#if defined(ESP32)
    portEXIT_CRITICAL(&data_mux_);
#endif
    return true;
}
#endif

bool ld2410::parse_data_frame_() {
    uint16_t intra_frame_data_length = radar_data_frame_[4] | (radar_data_frame_[5] << 8);

    // Frame total = header(4) + length(2) + intra-frame + footer(4) = intra + 10
    if (radar_data_frame_position_ != intra_frame_data_length + 10) {
        return false;
    }

    uint8_t data_type = radar_data_frame_[6];

#if defined(LD2410_VARIANT_S)
    // S protocol data types accepted on this path:
    //   0x01 (standard) — 70-byte intra (HLK-LD2410S §2.1 Table 2-1)
    //   0x03 (auto-threshold progress) — 3-byte intra (HLK §2.2.9), only
    //         emitted while the 0x09 autoUpdateThreshold command is running
    // Minimal frame (6E…62) is a different envelope handled by step 10c.
    // S does NOT use the 0xAA / 0x55 / 0x00 intra-frame markers that
    // base/C use, so we only validate length here.
    const bool is_standard       = (data_type == LD2410_DATA_TYPE_STANDARD       && intra_frame_data_length == 70);
    const bool is_auto_threshold = (data_type == LD2410_DATA_TYPE_AUTO_THRESHOLD && intra_frame_data_length == 3);
    if (!is_standard && !is_auto_threshold) {
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
    if (data_type == LD2410_DATA_TYPE_AUTO_THRESHOLD) {
        // 0x03 progress frame: 2-byte LE at offsets [7..8] (after head + length + type).
        // The radar encodes "progress × 100"; we store the raw 16-bit value so
        // the user owns the divide (e.g. 5000 = 50.00%).
        auto_threshold_progress_ = (uint16_t)radar_data_frame_[7] | ((uint16_t)radar_data_frame_[8] << 8);
        auto_threshold_received_ = true;
    } else {
        // 0x01 standard frame field decode (offsets are full-frame indices).
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
        memcpy(engineering_motion_energy_,     &radar_data_frame_[12],                         LD2410_GATE_COUNT);
        memcpy(engineering_stationary_energy_, &radar_data_frame_[12 + LD2410_GATE_COUNT],     LD2410_GATE_COUNT);
        engineering_data_received_ = true;
    }
#else
    // base/C basic-info layout (HLK Table 12 — full-frame indices):
    //   [8]    target state
    //   [9-10] moving target distance (cm, LE)
    //   [11]   moving target energy
    //   [12-13] stationary target distance (cm, LE)
    //   [14]   stationary target energy
    //   [15-16] detection distance (cm, LE)
    // The 16-bit fields land at odd byte offsets, so a `*(uint16_t*)`
    // cast is double-UB: strict-aliasing violation AND unaligned access
    // (HardFault on Cortex-M0/M0+, trap on Xtensa with odd offset).
    // memcpy is the portable form; on LE hosts with optimisation gcc
    // emits a single half-word load identical to a hypothetical safe
    // unaligned access — no perf cost, full portability.
    target_type_                = radar_data_frame_[8];
    memcpy(&moving_target_distance_,     &radar_data_frame_[9],  2);
    moving_target_energy_       = radar_data_frame_[11];
    memcpy(&stationary_target_distance_, &radar_data_frame_[12], 2);
    stationary_target_energy_   = radar_data_frame_[14];
    memcpy(&detection_distance_,         &radar_data_frame_[15], 2);

    // Engineering frame extras (HLK Table 14):
    //   [17]    max moving gate N (redundant with max_moving_gate from 0x61 ACK)
    //   [18]    max stationary gate N
    //   [19..27] motion energies for gate 0..8
    //   [28..36] stationary energies for gate 0..8
    //   then M reserved bytes + intra tail/check (already validated above).
    if (data_type == LD2410_DATA_TYPE_ENGINEERING && intra_frame_data_length >= 33) {
        memcpy(engineering_motion_energy_,     &radar_data_frame_[19], LD2410_GATE_COUNT);
        memcpy(engineering_stationary_energy_, &radar_data_frame_[28], LD2410_GATE_COUNT);
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
		return report_command_result_(latest_command_success_);
	}
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_END_CFG)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for leaving configuration mode: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
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
		return report_command_result_(latest_command_success_);
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
		if (latest_command_success_) {
			max_gate = radar_data_frame_[11];
			max_moving_gate = radar_data_frame_[12];
			max_stationary_gate = radar_data_frame_[13];
			memcpy(motion_sensitivity,     &radar_data_frame_[14], 9);
			memcpy(stationary_sensitivity, &radar_data_frame_[23], 9);
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
		}
		return report_command_result_(latest_command_success_);
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
		return report_command_result_(latest_command_success_);
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
		if (latest_command_success_) {
			firmware_major_version = radar_data_frame_[13];
			firmware_minor_version = radar_data_frame_[12];
			firmware_bugfix_version = radar_data_frame_[14];
			firmware_bugfix_version += radar_data_frame_[15]<<8;
			firmware_bugfix_version += radar_data_frame_[16]<<16;
			firmware_bugfix_version += radar_data_frame_[17]<<24;
		}
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_BAUD_RATE
	// HLK-LD2410 §2.2.9 — baud-rate ACK is 4-byte intra (cmd-word + 2-byte
	// status). Identical envelope to setMaxValues / restart / factory-reset.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_SET_BAUD_RATE)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for setting baud rate: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_BLUETOOTH
	// HLK-LD2410C §2.2.12 — Bluetooth on/off ACK is 4-byte intra
	// (cmd-word + 2-byte status). Same envelope as setBaudRate / restart.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_BLUETOOTH)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for Bluetooth on/off: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_SERIAL_NUMBER
	// HLK-LD2410S §2.2.5 — write-SN ACK is the standard 4-byte envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_WRITE_SN)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for write serial number: "));
		#endif
		return report_command_result_(latest_command_success_);
	}
	// HLK-LD2410S §2.2.6 — read-SN ACK is intra=14 = 2 (cmd) + 2 (status)
	// + 2 (length, always 8) + 8 (SN bytes). Frame offsets:
	//   [10][11] = SN length (LE; expected 0x0008)
	//   [12..19] = SN bytes
	else if(intra_frame_data_length_ == 14 && latest_ack_ == LD2410_OP_READ_SN)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for read serial number: "));
		#endif
		if (latest_command_success_) {
			memcpy(serial_number, &radar_data_frame_[12], 8);
		}
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_AUTO_THRESHOLD
	// HLK-LD2410S §2.2.9 — autoUpdateThreshold has no documented
	// command-channel ACK; the radar reports progress via the data-type
	// 0x03 frame instead. Tolerate firmwares that DO send a 4-byte ACK
	// matching the standard envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_AUTO_THRESHOLD)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for auto-threshold start: "));
		#endif
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_TRIGGER_THRESHOLD
	// HLK-LD2410S §2.2.10 — write-trigger-thresholds ACK is the standard
	// 4-byte success/fail envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_WRITE_TRIGGER_THRESH)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for write trigger thresholds: "));
		#endif
		return report_command_result_(latest_command_success_);
	}
	// HLK-LD2410S §2.2.11 — read-trigger-thresholds ACK: intra=68 = 4 +
	// 16 × 4. 16 LE 4-byte values starting at offset [10]; only the low
	// byte of each is kept (matches API doc).
	else if(intra_frame_data_length_ == 68 && latest_ack_ == LD2410_OP_READ_TRIGGER_THRESH)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for read trigger thresholds: "));
		#endif
		if (latest_command_success_) {
			for (uint8_t g = 0; g < 16; g++) {
				trigger_thresholds[g] = radar_data_frame_[10 + 4 * g];
			}
		}
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_HOLD_THRESHOLD
	// HLK-LD2410S §2.2.12 — write-hold-thresholds ACK is the standard
	// 4-byte success/fail envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_WRITE_HOLD_THRESH)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for write hold thresholds: "));
		#endif
		return report_command_result_(latest_command_success_);
	}
	// HLK-LD2410S §2.2.13 — read-hold-thresholds ACK: intra=68 = 4 + 16×4.
	else if(intra_frame_data_length_ == 68 && latest_ack_ == LD2410_OP_READ_HOLD_THRESH)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr) debug_uart_->print(F("\nACK for read hold thresholds: "));
		#endif
		if (latest_command_success_) {
			for (uint8_t g = 0; g < 16; g++) {
				hold_thresholds[g] = radar_data_frame_[10 + 4 * g];
			}
		}
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_GENERIC_PARAMS
	// HLK-LD2410S §2.2.7 — write-generic-parameters ACK is the standard
	// 4-byte success/fail envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_WRITE_GENERIC_PARAMS)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for write generic parameters: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
	// HLK-LD2410S §2.2.8 — read-generic-parameters ACK: intra=28 (= 4 +
	// 6×4), with 6 LE 4-byte values starting at offset [10]. Order matches
	// the request order: farthest, nearest, unmanned-delay, status-freq,
	// distance-freq, response-speed.
	// (PDF prints intra as 0x1A=26 but the documented payload sums to 0x1C=28
	// — typo in PDF; the math is authoritative.)
	else if(intra_frame_data_length_ == 28 && latest_ack_ == LD2410_OP_READ_GENERIC_PARAMS)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for read generic parameters: "));
		}
		#endif
		if (latest_command_success_) {
			detect_farthest_gate = (uint8_t)radar_data_frame_[10];
			detect_nearest_gate  = (uint8_t)radar_data_frame_[14];
			unmanned_delay_s     = (uint16_t)radar_data_frame_[18]
			                     | ((uint16_t)radar_data_frame_[19] << 8);
			status_report_freq   = (uint8_t)radar_data_frame_[22];
			distance_report_freq = (uint8_t)radar_data_frame_[26];
			response_speed       = (uint8_t)radar_data_frame_[30];
		}
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_OUTPUT_MODE
	// HLK-LD2410S §2.2.1 — switch-output-mode ACK is the standard
	// 4-byte success/fail envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_OUTPUT_MODE)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for switch output mode: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_BLUETOOTH
	// HLK-LD2410C §2.2.14 — obtainBluetoothPermissions ACK is the standard
	// 4-byte success/fail envelope. NOTE: the PDF says the radar replies
	// only over BLE, not UART — so this branch typically never fires
	// on the host side. Kept for protocol completeness in case some
	// firmware revisions mirror the ACK to UART.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_BLUETOOTH_PERMS)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for Bluetooth permissions: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
	// HLK-LD2410C §2.2.15 — setBluetoothPassword ACK is 4-byte standard.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_BLUETOOTH_PASSWORD)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for set Bluetooth password: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_MAC_ADDRESS
	// HLK-LD2410C §2.2.13 — MAC address ACK is 10-byte intra: cmd-word +
	// 2-byte status + 6 bytes MAC in WIRE / network order (big-endian as
	// printed by the radar — preserved as-is in mac_address[]).
	// Frame offsets (header is 4B, intra-len is 2B):
	//   [6][7]    = cmd-word ACK (A5 01)
	//   [8][9]    = status
	//   [10..15]  = MAC[0..5]
	else if(intra_frame_data_length_ == 10 && latest_ack_ == LD2410_OP_GET_MAC)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for MAC address: "));
		}
		#endif
		if (latest_command_success_) {
			memcpy(mac_address, &radar_data_frame_[10], 6);
		}
		return report_command_result_(latest_command_success_);
	}
#endif
#ifdef LD2410_HAS_DISTANCE_RESOLUTION
	// HLK-LD2410C §2.2.16 — set-distance-resolution ACK is the standard
	// 4-byte success/fail envelope.
	else if(intra_frame_data_length_ == 4 && latest_ack_ == LD2410_OP_DISTANCE_RESOLUTION_SET)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for set distance resolution: "));
		}
		#endif
		return report_command_result_(latest_command_success_);
	}
	// HLK-LD2410C §2.2.17 — query-distance-resolution ACK is 6-byte intra:
	// cmd-word + 2-byte status + 2-byte LE index (offsets [10][11]).
	else if(intra_frame_data_length_ == 6 && latest_ack_ == LD2410_OP_DISTANCE_RESOLUTION_GET)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for query distance resolution: "));
		}
		#endif
		if (latest_command_success_) {
			distance_resolution = (uint16_t)radar_data_frame_[10]
			                    | ((uint16_t)radar_data_frame_[11] << 8);
		}
		// Helper prints "OK"; we append " (<val>)" after it to preserve
		// the original verbose-debug format that included the resolved
		// distance-resolution value.
		const bool ok = report_command_result_(latest_command_success_);
		#ifdef LD2410_DEBUG_COMMANDS
		if (ok && debug_uart_ != nullptr) {
			debug_uart_->print(F(" ("));
			debug_uart_->print(distance_resolution);
			debug_uart_->print(F(")"));
		}
		#endif
		return ok;
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
		return report_command_result_(latest_command_success_);
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
		return report_command_result_(latest_command_success_);
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

// All "trivial" no-arg commands share the same on-the-wire body:
//   intra-length 02 00, opcode OP, padding 00.
// Helper bumps cmd_seq_, emits the full command envelope, and returns.
// Caller follows up with wait_for_ack_(opcode, timeout). Saves ~4 vcalls
// per command and removes ~8 lines of identical boilerplate from each
// of 9 callers (-200 B flash on AVR; on ESP32/ESP8266 the win is mainly
// code clarity).
void ld2410::send_simple_command_(uint8_t opcode)
{
	begin_command_(opcode);
	send_command_preamble_();
	const uint8_t cmd[4] = { 0x02, 0x00, opcode, 0x00 };
	radar_uart_->write(cmd, sizeof(cmd));
	send_command_postamble_();
}

// Canonical epilogue for an ACK branch in parse_command_frame_.
// Was duplicated 25 times in the chain — every branch ended with the
// same "if success { last_packet, debug OK, return true } else { debug
// failed, return false }" body. Collapse it here.
//
// Asymmetric debug gating is preserved verbatim from the original:
//   - "OK" is gated by LD2410_DEBUG_COMMANDS  (verbose flag)
//   - "failed" is NOT gated (always printed if debug_uart_ is set)
// because failure is operationally informative and worth surfacing
// even without the verbose flag.
bool ld2410::report_command_result_(bool success)
{
	if (success) {
		radar_uart_last_packet_ = millis();
		#ifdef LD2410_DEBUG_COMMANDS
		if (debug_uart_ != nullptr) {
			debug_uart_->print(F("OK"));
		}
		#endif
		return true;
	}
	if (debug_uart_ != nullptr) {
		debug_uart_->print(F("failed"));
	}
	return false;
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
	send_simple_command_(LD2410_OP_END_CFG);
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
		send_simple_command_(LD2410_OP_START_ENGINEERING);
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
		send_simple_command_(LD2410_OP_END_ENGINEERING);
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
		send_simple_command_(LD2410_OP_READ_PARAMS);
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
		send_simple_command_(LD2410_OP_FIRMWARE_VERSION);
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
		send_simple_command_(LD2410_OP_RESTART);
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
		send_simple_command_(LD2410_OP_FACTORY_RESET);
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

#ifdef LD2410_HAS_BAUD_RATE
// 0xA1 §2.2.9 — set serial port baud rate (base/C only).
// Send: cmd-word + 2-byte LE index (LD2410_BAUD_INDEX_*). Intra length = 4.
// ACK : cmd-word + 2-byte LE status. Intra length = 4 — handled in
//       parse_command_frame_ via the LD2410_OP_SET_BAUD_RATE branch.
// The new baud takes effect only after a module restart; the caller is
// responsible for reopening the host UART at the matching rate.
// See docs/method-coverage.md Table 1 row 0xA1 (regression vs v0.1.3,
// upstream issue #39).
bool ld2410::setBaudRate(uint16_t baud_index)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_SET_BAUD_RATE);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0004);                              // intra-frame data length (4 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_SET_BAUD_RATE);             // command word (LE)
		ld2410_write_le16(radar_uart_, baud_index);                          // baud-rate index (LE)
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_SET_BAUD_RATE, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_BLUETOOTH
// 0xA4 §2.2.12 (C only) — enable/disable the BLE radio. Intra=4
// (cmd-word + 2-byte LE state). ACK is the standard 4-byte success/fail
// envelope handled in parse_command_frame_. Effect is post-restart.
// See docs/method-coverage.md Table 1 row 0xA4.
bool ld2410::setBluetooth(bool on)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_BLUETOOTH);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0004);                              // intra-frame data length (4 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_BLUETOOTH);                 // command word (LE)
		ld2410_write_le16(radar_uart_, on ? LD2410_BLUETOOTH_ON : LD2410_BLUETOOTH_OFF);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_BLUETOOTH, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_SERIAL_NUMBER
// 0x10 §2.2.5 (S only) — write the 8-byte sensor serial number. Send:
// cmd-word + 2-byte length (always 8) + 8 SN bytes in wire order;
// intra=12. ACK is the standard 4-byte success/fail envelope.
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
// See docs/method-coverage.md Table 1 row 0x10.
bool ld2410::writeSerialNumber(const uint8_t sn[8])
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_WRITE_SN);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x000C);                              // intra-frame data length (12 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_WRITE_SN);                  // command word (LE)
		ld2410_write_le16(radar_uart_, 0x0008);                              // SN length (always 8)
		radar_uart_->write(sn, 8);                                           // SN bytes in wire order
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_WRITE_SN, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

// 0x11 §2.2.6 (S only) — read the 8-byte sensor serial number. Send:
// cmd-word only, intra=2. ACK envelope: cmd-word + 2-byte status +
// 2-byte length + 8 SN bytes (intra=14). Decoded by parse_command_frame_'s
// 0x11 branch into serial_number[8].
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
// See docs/method-coverage.md Table 1 row 0x11.
bool ld2410::requestSerialNumber()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		send_simple_command_(LD2410_OP_READ_SN);
		bool ok = wait_for_ack_(LD2410_OP_READ_SN, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_AUTO_THRESHOLD
// 0x09 §2.2.9 (S only) — start automatic threshold tuning sweep.
// Send envelope: cmd-word + 3 × 2-byte LE values, intra=8. The HLK PDF
// does not document a command-channel ACK — the radar instead emits
// data-type 0x03 progress frames on the data channel (already parsed
// by step 10b). We still call wait_for_ack_ with a short timeout so
// firmwares that happen to ACK report success; if no ACK arrives the
// method returns false but the sweep may still be running.
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
// See docs/method-coverage.md Table 1 row 0x09.
bool ld2410::autoUpdateThreshold(uint16_t trigger_factor,
                                 uint16_t retention_factor,
                                 uint16_t scanning_time_s)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_AUTO_THRESHOLD);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0008);                              // intra-frame data length (8 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_AUTO_THRESHOLD);            // command word (LE)
		ld2410_write_le16(radar_uart_, trigger_factor);                      // trigger factor (LE)
		ld2410_write_le16(radar_uart_, retention_factor);                    // retention factor (LE)
		ld2410_write_le16(radar_uart_, scanning_time_s);                     // scanning time (s, LE)
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_AUTO_THRESHOLD, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#if defined(LD2410_HAS_TRIGGER_THRESHOLD) || defined(LD2410_HAS_HOLD_THRESHOLD)
// Shared body for the 0x72 (trigger) / 0x76 (hold) write-thresholds
// commands. Both share the same envelope: cmd-word + 16 × (2-byte gate
// word LE + 4-byte value LE) = 98 byte intra (= 0x62). Only the opcode
// differs. Each gate's value is widened from uint8_t (the documented
// range) to uint32_t for the wire.
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
bool ld2410::write_per_gate_thresholds_(uint8_t opcode, const uint8_t thresholds[16])
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(opcode);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0062);                              // intra-frame data length (98 bytes)
		ld2410_write_le16(radar_uart_, opcode);                              // command word (LE)
		for (uint8_t g = 0; g < 16; g++) {
			ld2410_write_le16(radar_uart_, g);                               // gate index (LE)
			ld2410_write_le32(radar_uart_, thresholds[g]);                   // threshold (LE 4B, low byte = value)
		}
		send_command_postamble_();
		bool ok = wait_for_ack_(opcode, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

// Shared body for the 0x73 (trigger) / 0x77 (hold) read-thresholds
// commands. Send envelope: cmd-word + 16 × 2-byte gate index = 34 byte
// intra (= 0x22). ACK is decoded in parse_command_frame_'s 0x73/0x77
// branches into trigger_thresholds[] / hold_thresholds[] respectively.
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
bool ld2410::request_per_gate_thresholds_(uint8_t opcode)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(opcode);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0022);                              // intra-frame data length (34 bytes)
		ld2410_write_le16(radar_uart_, opcode);                              // command word (LE)
		for (uint8_t g = 0; g < 16; g++) {
			ld2410_write_le16(radar_uart_, g);                               // gate index (LE)
		}
		send_command_postamble_();
		bool ok = wait_for_ack_(opcode, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_TRIGGER_THRESHOLD
// 0x72 §2.2.10 (S only) — write per-gate motion (trigger) thresholds.
// See docs/method-coverage.md Table 1 row 0x72.
bool ld2410::writeTriggerThresholds(const uint8_t thresholds[16])
{
	return write_per_gate_thresholds_(LD2410_OP_WRITE_TRIGGER_THRESH, thresholds);
}

// 0x73 §2.2.11 (S only) — read per-gate motion (trigger) thresholds.
// On success the 16 values land in trigger_thresholds[].
// See docs/method-coverage.md Table 1 row 0x73.
bool ld2410::requestTriggerThresholds()
{
	return request_per_gate_thresholds_(LD2410_OP_READ_TRIGGER_THRESH);
}
#endif

#ifdef LD2410_HAS_HOLD_THRESHOLD
// 0x76 §2.2.12 (S only) — write per-gate stationary (hold) thresholds.
// See docs/method-coverage.md Table 1 row 0x76.
bool ld2410::writeHoldThresholds(const uint8_t thresholds[16])
{
	return write_per_gate_thresholds_(LD2410_OP_WRITE_HOLD_THRESH, thresholds);
}

// 0x77 §2.2.13 (S only) — read per-gate stationary (hold) thresholds.
// On success the 16 values land in hold_thresholds[].
// See docs/method-coverage.md Table 1 row 0x77.
bool ld2410::requestHoldThresholds()
{
	return request_per_gate_thresholds_(LD2410_OP_READ_HOLD_THRESH);
}
#endif

#ifdef LD2410_HAS_GENERIC_PARAMS
// 0x70 §2.2.7 (S only) — write all six generic parameters in one shot.
// Send envelope: cmd-word + 6 × (param-word LE 2B + value LE 4B) = 38 B intra
// (= 0x26). ACK is the standard 4-byte success/fail envelope.
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
// See docs/method-coverage.md Table 1 row 0x70.
bool ld2410::writeGenericParameters(uint8_t detect_farthest_gate_in,
                                    uint8_t detect_nearest_gate_in,
                                    uint16_t unmanned_delay_s_in,
                                    uint8_t status_report_freq_in,
                                    uint8_t distance_report_freq_in,
                                    uint8_t response_speed_in)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_WRITE_GENERIC_PARAMS);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0026);                              // intra-frame data length (38 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_WRITE_GENERIC_PARAMS);      // command word (LE)
		ld2410_write_le16(radar_uart_, LD2410_PARAM_FARTHEST_GATE);
		ld2410_write_le32(radar_uart_, detect_farthest_gate_in);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_NEAREST_GATE);
		ld2410_write_le32(radar_uart_, detect_nearest_gate_in);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_UNMANNED_DELAY);
		ld2410_write_le32(radar_uart_, unmanned_delay_s_in);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_STATUS_FREQ);
		ld2410_write_le32(radar_uart_, status_report_freq_in);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_DISTANCE_FREQ);
		ld2410_write_le32(radar_uart_, distance_report_freq_in);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_RESPONSE_SPEED);
		ld2410_write_le32(radar_uart_, response_speed_in);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_WRITE_GENERIC_PARAMS, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

// 0x71 §2.2.8 (S only) — read all six generic parameters in one shot.
// Send envelope: cmd-word + 6 × (param-word LE 2B) = 14 B intra (= 0x0E).
// ACK envelope: cmd-word + 2-byte status + 6 × (4-byte LE value) = 28 B intra
// (= 0x1C). The HLK PDF prints the ACK length as 0x1A (= 26) but the actual
// payload table sums to 28 — typo in the PDF; the math (4 + 6×4) is
// authoritative. ACK is decoded by parse_command_frame_'s 0x71 branch.
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
// See docs/method-coverage.md Table 1 row 0x71.
bool ld2410::requestGenericParameters()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_READ_GENERIC_PARAMS);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x000E);                              // intra-frame data length (14 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_READ_GENERIC_PARAMS);       // command word (LE)
		ld2410_write_le16(radar_uart_, LD2410_PARAM_FARTHEST_GATE);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_NEAREST_GATE);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_UNMANNED_DELAY);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_STATUS_FREQ);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_DISTANCE_FREQ);
		ld2410_write_le16(radar_uart_, LD2410_PARAM_RESPONSE_SPEED);
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_READ_GENERIC_PARAMS, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_OUTPUT_MODE
// 0x7A §2.2.1 (S only) — switch reporting envelope between standard
// (F4F3F2F1 / data type 0x01) and minimal (6E…62). Send: cmd-word +
// 6-byte payload (intra=8); ACK is standard 4-byte. The 6-byte payload
// is variant-defined in ld2410_s.h as constexpr arrays (the value does
// not fit a single uintN_t).
//
// Note on PDF discrepancy: HLK §2.2.1 prints the "standard" payload as
// "00 00 00 01 00 00" in the Command-value text but "00 00 01 00 00 00"
// in the Send-data table. The two differ in the position of the lone
// 0x01 byte. ld2410_s.h follows the Command-value text (offset 3); the
// Send-data table is treated as a documentation typo. If the wrong
// variant turns out to be active on real hardware, swap the constants
// in ld2410_s.h rather than editing this method.
//
// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
// See docs/method-coverage.md Table 1 row 0x7A.
bool ld2410::setOutputMode(bool standard)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_OUTPUT_MODE);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0008);                              // intra-frame data length (8 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_OUTPUT_MODE);               // command word (LE)
		const uint8_t* payload = standard ? LD2410_OUTPUT_MODE_STANDARD_PAYLOAD
		                                  : LD2410_OUTPUT_MODE_MINIMAL_PAYLOAD;
		radar_uart_->write(payload, 6);                                      // 6-byte mode payload (wire order)
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_OUTPUT_MODE, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_BLUETOOTH
// 0xA8 §2.2.14 (C only) — unlock BLE control APIs by presenting a 6-byte
// password (factory default = "HiLink"). Send: cmd-word + 6 password bytes
// in wire order (intra=8). The HLK PDF notes that the ACK is delivered
// "only over Bluetooth, not the serial port" — wait_for_ack_ on UART
// will therefore typically time out and this method returns false.
// Implemented for protocol completeness; the BLE-side flow is out of
// scope for the UART driver.
// See docs/method-coverage.md Table 1 row 0xA8.
bool ld2410::obtainBluetoothPermissions(const uint8_t password[LD2410_BLUETOOTH_PASSWORD_LENGTH])
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_BLUETOOTH_PERMS);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0008);                              // intra-frame data length (8 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_BLUETOOTH_PERMS);           // command word (LE)
		radar_uart_->write(password, LD2410_BLUETOOTH_PASSWORD_LENGTH);      // 6 password bytes in wire order
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_BLUETOOTH_PERMS, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

// 0xA9 §2.2.15 (C only) — set the 6-byte BLE control password. Send: cmd-word
// + 6 password bytes in wire order (intra=8). ACK is the standard 4-byte
// success/fail envelope, delivered on UART.
// See docs/method-coverage.md Table 1 row 0xA9.
bool ld2410::setBluetoothPassword(const uint8_t password[LD2410_BLUETOOTH_PASSWORD_LENGTH])
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_BLUETOOTH_PASSWORD);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0008);                              // intra-frame data length (8 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_BLUETOOTH_PASSWORD);        // command word (LE)
		radar_uart_->write(password, LD2410_BLUETOOTH_PASSWORD_LENGTH);      // 6 password bytes in wire order
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_BLUETOOTH_PASSWORD, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_MAC_ADDRESS
// 0xA5 §2.2.13 (C only) — read the BLE MAC address. Send: cmd-word +
// 2-byte selector 0x0001 (intra=4). ACK: intra=10 with cmd-word +
// 2-byte status + 6-byte MAC in wire order. The MAC is copied into
// mac_address[] inside parse_command_frame_'s 0xA5 branch.
// See docs/method-coverage.md Table 1 row 0xA5.
bool ld2410::requestMACAddress()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_GET_MAC);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0004);                              // intra-frame data length (4 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_GET_MAC);                   // command word (LE)
		ld2410_write_le16(radar_uart_, 0x0001);                              // documented selector (HLK §2.2.13)
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_GET_MAC, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}
#endif

#ifdef LD2410_HAS_DISTANCE_RESOLUTION
// 0xAA §2.2.16 (C only) — set the per-gate distance resolution.
// Intra=4 (cmd-word + 2-byte LE index from LD2410_DISTANCE_RESOLUTION_*).
// ACK is the standard 4-byte envelope. Effect is post-restart.
// See docs/method-coverage.md Table 1 row 0xAA.
bool ld2410::setDistanceResolution(uint16_t resolution_index)
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		begin_command_(LD2410_OP_DISTANCE_RESOLUTION_SET);
		send_command_preamble_();
		ld2410_write_le16(radar_uart_, 0x0004);                              // intra-frame data length (4 bytes)
		ld2410_write_le16(radar_uart_, LD2410_OP_DISTANCE_RESOLUTION_SET);   // command word (LE)
		ld2410_write_le16(radar_uart_, resolution_index);                    // 0x0000 = 0.75 m, 0x0001 = 0.2 m
		send_command_postamble_();
		bool ok = wait_for_ack_(LD2410_OP_DISTANCE_RESOLUTION_SET, radar_uart_command_timeout_);
		delay(50);
		leave_configuration_mode_();
		return ok;
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

// 0xAB §2.2.17 (C only) — query the current distance resolution. Send: cmd-word
// only (intra=2, no value). ACK: intra=6 with cmd-word + 2-byte status + 2-byte
// LE index, decoded into distance_resolution by parse_command_frame_'s 0xAB branch.
// See docs/method-coverage.md Table 1 row 0xAB.
bool ld2410::requestDistanceResolution()
{
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if(enter_configuration_mode_())
	{
		delay(50);
		send_simple_command_(LD2410_OP_DISTANCE_RESOLUTION_GET);
		bool ok = wait_for_ack_(LD2410_OP_DISTANCE_RESOLUTION_GET, radar_uart_command_timeout_);
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
    // Verify data-frame header — same constants as check_frame_start_(),
    // applied here directly because getFrameData() is const-correct (the
    // helper isn't) and is fixed to data-frame magic (no ack_frame_
    // branching needed).
    if (memcmp(radar_data_frame_, LD2410_DATA_FRAME_HEAD, 4) != 0) {
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

    // Verify data-frame footer at the runtime-determined length
    // (frame_length is clamped to last_valid_frame_length above, so it
    // may be < radar_data_frame_position_ — that is why this does NOT
    // call check_frame_end_(), which uses the live position).
    if (memcmp(&radar_data_frame_[frame_length - 4], LD2410_DATA_FRAME_TAIL, 4) != 0) {
        return {nullptr, 0};
    }

    // Se tutti i controlli sono passati, restituisci i dati validi
    return {radar_data_frame_, frame_length};
}
#endif
