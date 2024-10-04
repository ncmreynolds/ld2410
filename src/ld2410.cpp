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


uint8_t circular_buffer[LD2410_BUFFER_SIZE];
uint16_t buffer_head = 0;
uint16_t buffer_tail = 0;

// Enumerazione per lo stato del parsing
enum ParseState { WAITING_START, READING_LENGTH, READING_DATA, CHECKING_END };
ParseState current_state = WAITING_START;

// Buffer pre-allocato per il frame
#define MAX_FRAME_SIZE 1024
uint8_t frame_buffer[MAX_FRAME_SIZE];
uint16_t frame_position = 0;
uint16_t frame_length = 0;


ld2410::ld2410()	//Constructor function
{
}

ld2410::~ld2410()	//Destructor function
{
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

bool ld2410::find_frame_start() {
    uint8_t byte;

    // Continua a leggere dal buffer finché non trovi l'inizio del frame
    while (read_from_buffer(byte)) {
        // Controlla se il byte è l'inizio di un frame
        if (byte == 0xF4 || byte == 0xFD) {
            // Inizia un nuovo frame
            radar_data_frame_[0] = byte;
            radar_data_frame_position_ = 1;  // Reset della posizione del frame
            frame_started_ = true;
            ack_frame_ = (byte == 0xFD);  // Determina il tipo di frame (ack o data)
            return true;
        }
    }

    return false;  // Nessun frame trovato
}

bool ld2410::begin(Stream &radarStream, bool waitForRadar)	{
	radar_uart_ = &radarStream;		//Set the stream used for the LD2410
	if(debug_uart_ != nullptr)
	{
		debug_uart_->println(F("ld2410 started"));
	}
	if(waitForRadar)
	{
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nLD2410 firmware: "));
		}
		if(requestFirmwareVersion())
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F(" v"));
				debug_uart_->print(firmware_major_version);
				debug_uart_->print('.');
				debug_uart_->print(firmware_minor_version);
				debug_uart_->print('.');
				debug_uart_->print(firmware_bugfix_version);
			}
			return true;
		}
		else
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("no response"));
			}
		}
	}
	else
	{
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nLD2410 library configured"));
		}
		return true;
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

// Funzione per avviare il task
void ld2410::autoReadTask(uint32_t stack, uint32_t priority, uint32_t core) {
    xTaskCreatePinnedToCore(
        taskFunction,
        "LD2410Task",
        stack,
        this,
        priority,
        nullptr,
        core
    );
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



bool ld2410::check_frame_end_() {
    if (ack_frame_) {
        return (radar_data_frame_[0] == 0xFD &&
                radar_data_frame_[1] == 0xFC &&
                radar_data_frame_[2] == 0xFB &&
                radar_data_frame_[3] == 0xFA &&
                radar_data_frame_[radar_data_frame_position_ - 4] == 0x04 &&
                radar_data_frame_[radar_data_frame_position_ - 3] == 0x03 &&
                radar_data_frame_[radar_data_frame_position_ - 2] == 0x02 &&
                radar_data_frame_[radar_data_frame_position_ - 1] == 0x01);
    } else {
        return (radar_data_frame_[0] == 0xF4 &&
                radar_data_frame_[1] == 0xF3 &&
                radar_data_frame_[2] == 0xF2 &&
                radar_data_frame_[3] == 0xF1 &&
                radar_data_frame_[radar_data_frame_position_ - 4] == 0xF8 &&
                radar_data_frame_[radar_data_frame_position_ - 3] == 0xF7 &&
                radar_data_frame_[radar_data_frame_position_ - 2] == 0xF6 &&
                radar_data_frame_[radar_data_frame_position_ - 1] == 0xF5);
    }
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

bool ld2410::read_frame_() {
    uint8_t byte_read;
    while (read_from_buffer(byte_read)) {  // Corrected to pass byte_read by reference
        // If the frame has not started, check for the frame start
        if (!frame_started_) {
            if (byte_read == 0xF4 || byte_read == 0xFD) {
                radar_data_frame_[0] = byte_read;
                radar_data_frame_position_ = 1;
                frame_started_ = true;
                ack_frame_ = (byte_read == 0xFD);  // Determine the type of frame
            }
        } else {
            // Continue accumulating the frame bytes
            radar_data_frame_[radar_data_frame_position_++] = byte_read;

            // After reading at least 8 bytes, verify the frame length
            if (radar_data_frame_position_ == 8) {
                uint16_t intra_frame_data_length = radar_data_frame_[4] | (radar_data_frame_[5] << 8);

                // Check if the frame length exceeds the maximum allowed
                if (intra_frame_data_length + 10 > LD2410_MAX_FRAME_LENGTH) {
                    frame_started_ = false;
                    radar_data_frame_position_ = 0;
                    continue;  // Skip this frame
                }
            }

            // Check if the frame is complete
            if (radar_data_frame_position_ >= 8 && check_frame_end_()) {
                frame_started_ = false;  // Reset state for the next frame

                // Process the frame (command or data)
                if (ack_frame_) {
                    return parse_command_frame_();
                } else {
                    return parse_data_frame_();
                }
            }
        }
    }
    return false;  // No complete frame was found
}

bool ld2410::parse_data_frame_() {
    uint16_t intra_frame_data_length = radar_data_frame_[4] | (radar_data_frame_[5] << 8);

    // Verifica se la lunghezza del frame è corretta
    if (radar_data_frame_position_ != intra_frame_data_length + 10) {
        return false;
    }

    // Controllo dei byte specifici per validare il frame
    if (radar_data_frame_[6] == 0x02 && radar_data_frame_[7] == 0xAA &&
        radar_data_frame_[17] == 0x55 && radar_data_frame_[18] == 0x00) {

        target_type_ = radar_data_frame_[8];

        // Estrai distanze e energie dei bersagli
        stationary_target_distance_ = *(uint16_t*)(&radar_data_frame_[9]);
        moving_target_distance_ = *(uint16_t*)(&radar_data_frame_[15]);
        stationary_target_energy_ = radar_data_frame_[14];
        moving_target_energy_ = radar_data_frame_[11];

        last_valid_frame_length = radar_data_frame_position_;  // Aggiunto per tracciare la lunghezza del frame
        radar_uart_last_packet_ = millis();  // Aggiorna il timestamp dell'ultimo pacchetto
        return true;
    }

    return false;  // Frame non valido
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
	latest_ack_ = radar_data_frame_[6];
	latest_command_success_ = (radar_data_frame_[8] == 0x00 && radar_data_frame_[9] == 0x00);
	if(intra_frame_data_length_ == 8 && latest_ack_ == 0xFF)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0xFE)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0x60)
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
	else if(intra_frame_data_length_ == 28 && latest_ack_ == 0x61)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0x64)
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
	else if(intra_frame_data_length_ == 12 && latest_ack_ == 0xA0)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0xA2)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0xA3)
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
	//Command preamble
	radar_uart_->write((byte)0xFD);
	radar_uart_->write((byte)0xFC);
	radar_uart_->write((byte)0xFB);
	radar_uart_->write((byte)0xFA);
}

void ld2410::send_command_postamble_()
{
	//Command end
	radar_uart_->write((byte)0x04);
	radar_uart_->write((byte)0x03);
	radar_uart_->write((byte)0x02);
	radar_uart_->write((byte)0x01);
}

bool ld2410::enter_configuration_mode_()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write((byte) 0x04);	//Command is four bytes long
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) 0xFF);	//Request enter command mode
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) 0x01);
	radar_uart_->write((byte) 0x00);
	send_command_postamble_();
	radar_uart_last_command_ = millis();
	while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
	{
		if(read_frame_())
		{
			if(latest_ack_ == 0xFF && latest_command_success_)
			{
				return true;
			}
		}
	}
	return false;
}

bool ld2410::leave_configuration_mode_()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write((byte) 0x02);	//Command is two bytes long
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) 0xFE);	//Request leave command mode
	radar_uart_->write((byte) 0x00);
	send_command_postamble_();
	radar_uart_last_command_ = millis();
	while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
	{
		if(read_frame_())
		{
			if(latest_ack_ == 0xFE && latest_command_success_)
			{
				return true;
			}
		}
	}
	return false;
}

bool ld2410::requestStartEngineeringMode()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write((byte) 0x02);	//Command is two bytes long
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) 0x62);	//Request enter command mode
	radar_uart_->write((byte) 0x00);
	send_command_postamble_();
	radar_uart_last_command_ = millis();
	while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
	{
		if(read_frame_())
		{
			if(latest_ack_ == 0x62 && latest_command_success_)
			{
				return true;
			}
		}
	}
	return false;
}

bool ld2410::requestEndEngineeringMode()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write((byte) 0x02);	//Command is two bytes long
	radar_uart_->write((byte) 0x00);
	radar_uart_->write((byte) 0x63);	//Request leave command mode
	radar_uart_->write((byte) 0x00);
	send_command_postamble_();
	radar_uart_last_command_ = millis();
	while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
	{
		if(read_frame_())
		{
			if(latest_ack_ == 0x63 && latest_command_success_)
			{
				return true;
			}
		}
	}
	return false;
}

bool ld2410::requestCurrentConfiguration()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x61);	//Request current configuration
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0x61 && latest_command_success_)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool ld2410::requestFirmwareVersion()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xA0);	//Request firmware version
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			read_frame_();
			if(latest_ack_ == 0xA0 && latest_command_success_)
			{
				delay(50);
				leave_configuration_mode_();
				return true;
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool ld2410::requestRestart()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xA3);	//Request restart
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xA3 && latest_command_success_)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool ld2410::requestFactoryReset()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xA2);	//Request factory reset
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xA2 && latest_command_success_)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool ld2410::setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer)
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		radar_uart_->write((byte) 0x14);	//Command is 20 bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x60);	//Request set max values
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);	//Moving gate command
		radar_uart_->write((byte) 0x00);
		radar_uart_->write(char(moving & 0x00FF));	//Moving gate value
		radar_uart_->write(char((moving & 0xFF00)>>8));
		radar_uart_->write((byte) 0x00);	//Spacer
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x01);	//Stationary gate command
		radar_uart_->write((byte) 0x00);
		radar_uart_->write(char(stationary & 0x00FF));	//Stationary gate value
		radar_uart_->write(char((stationary & 0xFF00)>>8));
		radar_uart_->write((byte) 0x00);	//Spacer
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x02);	//Inactivity timer command
		radar_uart_->write((byte) 0x00);
		radar_uart_->write(char(inactivityTimer & 0x00FF));	//Inactivity timer
		radar_uart_->write(char((inactivityTimer & 0xFF00)>>8));
		radar_uart_->write((byte) 0x00);	//Spacer
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0x60 && latest_command_success_)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool ld2410::setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary)
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		radar_uart_->write((byte) 0x14);	//Command is 20 bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x64);	//Request set sensitivity values
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);	//Gate command
		radar_uart_->write((byte) 0x00);
		radar_uart_->write(char(gate));	//Gate value
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);	//Spacer
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x01);	//Motion sensitivity command
		radar_uart_->write((byte) 0x00);
		radar_uart_->write(char(moving));	//Motion sensitivity value
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);	//Spacer
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x02);	//Stationary sensitivity command
		radar_uart_->write((byte) 0x00);
		radar_uart_->write(char(stationary));	//Stationary sensitivity value
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);	//Spacer
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0x64 && latest_command_success_)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

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
