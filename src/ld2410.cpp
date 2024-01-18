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


ld2410::ld2410()	//Constructor function
{
}

ld2410::~ld2410()	//Destructor function
{
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

bool ld2410::read()
{
	return read_frame_();
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

uint8_t ld2410::movingTargetEnergy()
{
	//if(moving_target_distance_ > 0)
	{
		return moving_target_energy_;
	}
	//return 0;
}

bool ld2410::read_frame_()
{
	if(radar_uart_ -> available())
	{
		if(frame_started_ == false)
		{
			uint8_t byte_read_ = radar_uart_ -> read();
			if(byte_read_ == 0xF4)
			{
				#ifdef LD2410_DEBUG_DATA
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print(F("\nRcvd : 00 "));
				}
				#endif
				radar_data_frame_[radar_data_frame_position_++] = byte_read_;
				frame_started_ = true;
				ack_frame_ = false;
			}
			else if(byte_read_ == 0xFD)
			{
				#ifdef LD2410_DEBUG_COMMANDS
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print(F("\nRcvd : 00 "));
				}
				#endif
				radar_data_frame_[radar_data_frame_position_++] = byte_read_;
				frame_started_ = true;
				ack_frame_ = true;
			}
		}
		else
		{
			if(radar_data_frame_position_ < LD2410_MAX_FRAME_LENGTH)
			{
				#ifdef LD2410_DEBUG_DATA
				if(debug_uart_ != nullptr && ack_frame_ == false)
				{
					if(radar_data_frame_position_ < 0x10)
					{
						debug_uart_->print('0');
					}
					debug_uart_->print(radar_data_frame_position_, HEX);
					debug_uart_->print(' ');
				}
				#endif
				#ifdef LD2410_DEBUG_COMMANDS
				if(debug_uart_ != nullptr && ack_frame_ == true)
				{
					if(radar_data_frame_position_ < 0x10)
					{
						debug_uart_->print('0');
					}
					debug_uart_->print(radar_data_frame_position_, HEX);
					debug_uart_->print(' ');
				}
				#endif
				radar_data_frame_[radar_data_frame_position_++] = radar_uart_ -> read();
				if(radar_data_frame_position_ > 7)	//Can check for start and end
				{
					if(	radar_data_frame_[0]                              == 0xF4 &&	//Data frame end state
						radar_data_frame_[1]                              == 0xF3 &&
						radar_data_frame_[2]                              == 0xF2 &&
						radar_data_frame_[3]                              == 0xF1 &&
						radar_data_frame_[radar_data_frame_position_ - 4] == 0xF8 &&
						radar_data_frame_[radar_data_frame_position_ - 3] == 0xF7 &&
						radar_data_frame_[radar_data_frame_position_ - 2] == 0xF6 &&
						radar_data_frame_[radar_data_frame_position_ - 1] == 0xF5
					)
					{
						if(parse_data_frame_())
						{
							#ifdef LD2410_DEBUG_DATA
							if(debug_uart_ != nullptr)
							{
								debug_uart_->print(F("parsed data OK"));
							}
							#endif
							frame_started_ = false;
							radar_data_frame_position_ = 0;
							return true;
						}
						else
						{
							#ifdef LD2410_DEBUG_DATA
							if(debug_uart_ != nullptr)
							{
								debug_uart_->print(F("failed to parse data"));
							}
							#endif
							frame_started_ = false;
							radar_data_frame_position_ = 0;
						}
					}
					else if(radar_data_frame_[0]                              == 0xFD &&	//Command frame end state
							radar_data_frame_[1]                              == 0xFC &&
							radar_data_frame_[2]                              == 0xFB &&
							radar_data_frame_[3]                              == 0xFA &&
							radar_data_frame_[radar_data_frame_position_ - 4] == 0x04 &&
							radar_data_frame_[radar_data_frame_position_ - 3] == 0x03 &&
							radar_data_frame_[radar_data_frame_position_ - 2] == 0x02 &&
							radar_data_frame_[radar_data_frame_position_ - 1] == 0x01
						)
					{
						if(parse_command_frame_())
						{
							#ifdef LD2410_DEBUG_COMMANDS
							if(debug_uart_ != nullptr)
							{
								debug_uart_->print(F("parsed command OK"));
							}
							#endif
							frame_started_ = false;
							radar_data_frame_position_ = 0;
							return true;
						}
						else
						{
							#ifdef LD2410_DEBUG_COMMANDS
							if(debug_uart_ != nullptr)
							{
								debug_uart_->print(F("failed to parse command"));
							}
							#endif
							frame_started_ = false;
							radar_data_frame_position_ = 0;
						}
					}
				}
			}
			else
			{
				#if defined(LD2410_DEBUG_DATA) || defined(LD2410_DEBUG_COMMANDS)
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print(F("\nLD2410 frame overran"));
				}
				#endif
				frame_started_ = false;
				radar_data_frame_position_ = 0;
			}
		}
	}
	return false;
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

bool ld2410::parse_data_frame_()
{
	uint16_t intra_frame_data_length_ = radar_data_frame_[4] + (radar_data_frame_[5] << 8);
	if(radar_data_frame_position_ == intra_frame_data_length_ + 10)
	{
		#ifdef LD2410_DEBUG_DATA
		if(debug_uart_ != nullptr && ack_frame_ == false)
		{
			print_frame_();
		}
		#endif
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr && ack_frame_ == true)
		{
			print_frame_();
		}
		#endif
		if(radar_data_frame_[6] == 0x01 && radar_data_frame_[7] == 0xAA)	//Engineering mode data
		{
			target_type_ = radar_data_frame_[8];
			#ifdef LD2410_DEBUG_PARSE
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nEngineering data - "));
				if(target_type_ == 0x00)
				{
					debug_uart_->print(F("no target"));
				}
				else if(target_type_ == 0x01)
				{
					debug_uart_->print(F("moving target:"));
				}
				else if(target_type_ == 0x02)
				{
					debug_uart_->print(F("stationary target:"));
				}
				else if(target_type_ == 0x03)
				{
					debug_uart_->print(F("moving & stationary targets:"));
				}
			}
			#endif
			/*
			 *
			 *	To-do support engineering mode
			 *
			 */
		}
		else if(intra_frame_data_length_ == 13 && radar_data_frame_[6] == 0x02 && radar_data_frame_[7] == 0xAA && radar_data_frame_[17] == 0x55 && radar_data_frame_[18] == 0x00)	//Normal target data
		{
			target_type_ = radar_data_frame_[8];
			//moving_target_distance_ = radar_data_frame_[9] + (radar_data_frame_[10] << 8);
			stationary_target_distance_ = radar_data_frame_[9] + (radar_data_frame_[10] << 8);
			stationary_target_energy_ = radar_data_frame_[14];
			moving_target_energy_ = radar_data_frame_[11];
			//stationary_target_distance_ = radar_data_frame_[12] + (radar_data_frame_[13] << 8);
			moving_target_distance_ = radar_data_frame_[15] + (radar_data_frame_[16] << 8);
			#ifdef LD2410_DEBUG_PARSE
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nNormal data - "));
				if(target_type_ == 0x00)
				{
					debug_uart_->print(F("no target"));
				}
				else if(target_type_ == 0x01)
				{
					debug_uart_->print(F("moving target:"));
				}
				else if(target_type_ == 0x02)
				{
					debug_uart_->print(F("stationary target:"));
				}
				else if(target_type_ == 0x03)
				{
					debug_uart_->print(F("moving & stationary targets:"));
				}
				if(radar_data_frame_[8] & 0x01)
				{
					debug_uart_->print(F(" moving at "));
					debug_uart_->print(moving_target_distance_);
					debug_uart_->print(F("cm power "));
					debug_uart_->print(moving_target_energy_);
				}
				if(radar_data_frame_[8] & 0x02)
				{
					debug_uart_->print(F(" stationary at "));
					debug_uart_->print(stationary_target_distance_);
					debug_uart_->print(F("cm power "));
					debug_uart_->print(stationary_target_energy_);
				}
			}
			#endif
			radar_uart_last_packet_ = millis();
			return true;
		}
		else
		{
			#ifdef LD2410_DEBUG_DATA
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nUnknown frame type"));
			}
			#endif
			print_frame_();
		}
	}
	else
	{
		#ifdef LD2410_DEBUG_DATA
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nFrame length unexpected: "));
			debug_uart_->print(radar_data_frame_position_);
			debug_uart_->print(F(" not "));
			debug_uart_->print(intra_frame_data_length_ + 10);
		}
		#endif
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
    else if(intra_frame_data_length_ == 6 && latest_ack_ == 0xAB)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for read distance resolution: "));
		}
		#endif
		if(latest_command_success_)
		{
			resolution = radar_data_frame_[10];
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0xAA)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for set distance resolution: "));
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == 0xA4)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for set Bluetooth: "));
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
	else if(intra_frame_data_length_ == 10 && latest_ack_ == 0xA5)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for get MAC: "));
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

			mac[0] = radar_data_frame_[10];
			mac[1] = radar_data_frame_[11];
			mac[2] = radar_data_frame_[12];
			mac[3] = radar_data_frame_[13];
			mac[4] = radar_data_frame_[14];
			mac[5] = radar_data_frame_[15];

			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nMAC Address: "));
				debug_uart_->print(mac[0], HEX);
				debug_uart_->print(mac[1], HEX);
				debug_uart_->print(mac[2], HEX);
				debug_uart_->print(mac[3], HEX);
				debug_uart_->print(mac[4], HEX);
				debug_uart_->print(mac[5], HEX);
				debug_uart_->print(F("\n"));
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

bool ld2410::requestResolution()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request distance resolution
		radar_uart_->write((byte) 0x02);	//Command is two bytes long
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xAB);	//Request distance resolution setting
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xAB && latest_command_success_)
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

bool ld2410::setResolution(uint8_t res)
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		radar_uart_->write((byte) 0x04);	
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xAA);
		radar_uart_->write((byte) 0x00);

		radar_uart_->write(char(res));
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xAA && latest_command_success_)
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

bool ld2410::enableBluetooth()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		radar_uart_->write((byte) 0x04);	
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xA4);
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x01);
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xA4 && latest_command_success_)
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

bool ld2410::disableBluetooth()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		radar_uart_->write((byte) 0x04);	
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xA4);
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xA4 && latest_command_success_)
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

bool ld2410::getMAC()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		radar_uart_->write((byte) 0x04);	
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0xA5);
		radar_uart_->write((byte) 0x00);
		radar_uart_->write((byte) 0x01);
		radar_uart_->write((byte) 0x00);
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0xA5 && latest_command_success_)
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
#endif
