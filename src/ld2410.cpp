/*
 *	An Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor.
 *
 *  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 *	The code in this library is based heavily off this piece of work for ESPHome (https://github.com/rain931215/ESPHome-LD2410) and the manufacturer datasheet.
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
		if(request_firmware_version_())
		{
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F(" v"));
				debug_uart_->print(firmware_major_version_);
				debug_uart_->print('.');
				debug_uart_->print(firmware_minor_version_);
				debug_uart_->print('.');
				debug_uart_->print(firmware_bugfix_version_);
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
	if(millis() - radar_uart_last_packet_ < radar_uart_timeout)
	{
		return true;
	}
	if(read_frame_() && parse_frame_())
	{
		return true;
	}
	return false;
}


bool ld2410::read()
{
	return read_frame_();
	return false;
}
bool ld2410::presenceDetected()
{
	return target_type_ != 0;
}
uint16_t ld2410::stationaryTargetDistance()
{
	if(stationary_target_energy_ > 0)
	{
		return stationary_target_distance_;
	}
	return 0;
}
uint8_t ld2410::stationaryTargetEnergy()
{
	return stationary_target_energy_;
}

uint16_t ld2410::movingTargetDistance()
{
	if(moving_target_energy_ > 0)
	{
		return moving_target_distance_;
	}
	return 0;
}
uint8_t ld2410::movingTargetEnergy()
{
	return moving_target_energy_;
}

bool ld2410::read_frame_()
{
	if(radar_uart_ -> available())
	{
		if(frame_started_ == false)
		{
			uint8_t byte_read_ = radar_uart_ -> read();
			if(byte_read_ == 0xF4 || byte_read_ == 0xFD)
			{
				radar_data_frame_[radar_data_frame_position_++] = byte_read_;
				frame_started_ = true;
				#ifdef LD2410_DEBUG_FRAMES
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print(F("\nLD2410 frame started: ."));
				}
				#endif
			}
		}
		else
		{
			if(radar_data_frame_position_ < LD2410_MAX_FRAME_LENGTH)
			{
				radar_data_frame_[radar_data_frame_position_++] = radar_uart_ -> read();
				#ifdef LD2410_DEBUG_FRAMES
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print('.');
				}
				#endif
				if(radar_data_frame_position_ > 7 &&
					(
					radar_data_frame_[0]                              == 0xF4 &&	//Data frames
					radar_data_frame_[1]                              == 0xF3 &&
					radar_data_frame_[2]                              == 0xF2 &&
					radar_data_frame_[3]                              == 0xF1 &&
					radar_data_frame_[radar_data_frame_position_ - 4] == 0xF8 &&
					radar_data_frame_[radar_data_frame_position_ - 3] == 0xF7 &&
					radar_data_frame_[radar_data_frame_position_ - 2] == 0xF6 &&
					radar_data_frame_[radar_data_frame_position_ - 1] == 0xF5
					) || (
					radar_data_frame_[0]                              == 0xFD &&	//Command frames
					radar_data_frame_[1]                              == 0xFC &&
					radar_data_frame_[2]                              == 0xFB &&
					radar_data_frame_[3]                              == 0xFA &&
					radar_data_frame_[radar_data_frame_position_ - 4] == 0x04 &&
					radar_data_frame_[radar_data_frame_position_ - 3] == 0x03 &&
					radar_data_frame_[radar_data_frame_position_ - 2] == 0x02 &&
					radar_data_frame_[radar_data_frame_position_ - 1] == 0x01
					)
					)
				{
					#ifdef LD2410_DEBUG_FRAMES
					if(debug_uart_ != nullptr)
					{
						debug_uart_->print(F("ended"));
					}
					#endif
					if(parse_frame_())
					{
						frame_started_ = true;
						radar_data_frame_position_ = 0;
						return true;
					}
					else
					{
						frame_started_ = false;
						radar_data_frame_position_ = 0;
					}
				}
			}
			else
			{
				#ifdef LD2410_DEBUG_FRAMES
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
		debug_uart_->print(F("\nFrame: "));
		for(uint8_t i = 0; i < radar_data_frame_position_ ; i ++)
		{
			debug_uart_->print(radar_data_frame_[i],HEX);
			debug_uart_->print(' ');
		}
	}
}

bool ld2410::parse_frame_()
{
	if(			radar_data_frame_[0]                              == 0xF4 &&
				radar_data_frame_[1]                              == 0xF3 &&
				radar_data_frame_[2]                              == 0xF2 &&
				radar_data_frame_[3]                              == 0xF1 &&
				radar_data_frame_[radar_data_frame_position_ - 4] == 0xF8 &&
				radar_data_frame_[radar_data_frame_position_ - 3] == 0xF7 &&
				radar_data_frame_[radar_data_frame_position_ - 2] == 0xF6 &&
				radar_data_frame_[radar_data_frame_position_ - 1] == 0xF5)		//This is a data frame
	{
		uint16_t intra_frame_data_length_ = radar_data_frame_[4] + (radar_data_frame_[5] << 8);
		if(radar_data_frame_position_ == intra_frame_data_length_ + 10)
		{
			#ifdef LD2410_DEBUG_FRAMES
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nFrame payload: "));
				debug_uart_->print(intra_frame_data_length_);
				debug_uart_->print(F(" bytes"));
			}
			#endif
			if(radar_data_frame_[6] == 0x01 && radar_data_frame_[7] == 0xAA)	//Engineering mode data
			{
				target_type_ = radar_data_frame_[8];
				#ifdef LD2410_DEBUG_DATA
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print(F("\nEngineering data: "));
					if(target_type_ == 0x00)
					{
						debug_uart_->print(F("no target"));
					}
					else if(target_type_ == 0x01)
					{
						debug_uart_->print(F("moving target - "));
					}
					else if(target_type_ == 0x02)
					{
						debug_uart_->print(F("stationary target - "));
					}
					else if(target_type_ == 0x03)
					{
						debug_uart_->print(F("moving & stationary targets - "));
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
				moving_target_distance_ = radar_data_frame_[9] + (radar_data_frame_[10] << 8);
				moving_target_energy_ = radar_data_frame_[11];
				stationary_target_distance_ = radar_data_frame_[12] + (radar_data_frame_[13] << 8);
				stationary_target_energy_ = radar_data_frame_[14];
				detection_distance_ = radar_data_frame_[15] + (radar_data_frame_[16] << 8);
				#ifdef LD2410_DEBUG_DATA
				if(debug_uart_ != nullptr)
				{
					debug_uart_->print(F("\nNormal data: "));
					if(target_type_ == 0x00)
					{
						debug_uart_->print(F("no target"));
					}
					else if(target_type_ == 0x01)
					{
						debug_uart_->print(F("moving target - "));
					}
					else if(target_type_ == 0x02)
					{
						debug_uart_->print(F("stationary target - "));
					}
					else if(target_type_ == 0x03)
					{
						debug_uart_->print(F("moving & stationary targets - "));
					}
				}
				#endif
				#ifdef LD2410_DEBUG_DATA
				if(debug_uart_ != nullptr)
				{
					if(radar_data_frame_[8] & 0x01)
					{
						debug_uart_->print(F(" moving at "));
						debug_uart_->print(moving_target_distance_);
						debug_uart_->print(F("cm "));
						debug_uart_->print(F(" power "));
						debug_uart_->print(moving_target_energy_);
					}
					if(radar_data_frame_[8] & 0x02)
					{
						debug_uart_->print(F(" stationary at "));
						debug_uart_->print(stationary_target_distance_);
						debug_uart_->print(F("cm "));
						debug_uart_->print(F(" power "));
						debug_uart_->print(stationary_target_energy_);
					}
					if(radar_data_frame_[8] & 0x03)
					{
						debug_uart_->print(F(" detection at "));
						debug_uart_->print(detection_distance_);
						debug_uart_->print(F("cm"));
					}
				}
				#endif
				radar_uart_last_packet_ = millis();
				return true;
			}
			else
			{
				#ifdef LD2410_DEBUG_FRAMES
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
			#ifdef LD2410_DEBUG_FRAMES
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("\nFrame length unexpected: "));
				debug_uart_->print(radar_data_frame_position_);
				debug_uart_->print(F(" not "));
				debug_uart_->print(intra_frame_data_length_ + 10);
			}
			#endif
		}
	}
	else if(	radar_data_frame_[0]                              == 0xFD &&
				radar_data_frame_[1]                              == 0xFC &&
				radar_data_frame_[2]                              == 0xFB &&
				radar_data_frame_[3]                              == 0xFA &&
				radar_data_frame_[radar_data_frame_position_ - 4] == 0x04 &&
				radar_data_frame_[radar_data_frame_position_ - 3] == 0x03 &&
				radar_data_frame_[radar_data_frame_position_ - 2] == 0x02 &&
				radar_data_frame_[radar_data_frame_position_ - 1] == 0x01)		//This is an ACK/Command frame
	{
		#ifdef LD2410_DEBUG_FRAMES
		if(debug_uart_ != nullptr)
		{
			print_frame_();
		}
		#endif
		uint16_t intra_frame_data_length_ = radar_data_frame_[4] + (radar_data_frame_[5] << 8);
		#ifdef LD2410_DEBUG_FRAMES
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK frame payload: "));
			debug_uart_->print(intra_frame_data_length_);
			debug_uart_->print(F(" bytes"));
		}
		#endif
		latest_ack_ = radar_data_frame_[6];
		if(intra_frame_data_length_ == 4 && latest_ack_ == 0xA0)
		{
			firmware_major_version_ = radar_data_frame_[7];
			firmware_minor_version_ = radar_data_frame_[8];
			firmware_bugfix_version_ = radar_data_frame_[9];
			radar_uart_last_packet_ = millis();
			return true;
		}
	}
	return false;
}

void ld2410::send_command_preamble_()
{
	//Command preamble
	radar_uart_->write(char(0xFD));
	radar_uart_->write(char(0xFC));
	radar_uart_->write(char(0xFB));
	radar_uart_->write(char(0xFA));
}

void ld2410::send_command_postamble_()
{
	//Command end
	radar_uart_->write(char(0x04));
	radar_uart_->write(char(0x03));
	radar_uart_->write(char(0x02));
	radar_uart_->write(char(0x01));
}
bool ld2410::request_firmware_version_()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write(char(0x02));	//Command is two bytes long
	radar_uart_->write(char(0x00));
	radar_uart_->write(char(0xA0));	//Request firmware
	radar_uart_->write(char(0x00));
	send_command_postamble_();
	radar_uart_last_command_ = millis();
	while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
	{
		read_frame_();
		if(latest_ack_ == 0xA0)
		{
			return true;
		}
	}
	return false;
}

bool ld2410::request_restart_()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write(char(0x02));	//Command is two bytes long
	radar_uart_->write(char(0x00));
	radar_uart_->write(char(0xA3));	//Request restart
	radar_uart_->write(char(0x00));
	send_command_postamble_();
	radar_uart_last_command_ = millis();
	while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
	{
		read_frame_();
		if(latest_ack_ == 0xA3)
		{
			return true;
		}
	}
	return false;
}

#endif
