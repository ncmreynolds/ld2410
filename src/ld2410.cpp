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

/* Command / Response Frame
 *
 * REQUEST
 * FD FC FB FA -- Header
 *       dd dd -- Frame data length
 *       dd dd -- Command Word
 *         ... -- Command Value nBytes
 * 04 03 02 01 -- Footer
 *
 * RESPONSE
 * FD FC FB FA -- Header
 *       dd dd -- Frame data length
 *       dd dd -- ACK Word
 *         ... -- Response Values nBytes
 * 04 03 02 01 -- Footer
 */
bool ld2410::isProtocolDataFrame() {
	return (	radar_data_frame_[0]                              == FRAME_PROTOCOL_PREFIX &&	
				radar_data_frame_[1]                              == 0xFC &&
				radar_data_frame_[2]                              == 0xFB &&
				radar_data_frame_[3]                              == 0xFA &&
				radar_data_frame_[radar_data_frame_position_ - 4] == 0x04 &&
				radar_data_frame_[radar_data_frame_position_ - 3] == 0x03 &&
				radar_data_frame_[radar_data_frame_position_ - 2] == 0x02 &&
				radar_data_frame_[radar_data_frame_position_ - 1] == 0x01
			);
}

/* Data Frame
 *
 * F4 F3 F2 F1 -- header
 *       dd dd -- frame data length
 *          dd -- Type of Data (0x01=Engineering data, 0x02=Target data)
 *        0xAA -- Marker
 *         ... -- target data and engineering data
 *        0x55 -- Marker
 *        0x00 -- Check flag
 * F8 F7 F6 F5  - Footer
 * 
 * F4 F3 FA F1 0D 00 02 AA 02 A7 80 00 00 00 52 00 00 AD 80 F8 F7 F6 F5
 * F4 F3 F2 F9 8D 00 02 AA 02 5A 00 80 00 00 64 00 00 55 80 F8 F7 F6 F5
 * F4 F3 F2 F1 0D 00 82 AA 02 5A 00 00 00 80 64 00 00 55 00 F8 F7 F6 F5
*/
bool ld2410::isReportingDataFrame() {
	return (	radar_data_frame_[0]                              == FRAME_ENGINEERING_PREFIX &&
				radar_data_frame_[1]                              == 0xF3 &&
				radar_data_frame_[2]                              == 0xF2 &&
				radar_data_frame_[3]                              == 0xF1 &&
				radar_data_frame_[radar_data_frame_position_ - 4] == 0xF8 &&
				radar_data_frame_[radar_data_frame_position_ - 3] == 0xF7 &&
				radar_data_frame_[radar_data_frame_position_ - 2] == 0xF6 &&
				radar_data_frame_[radar_data_frame_position_ - 1] == 0xF5
			);
}

String ld2410::targetStateToString(uint8_t tartgetState) {
	switch(tartgetState) {
		case 0: 
			return String(F(" No Target"));
			break;
		case 1: 
			return String(F(" Sports Target"));
			break;
		case 2: 
			return String(F(" Stationary Target"));
			break;
		case 3: 
			return String(F(" Stationary & stationary Target"));
			break;
		default:
			return String(F( "Unknown Target Type"));
	}
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
					if(isReportingDataFrame())
					{
						if(parse_data_frame_())
						{
							#ifdef LD2410_DEBUG_DATA
							if(debug_uart_ != nullptr)
							{
								debug_uart_->print(F(" parsed data OK"));
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
								debug_uart_->print(F(" failed to parse data"));
							}
							#endif
							frame_started_ = false;
							radar_data_frame_position_ = 0;
						}
					}
					else if(isProtocolDataFrame())
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
		if(debug_uart_ != nullptr ) // && ack_frame_ == false)
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
		if(radar_data_frame_[6] == FRAME_ENGINEERING_TYPE && radar_data_frame_[7] == 0xAA)	//Engineering mode data
		{	
			uint8_t b_retcode = false;
			uint8_t pos = 19;
			/*   (Protocol) Target Data Reporting 
			* 02 AA         d6,7     data type (target data)
			* 02            d8       target type (stationary target)
			* 51 00         d9,10    stationary target distance 
			* 00            d11      stationary target energy 
			* 00 00         d12,13   moving target distance
			* 3B            d14      moving target energy
			* 00 00         d15,16   distance detection
			
			Engineering
			* 08        d17       Max moving distance gate
			* 08        d18       Max static distance gate
			* 3C 22 05 03 03 04 03 06 05       d19,27 Movement distance gate energy
			* 00 00 39 10 13 06 06 08 04       d28,36 Static distance gate energy
			* 03 05     d37,38    ?? v1283
			* 55 00     d39,40    Frame flag
			*/
			target_type_ = radar_data_frame_[8];
			stationary_target_distance_ = radar_data_frame_[9] + (radar_data_frame_[10] << 8);
			stationary_target_energy_ = radar_data_frame_[14];
			moving_target_energy_ = radar_data_frame_[11];
			moving_target_distance_ = radar_data_frame_[15] + (radar_data_frame_[16] << 8);
			max_moving_distance_gate = radar_data_frame_[17];
			max_static_distance_gate = radar_data_frame_[18];
			
			// motion_energy
			for(uint8_t gate = 0; gate < sizeof(movement_distance_gate_energy); gate++) {
				movement_distance_gate_energy[gate] = radar_data_frame_[pos++];
			}
			// stationary_engergy
			for(uint8_t gate = 0; gate < sizeof(static_distance_gate_engergy); gate++) {
				static_distance_gate_engergy[gate] = radar_data_frame_[pos++];
			}
			sensor_idle_time = radar_data_frame_[pos++] + (radar_data_frame_[pos] << 8); // maybe
			b_retcode = true;

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
				debug_uart_->print(F(" moving at "));
				debug_uart_->print(moving_target_distance_);
				debug_uart_->print(F("cm power "));
				debug_uart_->print(moving_target_energy_);

				debug_uart_->print(F(" max moving distance gate:"));
				debug_uart_->print(max_moving_distance_gate);
				debug_uart_->print(F(" max static distance gate:"));
				debug_uart_->print(max_static_distance_gate);
				debug_uart_->print(F(" moving/static distance gate energy: "));
				for(uint8_t gate = 0; gate < sizeof(movement_distance_gate_energy); gate++) {
					debug_uart_->printf("%d:[%d,%d] ", gate,movement_distance_gate_energy[gate],static_distance_gate_engergy[gate]);
				}
				debug_uart_->print(F("\n"));				
			}
			#endif

			radar_uart_last_packet_ = millis();

			return b_retcode;
		}
		else if(intra_frame_data_length_ == 13 && radar_data_frame_[6] == FRAME_PROTOCOL_TYPE && radar_data_frame_[7] == 0xAA && radar_data_frame_[17] == 0x55 && radar_data_frame_[18] == 0x00)	//Normal target data
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
					debug_uart_->print(F(" no target"));
				}
				else if(target_type_ == 0x01)
				{
					debug_uart_->print(F(" moving target:"));
				}
				else if(target_type_ == 0x02)
				{
					debug_uart_->print(F(" stationary target:"));
				}
				else if(target_type_ == 0x03)
				{
					debug_uart_->print(F(" moving & stationary targets:"));
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
		return true;
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
	if(intra_frame_data_length_ == 8 && latest_ack_ == CMD_CONFIGURATION_ENABLE)
	{
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for entering configuration mode: "));
		}
		#endif
		if(latest_command_success_)
		{
			configuration_protocol_version_ = radar_data_frame_[10] + (radar_data_frame_[11] << 8);
			configuration_buffer_size_ = radar_data_frame_[12] + (radar_data_frame_[13] << 8);
			radar_uart_last_packet_ = millis();
			#ifdef LD2410_DEBUG_COMMANDS
			if(debug_uart_ != nullptr)
			{
				debug_uart_->print(F("OK"));
				debug_uart_->printf(" protocol version:%d buffer_size:%d ",configuration_protocol_version_ ,configuration_buffer_size_);
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == CMD_CONFIGURATION_END)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == CMD_MAX_DISTANCE_AND_UNMANNED_DURATION)
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
	else if(intra_frame_data_length_ == 28 && latest_ack_ == CMD_READ_PARAMETER)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_== CMD_ENGINEERING_ENABLE) {	
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for end engineering mode: "));
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
	else if(intra_frame_data_length_ == 4 && latest_ack_== CMD_ENGINEERING_END) {	
		#ifdef LD2410_DEBUG_COMMANDS
		if(debug_uart_ != nullptr)
		{
			debug_uart_->print(F("\nACK for end engineering mode: "));
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == CMD_RANGE_GATE_SENSITIVITY)
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
	else if(intra_frame_data_length_ == 12 && latest_ack_ == CMD_READ_FIRMWARE_VERSION)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == CMD_FACTORY_RESET)
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
	else if(intra_frame_data_length_ == 4 && latest_ack_ == CMD_RESTART)
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

bool ld2410::enter_configuration_mode_()
{
	send_command_preamble_();
	//Request firmware
	radar_uart_->write(char(0x04));	//Command is four bytes long
	radar_uart_->write(char(0x00));
	radar_uart_->write(char(0xFF));	//Request enter command mode
	radar_uart_->write(char(0x00));
	radar_uart_->write(char(0x01));
	radar_uart_->write(char(0x00));
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
	radar_uart_->write(char(0x02));	//Command is four bytes long
	radar_uart_->write(char(0x00));
	radar_uart_->write(char(0xFE));	//Request leave command mode
	radar_uart_->write(char(0x00));
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
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		radar_uart_->write(char(0x02));	//Command is four bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x62));	//Request enter command mode
		radar_uart_->write(char(0x00));
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0x62 && latest_command_success_)
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

bool ld2410::requestEndEngineeringMode()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		enter_configuration_mode_();
		send_command_preamble_();
		//Request firmware
		radar_uart_->write(char(0x02));	//Command is four bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x63));	//Request leave command mode
		radar_uart_->write(char(0x00));
		send_command_postamble_();
		radar_uart_last_command_ = millis();
		while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_)
		{
			if(read_frame_())
			{
				if(latest_ack_ == 0x63 && latest_command_success_)
				{
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

bool ld2410::requestCurrentConfiguration()
{
	if(enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		radar_uart_->write(char(0x02));	//Command is two bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x61));	//Request current configuration
		radar_uart_->write(char(0x00));
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
		radar_uart_->write(char(0x02));	//Command is two bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0xA0));	//Request firmware version
		radar_uart_->write(char(0x00));
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
		radar_uart_->write(char(0x02));	//Command is two bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0xA3));	//Request restart
		radar_uart_->write(char(0x00));
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
		radar_uart_->write(char(0x02));	//Command is two bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0xA2));	//Request factory reset
		radar_uart_->write(char(0x00));
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
		radar_uart_->write(char(0x14));	//Command is 20 bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x60));	//Request set max values
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x00));	//Moving gate command
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(moving & 0x00FF));	//Moving gate value
		radar_uart_->write(char((moving & 0xFF00)>>8));
		radar_uart_->write(char(0x00));	//Spacer
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x01));	//Stationary gate command
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(stationary & 0x00FF));	//Stationary gate value
		radar_uart_->write(char((stationary & 0xFF00)>>8));
		radar_uart_->write(char(0x00));	//Spacer
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x02));	//Inactivity timer command
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(inactivityTimer & 0x00FF));	//Inactivity timer
		radar_uart_->write(char((inactivityTimer & 0xFF00)>>8));
		radar_uart_->write(char(0x00));	//Spacer
		radar_uart_->write(char(0x00));
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
		radar_uart_->write(char(0x14));	//Command is 20 bytes long
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x64));	//Request set sensitivity values
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x00));	//Gate command
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(gate));	//Gate value
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x00));	//Spacer
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x01));	//Motion sensitivity command
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(moving));	//Motion sensitivity value
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x00));	//Spacer
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x02));	//Stationary sensitivity command
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(stationary));	//Stationary sensitivity value
		radar_uart_->write(char(0x00));
		radar_uart_->write(char(0x00));	//Spacer
		radar_uart_->write(char(0x00));
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
