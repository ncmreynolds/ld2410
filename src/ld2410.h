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
#pragma once

#include <Arduino.h>

#define LD2410_MAX_FRAME_LENGTH 64

#define LD2410_DEBUG_DATA
#define LD2410_DEBUG_COMMANDS
#define LD2410_DEBUG_PARSE
/* 
 * Protocol Command Words
*/
#define LD2410_MAX_FRAME_LENGTH   0x40     // or 64 bytes

#define CMD_CONFIGURATION_ENABLE  0xFF
#define CMD_CONFIGURATION_END     0xFE
#define CMD_MAX_DISTANCE_AND_UNMANNED_DURATION  0x60
#define CMD_READ_PARAMETER        0x61
#define CMD_ENGINEERING_ENABLE    0x62
#define CMD_ENGINEERING_END       0x63
#define CMD_RANGE_GATE_SENSITIVITY 0x64
#define CMD_READ_FIRMWARE_VERSION 0xA0
#define CMD_SET_SERIAL_PORT_BAUD  0xA1
#define CMD_FACTORY_RESET         0xA2
#define CMD_RESTART               0xA3

/* 
 * Data Frame Formats
*/
#define FRAME_PROTOCOL_TYPE       0x02
#define FRAME_ENGINEERING_TYPE    0x01
#define FRAME_PROTOCOL_PREFIX     0xFD
#define FRAME_ENGINEERING_PREFIX  0xF4

class ld2410	{

	public:
		ld2410();														//Constructor function
		~ld2410();														//Destructor function
		bool begin(Stream &, bool waitForRadar = true);					//Start the ld2410
		void debug(Stream &);											//Start debugging on a stream
		bool isConnected();
		bool read();
		bool presenceDetected();
		bool stationaryTargetDetected();
		uint16_t stationaryTargetDistance();
		uint8_t stationaryTargetEnergy();
		bool movingTargetDetected();
		uint16_t movingTargetDistance();
		uint8_t movingTargetEnergy();
		bool requestFirmwareVersion();									//Request the firmware version
		uint8_t firmware_major_version = 0;								//Reported major version
		uint8_t firmware_minor_version = 0;								//Reported minor version
		uint32_t firmware_bugfix_version = 0;							//Reported bugfix version (coded as hex)
		bool requestCurrentConfiguration();								//Request current configuration
		uint8_t max_gate = 0;
		uint8_t max_moving_gate = 0;
		uint8_t max_stationary_gate = 0;
		uint16_t sensor_idle_time = 0;
		uint8_t motion_sensitivity[9] = {0,0,0,0,0,0,0,0,0};
		uint8_t stationary_sensitivity[9] = {0,0,0,0,0,0,0,0,0};
		bool requestRestart();
		bool requestFactoryReset();
		bool requestStartEngineeringMode();
		bool requestEndEngineeringMode();
		bool setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer);	//Realistically gate values are 0-8 but sent as uint16_t
		bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
		String targetStateToString(uint8_t targetState);                //from report or engineering data
	protected:
	private:
		Stream *radar_uart_ = nullptr;
		Stream *debug_uart_ = nullptr;									//The stream used for the debugging
		uint32_t radar_uart_timeout = 100;								//How long to give up on receiving some useful data from the LD2410
		uint32_t radar_uart_last_packet_ = 0;							//Time of the last packet from the radar
		uint32_t radar_uart_last_command_ = 0;							//Time of the last command sent to the radar
		uint32_t radar_uart_command_timeout_ = 100;						//Timeout for sending commands
		uint8_t latest_ack_ = 0;
		bool latest_command_success_ = false;
		uint8_t radar_data_frame_[LD2410_MAX_FRAME_LENGTH];				//Store the incoming data from the radar, to check it's in a valid format
		uint8_t radar_data_frame_position_ = 0;							//Where in the frame we are currently writing
		bool frame_started_ = false;									//Whether a frame is currently being read
		bool ack_frame_ = false;										//Whether the incoming frame is LIKELY an ACK frame
		bool waiting_for_ack_ = false;									//Whether a command has just been sent
		uint8_t target_type_ = 0;
		uint16_t moving_target_distance_ = 0;
		uint8_t moving_target_energy_ = 0;
		uint16_t stationary_target_distance_ = 0;
		uint8_t stationary_target_energy_ = 0;

		/*
		 * Protocol & Engineering Frame Data */
		uint16_t moving_target_distance_         = 0;                    //protocol mode info 
		uint8_t  moving_target_energy_           = 0;                    //protocol mode info 
		uint16_t stationary_target_distance_     = 0;                    //protocol mode info 
		uint8_t  stationary_target_energy_       = 0;                    //protocol mode info 
		uint16_t detection_distance_             = 0;                    //protocol & engineering mode info
		uint8_t  max_moving_distance_gate        = 0;                    //engineering mode info
		uint8_t  max_static_distance_gate        = 0;                    //engineering mode info
		uint8_t  movement_distance_gate_energy[9] = {0,0,0,0,0,0,0,0,0}; //Engineering mode info
		uint8_t  static_distance_gate_engergy[9]  = {0,0,0,0,0,0,0,0,0}; //Engineering mode info

		uint16_t configuration_protocol_version_ = 0;                    //From Enter Configuration Mode Response
		uint16_t configuration_buffer_size_ = LD2410_MAX_FRAME_LENGTH;   //From Enter Configuration Mode Response
		
		bool isProtocolDataFrame();                                     //Command -Determine type of Frame
		bool isReportingDataFrame();                                    //Data - Determine type of Frame


		bool read_frame_();												//Try to read a frame from the UART
		bool parse_data_frame_();										//Is the current data frame valid?
		bool parse_command_frame_();									//Is the current command frame valid?
		void print_frame_();											//Print the frame for debugging
		void send_command_preamble_();									//Commands have the same preamble
		void send_command_postamble_();									//Commands have the same postamble
		bool enter_configuration_mode_();								//Necessary before sending any command
		bool leave_configuration_mode_();								//Will not read values without leaving command mode
};
