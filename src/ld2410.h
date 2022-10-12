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
#ifndef ld2410_h
#define ld2410_h
#include <Arduino.h>

#define LD2410_MAX_FRAME_LENGTH 200
//#define LD2410_DEBUG_FRAMES
#define LD2410_DEBUG_ACKS
//#define LD2410_DEBUG_DATA

class ld2410	{

	public:
		ld2410();														//Constructor function
		~ld2410();														//Destructor function
		bool begin(Stream &, bool waitForRadar = true);					//Start the ld2410
		void debug(Stream &);											//Start debugging on a stream
		bool isConnected();
		bool read();
		bool presenceDetected();
		uint16_t stationaryTargetDistance();
		uint8_t stationaryTargetEnergy();
		uint16_t movingTargetDistance();
		uint8_t movingTargetEnergy();
	protected:
	private:
		Stream *radar_uart_ = nullptr;
		Stream *debug_uart_ = nullptr;									//The stream used for the debugging
		uint32_t radar_uart_timeout = 100;								//How long to give up on receiving some useful data from the LD2410
		uint32_t radar_uart_last_packet_ = 0;							//Time of the last packet from the radar
		uint32_t radar_uart_last_command_ = 0;							//Time of the last command sent to the radar
		uint32_t radar_uart_command_timeout_ = 1000;						//Timeout for sending commands
		uint8_t latest_ack_ = 0;
		uint8_t radar_data_frame_[LD2410_MAX_FRAME_LENGTH];				//Store the incoming data from the radar, to check it's in a valid format
		uint8_t radar_data_frame_position_ = 0;							//Where in the frame we are currently writing
		bool frame_started_ = false;									//Whether a frame is currently being read
		bool waiting_for_ack_ = false;									//Whether a command has just been sent
		uint8_t target_type_ = 0;
		uint16_t moving_target_distance_ = 0;
		uint8_t moving_target_energy_ = 0;
		uint16_t stationary_target_distance_ = 0;
		uint8_t stationary_target_energy_ = 0;
		uint16_t detection_distance_ = 0;
		uint8_t firmware_major_version_ = 0;
		uint8_t firmware_minor_version_ = 0;
		uint8_t firmware_bugfix_version_ = 0;
		
		bool read_frame_();												//Try to read a frame from the UART
		bool parse_frame_();											//Is the current frame valid
		void print_frame_();											//Print the frame for debugging
		void send_command_preamble_();
		void send_command_postamble_();
		bool request_firmware_version_();								//Request the firmware version
		bool request_restart_();										//Request the module restarts
};
#endif
