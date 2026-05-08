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
#ifndef ld2410_h
#define ld2410_h
#include <Arduino.h>
#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

// ---- Variant selection ----------------------------------------------------
// Pick exactly one of:
//   LD2410_VARIANT_BASE   (default — original LD2410, default UART 57600)
//   LD2410_VARIANT_C      (LD2410C — adds Bluetooth/MAC/distance-resolution,
//                          default UART 256000)
//   LD2410_VARIANT_S      (LD2410S — separate protocol, default UART 115200,
//                          16 gates; see ld2410_variants/ld2410_s.h for the
//                          UNVERIFIED-ON-HARDWARE caveat)
//
// Define the macro before #include <ld2410.h>, e.g.:
//     #define LD2410_VARIANT_C
//     #include <ld2410.h>
//
// Or via the toolchain:
//     PlatformIO   build_flags = -DLD2410_VARIANT_C
//     arduino-cli  --build-property "build.extra_flags=-DLD2410_VARIANT_C"
//
// Defaulting to BASE preserves historical behaviour: this fork has always
// targeted the LD2410 / LD2410C shared core, and base.h covers it.
#if (defined(LD2410_VARIANT_BASE) + defined(LD2410_VARIANT_C) + defined(LD2410_VARIANT_S)) > 1
#error "Define exactly one of LD2410_VARIANT_BASE / LD2410_VARIANT_C / LD2410_VARIANT_S"
#endif
#if !defined(LD2410_VARIANT_BASE) && !defined(LD2410_VARIANT_C) && !defined(LD2410_VARIANT_S)
#define LD2410_VARIANT_BASE
#endif

#if defined(LD2410_VARIANT_S)
#include "ld2410_variants/ld2410_s.h"
#elif defined(LD2410_VARIANT_C)
#include "ld2410_variants/ld2410_c.h"
#else
#include "ld2410_variants/ld2410_base.h"
#endif


// ---- Buffer sizing --------------------------------------------------------
// LD2410_MAX_FRAME_LENGTH covers the largest single frame the radar emits
// (worst case = data frame including all per-gate energies). Variant-aware
// because S inflates the per-gate block from 18..22 B (base/C dynamic) to
// a fixed 64 B inline:
//   base/C engineering frame: 35 B intra + 10 B envelope = 45 B (max ~38 B
//                              for the longest ACK 0x61) — 64 B is generous
//   S standard frame:         70 B intra + 10 B envelope = 80 B           — bump to 96 (≈ +20%)
//
// LD2410_BUFFER_SIZE is the circular buffer that absorbs UART bytes between
// the read path and the parser. Sized 4× LD2410_MAX_FRAME_LENGTH so it can
// hold several frames in flight (autoReadTask scenario, momentary pauses
// in user-space drain, etc.).
//
// Both are #ifndef-guarded so the user can override either independently
// before #include <ld2410.h>.
#ifndef LD2410_MAX_FRAME_LENGTH
#  if defined(LD2410_VARIANT_S)
#    define LD2410_MAX_FRAME_LENGTH 96
#  else
#    define LD2410_MAX_FRAME_LENGTH 64
#  endif
#endif

#ifndef LD2410_BUFFER_SIZE
#  define LD2410_BUFFER_SIZE (4 * LD2410_MAX_FRAME_LENGTH)
#endif
// ---- Debug flags ----------------------------------------------------------
// Compile-time gates for verbose Serial output via the user-provided
// debug stream (set with radar.debug(Stream&) at runtime). When a flag
// is NOT defined, the corresponding debug code paths are completely
// removed at compile time — no runtime overhead, no flash cost.
//
// All four flags are OFF by default. To enable, define the macro BEFORE
// #include <ld2410.h> in your sketch, or pass it via build flags:
//
//     // in the .ino, BEFORE the include
//     #define LD2410_DEBUG_COMMANDS
//     #include <ld2410.h>
//
//     // PlatformIO
//     build_flags = -DLD2410_DEBUG_COMMANDS
//
//     // arduino-cli
//     --build-property "compiler.cpp.extra_flags=-DLD2410_DEBUG_COMMANDS"
//
// Categories:
//   LD2410_DEBUG_COMMANDS  log every command sent + ACK received   [ACTIVE]
//   LD2410_DEBUG_DATA      dump every parsed data frame           [reserved]
//   LD2410_DEBUG_PARSE     trace the byte-level parser state      [reserved]
//   LD2410_DEBUG           meta-flag — turns on all three above
//
// "[ACTIVE]" means the .cpp currently gates code on this flag. The
// "[reserved]" flags are recognised here for forward compatibility but
// no code in the current .cpp gates on them — defining them is harmless
// but produces no extra output yet.
//
// At runtime each gate is also conditional on a non-null debug stream,
// so #define-ing a flag is harmless if radar.debug(...) is never called.
#if defined(LD2410_DEBUG)
#  ifndef LD2410_DEBUG_DATA
#    define LD2410_DEBUG_DATA
#  endif
#  ifndef LD2410_DEBUG_COMMANDS
#    define LD2410_DEBUG_COMMANDS
#  endif
#  ifndef LD2410_DEBUG_PARSE
#    define LD2410_DEBUG_PARSE
#  endif
#endif

struct FrameData {
    const uint8_t* data;
    uint16_t length;
};

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
		uint16_t detectionDistance();									//Last reported detection distance in cm (Table 12, last field)
		uint8_t movingEnergyAtGate(uint8_t gate);						//Per-gate motion energy from engineering frames (Table 14)
		uint8_t stationaryEnergyAtGate(uint8_t gate);				//Per-gate stationary energy from engineering frames (Table 14)
		bool engineeringRetrieved();								//True once at least one engineering-mode data frame has been parsed
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
    	FrameData getFrameData() const;
		bool isAutoReadTaskRunning();									//True iff autoReadTask() succeeded and the task hasn't been stopped (always false on non-ESP32)
#if defined(ESP32)
		bool autoReadTask(uint32_t stack = 4096, UBaseType_t priority = 1, BaseType_t core = tskNO_AFFINITY);
		void stopAutoReadTask();
#endif

	protected:
	private:
		Stream *radar_uart_ = nullptr;
		Stream *debug_uart_ = nullptr;									//The stream used for the debugging
		uint32_t radar_uart_timeout = 100;								//How long to give up on receiving some useful data from the LD2410
		uint32_t radar_uart_last_packet_ = 0;							//Time of the last packet from the radar
		uint32_t radar_uart_command_timeout_ = 100;						//Timeout for sending commands
		uint8_t latest_ack_ = 0;
		bool latest_command_success_ = false;
		uint8_t radar_data_frame_[LD2410_MAX_FRAME_LENGTH];				//Store the incoming data from the radar, to check it's in a valid format
		uint8_t radar_data_frame_position_ = 0;							//Where in the frame we are currently writing; 0 means no frame in progress
		bool ack_frame_ = false;										//Whether the incoming frame is LIKELY an ACK frame
		bool waiting_for_ack_ = false;									//Whether a command has just been sent
		uint8_t target_type_ = 0;
		uint16_t moving_target_distance_ = 0;
		uint8_t moving_target_energy_ = 0;
		uint16_t stationary_target_distance_ = 0;
		uint8_t stationary_target_energy_ = 0;
    	uint16_t last_valid_frame_length = 0;
		uint16_t detection_distance_ = 0;
		uint8_t engineering_motion_energy_[9] = {0,0,0,0,0,0,0,0,0};
		uint8_t engineering_stationary_energy_[9] = {0,0,0,0,0,0,0,0,0};
		bool engineering_data_received_ = false;
		uint8_t cmd_seq_ = 0;											//Monotonic counter; bumped before each command issue
		uint8_t cmd_ack_seq_ = 0;										//Mirrored by parser when an ACK matches expected_ack_opcode_
		uint8_t expected_ack_opcode_ = 0;								//Set by command issuer; checked by parse_command_frame_
#if defined(ESP32)
		TaskHandle_t taskHandle_ = nullptr;
		portMUX_TYPE data_mux_ = portMUX_INITIALIZER_UNLOCKED;
#endif

		uint8_t circular_buffer[LD2410_BUFFER_SIZE];
        uint16_t buffer_head = 0;
        uint16_t buffer_tail = 0;

		void add_to_buffer(uint8_t byte);
		bool read_from_buffer(uint8_t &byte);
		bool check_frame_end_();
		
		bool read_frame_();		
		bool parse_data_frame_();										//Is the current data frame valid?
		bool parse_command_frame_();									//Is the current command frame valid?
		void begin_command_(uint8_t expected_op);						//Bump cmd_seq_, reset stale state, set expected ACK opcode
		bool wait_for_ack_(uint8_t expected_op, uint32_t timeout_ms);	//Block until matching ACK arrives or timeout
		void print_frame_();											//Print the frame for debugging
		void send_command_preamble_();									//Commands have the same preamble
		void send_command_postamble_();									//Commands have the same postamble
		bool enter_configuration_mode_();								//Necessary before sending any command
		bool leave_configuration_mode_();								//Will not read values without leaving command mode
#if defined(ESP32)
		static void taskFunction(void* param);
#endif
};
#endif
