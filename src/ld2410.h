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
		// ---- Lifecycle ----------------------------------------------------
		ld2410();
		~ld2410();
		bool begin(Stream &, bool waitForRadar = true);
		void debug(Stream &);
		bool isConnected();
		bool read();

		// ---- Target-state accessors ---------------------------------------
		// Populated by parse_data_frame_() for every parsed frame, see
		// docs/method-coverage.md Table 2. State semantics differ between
		// base/C (4 states) and S (no-one / present); see roadmap §10.
		bool presenceDetected();
		bool stationaryTargetDetected();
		uint16_t stationaryTargetDistance();
		uint8_t stationaryTargetEnergy();
		bool movingTargetDetected();
		uint16_t movingTargetDistance();
		uint8_t movingTargetEnergy();
		uint16_t detectionDistance();
		uint8_t movingEnergyAtGate(uint8_t gate);
		uint8_t stationaryEnergyAtGate(uint8_t gate);
		bool engineeringRetrieved();

#ifdef LD2410_HAS_AUTO_THRESHOLD
		// 0x03 data type §2.2.9 — auto-threshold tuning progress, reported
		// by the radar while the 0x09 autoUpdateThreshold() command is
		// running. The radar encodes "progress × 100" (e.g. 5000 = 50.00%);
		// autoThresholdProgress() returns the raw 16-bit value so the user
		// can divide as needed. autoThresholdReceived() flips true the
		// first time a 0x03 frame is parsed and stays true.
		// See docs/method-coverage.md Table 2 row "Auto-threshold progress".
		uint16_t autoThresholdProgress();
		bool     autoThresholdReceived();
#endif

		// ---- Firmware version ---------------------------------------------
		// 0xA0 §2.2.8 (base/C) / 0x00 §2.2.2 (S) — read firmware version.
		// All three variants expose the capability; the .cpp currently
		// hardcodes 0xA0 so on S it returns false until roadmap §8.
		// See docs/method-coverage.md Table 1 row 0xA0/0x00.
		bool requestFirmwareVersion();
		uint8_t firmware_major_version = 0;
		uint8_t firmware_minor_version = 0;
		uint32_t firmware_bugfix_version = 0;

#ifdef LD2410_HAS_READ_PARAMS
		// 0x61 §2.2.4 — read current configuration. base/C only.
		// (S uses 0x71 generic-params; see docs/method-coverage.md
		// Table 1 rows 0x61 and 0x71. To be added in roadmap §11d.)
		bool requestCurrentConfiguration();
		uint8_t max_gate = 0;
		uint8_t max_moving_gate = 0;
		uint8_t max_stationary_gate = 0;
		uint16_t sensor_idle_time = 0;
		uint8_t motion_sensitivity[9] = {0,0,0,0,0,0,0,0,0};
		uint8_t stationary_sensitivity[9] = {0,0,0,0,0,0,0,0,0};
#endif

#ifdef LD2410_HAS_RESTART
		// 0xA3 §2.2.11 — restart module. base/C only.
		// (S has no restart command.) See docs/method-coverage.md Table 1.
		bool requestRestart();
#endif

#ifdef LD2410_HAS_FACTORY_RESET
		// 0xA2 §2.2.10 — factory reset. base/C only.
		// (S has no factory-reset command.) See docs/method-coverage.md Table 1.
		bool requestFactoryReset();
#endif

#ifdef LD2410_HAS_ENGINEERING_MODE
		// 0x62 §2.2.5 / 0x63 §2.2.6 — enable / close engineering mode.
		// base/C only. (S standard frame already includes per-gate energies
		// inline; toggle is unnecessary.) See docs/method-coverage.md Table 1.
		bool requestStartEngineeringMode();
		bool requestEndEngineeringMode();
#endif

#ifdef LD2410_HAS_MAX_VALUES
		// 0x60 §2.2.3 — set max gate + unmanned delay. base/C only.
		// (S uses 0x70 with different parameter words; see roadmap §11d.)
		// See docs/method-coverage.md Table 1 row 0x60.
		bool setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer);
#endif

#ifdef LD2410_HAS_GATE_SENSITIVITY
		// 0x64 §2.2.7 — gate sensitivity (motion + stationary in one shot).
		// base/C only. (S splits into trigger threshold 0x72 and hold
		// threshold 0x76; see roadmap §11d.)
		// See docs/method-coverage.md Table 1 row 0x64.
		bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
#endif

		// ---- Snapshot + auto-read task ------------------------------------
    	FrameData getFrameData() const;
		bool isAutoReadTaskRunning();
#if defined(ESP32)
		bool autoReadTask(uint32_t stack = 4096, UBaseType_t priority = 1, BaseType_t core = tskNO_AFFINITY);
		void stopAutoReadTask();
#endif

		// ---- Command serialization (concurrency safety) ------------------
		// RAII guard around lock_command_ / unlock_command_. Each
		// request* / set* helper constructs a CommandTransaction at the
		// top of the function; the destructor releases the mutex even on
		// early return. On non-ESP32 platforms the lock is a no-op (no
		// FreeRTOS scheduler, single-thread assumption).
		//
		// Without this lock, two FreeRTOS tasks calling e.g.
		// requestFirmwareVersion() and requestRestart() simultaneously
		// would interleave bytes on the radar UART TX. PR #3 already
		// protects the ACK-matching state via portMUX(data_mux_); this
		// extends the protection to the send path itself.
		//
		// Default timeout 1000 ms ≈ 5 commands worth of contention before
		// the second caller fails fast with .ok() == false.
		class CommandTransaction {
			ld2410 &sensor_;
			bool acquired_;
		public:
			explicit CommandTransaction(ld2410 &s, uint32_t timeout_ms = 1000)
				: sensor_(s), acquired_(s.lock_command_(timeout_ms)) {}
			~CommandTransaction() { if (acquired_) sensor_.unlock_command_(); }
			bool ok() const { return acquired_; }
			CommandTransaction(const CommandTransaction&) = delete;
			CommandTransaction& operator=(const CommandTransaction&) = delete;
		};

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
		uint8_t engineering_motion_energy_[LD2410_GATE_COUNT] = {};      // 9 on base/C, 16 on S — see variants/*.h
		uint8_t engineering_stationary_energy_[LD2410_GATE_COUNT] = {};
		bool engineering_data_received_ = false;
#ifdef LD2410_HAS_AUTO_THRESHOLD
		uint16_t auto_threshold_progress_ = 0;                          // populated by parse_data_frame_ on 0x03 frames
		bool     auto_threshold_received_ = false;                      // sticky flag — true once any 0x03 frame parsed
#if defined(LD2410_VARIANT_S)
		bool     minimal_frame_ = false;                                // true while read_frame_ is parsing a 6E…62 frame
#endif
#endif
		uint8_t cmd_seq_ = 0;											//Monotonic counter; bumped before each command issue
		uint8_t cmd_ack_seq_ = 0;										//Mirrored by parser when an ACK matches expected_ack_opcode_
		uint8_t expected_ack_opcode_ = 0;								//Set by command issuer; checked by parse_command_frame_
#if defined(ESP32)
		TaskHandle_t taskHandle_ = nullptr;
		portMUX_TYPE data_mux_ = portMUX_INITIALIZER_UNLOCKED;
		SemaphoreHandle_t cmd_mutex_ = nullptr;		//Serializes request*/set* on ESP32 (created in begin(), destroyed in dtor)
#endif

		uint8_t circular_buffer[LD2410_BUFFER_SIZE];
        uint16_t buffer_head = 0;
        uint16_t buffer_tail = 0;

		void add_to_buffer(uint8_t byte);
		bool read_from_buffer(uint8_t &byte);
		bool check_frame_start_();							//Validate F4F3F2F1 / FDFCFBFA at buffer[0..3]
		bool check_frame_end_();							//Validate F8F7F6F5 / 04030201 at buffer[position-4..position-1]
		
		bool read_frame_();		
		bool parse_data_frame_();										//Is the current data frame valid?
		bool parse_command_frame_();									//Is the current command frame valid?
#if defined(LD2410_VARIANT_S)
		bool parse_minimal_frame_();						//Decode 5-byte 6E…62 minimal frame (HLK-LD2410S §2.1)
#endif
		void begin_command_(uint8_t expected_op);						//Bump cmd_seq_, reset stale state, set expected ACK opcode
		bool wait_for_ack_(uint8_t expected_op, uint32_t timeout_ms);	//Block until matching ACK arrives or timeout
		bool lock_command_(uint32_t timeout_ms);				//Acquire cmd_mutex_ on ESP32; always-true stub on other platforms
		void unlock_command_();								//Release cmd_mutex_ on ESP32; no-op on other platforms
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
