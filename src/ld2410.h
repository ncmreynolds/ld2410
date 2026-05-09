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

// Atomic snapshot of the per-frame target state. Use snapshotTargetState()
// to fill this in one critical section, instead of calling the individual
// movingTarget*/stationaryTarget*/detectionDistance() getters separately
// — those return one field at a time and are not coherent against a
// concurrent autoReadTask write on ESP32 dual-core.
struct LD2410TargetState {
    uint8_t  target_type;
    uint16_t moving_distance;
    uint8_t  moving_energy;
    uint16_t stationary_distance;
    uint8_t  stationary_energy;
    uint16_t detection_distance;
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

		// Atomic snapshot of the per-gate engineering arrays. Use these
		// when iterating all LD2410_GATE_COUNT gates from a context that
		// may run concurrently with autoReadTask: the array is memcpy'd
		// under portENTER_CRITICAL(data_mux_) on ESP32 so the snapshot
		// is a self-consistent picture of one frame instead of nine
		// independent reads that the task may interleave its own writes
		// between. On non-ESP32 platforms the lock degenerates to a
		// plain memcpy (single-thread assumption — no FreeRTOS).
		// `out` MUST be at least LD2410_GATE_COUNT bytes wide.
		void snapshotEngineeringMotionEnergies(uint8_t out[LD2410_GATE_COUNT]) const;
		void snapshotEngineeringStationaryEnergies(uint8_t out[LD2410_GATE_COUNT]) const;

		// Atomic snapshot of the basic target-state block (target_type,
		// moving distance/energy, stationary distance/energy, detection
		// distance). On ESP32 these six fields are updated as a group by
		// parse_data_frame_() running on the autoReadTask core; calling
		// the individual getters from the user loop core can observe a
		// torn state where target_type already reflects the new frame
		// but the distance/energy fields still hold values from the
		// previous frame. Reading via snapshotTargetState() is the only
		// way to get a self-consistent picture under FreeRTOS — the copy
		// happens entirely inside portENTER_CRITICAL(data_mux_).
		// On non-ESP32 platforms (single-thread assumption) this just
		// degenerates to six field reads with no lock cost.
		void snapshotTargetState(LD2410TargetState& out) const;

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

		// 0x09 §2.2.9 (S only) — kick off the automatic threshold tuning
		// sweep. Send: cmd-word + 3 × 2-byte LE values (trigger factor,
		// retention factor, scanning time in seconds); intra=8.
		//
		// HLK PDF does NOT document a command-channel ACK — the radar
		// instead reports progress via the data-type 0x03 frame
		// (autoThresholdProgress()). The implementation issues a
		// short wait_for_ack_ and returns its result so firmwares that
		// happen to ACK still report success here, but a `false` return
		// does NOT necessarily mean the sweep failed: monitor
		// autoThresholdProgress() until it reaches 10000 (= 100.00%) for
		// a definitive answer.
		//
		// Defaults (2 / 1 / 120) come from the §2.2.9 PDF example.
		// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
		// See docs/method-coverage.md Table 1 row 0x09.
		bool autoUpdateThreshold(uint16_t trigger_factor   = LD2410_AUTO_THRESH_TRIGGER_DEFAULT,
		                         uint16_t retention_factor = LD2410_AUTO_THRESH_RETENTION_DEFAULT,
		                         uint16_t scanning_time_s  = LD2410_AUTO_THRESH_SCANTIME_DEFAULT);
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

#ifdef LD2410_HAS_BAUD_RATE
		// 0xA1 §2.2.9 — set serial port baud rate. base/C only.
		// (S has no 0xA1 command — its baud is fixed at the variant default.)
		// Pass one of the LD2410_BAUD_INDEX_* constants from the variant
		// header (e.g. LD2410_BAUD_INDEX_256000 = 0x0007). Setting is
		// non-volatile and takes effect after the next module restart;
		// the host UART must be reopened at the new rate before further I/O.
		// See docs/method-coverage.md Table 1 row 0xA1 (closes regression
		// vs v0.1.3, upstream issue #39).
		bool setBaudRate(uint16_t baud_index);
#endif

#ifdef LD2410_HAS_TRIGGER_THRESHOLD
		// 0x72 / 0x73 §2.2.10-11 (S only) — write / read the per-gate
		// MOTION (trigger) thresholds. S splits what base/C sends as a
		// single 0x64 sensitivity command into two: 0x72 = motion-side
		// (trigger), 0x76 = stationary-side (hold).
		// All 16 gates are written / read in one shot. The radar accepts
		// 4-byte values per gate but the documented examples only ever
		// use the low byte, so the API stores them as uint8_t.
		// requestTriggerThresholds() populates trigger_thresholds[16].
		// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
		// See docs/method-coverage.md Table 1 rows 0x72 / 0x73.
		bool writeTriggerThresholds(const uint8_t thresholds[16]);
		bool requestTriggerThresholds();
		uint8_t trigger_thresholds[16] = {};
#endif

#ifdef LD2410_HAS_HOLD_THRESHOLD
		// 0x76 / 0x77 §2.2.12-13 (S only) — write / read the per-gate
		// STATIONARY (hold) thresholds. Mirror image of 0x72/0x73 above.
		// requestHoldThresholds() populates hold_thresholds[16].
		// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
		// See docs/method-coverage.md Table 1 rows 0x76 / 0x77.
		bool writeHoldThresholds(const uint8_t thresholds[16]);
		bool requestHoldThresholds();
		uint8_t hold_thresholds[16] = {};
#endif

#ifdef LD2410_HAS_SERIAL_NUMBER
		// 0x10 / 0x11 §2.2.5-6 (S only) — write / read the 8-byte sensor
		// serial number. The HLK PDF examples always use SN length = 8
		// bytes (e.g. ASCII "12345678"); the protocol defines a 2-byte
		// length prefix but no firmware ships with a different length,
		// so the API uses a fixed 8-byte buffer for simplicity.
		// requestSerialNumber() populates serial_number[8] in wire order.
		// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
		// See docs/method-coverage.md Table 1 rows 0x10 / 0x11.
		bool writeSerialNumber(const uint8_t sn[8]);
		bool requestSerialNumber();
		uint8_t serial_number[8] = {0,0,0,0,0,0,0,0};
#endif

#ifdef LD2410_HAS_GENERIC_PARAMS
		// 0x70 / 0x71 §2.2.7-8 (S only) — write / read the six "generic"
		// configuration parameters (HLK Table 2-2):
		//   detect_farthest_gate  (1..16)        — LD2410_PARAM_FARTHEST_GATE  0x05
		//   detect_nearest_gate   (0..16)        — LD2410_PARAM_NEAREST_GATE   0x0A
		//   unmanned_delay_s      (10..120)      — LD2410_PARAM_UNMANNED_DELAY 0x06
		//   status_report_freq    (Hz×10, 5..80) — LD2410_PARAM_STATUS_FREQ    0x02
		//   distance_report_freq  (Hz×10, 5..80) — LD2410_PARAM_DISTANCE_FREQ  0x0C
		//   response_speed        (5 normal / 10 fast) — LD2410_PARAM_RESPONSE_SPEED 0x0B
		// All six are sent in one shot (write) or queried in one shot (read);
		// the partial-write API can be added later if needed.
		// requestGenericParameters() populates the matching fields below.
		// UNVERIFIED ON HARDWARE — see ld2410_s.h banner.
		// See docs/method-coverage.md Table 1 rows 0x70 / 0x71.
		bool writeGenericParameters(uint8_t detect_farthest_gate,
		                            uint8_t detect_nearest_gate,
		                            uint16_t unmanned_delay_s,
		                            uint8_t status_report_freq,
		                            uint8_t distance_report_freq,
		                            uint8_t response_speed);
		bool requestGenericParameters();
		uint8_t  detect_farthest_gate  = 0;
		uint8_t  detect_nearest_gate   = 0;
		uint16_t unmanned_delay_s      = 0;
		uint8_t  status_report_freq    = 0;
		uint8_t  distance_report_freq  = 0;
		uint8_t  response_speed        = 0;
#endif

#ifdef LD2410_HAS_OUTPUT_MODE
		// 0x7A §2.2.1 (S only) — switch the radar between minimal-frame
		// (6E…62, 5 bytes total) and standard-frame (F4F3F2F1 / data type
		// 0x01) reporting. Send envelope is intra=8: cmd-word + 6-byte
		// LD2410_OUTPUT_MODE_*_PAYLOAD. ACK is the standard 4-byte
		// success/fail envelope.
		// Setting is non-volatile (per HLK PDF) and takes effect immediately
		// — host parser already handles both envelopes (see steps 10 / 10c).
		// UNVERIFIED ON HARDWARE: see ld2410_s.h banner.
		// See docs/method-coverage.md Table 1 row 0x7A.
		bool setOutputMode(bool standard);
#endif

#ifdef LD2410_HAS_BLUETOOTH
		// 0xA4 §2.2.12 (C only) — enable / disable the on-board BLE radio.
		// Setting is non-volatile and takes effect after the next restart.
		// Closes regression vs v0.1.3 (upstream issue #39).
		// See docs/method-coverage.md Table 1 row 0xA4.
		bool setBluetooth(bool on);

		// 0xA8 §2.2.14 (C only) — present the 6-byte password to the radar
		// in order to unlock the BLE control APIs. Factory default is
		// "HiLink" (LD2410_BLUETOOTH_PASSWORD_DEFAULT). Note from the HLK
		// PDF: "this response only answers to Bluetooth, not to the serial
		// port" — i.e. ACK is delivered over the BLE channel, not UART, so
		// the host UART will time out even on a successful unlock. The
		// method still queues + sends the command but should be expected
		// to return false unless the radar firmware also mirrors the ACK
		// to UART. See docs/method-coverage.md Table 1 row 0xA8.
		bool obtainBluetoothPermissions(const uint8_t password[LD2410_BLUETOOTH_PASSWORD_LENGTH]);

		// 0xA9 §2.2.15 (C only) — set the 6-byte BLE control password.
		// Effect is non-volatile. ACK is the standard 4-byte success/fail
		// envelope on UART (unlike 0xA8 which only ACKs over BLE).
		// See docs/method-coverage.md Table 1 row 0xA9.
		bool setBluetoothPassword(const uint8_t password[LD2410_BLUETOOTH_PASSWORD_LENGTH]);
#endif

#ifdef LD2410_HAS_MAC_ADDRESS
		// 0xA5 §2.2.13 (C only) — query the BLE MAC address.
		// On success the 6-byte address is copied into mac_address[]
		// in the wire (big-endian / "network") order printed by the radar.
		// Returns false if the radar did not ACK, or replied with status≠0.
		// See docs/method-coverage.md Table 1 row 0xA5.
		bool requestMACAddress();
		uint8_t mac_address[6] = {0,0,0,0,0,0};
#endif

#ifdef LD2410_HAS_DISTANCE_RESOLUTION
		// 0xAA / 0xAB §2.2.16-17 (C only) — set / query the per-gate
		// distance resolution (LD2410_DISTANCE_RESOLUTION_075M = 0.75 m,
		// LD2410_DISTANCE_RESOLUTION_02M = 0.2 m). Setting is non-volatile
		// and takes effect after the next restart; downstream code that
		// converts gate index → metres must reread distance_resolution
		// after a successful requestDistanceResolution().
		// See docs/method-coverage.md Table 1 rows 0xAA / 0xAB.
		bool setDistanceResolution(uint16_t resolution_index);
		bool requestDistanceResolution();
		uint16_t distance_resolution = 0xFFFF;     // 0xFFFF = "not yet queried"
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
		mutable portMUX_TYPE data_mux_ = portMUX_INITIALIZER_UNLOCKED;   // mutable so const snapshot* methods can acquire it
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
		// 7+ commands ("trivial" no-arg: leave-cfg, start/end engineering,
		// read-params, fw-version, restart, factory-reset, read-SN, query-
		// distance-resolution) share the exact same 4-byte payload
		// `02 00 OP 00`. Helper bumps cmd_seq_ via begin_command_(opcode),
		// emits the full envelope (preamble + 4-byte body + postamble),
		// and returns. Caller is responsible for the wait_for_ack_().
		void send_simple_command_(uint8_t opcode);
		bool enter_configuration_mode_();								//Necessary before sending any command
		bool leave_configuration_mode_();								//Will not read values without leaving command mode
#if defined(LD2410_HAS_TRIGGER_THRESHOLD) || defined(LD2410_HAS_HOLD_THRESHOLD)
		// Shared body for the 0x72/0x76 write commands (trigger / hold).
		// Both build the same 98-byte intra payload (cmd-word + 16 ×
		// (gate-word LE + value LE 4B)); only the opcode differs.
		bool write_per_gate_thresholds_(uint8_t opcode, const uint8_t thresholds[16]);
		// Shared body for the 0x73/0x77 read commands (trigger / hold).
		bool request_per_gate_thresholds_(uint8_t opcode);
#endif
#if defined(ESP32)
		static void taskFunction(void* param);
#endif
};
#endif
