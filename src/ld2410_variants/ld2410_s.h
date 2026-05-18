/*
 *	LD2410S — opcode, parameter and feature definitions.
 *
 *	Reference: docs/HLK-LD2410S_protocol.md (V1.00, 2024-08-23).
 *
 *	==========================================================================
 *	STATUS — UNVERIFIED ON HARDWARE
 *
 *	Every value in this header is transcribed directly from the HLK-LD2410S
 *	V1.00 protocol PDF dated 2024-08-23. NONE of these opcodes, parameter
 *	encodings, frame layouts, or feature flags has been exercised against a
 *	real LD2410S unit by the maintainers of this fork (we own and develop
 *	against LD2410C hardware only). Treat this file as a faithful translation
 *	of the spec, not as field-validated code.
 *
 *	If you have an LD2410S on your bench: please report any discrepancy
 *	between the spec values here and what the firmware actually accepts —
 *	especially around frame layouts, the 0x7A output-mode payload, and the
 *	per-gate energy ordering inside data-type 0x01 frames. PRs welcome.
 *	==========================================================================
 *
 *	The S variant uses a SEPARATE protocol: it shares only the
 *	configuration-mode envelope (0xFF / 0xFE) with base/C. Every other
 *	opcode and the entire data-frame meaning differ. This header is
 *	therefore *standalone* — it intentionally does NOT include
 *	ld2410_base.h, so consumers compiled with LD2410_VARIANT_S see
 *	only opcodes the S firmware can actually answer.
 *
 *	Notable differences from base/C:
 *	  - 16 distance gates (vs 9), with both "nearest" and "farthest"
 *	    settable max-gate parameters.
 *	  - Default UART baud 115200 (vs 57600 / 256000).
 *	  - Per-gate sensitivity is split into TWO independent parameters:
 *	    "trigger threshold" (0x72/0x73) and "hold threshold" (0x76/0x77).
 *	  - No engineering-mode toggle: the per-gate energy block is part
 *	    of the always-on "standard" frame (data type 0x01).
 *	  - "Minimal" frame format with 1-byte head (0x6E) and 1-byte tail
 *	    (0x62), selected via 0x7A "switch output mode".
 *	  - Auto-threshold tuning command (0x09) reporting progress on a
 *	    dedicated data type (0x03).
 *	  - Serial-number commands (0x10 / 0x11).
 *	  - No factory reset, no restart, no baud-rate change, no Bluetooth,
 *	    no MAC, no distance-resolution switch.
 */
#ifndef ld2410_s_h
#define ld2410_s_h

#include <stdint.h>

// ---- Variant identification -----------------------------------------------
#define LD2410_VARIANT_NAME "LD2410S"

// ---- Distance gates (HLK-S §2.2.7 Table 2-2) ------------------------------
// 16 gates indexed 0..15. Two independent settable max-gate parameters:
// "farthest" (1..16) and "nearest" (0..16). Gate width is NOT specified
// in the V1.00 protocol document, so LD2410_GATE_WIDTH_CM is intentionally
// undefined here — derive distance from the firmware-reported "object
// distance" field instead of computing gate × width.
#define LD2410_GATE_COUNT             16
#define LD2410_GATE_INDEX_FIRST       0
#define LD2410_GATE_INDEX_LAST        15

#define LD2410_MAX_GATE_MIN           1   // farthest gate minimum
#define LD2410_MAX_GATE_MAX           16  // farthest gate maximum

#define LD2410_NEAREST_GATE_MIN       0
#define LD2410_NEAREST_GATE_MAX       16

// ---- UART defaults --------------------------------------------------------
// 115200 fixed (no 0xA1-style baud-change command on S).
#define LD2410_DEFAULT_BAUD           115200

// ---- Data-frame types -----------------------------------------------------
// S protocol semantics differ from base/C:
//   0x01 on S  = "standard" frame, ALWAYS includes per-gate energies inline.
//                (On base/C, 0x01 means "engineering frame appended".)
//   0x03 on S  = auto-threshold progress (only emitted while 0x09 runs).
// Code that needs to disambiguate at compile time uses
// LD2410_HAS_OUTPUT_MODE (defined here only) as the discriminator.
#define LD2410_DATA_TYPE_STANDARD       0x01
#define LD2410_DATA_TYPE_AUTO_THRESHOLD 0x03

// ---- Minimal-frame envelope (S only) --------------------------------------
// One-byte head/tail used in "minimal" output mode. Selected via 0x7A.
// The full standard envelope (F4 F3 F2 F1 / F8 F7 F6 F5) lives in
// ld2410_frame.h and is shared.
#define LD2410_MINIMAL_FRAME_HEAD     0x6E
#define LD2410_MINIMAL_FRAME_TAIL     0x62

// ---- Configuration commands (shared with base/C) --------------------------
// Same opcodes/values as the base/C envelope; redefined here so this
// header is standalone. ACK fields differ slightly: S returns protocol
// version 0x0003 and buffer size 0x0080 (vs 0x0001 / 0x0040 on base/C).
#define LD2410_OP_ENABLE_CFG          0xFF  // §2.2.3 enable configuration mode
#define LD2410_OP_END_CFG             0xFE  // §2.2.4 end configuration mode

// ---- S-only command opcodes (HLK-S §2.2) ----------------------------------
#define LD2410_OP_FIRMWARE_VERSION    0x00  // §2.2.2 read firmware version
#define LD2410_OP_AUTO_THRESHOLD      0x09  // §2.2.9 automatic threshold update
#define LD2410_OP_WRITE_SN            0x10  // §2.2.5 write serial number
#define LD2410_OP_READ_SN             0x11  // §2.2.6 read serial number
#define LD2410_OP_WRITE_GENERIC_PARAMS 0x70 // §2.2.7 write generic parameters
#define LD2410_OP_READ_GENERIC_PARAMS  0x71 // §2.2.8 read generic parameters
#define LD2410_OP_WRITE_TRIGGER_THRESH 0x72 // §2.2.10 write trigger threshold
#define LD2410_OP_READ_TRIGGER_THRESH  0x73 // §2.2.11 read trigger threshold
#define LD2410_OP_WRITE_HOLD_THRESH    0x76 // §2.2.12 write hold threshold
#define LD2410_OP_READ_HOLD_THRESH     0x77 // §2.2.13 read hold threshold
#define LD2410_OP_OUTPUT_MODE          0x7A // §2.2.1 switch output mode

// ---- 0x70 / 0x71 generic parameter words (HLK-S Table 2-2) ----------------
#define LD2410_PARAM_FARTHEST_GATE    0x05
#define LD2410_PARAM_NEAREST_GATE     0x0A
#define LD2410_PARAM_UNMANNED_DELAY   0x06
#define LD2410_PARAM_STATUS_FREQ      0x02
#define LD2410_PARAM_DISTANCE_FREQ    0x0C
#define LD2410_PARAM_RESPONSE_SPEED   0x0B

// Unmanned-delay range (seconds; sent as the 4-byte parameter value).
// Note S has a tighter range than base/C (which accepts 0..65535).
#define LD2410_UNMANNED_DELAY_MIN_S   10
#define LD2410_UNMANNED_DELAY_MAX_S   120

// Reporting-frequency encoding: byte value = Hz × 10
//   0.5 Hz  → 5
//   1.0 Hz  → 10
//   8.0 Hz  → 80
// Valid range 0.5..8 Hz, step 0.5.
#define LD2410_REPORTING_FREQ_BYTE_MIN  5
#define LD2410_REPORTING_FREQ_BYTE_MAX  80
#define LD2410_REPORTING_FREQ_TO_BYTE(hz) ((uint8_t)((hz) * 10))

// Response-speed values (sent verbatim as the 4-byte parameter value).
#define LD2410_RESPONSE_SPEED_NORMAL  5
#define LD2410_RESPONSE_SPEED_FAST    10

// ---- 0x7A "switch output mode" payloads -----------------------------------
// 6-byte parameter sequence sent after the command word. Defined as
// constexpr arrays because the value does not fit a single uintN_t.
constexpr uint8_t LD2410_OUTPUT_MODE_STANDARD_PAYLOAD[6] = {
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00
};
constexpr uint8_t LD2410_OUTPUT_MODE_MINIMAL_PAYLOAD[6] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ---- 0x09 auto-threshold parameter ranges (HLK-S §2.2.9) -----------------
// All three are 2-byte LE values transmitted in the order:
//   trigger_factor + retention_factor + scanning_time
// The example in §2.2.9 uses (2, 1, 120). Document does not give explicit
// min/max — these defaults are the reference values from the example.
#define LD2410_AUTO_THRESH_TRIGGER_DEFAULT    2
#define LD2410_AUTO_THRESH_RETENTION_DEFAULT  1
#define LD2410_AUTO_THRESH_SCANTIME_DEFAULT   120

// ---- Feature flags --------------------------------------------------------
// LD2410_HAS_OUTPUT_MODE is the single most useful flag for variant-aware
// code: it is defined ONLY on S, so #ifdef LD2410_HAS_OUTPUT_MODE serves
// as a clean "we are compiling for S" predicate.
#define LD2410_HAS_CONFIGURATION_MODE
#define LD2410_HAS_FIRMWARE_VERSION
#define LD2410_HAS_GENERIC_PARAMS
#define LD2410_HAS_TRIGGER_THRESHOLD
#define LD2410_HAS_HOLD_THRESHOLD
#define LD2410_HAS_AUTO_THRESHOLD
#define LD2410_HAS_OUTPUT_MODE
#define LD2410_HAS_SERIAL_NUMBER

// Capabilities NOT present on the S variant (intentionally undefined):
//   LD2410_HAS_MAX_VALUES         (replaced by GENERIC_PARAMS / 0x70)
//   LD2410_HAS_READ_PARAMS        (replaced by GENERIC_PARAMS / 0x71)
//   LD2410_HAS_ENGINEERING_MODE   (no toggle — std mode is always per-gate)
//   LD2410_HAS_GATE_SENSITIVITY   (replaced by TRIGGER + HOLD thresholds)
//   LD2410_HAS_BAUD_RATE          (115200 fixed)
//   LD2410_HAS_FACTORY_RESET
//   LD2410_HAS_RESTART
//   LD2410_HAS_BLUETOOTH
//   LD2410_HAS_MAC_ADDRESS
//   LD2410_HAS_DISTANCE_RESOLUTION

#endif
