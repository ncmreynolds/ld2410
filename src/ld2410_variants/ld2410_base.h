/*
 *	LD2410 (base) — opcode, parameter and feature definitions.
 *
 *	Reference: docs/HLK-LD2410_protocol.md (protocol V1.02 / V1.03).
 *
 *	This header is included automatically when LD2410_VARIANT_BASE is
 *	defined (the default). The C-variant header (ld2410_c.h) includes
 *	this one and adds Bluetooth / MAC / distance-resolution opcodes.
 *	The S-variant (ld2410_s.h) is standalone — its protocol diverges
 *	too much to share definitions.
 *
 *	All opcodes are the LOW byte of the 2-byte little-endian command
 *	word; the high byte is always 0x00 for this protocol family.
 */
#ifndef ld2410_base_h
#define ld2410_base_h

#include <stdint.h>

// ---- Variant identification -----------------------------------------------
#define LD2410_VARIANT_NAME "LD2410"

// ---- Distance gates (HLK §1.2.2 + §2.2.3) ---------------------------------
// 9 gates at 0.75 m each. Two distinct ranges apply, do NOT confuse them:
//
//   1. Gate INDICES — used for array storage (motion_sensitivity[i],
//      stationary_sensitivity[i], engineering-frame Table 14 offsets) and
//      as the gate selector in the 0x64 per-gate sensitivity command:
//      valid range is 0..8 (9 values).
//
//   2. Max-distance-gate VALUES — sent as the parameter value of the 0x60
//      command parameter words LD2410_PARAM_MAX_MOVING /
//      LD2410_PARAM_MAX_STATIONARY: valid range is 1..8. Sending 0 is
//      rejected by the radar (HLK §1.2.2: "the setting range is 1 to 8";
//      see also upstream issue #28).
//
// Stationary sensitivity for gates 0 and 1 is factory-locked (HLK Table 7);
// the 0x64 command accepts it but the value is ignored by the firmware.
#define LD2410_GATE_COUNT             9
#define LD2410_GATE_INDEX_FIRST       0
#define LD2410_GATE_INDEX_LAST        8
#define LD2410_GATE_WIDTH_CM          75   // 0.75 m

#define LD2410_MAX_GATE_MIN           1
#define LD2410_MAX_GATE_MAX           8

// ---- UART defaults --------------------------------------------------------
// 57600 since protocol V1.03 (2023-03-22). V1.02 used a different default.
#define LD2410_DEFAULT_BAUD        57600

// ---- Data-frame types (HLK Table 11) --------------------------------------
#define LD2410_DATA_TYPE_BASIC         0x02
#define LD2410_DATA_TYPE_ENGINEERING   0x01

// ---- Command opcodes (HLK §2.2) -------------------------------------------
#define LD2410_OP_ENABLE_CFG         0xFF  // §2.2.1 enable configuration mode
#define LD2410_OP_END_CFG            0xFE  // §2.2.2 end configuration mode
#define LD2410_OP_SET_MAX_VALUES     0x60  // §2.2.3 max gate + unmanned duration
#define LD2410_OP_READ_PARAMS        0x61  // §2.2.4 read current configuration
#define LD2410_OP_START_ENGINEERING  0x62  // §2.2.5 enable engineering mode
#define LD2410_OP_END_ENGINEERING    0x63  // §2.2.6 close engineering mode
#define LD2410_OP_GATE_SENSITIVITY   0x64  // §2.2.7 range gate sensitivity
#define LD2410_OP_FIRMWARE_VERSION   0xA0  // §2.2.8 read firmware version
#define LD2410_OP_SET_BAUD_RATE      0xA1  // §2.2.9 set serial port baud rate
#define LD2410_OP_FACTORY_RESET      0xA2  // §2.2.10 restore factory settings
#define LD2410_OP_RESTART            0xA3  // §2.2.11 restart module

// ---- 0x60 (set max values) parameter words --------------------------------
#define LD2410_PARAM_MAX_MOVING        0x0000
#define LD2410_PARAM_MAX_STATIONARY    0x0001
#define LD2410_PARAM_UNMANNED_DELAY    0x0002

// ---- 0x64 (gate sensitivity) parameter words ------------------------------
#define LD2410_PARAM_GATE              0x0000
#define LD2410_PARAM_MOTION_SENS       0x0001
#define LD2410_PARAM_STATIONARY_SENS   0x0002
#define LD2410_PARAM_GATE_BROADCAST    0xFFFF  // all-gates magic value

// ---- 0xA1 (set baud rate) index values (HLK Table 6) ---------------------
#define LD2410_BAUD_INDEX_9600         0x0001
#define LD2410_BAUD_INDEX_19200        0x0002
#define LD2410_BAUD_INDEX_38400        0x0003
#define LD2410_BAUD_INDEX_57600        0x0004
#define LD2410_BAUD_INDEX_115200       0x0005
#define LD2410_BAUD_INDEX_230400       0x0006
#define LD2410_BAUD_INDEX_256000       0x0007
#define LD2410_BAUD_INDEX_460800       0x0008

// ---- Feature flags --------------------------------------------------------
// LD2410_HAS_<feature> is defined when the variant supports that
// command path. Variant-aware code in ld2410.cpp uses these as
// #ifdef gates so that methods only exist when the underlying chip
// can answer them.
#define LD2410_HAS_CONFIGURATION_MODE
#define LD2410_HAS_MAX_VALUES
#define LD2410_HAS_READ_PARAMS
#define LD2410_HAS_ENGINEERING_MODE
#define LD2410_HAS_GATE_SENSITIVITY
#define LD2410_HAS_FIRMWARE_VERSION
#define LD2410_HAS_BAUD_RATE
#define LD2410_HAS_FACTORY_RESET
#define LD2410_HAS_RESTART

// Capabilities NOT present in this variant (for reference — these
// macros are intentionally left undefined here):
//   LD2410_HAS_BLUETOOTH            (C only)
//   LD2410_HAS_MAC_ADDRESS          (C only)
//   LD2410_HAS_DISTANCE_RESOLUTION  (C only)
//   LD2410_HAS_TRIGGER_THRESHOLD    (S only)
//   LD2410_HAS_HOLD_THRESHOLD       (S only)
//   LD2410_HAS_AUTO_THRESHOLD       (S only)
//   LD2410_HAS_OUTPUT_MODE          (S only)
//   LD2410_HAS_SERIAL_NUMBER        (S only)
//   LD2410_HAS_GENERIC_PARAMS       (S only)

#endif
