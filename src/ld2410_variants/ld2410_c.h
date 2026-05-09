/*
 *	LD2410C — opcode, parameter and feature definitions.
 *
 *	Reference: docs/HLK-LD2410C_protocol.md (V1.00, 2022-11-07).
 *
 *	The C variant shares the entire base protocol (opcodes 0x60..0x64,
 *	0xA0..0xA3, 0xFF/0xFE) with LD2410, and adds six extension opcodes:
 *	Bluetooth control, MAC address, distance-resolution switching.
 *
 *	This header is included automatically when LD2410_VARIANT_C is
 *	defined; it pulls in ld2410_base.h then layers C-specific symbols
 *	on top.
 */
#ifndef ld2410_c_h
#define ld2410_c_h

#include "ld2410_base.h"

// ---- Variant identification (override base) -------------------------------
#undef  LD2410_VARIANT_NAME
#define LD2410_VARIANT_NAME "LD2410C"

// ---- UART defaults (override base; HLK §2.2.9 factory index = 0x0007) -----
// Factory baud rate is 256000 on the C variant (vs 57600 on base).
#undef  LD2410_DEFAULT_BAUD
#define LD2410_DEFAULT_BAUD 256000

// ---- Distance gates -------------------------------------------------------
// Gate count and 0.75 m default width are inherited from base. The C
// variant additionally supports a RUNTIME-switchable resolution via the
// 0xAA command (LD2410_OP_DISTANCE_RESOLUTION_SET): default 0.75 m,
// alternate 0.2 m. This is a runtime mode change, not a compile-time
// constant — the actual gate width on the wire follows whatever was
// last written via 0xAA.
//
// LD2410_GATE_WIDTH_CM (= 75) is therefore the *default* width; user
// code that switched to 0.2 m must remember its setting locally or
// query it back via 0xAB.

// ---- C-extension command opcodes (HLK §2.2.12 .. §2.2.17) -----------------
#define LD2410_OP_BLUETOOTH               0xA4  // §2.2.12 enable/disable Bluetooth
#define LD2410_OP_GET_MAC                 0xA5  // §2.2.13 get Bluetooth MAC
#define LD2410_OP_BLUETOOTH_PERMS         0xA8  // §2.2.14 obtain Bluetooth permissions
#define LD2410_OP_BLUETOOTH_PASSWORD      0xA9  // §2.2.15 set Bluetooth password
#define LD2410_OP_DISTANCE_RESOLUTION_SET 0xAA  // §2.2.16 set distance resolution
#define LD2410_OP_DISTANCE_RESOLUTION_GET 0xAB  // §2.2.17 query distance resolution

// ---- 0xA8 / 0xA9 (Bluetooth password) ------------------------------------
// HLK §2.2.14 / §2.2.15: both commands take a 6-byte password value sent
// byte-by-byte in wire order (the PDF describes it as "6 bytes of password
// value (every 2 bytes in little-endian order)", but the §2.2.14 example
// shows the ASCII string "HiLink" → bytes 48 69 4C 69 6E 6B emitted left to
// right on the wire). Factory default password is "HiLink".
#define LD2410_BLUETOOTH_PASSWORD_LENGTH  6
#define LD2410_BLUETOOTH_PASSWORD_DEFAULT "HiLink"

// ---- 0xA4 (Bluetooth on/off) parameter values ----------------------------
// HLK PDF §2.2.12 prints the value as "0x0100 turn on bluetooth", but the
// example send payload right next to it is `A4 00 | 01 00` — i.e. the
// two value bytes are 01 00 in WIRE order. Interpreted little-endian
// (matching every other 16-bit parameter on this protocol), that's
// 0x0001, not 0x0100. The PDF wording mixes wire-byte ordering with a
// scalar reading; the table example is authoritative.
#define LD2410_BLUETOOTH_ON               0x0001
#define LD2410_BLUETOOTH_OFF              0x0000

// ---- 0xAA / 0xAB (distance resolution) values (HLK Table 8) ---------------
// Note: the original HLK PDF has a typo claiming the factory default is
// 0x0001; the table mapping confirms 0x0000 (= 0.75 m) is correct.
#define LD2410_DISTANCE_RESOLUTION_075M   0x0000
#define LD2410_DISTANCE_RESOLUTION_02M    0x0001
#define LD2410_DISTANCE_RESOLUTION_DEFAULT LD2410_DISTANCE_RESOLUTION_075M

// ---- C-extension feature flags --------------------------------------------
// These compose with the base feature flags already defined by
// ld2410_base.h (LD2410_HAS_FACTORY_RESET, RESTART, BAUD_RATE,
// ENGINEERING_MODE, etc.) so that variant-aware code in ld2410.cpp can
// use a single set of #ifdef gates regardless of which variant header
// was included.
#define LD2410_HAS_BLUETOOTH
#define LD2410_HAS_MAC_ADDRESS
#define LD2410_HAS_DISTANCE_RESOLUTION

// Capabilities still NOT present on the C variant (S-only):
//   LD2410_HAS_TRIGGER_THRESHOLD
//   LD2410_HAS_HOLD_THRESHOLD
//   LD2410_HAS_AUTO_THRESHOLD
//   LD2410_HAS_OUTPUT_MODE
//   LD2410_HAS_SERIAL_NUMBER
//   LD2410_HAS_GENERIC_PARAMS

#endif
