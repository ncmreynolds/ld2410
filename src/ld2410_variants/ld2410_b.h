/*
 *	LD2410B — opcode, parameter and feature definitions.
 *
 *	Reference: docs/HLK-LD2410B_protocol.md (V1.06, 2023-02-20).
 *
 *	The B variant is a strict superset of the C variant in terms of
 *	UART protocol: it shares every opcode 0x60..0xAB with the C
 *	(Bluetooth + MAC + distance-resolution included) and adds two
 *	new commands:
 *
 *	  0xAD — set auxiliary (light-sense) control mode + threshold
 *	         + OUT-pin default level  (HLK §2.2.18)
 *	  0xAE — query auxiliary control configuration  (HLK §2.2.19)
 *
 *	On top of the command additions, the B's engineering-mode data
 *	frame specifies two extra trailer bytes (photosensitivity value
 *	+ OUT pin output state, HLK §2.3.2 Table 15). These occupy the
 *	SAME wire slot that base/C document as variable "M reserved",
 *	so the total frame length matches base/C — the B simply assigns
 *	specific semantics to those bytes.
 *
 *	Physical pinout follows the BASE layout (OUT on pin 1, UART_Tx
 *	on pin 2, UART_Rx on pin 3, GND on pin 4, VCC on pin 5), NOT
 *	the C layout. This affects wiring but not the UART protocol the
 *	library speaks, so the variant header does not need to model it.
 *
 *	The default UART baud rate matches the C (256000), so the
 *	LD2410_DEFAULT_BAUD inherited from ld2410_c.h is correct.
 *
 *	UNVERIFIED ON HARDWARE: the maintainer's bench does not currently
 *	have an LD2410B sample. The implementation has been reviewed
 *	against the PDF and exercises code paths that are byte-identical
 *	to the LD2410C ones (already hardware-validated), but the new
 *	0xAD/0xAE branches and the engineering-frame trailer extraction
 *	have not been wire-tested.
 *
 *	This header is included automatically when LD2410_VARIANT_B is
 *	defined; it pulls in ld2410_c.h (which itself pulls in
 *	ld2410_base.h) so that variant-aware code in ld2410.cpp can use
 *	a single set of #ifdef gates regardless of which variant header
 *	was included.
 */
#ifndef ld2410_b_h
#define ld2410_b_h

#include "ld2410_c.h"

// ---- Variant identification (override C → B) ------------------------------
#undef  LD2410_VARIANT_NAME
#define LD2410_VARIANT_NAME "LD2410B"

// ---- B-extension command opcodes (HLK §2.2.18 .. §2.2.19) -----------------
#define LD2410_OP_AUX_CONTROL_SET         0xAD  // §2.2.18 set light-sense auxiliary control
#define LD2410_OP_AUX_CONTROL_GET         0xAE  // §2.2.19 query light-sense auxiliary control

// ---- 0xAD / 0xAE (auxiliary control) value layout (HLK Table 9) -----------
// The 4-byte command value (and the 4-byte payload returned by 0xAE) is
// laid out as:
//
//   byte 0  — mode:  0x00 = off (OUT not affected by light sense)
//                    0x01 = trigger when light value < threshold
//                    0x02 = trigger when light value > threshold
//   byte 1  — threshold (0x00..0xFF), default 0x80
//   byte 2  — OUT pin default level: 0x00 = idle LOW (factory default;
//                    triggered = HIGH), 0x01 = idle HIGH (triggered = LOW)
//   byte 3  — reserved (always 0x00 in the PDF example; meaning unspecified)
#define LD2410_AUX_MODE_OFF                  0x00
#define LD2410_AUX_MODE_TRIGGER_BELOW        0x01
#define LD2410_AUX_MODE_TRIGGER_ABOVE        0x02
#define LD2410_AUX_THRESHOLD_DEFAULT         0x80
#define LD2410_AUX_OUT_DEFAULT_LOW           0x00
#define LD2410_AUX_OUT_DEFAULT_HIGH          0x01

// ---- B-extension feature flags --------------------------------------------
// Composed on top of the C feature flags (LD2410_HAS_BLUETOOTH,
// LD2410_HAS_MAC_ADDRESS, LD2410_HAS_DISTANCE_RESOLUTION) and the base
// flags pulled in via ld2410_c.h → ld2410_base.h.
#define LD2410_HAS_AUX_CONTROL

// Capabilities NOT present on the B variant (S-only):
//   LD2410_HAS_TRIGGER_THRESHOLD
//   LD2410_HAS_HOLD_THRESHOLD
//   LD2410_HAS_AUTO_THRESHOLD
//   LD2410_HAS_OUTPUT_MODE
//   LD2410_HAS_SERIAL_NUMBER
//   LD2410_HAS_GENERIC_PARAMS

#endif
