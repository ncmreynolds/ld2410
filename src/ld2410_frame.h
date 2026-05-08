/*
 *	Frame envelope shared by every LD2410 family (base, C, S).
 *
 *	Per HLK protocol §2.1 the command path always uses the same magic
 *	bytes around the intra-frame data, and the data path always uses the
 *	same magic bytes around the radar-reported payload. The opcodes
 *	*inside* the envelope vary by variant (see ld2410_variants/) but the
 *	envelope itself never does.
 *
 *	Including this header is safe regardless of which LD2410_VARIANT_*
 *	is selected — it does not depend on the variant.
 */
#ifndef ld2410_frame_h
#define ld2410_frame_h

#include <Arduino.h>

// Command path (host → radar): see HLK protocol Tables 2/4.
constexpr uint8_t LD2410_CMD_FRAME_HEAD[4] = { 0xFD, 0xFC, 0xFB, 0xFA };
constexpr uint8_t LD2410_CMD_FRAME_TAIL[4] = { 0x04, 0x03, 0x02, 0x01 };

// Data path (radar → host) — standard / engineering frames: see HLK Tables 8/9.
// Both base/C and S use these 4-byte magic sequences for non-minimal frames.
constexpr uint8_t LD2410_DATA_FRAME_HEAD[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
constexpr uint8_t LD2410_DATA_FRAME_TAIL[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };

// Intra-frame head/tail bytes inside a non-minimal data frame: see HLK Table 9.
constexpr uint8_t LD2410_DATA_INTRA_HEAD = 0xAA;
constexpr uint8_t LD2410_DATA_INTRA_TAIL = 0x55;
constexpr uint8_t LD2410_DATA_INTRA_CHECK = 0x00;

// Helpers — write the four bytes of the command-frame envelope to a Stream.
// These replace the inline radar_uart_->write((byte)0xFD)... blocks in
// send_command_preamble_() / send_command_postamble_() and centralise the
// fact that all three variants share the same envelope.
inline void ld2410_write_cmd_frame_head(Stream * uart) {
	uart->write(LD2410_CMD_FRAME_HEAD[0]);
	uart->write(LD2410_CMD_FRAME_HEAD[1]);
	uart->write(LD2410_CMD_FRAME_HEAD[2]);
	uart->write(LD2410_CMD_FRAME_HEAD[3]);
}

inline void ld2410_write_cmd_frame_tail(Stream * uart) {
	uart->write(LD2410_CMD_FRAME_TAIL[0]);
	uart->write(LD2410_CMD_FRAME_TAIL[1]);
	uart->write(LD2410_CMD_FRAME_TAIL[2]);
	uart->write(LD2410_CMD_FRAME_TAIL[3]);
}

// Helpers for emitting little-endian multi-byte values onto the radar UART.
// The protocol uses LE consistently for length fields, command words, and
// parameter values; centralising the byte split here lets call sites read
// at the level of the protocol (a 16-bit param word) instead of the wire
// (two raw bytes) and avoids the "& 0xFF / >> 8" boilerplate.
inline void ld2410_write_le16(Stream * uart, uint16_t value) {
	uart->write((uint8_t)(value & 0xFF));
	uart->write((uint8_t)((value >> 8) & 0xFF));
}

inline void ld2410_write_le32(Stream * uart, uint32_t value) {
	uart->write((uint8_t)(value & 0xFF));
	uart->write((uint8_t)((value >> 8) & 0xFF));
	uart->write((uint8_t)((value >> 16) & 0xFF));
	uart->write((uint8_t)((value >> 24) & 0xFF));
}

#endif
