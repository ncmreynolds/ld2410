# 08 — Advanced API and build configuration

Topics for users who already have a working sketch and want to go
further: raw frame access, debug instrumentation, build flags, and
internals that don't appear in the per-variant API pages.

---

## `getFrameData`

### `FrameData getFrameData() const`

Return the most recent valid **data frame** (not command/ACK frame)
as `{ const uint8_t* data; uint16_t length; }`. Both fields are zero
if no valid frame is available since `begin()`. The pointer is into
the library's internal buffer — copy out before issuing more reads,
or wrap your access in a snapshot routine.

Frame layout (HLK protocol §2.3):

```
[0..3]   F4 F3 F2 F1                  4-byte data-frame magic header
[4..5]   intra-length (uint16 LE)     N
[6..N+5] intra-frame data             N bytes
[N+6..N+9] F8 F7 F6 F5                4-byte data-frame magic footer
```

Total length = `intra_length + 10`.

```cpp
// 1. Forward raw frames to a logger
FrameData fd = radar.getFrameData();
if (fd.data) myLog.write(fd.data, fd.length);

// 2. Inspect a specific intra-byte for a custom field not yet exposed
FrameData fd = radar.getFrameData();
if (fd.data && fd.length >= 17) Serial.println(fd.data[16], HEX);

// 3. Snapshot raw bytes and process out-of-thread
static uint8_t copy[LD2410_MAX_FRAME_LENGTH];
static uint16_t copy_len = 0;
{
  FrameData fd = radar.getFrameData();
  if (fd.data) { memcpy(copy, fd.data, fd.length); copy_len = fd.length; }
}
processInBackground(copy, copy_len);
```

`getFrameData()` does NOT lock — under `autoReadTask` on ESP32 the
frame may be partially overwritten between the time you grab the
pointer and the time you finish reading. For coherent multi-byte
access either copy the bytes inline or use the higher-level
`snapshotTargetState` / `snapshotEngineering*` accessors.

---

## Debug flags

The library can emit verbose serial diagnostics if you opt in at
compile time via these macros:

| Macro | What it logs | Cost |
|---|---|---|
| `LD2410_DEBUG_PARSE`    | Per-frame parse traces (header sync, intra length, validation) | High — chatty |
| `LD2410_DEBUG_COMMANDS` | Per-ACK opcode + intra length + OK/failed | Low |
| `LD2410_DEBUG_DATA`     | (legacy alias of `LD2410_DEBUG_PARSE`) | High |

Enable by either defining the macro before `#include <ld2410.h>` (or
the variant entry header such as `<ld2410c.h>`), or by passing a `-D`
build flag at compile time. Because the library is header-only, the
macro is visible to every translation unit that consumes the header
— there's no library `.cpp` to recompile separately. The library
still requires you to call `radar.debug(stream)` to attach a
`Stream` for the output.

```cpp
// Build with -DLD2410_DEBUG_COMMANDS
#define LD2410_DEBUG_COMMANDS
#include <ld2410.h>

void setup() {
  Serial.begin(115200);
  radar.debug(Serial);                 // diagnostics go to USB Serial
  radar.begin(RADAR_SERIAL);
  radar.setMaxValues(8, 8, 5);         // will print: "ACK for setting max values: OK"
}
```

There is also a top-level `LD2410_DEBUG` umbrella that turns on all
of the above. Use it when bringing up a new variant or wiring; turn
it off in production builds (debug prints add ~2 KB flash and
introduce small UART overhead).

```
#define LD2410_DEBUG          // turns on _PARSE + _COMMANDS
```

---

## Build-time variant selection (recap)

The variant macros control which `LD2410_HAS_*` capability flags are
enabled:

| Macro | Set automatically by | Default | Enables |
|---|---|---|---|
| `LD2410_VARIANT_BASE` | `<ld2410.h>` when no other is set | yes | base + common subset |
| `LD2410_VARIANT_B`    | `<ld2410b.h>` (or `-DLD2410_VARIANT_B`) | no | base + bluetooth + MAC + baud + distance resolution + B-only aux-control |
| `LD2410_VARIANT_C`    | `<ld2410c.h>` (or `-DLD2410_VARIANT_C`) | no | base + bluetooth + MAC + baud + distance resolution |
| `LD2410_VARIANT_S`    | `<ld2410s.h>` (or `-DLD2410_VARIANT_S`) | no | S-only commands (generic params, thresholds, output mode, …) |

Define exactly one (or include exactly one entry header — they're
idempotent under `#ifndef` guards, so mixing entry header +
matching `-D` is safe). Setting two different ones simultaneously
hits the mutual-exclusion guard in `src/ld2410.h:46-48` and produces
a `#error` at compile time. See
[`02-variants.md`](02-variants.md#selecting-the-variant-at-build-time)
for the full comparison.

---

## Capability flags

The capability flags are introspectable at compile time, useful for
writing portable sketches that adapt to the active variant:

```cpp
#include <ld2410.h>

void setup() {
  // ...
  #ifdef LD2410_HAS_BLUETOOTH
  radar.setBluetooth(false);
  #endif

  #ifdef LD2410_HAS_AUTO_THRESHOLD
  radar.autoUpdateThreshold(2, 1, 120);
  #endif
}
```

Full list of flags and their corresponding methods is in
[`method-coverage.md`](method-coverage.md) (the right-most column of
Table 1).

---

## Compile-time sizing constants

| Constant | Value | Notes |
|---|---|---|
| `LD2410_GATE_COUNT` | 9 (base/C) or 16 (S) | Number of energy gates per frame |
| `LD2410_MAX_FRAME_LENGTH` | 64 (base/C) or 96 (S) | Max bytes per data frame including envelope |
| `LD2410_BUFFER_SIZE` | 4 × `LD2410_MAX_FRAME_LENGTH` | Internal circular UART buffer (256 / 384 B) |
| `LD2410_DEFAULT_BAUD` | 57600 / 256000 / 115200 | UART default by variant |

Override `LD2410_BUFFER_SIZE` in your build flags if you have plenty
of RAM and want headroom against slow `loop()` cycles:

```
-DLD2410_BUFFER_SIZE=2048
```

---

## Thread-safety model

The library is **not** thread-safe in general. The exceptions:

1. **`autoReadTask` is internally synchronised.** The task uses
   `portENTER_CRITICAL(&data_mux_)` to update the field cache, so
   the snapshot APIs (`snapshotTargetState`, `snapshotEngineering*`)
   are race-safe. The individual getters are NOT race-safe — see
   [`03-runtime-models.md#the-dual-core-race-and-how-the-library-solves-it`](03-runtime-models.md#the-dual-core-race-and-how-the-library-solves-it).

2. **Command issuers take a mutex.** All `request*` / `set*` /
   `write*` methods on ESP32 acquire `cmd_mutex_` (a FreeRTOS
   semaphore) so two contexts can't issue overlapping commands. On
   non-ESP32 platforms the mutex is a no-op stub.

What's NOT safe:

- Calling `read()` from one core while `autoReadTask` is running on
  another (UART buffer corruption).
- Calling getters from an ISR (no synchronisation, ISRs can fire
  mid-update).
- Sharing one `ld2410` instance across two physical radars (use one
  instance per radar).

---

## Internal architecture (for contributors)

This section describes implementation, not API. Skip unless you're
modifying the library.

### Parser pipeline

```
UART RX → circular_buffer (ring) → read_frame_() (state machine) →
  ├── parse_data_frame_()      → updates target/engineering fields
  ├── parse_command_frame_()   → updates latest_ack_/last_command_success_
  └── parse_minimal_frame_()   → S-only 5-byte frame
```

State variables:

- `radar_data_frame_position_` — current byte position in the
  in-flight frame (0 = no frame in progress)
- `ack_frame_` — true if the in-flight frame is a command ACK
- `expected_ack_opcode_` — set by `begin_command_()`, read by
  `parse_command_frame_()` for ACK matching
- `cmd_seq_` / `cmd_ack_seq_` — monotonic counter to discard stale
  ACKs from previously-timed-out commands

### Command transaction guard

`CommandTransaction` is an RAII guard around `cmd_mutex_` (ESP32) /
no-op (others). Every public `request*` / `set*` method begins with:

```cpp
CommandTransaction tx(*this);
if (!tx.ok()) return false;
```

ensuring that two concurrent issuers serialise.

### Snapshot pattern

The snapshot accessors (`snapshotTargetState`,
`snapshotEngineeringMotionEnergies`,
`snapshotEngineeringStationaryEnergies`) all use the same template:

```cpp
#if defined(ESP32)
  portENTER_CRITICAL(&data_mux_);
#endif
  // copy fields / memcpy arrays
#if defined(ESP32)
  portEXIT_CRITICAL(&data_mux_);
#endif
```

`data_mux_` is `mutable` so const methods can acquire it. On
non-ESP32 platforms the lock degenerates to a plain copy — single
thread assumption.

---

## See also

- [`04-api-core.md`](04-api-core.md) through [`07-api-ld2410s.md`](07-api-ld2410s.md) — per-variant API
- [`09-troubleshooting.md`](09-troubleshooting.md) — common runtime issues
- [`method-coverage.md`](method-coverage.md) — capability matrix (source of truth)
- [`src/ld2410.h`](../src/ld2410.h) — class declaration with field-by-field documentation
