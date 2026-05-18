# 09 — Troubleshooting

Common problems and their root causes, in roughly the order people
hit them when bringing up a new bench.

If your symptom isn't here, please open an issue on GitHub — and
include the output of:

```cpp
#define LD2410_DEBUG
radar.debug(Serial);
```

so we can see the parser traces.

---

## "radar.begin() returns false"

Most common causes, in order of probability:

1. **Wires swapped (TX↔RX).** The radar's TX must connect to the
   MCU's RX, and vice versa. The pin labels on the radar header
   refer to the radar's perspective. Re-check
   [`02-variants.md#hardware-quick-reference`](02-variants.md#hardware-quick-reference).

2. **Wrong baud rate.** The library uses `LD2410_DEFAULT_BAUD` which
   resolves to 256000 (C) / 57600 (base) / 115200 (S). If you've
   changed the radar's baud previously via `setBaudRate()` and
   didn't restore, you need to open `RADAR_SERIAL.begin()` at the
   actual current baud.

3. **Wrong variant macro.** Defined `LD2410_VARIANT_S` while wiring
   an LD2410C? The default-baud constant resolves to 115200 instead
   of 256000 → no sync. Set the right macro, recompile.

4. **Power problem.** LD2410S needs 3.3 V (NOT 5 V). The base/C
   tolerate 5 V but draw > 200 mA at peak — a weak USB supply or a
   long thin wire to the regulator can brown out the radar at every
   detection peak.

5. **Radar bricked from a bad write.** Most commonly from issuing
   `setBaudRate` to a value not supported by your USB-UART bridge,
   then losing comms. Recovery: try every supported baud (9600 to
   460800) until one syncs. If none work, the radar's NVS may need
   a factory reset via the HLK Android app over Bluetooth (LD2410C
   only).

---

## "Garbled bytes in Serial Monitor (`���...`)"

Two distinct causes — diagnose by reading exactly what's garbled:

### 1. Boot ROM at a different baud (one-shot)

The ESP32 ROM bootloader prints to Serial at 74880 baud before the
sketch runs. If your monitor is at 115200, you'll see ~10–30 garbage
characters at every reset, then clean output once `setup()` runs and
opens Serial at your baud. **Harmless — ignore.**

### 2. CP2102 USB-UART clone clock skew at high baud (recurring)

Some cheap CP2102 clones (and CH340 too) have crystal tolerances of
±2-3 % which produce sporadic single-byte framing errors at 115200
baud on long status prints. Symptom: a "missing comma" inside an
array, e.g. `100,67` rendered as `10067`, or an occasional truncated
line. Reproduces with both `arduino-cli monitor` AND `picocom`, and
in synchronous mode without FreeRTOS — so it's neither a race nor a
monitor-side issue.

Fix that worked on the maintainer's bench (2026-05-09):

```cpp
// In your sketch, BEFORE Serial.begin():
Serial.setTxBufferSize(1024);
Serial.begin(57600);                  // drop monitor baud to 57600
```

The `setTxBufferSize(1024)` lets the full status line fit in one
shot and avoids HardwareSerial chunking it across multiple
`uart_write_bytes()` calls — fewer chances for a clock-skew framing
error to land between bytes.

The radar's own UART (256000 baud) is unaffected because the radar
chip uses a tighter crystal.

---

## "Random `0` values in moving/stationary distance"

You're likely calling several getters in a row from the user `loop()`
while `autoReadTask` is running on ESP32 dual-core. This is the
documented inter-field race — see
[`03-runtime-models.md#the-dual-core-race-and-how-the-library-solves-it`](03-runtime-models.md#the-dual-core-race-and-how-the-library-solves-it).

Replace with a snapshot:

```cpp
// BEFORE (race-prone):
if (radar.movingTargetDetected())
  log(radar.movingTargetDistance(), radar.movingTargetEnergy());

// AFTER (race-safe):
LD2410TargetState s; radar.snapshotTargetState(s);
if (s.target_type & 0x01) log(s.moving_distance, s.moving_energy);
```

Same fix for the per-gate engineering arrays:
[`snapshotEngineeringMotionEnergies / Stationary`](04-api-core.md#engineering-mode--per-gate-energy)
or example
[`../examples/snapshotAtomic/snapshotAtomic.ino`](../examples/snapshotAtomic/snapshotAtomic.ino).

---

## "Frame counter stops climbing / `isConnected()` flips false"

Your `loop()` is too slow. The library's circular buffer (256 B on
base/C, 384 B on S) holds ~4 frames; if `loop()` cycles in seconds,
you'll overflow.

Options:

1. **Speed up `loop()`** — usually the right answer. Identify the slow
   path with `delay()` / `millis()` profiling.
2. **Move to `autoReadTask`** on ESP32 — a background task drains
   UART continuously regardless of `loop()` latency.
3. **Increase the buffer** — `-DLD2410_BUFFER_SIZE=2048` if you have
   the RAM (256 B × 16 frames of headroom).

---

## "ACK timeout on a setBluetooth/setBaudRate/etc."

Two known causes:

### 1. The command is genuinely failing

Some firmwares (notably very old LD2410 base) silently reject
commands without an ACK. The ACK timeout is 100 ms by default. To
diagnose, enable `LD2410_DEBUG_COMMANDS`:

```cpp
#define LD2410_DEBUG_COMMANDS
#include <ld2410.h>
// ... radar.debug(Serial); ...
radar.setBluetooth(false);
// expected: "ACK for Bluetooth on/off: OK"
// if you see the request but no ACK: firmware doesn't support it
```

### 2. The command's ACK comes only over BLE

`obtainBluetoothPermissions()` is documented in the HLK PDF as
returning its ACK only over BLE, not UART. The library returns
`false` in this case. Treat it as "no UART feedback, expected".

---

## "engineeringRetrieved() never returns true"

You haven't enabled engineering mode. On base/C you must call
`requestStartEngineeringMode()` first. On S, engineering data is part
of the standard 0x01 frame — but only if the output mode is set to
standard (which is the default). If you previously called
`setOutputMode(false)` (minimal frame), engineering data isn't
emitted at all.

```cpp
// LD2410 base / C
radar.requestStartEngineeringMode();
delay(100);
radar.read();
if (radar.engineeringRetrieved()) Serial.println("got per-gate data");

// LD2410S
#ifdef LD2410_VARIANT_S
radar.setOutputMode(true);                // standard frame
delay(100);
radar.read();
if (radar.engineeringRetrieved()) Serial.println("got per-gate data");
#endif
```

---

## "Compile error: 'LD2410_OP_X' was not declared" / "'class ld2410' has no member named 'setBluetooth'"

You're calling a method on a variant that doesn't expose that
opcode. Example: `setBluetooth()` on the default
`LD2410_VARIANT_BASE` build will fail because `LD2410_HAS_BLUETOOTH`
is not defined.

Solution — pick the variant via the entry header (simplest, works
on every build system including the Arduino IDE GUI):

```cpp
#include <ld2410c.h>           // (or <ld2410b.h>, <ld2410s.h>)
```

Equivalent on PlatformIO / arduino-cli only — pass `-DLD2410_VARIANT_C`
as a build flag, or define before include:

```cpp
#define LD2410_VARIANT_C        // for setBluetooth, setBaudRate, etc.
#include <ld2410.h>
```

> ⚠️ The Arduino IDE GUI has **no** per-sketch `-D` build-flag
> mechanism, so `#define` before `#include` in your `.ino` is **not
> enough** on its own with a non-header-only library — that's why the
> entry-header route is the recommended one. The ld2410 library is
> header-only by design (since the variant-abstraction refactor) so
> all three routes now work, but the entry header is the most
> foolproof.

See [`02-variants.md`](02-variants.md) for the capability matrix
and [`method-coverage.md`](method-coverage.md) for the per-opcode
breakdown.

---

## "After requestRestart, no frames arrive for ~1 s"

Expected. The library suspends `autoReadTask` (if running) for the
~800 ms reboot blackout, drains the UART RX FIFO twice, clears the
parser state, then resumes. During this window:

- `read()` returns `false` (no data).
- `isConnected()` returns `false` (no recent frame).
- Getters return whatever value was current before the restart
  (cached, not zeroed — the library doesn't know if the frame is
  "still valid" or "stale").

After the blackout the parser resyncs on the first valid header
within the next few ms.

---

## "Two radars on the same MCU"

Use two `ld2410` instances bound to two different `Stream` objects.
There is no class-level shared state.

```cpp
ld2410 radarA;
ld2410 radarB;

void setup() {
  Serial1.begin(256000, SERIAL_8N1, RX_A, TX_A);
  Serial2.begin(256000, SERIAL_8N1, RX_B, TX_B);
  radarA.begin(Serial1);
  radarB.begin(Serial2);
}

void loop() {
  radarA.read();
  radarB.read();
  // ... process both independently
}
```

Each instance has its own circular buffer, its own task (if you call
`autoReadTask` on each), its own field cache.

---

## See also

- [`01-getting-started.md`](01-getting-started.md) — initial wiring + pin map
- [`02-variants.md`](02-variants.md) — variant capabilities + warnings
- [`03-runtime-models.md`](03-runtime-models.md) — sync vs async runtime model
- [`08-advanced.md`](08-advanced.md) — debug flags + thread-safety model
