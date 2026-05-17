# 01 — Getting started

This guide gets you from "radar in a bag" to "values printed in the
Serial Monitor" in three steps. It assumes the original LD2410 or
LD2410C variant; for LD2410S see the variant-specific notes in
[`02-variants.md`](02-variants.md).

## What this library does

`ld2410` is an Arduino library that talks the HLK serial
protocol used by the LD2410 / LD2410C / LD2410S 24 GHz mmWave presence
radars. The radar streams target detection data continuously over UART
at 256000 baud (C) / 57600 baud (base) / 115200 baud (S); this library
parses the frames, exposes the latest values via getters, and lets you
issue configuration commands.

What you can do with it:

- detect human presence and distinguish moving from stationary targets
- read distance + energy per target type
- read per-gate energy in engineering mode (signal strength bucketed
  by 0.75 m or 0.20 m distance ranges, depending on the variant)
- configure thresholds, gate sensitivity, idle timeout
- read/write firmware metadata (version, MAC, serial number)
- (LD2410C) toggle Bluetooth, change baud rate, change distance resolution
- (LD2410S) configure auto-threshold tuning, generic params, output mode

What it does **not** do: provide BLE pairing (HLK has its own Android app
for that), drive the radar's OUT pin, or perform any signal-processing
beyond what the radar firmware already does.

## Hardware

| What you need | Notes |
|---|---|
| 1× HLK LD2410 / LD2410C / LD2410S | LD2410C is the most widely available; works at 5 V. LD2410S needs **3.3 V** — wiring it to 5 V will brick it. |
| 1× MCU with a free hardware UART | ESP32 (any), ESP8266, RP2040, ATmega32u4 (Leonardo / Pro Micro). Plain UNO does NOT work — its single UART is needed for the Serial Monitor. |
| 4 jumper wires | VCC, GND, radar TX → MCU RX, radar RX → MCU TX. |

The pin labels on the radar header **differ between variants** — see the
table in [`02-variants.md`](02-variants.md) before wiring. A radar
labelled "LD2410" can be either the original or the C variant, and they
have OUT and TX on different pins.

## Hello-world sketch

The smallest useful sketch is in
[`../examples/basicSensor/basicSensor.ino`](../examples/basicSensor/basicSensor.ino) — copy it and
upload. The essential parts are:

```cpp
#include <ld2410.h>

ld2410 radar;

void setup() {
  Serial.begin(115200);                                      // monitor
  Serial1.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  if (!radar.begin(Serial1)) {                               // sync first frame
    Serial.println("radar.begin: FAIL");
    while (true) delay(1000);
  }
}

void loop() {
  radar.read();                                              // drain UART, parse one frame if available
  if (radar.presenceDetected()) {
    Serial.print("detection at ");
    Serial.print(radar.detectionDistance());
    Serial.println(" cm");
  }
  delay(100);
}
```

Three things to know about that loop:

1. **`radar.read()` is non-blocking.** It returns true only when a
   complete frame was parsed; otherwise it just drains pending UART
   bytes and returns. Call it as often as you like.
2. **Getters return the *latest* values**, not stream events. They
   never block. The radar emits a frame every ~50 ms at default
   settings, so polling at 10 Hz will see every frame.
3. **No callback, no event loop.** This is a polling library by design,
   to keep it portable across AVR / ESP / RP2040. If you need a
   background reader on ESP32, see
   [`03-runtime-models.md`](03-runtime-models.md).

## What baud rate?

| Variant | Default radar baud | Why |
|---|---|---|
| LD2410 (base) | 57600 | original spec |
| LD2410C | 256000 | bumped in firmware revision |
| LD2410S | 115200 | new variant |

The constant `LD2410_DEFAULT_BAUD` resolves to the correct value at
compile time based on the active variant macro
(`LD2410_VARIANT_BASE` / `_C` / `_S`). Use it instead of hard-coding
the number:

```cpp
RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
```

If you change the radar's baud (LD2410C only — see
[`06-api-ld2410c.md`](06-api-ld2410c.md)), remember the new rate is
non-volatile: re-open `RADAR_SERIAL.begin(NEW_BAUD, ...)` after the
restart.

## What variant am I building for?

By default the library compiles for the original LD2410. There are two
ways to target a different variant; both produce the exact same build.

### Recommended: use the variant entry header (works everywhere)

Include one of the variant-specific headers instead of `<ld2410.h>`.
That's it — no `#define`, no build flag:

| Sensor | Include this |
|---|---|
| LD2410 (base) | `#include <ld2410.h>` |
| LD2410**B** | `#include <ld2410b.h>` |
| LD2410**C** | `#include <ld2410c.h>` |
| LD2410**S** | `#include <ld2410s.h>` |

```cpp
#include <ld2410c.h>
ld2410 radar;
```

This is the only route that works **identically on the Arduino IDE
GUI, arduino-cli, and PlatformIO**. The Arduino IDE GUI does not
support per-sketch `-D` build flags, so the entry-header approach is
the one to recommend to non-PlatformIO users.

> **Important — multi-file projects.** Use the same entry header in
> *every* translation unit (every `.ino`, `.cpp` in the sketch folder
> that touches the library). Including `<ld2410.h>` in one file and
> `<ld2410c.h>` in another silently produces two different `class
> ld2410` layouts → undefined behaviour. One header, used
> consistently, fixes this.

### Alternative: define the variant macro before `<ld2410.h>`

Equivalent — same final build. Use this if you prefer to drive the
selection from a build flag (PlatformIO `build_flags`, arduino-cli
`--build-property "build.extra_flags=..."`), or if you have an existing
sketch using this pattern:

```cpp
#define LD2410_VARIANT_C    // or LD2410_VARIANT_B / _S, default is BASE
#include <ld2410.h>
```

Or as a build flag:

```
-DLD2410_VARIANT_C
```

The two routes compose: defining `LD2410_VARIANT_C` and then including
`<ld2410c.h>` is a no-op (the entry header is idempotent).

### What happens if I call a method that doesn't exist on my variant?

Clean compile error pointing at the missing `LD2410_HAS_*` flag — no
silent runtime failures. See [`02-variants.md`](02-variants.md) for
the capability matrix.

## What next?

- Configure thresholds and sensitivity:
  [`05-api-ld2410-base.md`](05-api-ld2410-base.md)
- Run a background reader task on ESP32:
  [`03-runtime-models.md`](03-runtime-models.md) +
  [`../examples/autoReadTask/autoReadTask.ino`](../examples/autoReadTask/autoReadTask.ino)
- Read per-gate energy values:
  [`04-api-core.md#engineering-mode-accessors`](04-api-core.md) +
  [`../examples/engineeringMode/engineeringMode.ino`](../examples/engineeringMode/engineeringMode.ino)
- Change distance resolution or Bluetooth (LD2410C only):
  [`06-api-ld2410c.md`](06-api-ld2410c.md)
- Something doesn't work: [`09-troubleshooting.md`](09-troubleshooting.md)
