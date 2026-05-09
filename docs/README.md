# LD2410 library — documentation index

User-facing reference for the `ld2410` Arduino library — see
[`../library.properties`](../library.properties) for the canonical
project URL and version.

This index is the jumping-off point. Every page below is self-contained
and cross-links to its peers; you don't need to read them in order.

## Start here

| File | Purpose |
|---|---|
| [`01-getting-started.md`](01-getting-started.md) | Wiring, hello-world sketch, first read in <50 lines. |
| [`02-variants.md`](02-variants.md) | Differences between LD2410 / LD2410C / LD2410S — pick the right one. |
| [`03-runtime-models.md`](03-runtime-models.md) | When to use synchronous `read()` vs the FreeRTOS `autoReadTask()`. |

## API reference (per variant)

| File | Methods covered | Variants |
|---|---|---|
| [`04-api-core.md`](04-api-core.md) | `begin`, `read`, presence/distance/energy getters, engineering frame access | all |
| [`05-api-ld2410-base.md`](05-api-ld2410-base.md) | Methods available on the original LD2410 | BASE + C |
| [`06-api-ld2410c.md`](06-api-ld2410c.md) | C-only extensions: baud rate, Bluetooth, MAC, distance resolution | LD2410C |
| [`07-api-ld2410s.md`](07-api-ld2410s.md) | S-only extensions: generic params, thresholds, minimal frame | LD2410S |
| [`08-advanced.md`](08-advanced.md) | Atomic snapshots, raw frame access, debug streams, build flags | all |

## Operations

| File | Purpose |
|---|---|
| [`09-troubleshooting.md`](09-troubleshooting.md) | CP2102 baud skew, torn reads, parser hangs, common pitfalls. |

## Reference material (technical, source of truth)

These files are the authoritative reference and should not be modified
except in lockstep with code changes:

| File | Purpose |
|---|---|
| [`method-coverage.md`](method-coverage.md) | Matrix of every public method × variant × opcode (source of truth for `LD2410_HAS_*` gates). |
| [`ld2410-variants-comparison.md`](ld2410-variants-comparison.md) | Byte-level side-by-side of the three protocols. |
| [`HLK-LD2410_protocol.md`](HLK-LD2410_protocol.md) | Original HLK protocol spec for the LD2410 base. |
| [`HLK-LD2410C_protocol.md`](HLK-LD2410C_protocol.md) | Original HLK protocol spec for the LD2410C. |
| [`HLK-LD2410S_protocol.md`](HLK-LD2410S_protocol.md) | Original HLK protocol spec for the LD2410S. |

## Examples

All sketches under [`../examples/`](../examples/) compile against the
default ESP32 / ESP8266 / RP2040 cores in CI. Each API page links to the
example sketch that exercises the relevant functions.

| Sketch | Demonstrates |
|---|---|
| [`basicSensor`](../examples/basicSensor/basicSensor.ino) | Synchronous `read()` + basic getters. Multi-platform. |
| [`basicSensorEsp8266`](../examples/basicSensorEsp8266/basicSensorEsp8266.ino) | Same on ESP8266 with SoftwareSerial. |
| [`autoReadTask`](../examples/autoReadTask/autoReadTask.ino) | FreeRTOS background reader on ESP32. |
| [`setupSensor`](../examples/setupSensor/setupSensor.ino) | Configuration writes (`setMaxValues`, `setGateSensitivityThreshold`). |
| [`engineeringMode`](../examples/engineeringMode/engineeringMode.ino) | Per-gate energy via `requestStartEngineeringMode` + atomic snapshots. |
| [`bluetoothControl`](../examples/bluetoothControl/bluetoothControl.ino) | LD2410C-only: Bluetooth on/off, MAC readback, password. |
| [`distanceResolution`](../examples/distanceResolution/distanceResolution.ino) | LD2410C-only: 0.75 m ↔ 0.20 m gate quantisation. |
| [`snapshotAtomic`](../examples/snapshotAtomic/snapshotAtomic.ino) | ESP32 dual-core race-safe getter pattern. |

## Other resources

- [`../README.md`](../README.md) — top-level project README.
- [`../CLAUDE.md`](../CLAUDE.md) — contribution rules (PR-only, no direct push to `main`).
- Release history: see the GitHub releases page of the upstream repo (URL in [`../library.properties`](../library.properties), key `url=`).
