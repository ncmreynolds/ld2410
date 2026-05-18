# 02 — Variants: LD2410 vs LD2410B vs LD2410C vs LD2410S

The Hi-Link "LD2410" family is **four different sensors** sharing a
similar product name and a related (but not identical) UART protocol.
Picking the wrong one — or wiring it like the wrong one — wastes a
weekend. This page summarises the practical differences.

For byte-level protocol differences see
[`ld2410-variants-comparison.md`](ld2410-variants-comparison.md). For
the per-method capability matrix see
[`method-coverage.md`](method-coverage.md).

## TL;DR — which one should I buy?

| If you want... | Pick... |
|---|---|
| Cheapest, just presence + distance, 5 V tolerant | **LD2410C** |
| Configurable Bluetooth + Android app | **LD2410B** or **LD2410C** (both expose BT) |
| Bluetooth **plus** on-board light sensor (occupancy + ambient gating) | **LD2410B** (only variant with the photodiode + 0xAD/0xAE light-sense control) |
| Original cheap version, no BT, 5 V | **LD2410** (base) — increasingly hard to find new |
| Battery-powered / 3.3 V, finer near-field, auto-tuning thresholds | **LD2410S** |

The LD2410B and LD2410C are what most resellers ship as "LD2410" today. If your
board has Bluetooth-related silkscreen or a chip antenna near one
edge, it's a B or C — the easiest way to tell them apart is the OUT-pin
position (pin **1** on B, pin **3** on C) and the presence of a small
photodiode beside the antenna patch (B only).

## Hardware quick-reference

| | **LD2410 (base)** | **LD2410B** | **LD2410C** | **LD2410S** |
|---|---|---|---|---|
| Supply voltage | 5 V | 5 V | 5 V | **3.3 V** (3.0–3.6 V) |
| Min current | > 200 mA | > 200 mA | > 200 mA | ultra-low (battery target) |
| Default UART baud | 57600 | 256000 | 256000 | 115200 |
| Max gates / range | 9 × 0.75 m = 6.75 m | 9 × 0.75 m or 9 × 0.20 m = 6.75 / 1.80 m | 9 × 0.75 m or 9 × 0.20 m = 6.75 / 1.80 m | 16 gates (width not specified by HLK V1.00; use `detectionDistance()` for absolute distance) |
| Distance resolution | fixed 0.75 m | switchable 0.75 / 0.20 m | switchable 0.75 / 0.20 m | not specified by HLK V1.00 — `detectionDistance()` reports cm directly |
| OUT pin position | pin **1** | pin **1** | pin **3** | pin **5 (J2)** |
| On-board photodiode (light sensor) | ❌ | ✅ — queried in engineering frame + can gate the OUT pin via `0xAD`/`0xAE` | ❌ | ❌ |

> ⚠️ **Wiring danger:** the base/B and C variants have **opposite** OUT-pin
> layouts on a 5-pin header (B follows the base layout — OUT on pin 1;
> C moves OUT to pin 3). A board labelled just "LD2410" can be any of
> the three; check the silkscreen and the OUT-pin position before
> connecting power, or you may feed 5 V into a 3.3 V data pin. Full
> pinout in
> [`ld2410-variants-comparison.md`](ld2410-variants-comparison.md#01-pin-pinout-side-by-side).

## Capability matrix (high level)

A green ✅ means the method is exposed on that variant. A red ❌ means
calling it produces a clean compile-time error pointing at the
`LD2410_HAS_*` flag.

| Capability | base | B | C | S | API page |
|---|:-:|:-:|:-:|:-:|---|
| Connection + sync read | ✅ | ✅ | ✅ | ✅ | [04 core](04-api-core.md) |
| FreeRTOS background task (ESP32) | ✅ | ✅ | ✅ | ✅ | [04 core](04-api-core.md) |
| Presence + distance + energy getters | ✅ | ✅ | ✅ | ✅ | [04 core](04-api-core.md) |
| Engineering mode (per-gate energy) | ✅ | ✅ | ✅ | ✅ | [04 core](04-api-core.md) |
| Atomic snapshots (basic + per-gate) | ✅ | ✅ | ✅ | ✅ | [08 advanced](08-advanced.md) |
| Set max distance + idle delay | ✅ | ✅ | ✅ | ❌ | [05 base](05-api-ld2410-base.md) |
| Set per-gate sensitivity | ✅ | ✅ | ✅ | ❌ | [05 base](05-api-ld2410-base.md) |
| Read current configuration | ✅ | ✅ | ✅ | ❌ | [05 base](05-api-ld2410-base.md) |
| Read firmware version | ✅ | ✅ | ✅ | ✅ | [05 base](05-api-ld2410-base.md) |
| Restart / factory reset | ✅ | ✅ | ✅ | ❌ | [05 base](05-api-ld2410-base.md) |
| Change baud rate | ✅ | ✅ | ✅ | ❌ | [06 C](06-api-ld2410c.md) |
| Bluetooth on/off | ❌ | ✅ | ✅ | ❌ | [06 C](06-api-ld2410c.md) |
| Read MAC address | ❌ | ✅ | ✅ | ❌ | [06 C](06-api-ld2410c.md) |
| Bluetooth password / permissions | ❌ | ✅ | ✅ | ❌ | [06 C](06-api-ld2410c.md) |
| Distance resolution 0.75 ↔ 0.20 m | ❌ | ✅ | ✅ | ❌ | [06 C](06-api-ld2410c.md) |
| Photodiode / light-sense auxiliary control (`0xAD`/`0xAE`) | ❌ | ✅ B-only (`setAuxiliaryControl`, `requestAuxiliaryControl`) | ❌ | ❌ | [method-coverage.md](method-coverage.md#ld2410b) |
| Engineering-frame light-sense + OUT pin state trailer (2 B, same slot as base/C "M reserved") | ❌ | ✅ B-only (public fields `photosensitivity_value`, `out_pin_state`) | ❌ | ❌ | [method-coverage.md](method-coverage.md#ld2410b) |
| Auto-threshold tuning | ❌ | ❌ | ❌ | ✅ | [07 S](07-api-ld2410s.md) |
| Generic-parameter set (6 fields in one call) | ❌ | ❌ | ❌ | ✅ | [07 S](07-api-ld2410s.md) |
| Per-gate trigger / hold thresholds | ❌ | ❌ | ❌ | ✅ | [07 S](07-api-ld2410s.md) |
| Read/write serial number | ❌ | ❌ | ❌ | ✅ | [07 S](07-api-ld2410s.md) |
| Switch to minimal output frame | ❌ | ❌ | ❌ | ✅ | [07 S](07-api-ld2410s.md) |

For the canonical opcode-by-opcode breakdown see
[`method-coverage.md`](method-coverage.md). That file is the
**source of truth** for the `LD2410_HAS_*` feature gates and is
updated in lockstep with the code.

## Selecting the variant at build time

The library uses four mutually-exclusive macros to pick the variant.
There are two equivalent ways to set them; both produce identical
builds.

### Recommended: include the variant entry header

| Sensor | Include |
|---|---|
| LD2410 (base) | `#include <ld2410.h>` |
| LD2410**B** | `#include <ld2410b.h>` |
| LD2410**C** | `#include <ld2410c.h>` |
| LD2410**S** | `#include <ld2410s.h>` |

```cpp
#include <ld2410c.h>
```

Each variant entry header is a 5-line shim that defines the
corresponding macro (if not already defined) and then includes
`<ld2410.h>`. This route is the only one that works **on every
build system including the Arduino IDE GUI**, which cannot pass
per-sketch `-D` build flags.

> Use the same entry header in every translation unit of your
> sketch (every `.ino`/`.cpp`). Mixing `<ld2410.h>` in one file with
> `<ld2410c.h>` in another silently produces two different `class
> ld2410` layouts → undefined behaviour.

### Alternative: define the macro before `<ld2410.h>`

Equivalent — useful for PlatformIO / arduino-cli where you can push
the macro via a build flag:

```cpp
#define LD2410_VARIANT_BASE   // original LD2410 — this is the default if nothing is defined
#define LD2410_VARIANT_B      // LD2410B
#define LD2410_VARIANT_C      // LD2410C
#define LD2410_VARIANT_S      // LD2410S
```

Define exactly one **before** `#include <ld2410.h>`. With PlatformIO
use `build_flags = -DLD2410_VARIANT_C`. With arduino-cli use
`--build-property "build.extra_flags=-DLD2410_VARIANT_C"`. The
Arduino IDE GUI has no per-sketch equivalent — use the entry-header
route above instead.

The two routes compose safely: setting `-DLD2410_VARIANT_C` and then
including `<ld2410c.h>` is a no-op (the entry header is idempotent).

### What the macro controls

The variant macro toggles a set of capability flags
(`LD2410_HAS_BLUETOOTH`, `LD2410_HAS_AUTO_THRESHOLD`, …) which gate the
public methods. Calling a method that's not in your variant produces a
linker / compiler error — no silent fall-through.

## Status of LD2410B support

✅ First-class compile-time variant since the LD2410B addition.
Selected via `#include <ld2410b.h>` or `-DLD2410_VARIANT_B`. The
shared command surface is identical to LD2410C's, plus two B-only
methods (`setAuxiliaryControl()` / `requestAuxiliaryControl()`)
behind `LD2410_HAS_AUX_CONTROL`, and two B-only public fields
(`photosensitivity_value`, `out_pin_state`) populated from the
2-byte trailer the B emits inside each engineering-mode frame at
the same wire slot the base/C parser documents as "M reserved".

> ⚠️ **UNVERIFIED on hardware.** The B-only code path was
> developed against the V1.06 protocol PDF
> ([HLK-LD2410B_protocol.md](HLK-LD2410B_protocol.md)) and reviewed
> for correctness, but the maintainer's bench has only an LD2410C
> sample. Host tests (27 B-specific cases) and the 16-cell compile
> matrix exercise the code; bench validation is pending an
> LD2410B sample. If you have one and want to help, please open an
> issue with a serial-monitor capture from
> `tests/hw/ld2410c_full_test` rebuilt with `<ld2410b.h>`.

**Backward-compat option.** A user with an LD2410B board who does
not need the two B-only methods or the trailer fields can still
build with `<ld2410c.h>` instead — every shared command works
identically because B is a wire-level superset of C. The parser
will silently absorb (without exposing) the 2 trailer bytes of
the engineering frame.

## Status of LD2410S support

⚠️ The S variant code path was developed against the V1.00 protocol PDF
and reviewed for correctness, but **has not been validated against
physical hardware** because the maintainer's bench does not currently
have an LD2410S sample. The host test suite (19 S-specific cases)
covers the byte-level parsing logic, and the 16-cell CI compile
matrix builds the S variant on every supported board
(ESP32 / ESP8266 / RP2040 / AVR128DA32), but the per-gate frame
layout assumption (64-byte block split as 16 motion + 16 stationary,
mirroring base/C convention) is informed but not measured.

If you have an LD2410S and find a discrepancy, please open an issue
with a captured frame dump.

## What about pin-compatible competitors?

The protocol is HLK-proprietary. There are visually-similar 24 GHz
mmWave radars from other Chinese vendors (Seeed MR60BHA1, DFRobot
SEN0395, …) that use entirely different protocols and are **not**
compatible with this library — even if they look the same on the
bench.

## Going deeper

- Pick the right runtime model for your platform: [`03-runtime-models.md`](03-runtime-models.md)
- Wire up your radar: [`01-getting-started.md`](01-getting-started.md)
- Per-byte protocol comparison: [`ld2410-variants-comparison.md`](ld2410-variants-comparison.md)
- Original HLK PDFs (translated from Chinese): [`HLK-LD2410_protocol.md`](HLK-LD2410_protocol.md), [`HLK-LD2410B_protocol.md`](HLK-LD2410B_protocol.md), [`HLK-LD2410C_protocol.md`](HLK-LD2410C_protocol.md), [`HLK-LD2410S_protocol.md`](HLK-LD2410S_protocol.md)
