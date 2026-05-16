# LD2410 — method coverage matrix

How the public API of `class ld2410` maps to the documented protocol
capabilities of each sensor variant (LD2410 base / LD2410C / LD2410S).
This file is the **source of truth** for the `#ifdef LD2410_HAS_*`
gates in `src/ld2410.h` and `src/ld2410.cpp`.

When you add, remove, or rename a public method, update the relevant
row of Table 1 here and the corresponding `#ifdef` gate in the header
(plus the symmetric gate in the `.cpp`).

References:
- Per-variant opcodes: [`docs/HLK-LD2410_protocol.md`](HLK-LD2410_protocol.md),
  [`docs/HLK-LD2410B_protocol.md`](HLK-LD2410B_protocol.md),
  [`docs/HLK-LD2410C_protocol.md`](HLK-LD2410C_protocol.md),
  [`docs/HLK-LD2410S_protocol.md`](HLK-LD2410S_protocol.md)
- Side-by-side byte-level differences: [`docs/ld2410-variants-comparison.md`](ld2410-variants-comparison.md)
- Variant header definitions: `src/ld2410_variants/{ld2410_base,ld2410_c,ld2410_s}.h`

Legend:
- ✅ exposed via a public method
- ❌ documented but not exposed yet
- 🟡 exposed but with the wrong opcode for this variant — needs internal dispatch
- — not part of this variant's protocol

---

## Table 1 — Commands (host → radar)

| Opcode | Section | Capability | base | B | C | S | Public method | Feature flag |
|---|---|---|---|---|---|---|---|---|
| `0xFF` | §2.2.1 base/B/C, §2.2.3 S | Enable configuration | ✅ | ✅\* | ✅ | ✅ | `enter_configuration_mode_()` (private) | `LD2410_HAS_CONFIGURATION_MODE` |
| `0xFE` | §2.2.2 base/B/C, §2.2.4 S | End configuration | ✅ | ✅\* | ✅ | ✅ | `leave_configuration_mode_()` (private) | `LD2410_HAS_CONFIGURATION_MODE` |
| `0x60` | §2.2.3 base/B/C | Set max gate + unmanned delay | ✅ | ✅\* | ✅ | — | `setMaxValues(moving, stationary, inactivityTimer)` | `LD2410_HAS_MAX_VALUES` |
| `0x61` | §2.2.4 base/B/C | Read current configuration | ✅ | ✅\* | ✅ | — | `requestCurrentConfiguration()` | `LD2410_HAS_READ_PARAMS` |
| `0x62` | §2.2.5 base/B/C | Enable engineering mode | ✅ | ✅\* | ✅ | — | `requestStartEngineeringMode()` | `LD2410_HAS_ENGINEERING_MODE` |
| `0x63` | §2.2.6 base/B/C | Close engineering mode | ✅ | ✅\* | ✅ | — | `requestEndEngineeringMode()` | `LD2410_HAS_ENGINEERING_MODE` |
| `0x64` | §2.2.7 base/B/C | Range gate sensitivity | ✅ | ✅\* | ✅ | — | `setGateSensitivityThreshold(gate, moving, stationary)` | `LD2410_HAS_GATE_SENSITIVITY` |
| `0xA0` (base/B/C) / `0x00` (S) | §2.2.8 base/B/C, §2.2.2 S | Read firmware version | ✅ | ✅\* | ✅ | ✅ | `requestFirmwareVersion()` — variant-aware send opcode (step 8) and variant-aware ACK length/offsets (step 9). On S, major/minor 16-bit values are stored as their LE low byte into the existing uint8_t fields; patch is stored full-width 16-bit in firmware_bugfix_version. | `LD2410_HAS_FIRMWARE_VERSION` |
| `0xA1` | §2.2.9 base/B/C | Set serial port baud rate | ✅ | ✅\* | ✅ | — | `setBaudRate(uint16_t baud_index)` — index from `LD2410_BAUD_INDEX_*`; takes effect after restart | `LD2410_HAS_BAUD_RATE` |
| `0xA2` | §2.2.10 base/B/C | Factory reset | ✅ | ✅\* | ✅ | — | `requestFactoryReset()` | `LD2410_HAS_FACTORY_RESET` |
| `0xA3` | §2.2.11 base/B/C | Restart module | ✅ | ✅\* | ✅ | — | `requestRestart()` (with 800 ms reboot blackout for ESP32 autoReadTask) | `LD2410_HAS_RESTART` |
| `0xA4` | §2.2.12 B/C | Bluetooth on/off | — | ✅\* | ✅ | — | `setBluetooth(bool on)` — non-volatile, takes effect after restart | `LD2410_HAS_BLUETOOTH` |
| `0xA5` | §2.2.13 B/C | Get MAC address | — | ✅\* | ✅ | — | `requestMACAddress()` → populates `mac_address[6]` (wire/big-endian order) | `LD2410_HAS_MAC_ADDRESS` |
| `0xA8` | §2.2.14 B/C | Obtain Bluetooth permissions | — | ✅\* | ✅ | — | `obtainBluetoothPermissions(uint8_t pwd[6])` — note: ACK is delivered over BLE, not UART, so on most firmwares this returns false | `LD2410_HAS_BLUETOOTH` |
| `0xA9` | §2.2.15 B/C | Set Bluetooth password | — | ✅\* | ✅ | — | `setBluetoothPassword(uint8_t pwd[6])` — 4-byte ACK on UART | `LD2410_HAS_BLUETOOTH` |
| `0xAA` | §2.2.16 B/C | Set distance resolution (0.75 / 0.2 m) | — | ✅\* | ✅ | — | `setDistanceResolution(LD2410_DISTANCE_RESOLUTION_*)` — non-volatile, takes effect after restart | `LD2410_HAS_DISTANCE_RESOLUTION` |
| `0xAB` | §2.2.17 B/C | Query distance resolution | — | ✅\* | ✅ | — | `requestDistanceResolution()` → populates `distance_resolution` (LE index) | `LD2410_HAS_DISTANCE_RESOLUTION` |
| `0xAD` | §2.2.18 B | Set auxiliary (light-sense) control: mode + threshold + OUT default level | — | ❌ | — | — | **MISSING** — proposed `setAuxiliaryControl(uint8_t mode, uint8_t threshold, uint8_t out_default_level)` | **MISSING** — proposed `LD2410_HAS_AUX_CONTROL` |
| `0xAE` | §2.2.19 B | Query auxiliary control configuration | — | ❌ | — | — | **MISSING** — proposed `requestAuxiliaryControl()` → populates `aux_control_mode`, `aux_control_threshold`, `aux_control_out_default_level` | **MISSING** — proposed `LD2410_HAS_AUX_CONTROL` |
| `0x09` | §2.2.9 S | Auto-update threshold | — | — | — | ✅ | `autoUpdateThreshold(trigger=2, retention=1, scan_s=120)` — return value is best-effort (HLK does not document an ACK); use `autoThresholdProgress()` for real progress. UNVERIFIED on hardware | `LD2410_HAS_AUTO_THRESHOLD` |
| `0x10` | §2.2.5 S | Write serial number | — | — | — | ✅ | `writeSerialNumber(uint8_t sn[8])` — UNVERIFIED on hardware | `LD2410_HAS_SERIAL_NUMBER` |
| `0x11` | §2.2.6 S | Read serial number | — | — | — | ✅ | `requestSerialNumber()` → populates `serial_number[8]` — UNVERIFIED on hardware | `LD2410_HAS_SERIAL_NUMBER` |
| `0x70` | §2.2.7 S | Write generic parameters | — | — | — | ✅ | `writeGenericParameters(farthest, nearest, delay_s, status_freq, distance_freq, speed)` — UNVERIFIED on hardware | `LD2410_HAS_GENERIC_PARAMS` |
| `0x71` | §2.2.8 S | Read generic parameters | — | — | — | ✅ | `requestGenericParameters()` → populates 6 public fields — UNVERIFIED on hardware | `LD2410_HAS_GENERIC_PARAMS` |
| `0x72` | §2.2.10 S | Write trigger threshold | — | — | — | ✅ | `writeTriggerThresholds(uint8_t[16])` — UNVERIFIED on hardware | `LD2410_HAS_TRIGGER_THRESHOLD` |
| `0x73` | §2.2.11 S | Read trigger threshold | — | — | — | ✅ | `requestTriggerThresholds()` → populates `trigger_thresholds[16]` — UNVERIFIED on hardware | `LD2410_HAS_TRIGGER_THRESHOLD` |
| `0x76` | §2.2.12 S | Write hold threshold | — | — | — | ✅ | `writeHoldThresholds(uint8_t[16])` — UNVERIFIED on hardware | `LD2410_HAS_HOLD_THRESHOLD` |
| `0x77` | §2.2.13 S | Read hold threshold | — | — | — | ✅ | `requestHoldThresholds()` → populates `hold_thresholds[16]` — UNVERIFIED on hardware | `LD2410_HAS_HOLD_THRESHOLD` |
| `0x7A` | §2.2.1 S | Switch output mode (standard / minimal) | — | — | — | ✅ | `setOutputMode(bool standard)` — UNVERIFIED on hardware | `LD2410_HAS_OUTPUT_MODE` |

\* The library does not yet expose `LD2410_VARIANT_B`. Rows marked `✅*` for the B column work today by building with `LD2410_VARIANT_C` — the B's command opcodes are a strict superset of the C's, so every shared-core method drives a B board correctly. Adding first-class B support (with the two new `0xAD`/`0xAE` methods and the engineering-frame trailer) is tracked in [Table 3 → LD2410B](#ld2410b) below.

---

## Table 2 — Data path (radar → host)

| Frame | Section | base | B | C | S | Parser status |
|---|---|---|---|---|---|---|
| Basic frame (data type `0x02`, 4-byte envelope) | §2.3.1-2 base/B/C | ✅ | ✅\* | ✅ | — | `parse_data_frame_()` decodes Tabella 12 fields |
| Engineering frame (data type `0x01`, per-gate appended, 9 gates) | §2.3.2 base/B/C | ✅ | 🟡 | ✅ | — | `parse_data_frame_()` populates `engineering_motion_energy_[9]` and `engineering_stationary_energy_[9]` from offsets `[19..27]` and `[28..36]`. On B the per-gate offsets and total frame length match base/C; **the 2 trailing bytes (photosensitivity + OUT pin state, HLK §2.3.2 Table 15) occupy the SAME wire slot that base/C document as variable "M reserved"** — the length-driven parser already absorbs them correctly, but they are silently dropped instead of exposed via public fields. |
| Standard frame (data type `0x01`, per-gate INLINE 64 B, 16 gates) | §2.1 S | — | — | — | ✅ | `parse_data_frame_()` decodes target state + per-gate energies for the S branch under `LD2410_VARIANT_S`. Per-gate uses `LD2410_GATE_COUNT == 16` from `src/ld2410_variants/ld2410_s.h`. Offsets `[12..27]` motion + `[12+GATE_COUNT..]` stationary. **UNVERIFIED on hardware** — the V1.00 PDF specifies 64 bytes total for the per-gate block but does not detail the per-gate breakdown; the assumption is the same 1-byte-per-energy convention as base/C. |
| Auto-threshold progress frame (data type `0x03`) | §2.1, §2.2.9 S | — | — | — | ✅ | `parse_data_frame_` decodes 2-byte LE progress; exposed via `autoThresholdProgress()` and `autoThresholdReceived()` |
| Minimal frame (`6E … 62`, 5 bytes total) | §2.1 S | — | — | — | ✅ | `read_frame_` recognises `0x6E` as a third header start; `parse_minimal_frame_` decodes target state + 2-byte object distance. Resync supported (a stray `0x6E` mid-stream restarts a minimal-frame attempt). |

---

## Table 3 — Per-variant missing capabilities (priority-ordered)

### LD2410 base
*All documented commands now exposed.*

→ **0 capabilities missing**.

### LD2410C
*All documented commands now exposed.*

→ **0 capabilities missing**.

<a id="ld2410b"></a>
### LD2410B

*Not yet selectable as a build variant.* The B's command set is a
strict superset of the C's (every opcode `0x60`..`0xAB` is shared,
plus the B adds `0xAD`/`0xAE`), so today a user builds with
`LD2410_VARIANT_C` and gets the shared-core methods for free. What is
still missing to make B a first-class variant:

**A. Build-system / variant plumbing (4 items)**

| Item | Where it lives today (or should) | Status |
|---|---|---|
| Variant header `src/ld2410_variants/ld2410_b.h` — should `#include "ld2410_c.h"` (or `ld2410_base.h` + the BT/MAC/distance-resolution opcodes copied from `ld2410_c.h`) and layer the `0xAD`/`0xAE` opcodes on top. Should `#undef LD2410_VARIANT_NAME` → `"LD2410B"`. Default baud and pinout already match C/base so no overrides needed. | `src/ld2410_variants/ld2410_b.h` (new file) | ❌ MISSING |
| Selection macro `LD2410_VARIANT_B` recognised in the `#if (defined(...) + defined(...) + ...) > 1` mutual-exclusion guard at `src/ld2410.h:40-45` and the `#if/#elif/#else` dispatch at `src/ld2410.h:47-53`. | `src/ld2410.h` lines ~40-53 | ❌ MISSING |
| Opcode macros `LD2410_OP_AUX_CONTROL_SET = 0xAD` and `LD2410_OP_AUX_CONTROL_GET = 0xAE`, defined inside the new `ld2410_b.h`. | `src/ld2410_variants/ld2410_b.h` | ❌ MISSING |
| Feature flag `LD2410_HAS_AUX_CONTROL`, defined inside the new `ld2410_b.h`. Used as the `#ifdef` gate around the new methods in `src/ld2410.h` / `src/ld2410.cpp`. | `src/ld2410_variants/ld2410_b.h` | ❌ MISSING |

**B. Public API additions (2 methods + 4 fields)**

All gated by `#ifdef LD2410_HAS_AUX_CONTROL`. Inserted alongside the
existing `setBluetooth` / `requestMACAddress` group in `src/ld2410.h`:

```cpp
#ifdef LD2410_HAS_AUX_CONTROL
    // 0xAD §2.2.18 (B only) — configure the on-board photodiode → OUT-pin
    // gating. `mode`: 0 off, 1 trigger when light < threshold, 2 trigger
    // when light > threshold. `out_default_level`: 0 = idle LOW / triggered
    // HIGH (factory default), 1 = inverted. Setting is non-volatile.
    bool setAuxiliaryControl(uint8_t mode,
                             uint8_t threshold = 0x80,
                             uint8_t out_default_level = 0);

    // 0xAE §2.2.19 (B only) — read back the current 4-byte aux control
    // configuration. Populates the three public fields below.
    bool requestAuxiliaryControl();

    uint8_t aux_control_mode              = 0;
    uint8_t aux_control_threshold         = 0;
    uint8_t aux_control_out_default_level = 0;
    bool    aux_control_received          = false;
#endif
```

**C. Parser extension — engineering-frame trailer (1 branch + 2 fields)**

The B's engineering frame specifies 2 fixed trailer bytes after the per-gate energy block (HLK §2.3.2 Table 15) — they occupy the SAME wire slot that base/C document as variable "M reserved", so the frame total length matches base/C:

| Byte | Field | Range |
|---|---|---|
| trailer + 0 | Photosensitivity detection value | 0 ~ 255 |
| trailer + 1 | OUT pin output state | 0 = no one, 1 = someone |

Today `parse_data_frame_()` (`src/ld2410.cpp`) ignores them because they fall in the "variable reserved tail" the parser already tolerates. For first-class support these need to be captured:

```cpp
// New public fields in src/ld2410.h (gated by LD2410_VARIANT_B,
// or by LD2410_HAS_AUX_CONTROL since both arrive together on B):
uint8_t photosensitivity_value = 0;
uint8_t out_pin_state          = 0;
```

And in `parse_data_frame_()`, the engineering branch needs a
`#if defined(LD2410_VARIANT_B)` block that reads
`radar_data_frame_[28 + LD2410_GATE_COUNT]` and `... + 1` into those
fields (i.e. immediately after the stationary energies, before the
`0x55 0x00` tail).

**D. Command parser ACK branches (in `parse_command_frame_`)**

Two new branches handling `0xAD` ACK (4-byte success/fail) and `0xAE`
ACK (4-byte status + 4-byte returned config). Both follow the existing
`report_command_result_()` epilogue pattern used by every other ACK
branch.

**E. Optional but recommended**

| Item | Rationale |
|---|---|
| New API page `docs/06b-api-ld2410b.md` — clone of `06-api-ld2410c.md` minus C-only quirks plus the aux-control + photodiode getters | Keeps the per-variant API pages parallel; user-discoverable from `docs/README.md` |
| New example sketch `examples/lightSenseControl/lightSenseControl.ino` — toggle aux control on/off, print `photosensitivity_value` and `out_pin_state` from each engineering frame | Provides a one-shot bench validation entry point |
| CI compile-matrix cell `(esp32, B)` / `(esp8266, B)` / `(rp2040, B)` added to `tests/compile_matrix.sh` and `.github/workflows/ci.yml` | Catches build regressions on the new variant in lockstep with the others |
| HW bench validation against a real LD2410B board — confirm the engineering-frame trailer offsets, the `0xAD`/`0xAE` ACK structures, and the OUT-pin behaviour under different mode bytes | Lift the eventual `UNVERIFIED ON HARDWARE` caveat |

→ **5 capabilities missing on B today** (variant plumbing, 2 new methods, engineering-frame trailer parsing, ACK branches). All are mechanical; there is no protocol ambiguity to resolve. Roadmap step 13 below tracks the work.

### LD2410S
*All documented commands now exposed; all three parser modes implemented.*

→ **0 capabilities missing**, but **all S code is UNVERIFIED on hardware**
(see banner in `src/ld2410_variants/ld2410_s.h`). Bench validation against
a real LD2410S sample is the next gating step before declaring S production-ready.

---

## Implementation roadmap (status)

| Step | Description | Status |
|---|---|---|
| 1 | `src/ld2410_frame.h` — shared envelope helpers | ✅ done (commit `0d95f1c`) |
| 2 | `src/ld2410_variants/ld2410_base.h` — base vocabulary | ✅ done (commits `9fb0c49`, `f5cbb97`) |
| 3 | `src/ld2410_variants/ld2410_c.h` — C extensions on top of base | ✅ done (commit `0f0566f`) |
| 4 | `src/ld2410_variants/ld2410_s.h` — S standalone (UNVERIFIED) | ✅ done (commits `f79816f`, `c184679`) |
| 5 | Variant dispatcher + variant-aware buffer sizing in `ld2410.h` | ✅ done (commits `cfaca07`, `b53b9f7`) |
| 5b | Debug flags made opt-in | ✅ done (commit `ccc9800`) |
| 6 | Apply `LD2410_HAS_*` gates to existing `class ld2410` methods + .cpp definitions + parse_command_frame_ branches | ✅ done (commit `95e22dd`) |
| 7 | `CommandTransaction` lock for concurrency safety across `request*/set*` calls | ✅ done (commit `f7f9089`) |
| 8 | Refactor `request*/set*` to use `LD2410_OP_*` macros + use `frame.h` helpers in send_command_preamble_/postamble_ | ✅ done (commit `95605bb`) |
| 8b | Use `LD2410_PARAM_*` for parameter words in setMaxValues / setGateSensitivityThreshold (+ ld2410_write_le16/le32 helpers) | ✅ done (commit `e12610d`) |
| **9** | **Refactor `parse_command_frame_` ACK branches with macros + variant-aware FW version handling** | ✅ done (this commit) |
| 10-min | Refactor `parse_data_frame_` + `check_frame_end_` + `read_frame_` to use frame.h constants; engineering arrays sized via `LD2410_GATE_COUNT`; S `data type 0x01` (standard) decode | ✅ done (this commit) |
| 10b | Add S auto-threshold-progress (`data type 0x03`) parsing + `autoThresholdProgress()`/`autoThresholdReceived()` accessors | ✅ done (this commit) |
| 10c | Add S minimal frame (`6E … 62`) parsing — `read_frame_` recognises 3 header types; new `parse_minimal_frame_` private method | ✅ done (this commit) |
| 11a | Add `setBaudRate()` for base/C (regression fix, upstream issue #39) | ✅ done (this commit) |
| 11b | Add `setBluetooth/requestMACAddress/setDistanceResolution/requestDistanceResolution` for C (regression fixes vs v0.1.3) | ✅ done (this commit) |
| 11c | Add `obtainBluetoothPermissions/setBluetoothPassword` for C (never exposed before) | ✅ done (this commit) |
| 11d.1 | S — `setOutputMode` (0x7A) | ✅ done (this commit) |
| 11d.2 | S — `writeGenericParameters` / `requestGenericParameters` (0x70/0x71) | ✅ done (this commit) |
| 11d.3 | S — `write/requestTriggerThresholds` (0x72/0x73) + `write/requestHoldThresholds` (0x76/0x77) | ✅ done (this commit) |
| 11d.4 | S — `autoUpdateThreshold` (0x09) | ✅ done (this commit) |
| 11d.5 | S — `write/requestSerialNumber` (0x10/0x11) | ✅ done |
| 12 | End-to-end verification: dual-binary host suite + 3-variant × 3-board arduino-cli matrix, both wired into CI | ✅ done (this commit) |

### Step 12 details

The verification was previously done by hand after each step. Step 12
makes it permanent and reproducible:

- **`tests/run.sh`** — host parser test suite, compiled THREE ways
  (default base / `-DLD2410_VARIANT_C` / `-DLD2410_VARIANT_S`).
  57 tests total across the three binaries — count includes the
  `test_snapshot_target_state` checks added with the atomic-snapshot
  API.
- **`tests/compile_matrix.sh`** — orchestrates `arduino-cli compile`
  across the cross-product of (esp32, esp8266, rp2040) × (default, C, S).
  Reports a per-cell pass/fail line and exits non-zero on any failure.
  9 cells; rp2040 uses the now-extended `examples/basicSensor.ino`
  (a `defined(ARDUINO_ARCH_RP2040)` branch was added in this step).
- **`.github/workflows/ci.yml`** — GitHub Actions workflow that runs
  on every push to `main`/`feat/*`/`fix/*`/`refactor/*` and on every
  PR targeting `main`. Layout:
  - `host-tests` job — runs `bash tests/run.sh` standalone (no board
    cores installed). Independent canary for pure-C++ regressions.
  - `compile-cell` matrix job — `strategy.fail-fast: false` over
    boards × variants = 9 cells. Each cell installs ONLY its own
    board core, so a transient 502 from a single core mirror only
    fails the affected cell(s) and the rest of the matrix still
    runs. Failed cells can be re-triggered in isolation from the
    GitHub Actions UI without rerunning the entire workflow.
  The 9 cells reproduce the same logic as `tests/compile_matrix.sh`
  used locally; the script remains the single-shot way to run the
  matrix without GitHub.

### Step 13 — LD2410B first-class variant (planned)

| Sub-step | Description | Status |
|---|---|---|
| 13a | New `src/ld2410_variants/ld2410_b.h` — `#include "ld2410_c.h"`, override `LD2410_VARIANT_NAME`, add `LD2410_OP_AUX_CONTROL_SET` (`0xAD`) + `LD2410_OP_AUX_CONTROL_GET` (`0xAE`), define `LD2410_HAS_AUX_CONTROL` | ❌ not started |
| 13b | `src/ld2410.h` — add `LD2410_VARIANT_B` to the mutual-exclusion check + dispatch include | ❌ not started |
| 13c | `src/ld2410.h` / `src/ld2410.cpp` — add `setAuxiliaryControl()` + `requestAuxiliaryControl()` + 4 public fields (gated by `LD2410_HAS_AUX_CONTROL`) | ❌ not started |
| 13d | `src/ld2410.cpp` — `parse_command_frame_` branches for `0xAD` and `0xAE` ACKs | ❌ not started |
| 13e | `src/ld2410.cpp` — `parse_data_frame_` engineering-mode branch: capture the 2 trailing bytes (`photosensitivity_value`, `out_pin_state`) into new public fields when `LD2410_VARIANT_B` is set. Note: same wire slot the base/C parser already absorbs as "M reserved" — change is read-and-expose, not new offset arithmetic. | ❌ not started |
| 13f | New API page `docs/06b-api-ld2410b.md`; cross-link from `docs/README.md` and `docs/02-variants.md` | ❌ not started |
| 13g | New example `examples/lightSenseControl/lightSenseControl.ino` | ❌ not started |
| 13h | Extend `tests/run.sh` (4th host build with `-DLD2410_VARIANT_B`) and `tests/compile_matrix.sh` + `.github/workflows/ci.yml` (3 new compile cells: esp32/esp8266/rp2040 × B) | ❌ not started |
| 13i | HW bench validation against a real LD2410B sample — confirm engineering-frame trailer offsets and `0xAD`/`0xAE` ACK structures | ❌ not started (no LD2410B on bench yet) |
