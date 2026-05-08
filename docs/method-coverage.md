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

| Opcode | Section | Capability | base | C | S | Public method | Feature flag |
|---|---|---|---|---|---|---|---|
| `0xFF` | §2.2.1 base/C, §2.2.3 S | Enable configuration | ✅ | ✅ | ✅ | `enter_configuration_mode_()` (private) | `LD2410_HAS_CONFIGURATION_MODE` |
| `0xFE` | §2.2.2 base/C, §2.2.4 S | End configuration | ✅ | ✅ | ✅ | `leave_configuration_mode_()` (private) | `LD2410_HAS_CONFIGURATION_MODE` |
| `0x60` | §2.2.3 base/C | Set max gate + unmanned delay | ✅ | ✅ | — | `setMaxValues(moving, stationary, inactivityTimer)` | `LD2410_HAS_MAX_VALUES` |
| `0x61` | §2.2.4 base/C | Read current configuration | ✅ | ✅ | — | `requestCurrentConfiguration()` | `LD2410_HAS_READ_PARAMS` |
| `0x62` | §2.2.5 base/C | Enable engineering mode | ✅ | ✅ | — | `requestStartEngineeringMode()` | `LD2410_HAS_ENGINEERING_MODE` |
| `0x63` | §2.2.6 base/C | Close engineering mode | ✅ | ✅ | — | `requestEndEngineeringMode()` | `LD2410_HAS_ENGINEERING_MODE` |
| `0x64` | §2.2.7 base/C | Range gate sensitivity | ✅ | ✅ | — | `setGateSensitivityThreshold(gate, moving, stationary)` | `LD2410_HAS_GATE_SENSITIVITY` |
| `0xA0` (base/C) / `0x00` (S) | §2.2.8 base/C, §2.2.2 S | Read firmware version | ✅ | ✅ | ✅ | `requestFirmwareVersion()` — variant-aware send opcode (step 8) and variant-aware ACK length/offsets (step 9). On S, major/minor 16-bit values are stored as their LE low byte into the existing uint8_t fields; patch is stored full-width 16-bit in firmware_bugfix_version. | `LD2410_HAS_FIRMWARE_VERSION` |
| `0xA1` | §2.2.9 base/C | Set serial port baud rate | ✅ | ✅ | — | `setBaudRate(uint16_t baud_index)` — index from `LD2410_BAUD_INDEX_*`; takes effect after restart | `LD2410_HAS_BAUD_RATE` |
| `0xA2` | §2.2.10 base/C | Factory reset | ✅ | ✅ | — | `requestFactoryReset()` | `LD2410_HAS_FACTORY_RESET` |
| `0xA3` | §2.2.11 base/C | Restart module | ✅ | ✅ | — | `requestRestart()` (with 800 ms reboot blackout for ESP32 autoReadTask) | `LD2410_HAS_RESTART` |
| `0xA4` | §2.2.12 C | Bluetooth on/off | — | ❌ | — | *missing* (regression vs v0.1.3) | `LD2410_HAS_BLUETOOTH` |
| `0xA5` | §2.2.13 C | Get MAC address | — | ❌ | — | *missing* (regression vs v0.1.3) | `LD2410_HAS_MAC_ADDRESS` |
| `0xA8` | §2.2.14 C | Obtain Bluetooth permissions | — | ❌ | — | *missing* (never exposed) | `LD2410_HAS_BLUETOOTH` |
| `0xA9` | §2.2.15 C | Set Bluetooth password | — | ❌ | — | *missing* (never exposed) | `LD2410_HAS_BLUETOOTH` |
| `0xAA` | §2.2.16 C | Set distance resolution (0.75 / 0.2 m) | — | ❌ | — | *missing* (regression vs v0.1.3) | `LD2410_HAS_DISTANCE_RESOLUTION` |
| `0xAB` | §2.2.17 C | Query distance resolution | — | ❌ | — | *missing* (regression vs v0.1.3) | `LD2410_HAS_DISTANCE_RESOLUTION` |
| `0x09` | §2.2.9 S | Auto-update threshold | — | — | ❌ | *missing* | `LD2410_HAS_AUTO_THRESHOLD` |
| `0x10` | §2.2.5 S | Write serial number | — | — | ❌ | *missing* | `LD2410_HAS_SERIAL_NUMBER` |
| `0x11` | §2.2.6 S | Read serial number | — | — | ❌ | *missing* | `LD2410_HAS_SERIAL_NUMBER` |
| `0x70` | §2.2.7 S | Write generic parameters | — | — | ❌ | *missing* | `LD2410_HAS_GENERIC_PARAMS` |
| `0x71` | §2.2.8 S | Read generic parameters | — | — | ❌ | *missing* | `LD2410_HAS_GENERIC_PARAMS` |
| `0x72` | §2.2.10 S | Write trigger threshold | — | — | ❌ | *missing* | `LD2410_HAS_TRIGGER_THRESHOLD` |
| `0x73` | §2.2.11 S | Read trigger threshold | — | — | ❌ | *missing* | `LD2410_HAS_TRIGGER_THRESHOLD` |
| `0x76` | §2.2.12 S | Write hold threshold | — | — | ❌ | *missing* | `LD2410_HAS_HOLD_THRESHOLD` |
| `0x77` | §2.2.13 S | Read hold threshold | — | — | ❌ | *missing* | `LD2410_HAS_HOLD_THRESHOLD` |
| `0x7A` | §2.2.1 S | Switch output mode (standard / minimal) | — | — | ❌ | *missing* | `LD2410_HAS_OUTPUT_MODE` |

---

## Table 2 — Data path (radar → host)

| Frame | Section | base | C | S | Parser status |
|---|---|---|---|---|---|
| Basic frame (data type `0x02`, 4-byte envelope) | §2.3.1-2 base/C | ✅ | ✅ | — | `parse_data_frame_()` decodes Tabella 12 fields |
| Engineering frame (data type `0x01`, per-gate appended, 9 gates) | §2.3.2 base/C | ✅ | ✅ | — | `parse_data_frame_()` populates `engineering_motion_energy_[9]` and `engineering_stationary_energy_[9]` from offsets `[19..27]` and `[28..36]` |
| Standard frame (data type `0x01`, per-gate INLINE 64 B, 16 gates) | §2.1 S | — | — | ❌ | parser loop is `gate < 9`, doesn't read 16 gates; offsets are wrong — see roadmap §6.10 |
| Auto-threshold progress frame (data type `0x03`) | §2.1, §2.2.9 S | — | — | ✅ | `parse_data_frame_` decodes 2-byte LE progress; exposed via `autoThresholdProgress()` and `autoThresholdReceived()` |
| Minimal frame (`6E … 62`, 5 bytes total) | §2.1 S | — | — | ✅ | `read_frame_` recognises `0x6E` as a third header start; `parse_minimal_frame_` decodes target state + 2-byte object distance. Resync supported (a stray `0x6E` mid-stream restarts a minimal-frame attempt). |

---

## Table 3 — Per-variant missing capabilities (priority-ordered)

### LD2410 base
*All documented commands now exposed.*

→ **0 capabilities missing**.

### LD2410C
| Missing | Opcode | Severity |
|---|---|---|
| `setBluetooth()` | `0xA4` | regression vs v0.1.3 |
| `getMACAddress()` | `0xA5` | regression vs v0.1.3 |
| `obtainBluetoothPermissions()` | `0xA8` | never exposed |
| `setBluetoothPassword()` | `0xA9` | never exposed |
| `setDistanceResolution()` | `0xAA` | regression vs v0.1.3 |
| `getDistanceResolution()` | `0xAB` | regression vs v0.1.3 |

→ **6 capabilities missing** (4 are regressions vs v0.1.3).

### LD2410S
Almost everything is missing — only enter/leave configuration currently
work because they share opcodes with base/C. `requestFirmwareVersion()`
is exposed but uses the wrong opcode (0xA0 instead of 0x00).

| Missing | Opcode / scope | Severity |
|---|---|---|
| Fix `requestFirmwareVersion()` to dispatch 0xA0 (base/C) vs 0x00 (S) | command opcode | blocking |
| `setOutputMode(standard / minimal)` | `0x7A` | never exposed |
| `writeGenericParams()` (S equivalent of setMaxValues) | `0x70` | never exposed |
| `readGenericParams()` (S equivalent of requestCurrentConfiguration) | `0x71` | never exposed |
| `writeTriggerThreshold()` (S equivalent of setGateSensitivityThreshold, motion half) | `0x72` | never exposed |
| `readTriggerThreshold()` | `0x73` | never exposed |
| `writeHoldThreshold()` (S equivalent of setGateSensitivityThreshold, hold half) | `0x76` | never exposed |
| `readHoldThreshold()` | `0x77` | never exposed |
| `autoUpdateThreshold()` | `0x09` | never exposed |
| `writeSerialNumber()` | `0x10` | never exposed |
| `readSerialNumber()` | `0x11` | never exposed |
| Standard frame parser (16 gates inline) | data type `0x01` (S meaning) | blocking |
| Auto-threshold progress parser | data type `0x03` | blocking for `0x09` |
| Minimal frame parser (`6E … 62`) | — | blocking for `0x7A` minimal mode |

→ **11 commands + 3 parser modes missing**.

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
| 11b | Add `setBluetooth/getMACAddress/setDistanceResolution/getDistanceResolution` for C (regression fixes) | pending |
| 11c | Add `setBluetoothPassword/obtainBluetoothPermissions` for C | pending (low priority) |
| 11d | Add 11 S commands (`setOutputMode`, `write/readTriggerThreshold`, `write/readHoldThreshold`, `autoUpdateThreshold`, `write/readSerialNumber`, `write/readGenericParams`) | pending — blocking for S support |
| 12 | End-to-end verification: tests + arduino-cli compile across all 3 variants × 3 boards | continuous |
