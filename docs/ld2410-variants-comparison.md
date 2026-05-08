# LD2410 / LD2410C / LD2410S — Byte-level protocol comparison

A side-by-side mapping of every byte sequence in the three HLK radar variants. Built directly from:

- `HLK-LD2410_protocol.md` — base, V1.02 / V1.03 (2022-12 / 2023-03)
- `HLK-LD2410C_protocol.md` — V1.00 (2022-11-07)
- `HLK-LD2410S_protocol.md` — V1.00 (2024-08-23)

Legend: ✅ supported · ❌ not supported · 🟡 same opcode but different semantics or value range · ⚠️ structurally different.

---

## 0. Hardware & UART defaults

|                         | **LD2410 (base)**       | **LD2410C**             | **LD2410S**                         |
|-------------------------|-------------------------|-------------------------|-------------------------------------|
| Supply voltage          | 5 V                     | 5 V                     | **3.3 V** (3.0 ~ 3.6 V)             |
| Min current             | > 200 mA                | > 200 mA                | ultra-low power (battery target)    |
| Pin count (data header) | 5 (J0)                  | 5                       | 5 (J2) + 4 (J1 = SWD debug)         |
| **Default baud**        | **57600** (V1.03)       | **256000**              | **115200**                          |
| Frame format            | 8N1                     | 8N1                     | 8N1                                 |
| OUT pin position        | **pin 1**               | **pin 3**               | **pin 5 (J2 — OT2)**                |
| OUT pin level           | 3.3 V                   | 3.3 V                   | 0 ~ 3.3 V                           |
| Auxiliary OUT pins      | —                       | —                       | OT1 = UART_Tx, OT2 = presence flag  |

### 0.1 Pin pinout side-by-side

| Pin | **LD2410 (base)** | **LD2410C**   | **LD2410S (J2)**         |
|-----|-------------------|---------------|--------------------------|
| 1   | OUT (status)      | UART_Tx       | 3V3 (power)              |
| 2   | UART_Tx           | UART_Rx       | GND                      |
| 3   | UART_Rx           | OUT (status)  | OT1 (UART_Tx)            |
| 4   | GND               | GND           | RX (UART_Rx)             |
| 5   | VCC (5 V)         | VCC (5 V)     | OT2 (presence high/low)  |

> **Wiring danger:** the base and C variants use **opposite** OUT-pin layouts (1 vs 3) on a 5-pin header. A board labelled "LD2410" can be either: check before connecting, or you'll feed 5 V into a 3.3 V data pin.

---

## 1. Command-frame envelope (constant across all three variants)

```text
┌────────────────────┬──────────┬───────────────────────────┬───────────────────┐
│ Frame header       │ Length   │ Intra-frame data          │ Frame end (MFR)   │
│ FD FC FB FA        │ 2 bytes  │ command word + value      │ 04 03 02 01       │
└────────────────────┴──────────┴───────────────────────────┴───────────────────┘
```

ACK reply: same envelope, with the command word in the intra-frame data set to `<sent> | 0x0100` (so `0x00A0` send → `0x01A0` ACK, transmitted little-endian as `A0 01`).

This is the **only** part of the protocol shared identically by all three chips.

---

## 2. Data-frame envelope (sensor → host)

|                                 | **LD2410 (base)** | **LD2410C**   | **LD2410S**                                |
|---------------------------------|-------------------|---------------|--------------------------------------------|
| Header (standard / engineering) | `F4 F3 F2 F1`     | `F4 F3 F2 F1` | `F4 F3 F2 F1`                              |
| Trailer (standard / engineering)| `F8 F7 F6 F5`     | `F8 F7 F6 F5` | `F8 F7 F6 F5`                              |
| **"Minimal" frame**             | ❌                | ❌            | ✅ `6E … 62` (head/end, single-byte each)  |
| Intra-frame head byte           | `0xAA`            | `0xAA`        | (none — minimal) / `0xAA` for standard     |
| Intra-frame tail byte           | `0x55 0x00`       | `0x55 0x00`   | (none for minimal) / `0x55 0x00` for std   |

### 2.1 Data-type byte (right after the length field)

| Value | **LD2410 (base)**   | **LD2410C**         | **LD2410S**                                          |
|-------|---------------------|---------------------|------------------------------------------------------|
| `0x01`| Engineering mode    | Engineering mode    | **Standard data** (always includes per-gate energies)|
| `0x02`| Basic target data   | Basic target data   | ❌                                                   |
| `0x03`| ❌                  | ❌                  | **Auto-threshold progress** (only while running)     |

> The S variant **inverts the meaning of `0x01`**: on base/C it means "engineering frame appended"; on S it means "standard standard" (which on S already carries the per-gate block — there is no separate engineering mode). A parser written for base/C will mis-decode S frames if reused as-is.

### 2.2 Standard / basic target frame layout

Field width per byte, in transmission order, after the data-type byte and intra-frame head `0xAA`:

| Field                          | **LD2410 (base)** | **LD2410C** | **LD2410S** (standard mode) |
|--------------------------------|-------------------|-------------|-----------------------------|
| Target state                   | 1 B               | 1 B         | 1 B (different code map)    |
| Movement distance              | 2 B (cm)          | 2 B (cm)    | —                           |
| Movement energy                | 1 B               | 1 B         | —                           |
| Stationary distance            | **1 B** (cm) ⚠️   | **2 B** (cm)| —                           |
| Stationary energy              | 1 B               | 1 B         | —                           |
| Detection distance             | **1 B** (cm) ⚠️   | **2 B** (cm)| 2 B (cm) — "object distance"|
| Reserved bits                  | —                 | —           | 2 B                         |
| Per-gate energies block        | (only in eng. frame) | (only in eng. frame) | **64 B inline** |
| Tail                           | `55 00`           | `55 00`     | `55 00`                     |

> The base PDF marks stationary distance and detection distance as 1 B each, while the C PDF marks them as 2 B. In practice, **most real-world LD2410 / LD2410B firmwares emit 2 B fields** — the base PDF is widely considered to under-document the field width. The library assumes 2 B.

### 2.3 Target-state code map

| Code    | **base / C**                          | **S**                                |
|---------|---------------------------------------|--------------------------------------|
| `0x00`  | No target                             | "no one" (low nibble 0)              |
| `0x01`  | Moving target                         | "no one" (low nibble 1)              |
| `0x02`  | Stationary target                     | "someone present"                    |
| `0x03`  | Moving + stationary                   | "someone present"                    |

> S collapses the four-state map into "no one (0/1) vs present (2/3)". A boolean-only consumer can read either family, but anything that distinguishes moving from stationary needs different handling on S.

### 2.4 Engineering / per-gate energy block

|                                | **LD2410 (base)**       | **LD2410C**             | **LD2410S**                       |
|--------------------------------|-------------------------|-------------------------|-----------------------------------|
| Activation                     | command `0x0062`        | command `0x0062`        | always-on in standard mode        |
| Number of gates exposed        | configured `N+1` (≤ 9)  | configured `N+1` (≤ 9)  | fixed **16 gates** (0 ~ 15)       |
| Bytes per gate (motion + static)| 2 B/gate (1+1)         | 2 B/gate (1+1)          | 2 B/gate (1+1) — 32 B total       |
| "Reserved / extra info" tail   | `M` bytes (variable)    | `M` bytes (variable)    | included in 64 B inline block     |
| Block length                   | dynamic (≈ 18 ~ 22 B)   | dynamic (≈ 18 ~ 22 B)   | fixed **64 B**                    |

---

## 3. Command opcode matrix

Opcodes are 16-bit little-endian. The "Word" column shows the byte sequence as transmitted (LE).

| Word    | Command name                            | **base**                 | **C**                    | **S**                                                  |
|---------|-----------------------------------------|--------------------------|--------------------------|--------------------------------------------------------|
| `00 00` | Read firmware version                   | ❌                       | ❌                       | ✅ §2.2.2                                              |
| `09 00` | Auto-update threshold                   | ❌                       | ❌                       | ✅ §2.2.9 (trigger/retention/scan-time)                |
| `10 00` | Write serial number                     | ❌                       | ❌                       | ✅ §2.2.5 (8-byte SN)                                  |
| `11 00` | Read serial number                      | ❌                       | ❌                       | ✅ §2.2.6                                              |
| `60 00` | Set max gate + unmanned duration        | ✅ §2.2.3                | ✅ §2.2.3                | ❌ (replaced by `0x0070`)                              |
| `61 00` | Read parameters                         | ✅ §2.2.4                | ✅ §2.2.4                | ❌ (replaced by `0x0071`)                              |
| `62 00` | Enable engineering mode                 | ✅ §2.2.5                | ✅ §2.2.5                | ❌ (no separate mode)                                  |
| `63 00` | Close engineering mode                  | ✅ §2.2.6                | ✅ §2.2.6                | ❌                                                     |
| `64 00` | Range-gate sensitivity                  | ✅ §2.2.7 (single sens.) | ✅ §2.2.7 (single sens.) | ❌ (split into `0x0072` + `0x0076`)                    |
| `70 00` | Write generic parameters                | ❌                       | ❌                       | ✅ §2.2.7 (6 parameter words, see Table 2-2)           |
| `71 00` | Read generic parameters                 | ❌                       | ❌                       | ✅ §2.2.8                                              |
| `72 00` | Write trigger threshold (per gate)      | ❌                       | ❌                       | ✅ §2.2.10                                             |
| `73 00` | Read trigger threshold                  | ❌                       | ❌                       | ✅ §2.2.11                                             |
| `76 00` | Write hold threshold (per gate)         | ❌                       | ❌                       | ✅ §2.2.12                                             |
| `77 00` | Read hold threshold                     | ❌                       | ❌                       | ✅ §2.2.13                                             |
| `7A 00` | Switch output mode (minimal / standard) | ❌                       | ❌                       | ✅ §2.2.1                                              |
| `A0 00` | Read firmware version                   | ✅ §2.2.8                | ✅ §2.2.8                | ❌ (S uses `0x0000`)                                   |
| `A1 00` | Set serial port baud rate               | ✅ §2.2.9 (8 indices)    | ✅ §2.2.9 (8 indices)    | ❌ (115200 fixed)                                      |
| `A2 00` | Factory reset                           | ✅ §2.2.10               | ✅ §2.2.10               | ❌                                                     |
| `A3 00` | Restart module                          | ✅ §2.2.11               | ✅ §2.2.11               | ❌                                                     |
| `A4 00` | Bluetooth on/off                        | ❌                       | ✅ §2.2.12               | ❌                                                     |
| `A5 00` | Get MAC address                         | ❌                       | ✅ §2.2.13               | ❌                                                     |
| `A8 00` | Obtain Bluetooth permissions            | ❌                       | ✅ §2.2.14               | ❌                                                     |
| `A9 00` | Set Bluetooth password                  | ❌                       | ✅ §2.2.15               | ❌                                                     |
| `AA 00` | Distance resolution setting             | ❌                       | ✅ §2.2.16 (0.75 / 0.2 m)| ❌                                                     |
| `AB 00` | Query distance resolution               | ❌                       | ✅ §2.2.17               | ❌                                                     |
| `FE 00` | End configuration                       | ✅                       | ✅                       | ✅                                                     |
| `FF 00` | Enable configuration                    | ✅                       | ✅                       | ✅                                                     |

### 3.1 Visual partition

```text
opcodes shared by all three:    FE 00, FF 00                         (enter / leave config)
opcodes shared by base + C:     60 00, 61 00, 62 00, 63 00, 64 00,
                                A0 00, A1 00, A2 00, A3 00
opcodes only on C:              A4 00, A5 00, A8 00, A9 00,
                                AA 00, AB 00                         (Bluetooth + distance resolution)
opcodes only on S:              00 00, 09 00, 10 00, 11 00, 70 00,
                                71 00, 72 00, 73 00, 76 00, 77 00,
                                7A 00                                (read-FW reuses 0x0000;
                                                                      param model is gate0..15
                                                                      with split trigger/hold)
```

---

## 4. Byte-by-byte send/ACK for the shared core (FF, FE, 62, 63, A3)

### 4.1 Enable configuration (`0xFF`)

| Variant | Send                                                 | ACK (success)                                                                         |
|---------|------------------------------------------------------|---------------------------------------------------------------------------------------|
| base    | `FD FC FB FA  04 00  FF 00  01 00  04 03 02 01`      | `FD FC FB FA  08 00  FF 01  00 00  01 00  40 00  04 03 02 01`                         |
| C       | `FD FC FB FA  04 00  FF 00  01 00  04 03 02 01`      | `FD FC FB FA  08 00  FF 01  00 00  01 00  40 00  04 03 02 01`                         |
| S       | `FD FC FB FA  04 00  FF 00  01 00  04 03 02 01`      | `FD FC FB FA  08 00  FF 01  00 00  03 00  80 00  04 03 02 01`  ← protocol = 3, buf 128 |

**Differences:** payload bytes are identical, but the S variant returns a **different protocol version** (`0x0003`) and **different buffer size** (`0x0080` = 128) inside the ACK. Parsers that only check the trailing magic + the success byte work everywhere; parsers that hardcode protocol/buffer values to `01 00 / 40 00` (as on base + C) will mis-flag the S as "wrong protocol".

### 4.2 End configuration (`0xFE`) — fully identical on all three

`FD FC FB FA  02 00  FE 00  04 03 02 01` → `FD FC FB FA  04 00  FE 01  00 00  04 03 02 01`

### 4.3 Engineering mode toggles (`0x62` / `0x63`)

| Variant | `enableEngineering` | `closeEngineering` |
|---------|---------------------|--------------------|
| base    | `…  02 00  62 00  …` → `…  04 00  62 01  00 00  …` | `…  02 00  63 00  …` → `…  04 00  63 01  00 00  …` |
| C       | identical to base   | identical to base  |
| S       | ❌ (`0x62` / `0x63` not part of the S protocol — the engineering data is folded into the always-on "standard" frame; toggle the format with `0x7A` instead) |

### 4.4 Restart (`0xA3`)

| Variant | Send                                       | ACK (success)                                       |
|---------|--------------------------------------------|-----------------------------------------------------|
| base    | `FD FC FB FA  02 00  A3 00  04 03 02 01`   | `FD FC FB FA  04 00  A3 01  00 00  04 03 02 01`     |
| C       | identical                                  | identical                                           |
| S       | ❌                                         |                                                     |

---

## 5. Variant-specific commands (no equivalent on the others)

### 5.1 LD2410C-only — Bluetooth, MAC, distance resolution

| Function                 | Word    | Send (key bytes)                                          | ACK |
|--------------------------|---------|-----------------------------------------------------------|-----|
| Bluetooth on             | `0xA4`  | `A4 00  01 00`                                            | `A4 01  00 00` |
| Bluetooth off            | `0xA4`  | `A4 00  00 00`                                            | `A4 01  00 00` |
| Get MAC                  | `0xA5`  | `A5 00  01 00`                                            | `A5 01  00 00  00  <3 B MAC, big-endian>` |
| BT permissions           | `0xA8`  | `A8 00  …`                                                | `A8 01  00 00`                            |
| Set BT password          | `0xA9`  | `A9 00  <6 B password>`                                   | `A9 01  00 00`                            |
| Distance resolution set  | `0xAA`  | `AA 00  00 00` (0.75 m) / `AA 00  01 00` (0.2 m)          | `AA 01  00 00`                            |
| Query distance resolution| `0xAB`  | `AB 00`                                                   | `AB 01  00 00  00 00` (0.75) / `01 00`    |

### 5.2 LD2410S-only — output mode + parameter model + thresholds + auto-threshold + SN

| Function                    | Word    | Notable structure                                                                                    |
|-----------------------------|---------|------------------------------------------------------------------------------------------------------|
| Switch output mode          | `0x7A`  | `7A 00  00 00 01 00 00 00` → standard, `… 00 00 00 00 00 00` → minimal                              |
| Read firmware (S-only opcode)| `0x00` | `00 00` → ACK has 2 B major / 2 B minor / 2 B patch                                                  |
| Write SN                    | `0x10`  | `10 00  <2 B length>  <8 B ASCII serial>`                                                            |
| Read SN                     | `0x11`  | `11 00` → ACK includes 2 B length + 8 B SN                                                           |
| Write generic parameters    | `0x70`  | (`<2 B word>  <4 B value>`) × N — words: `05` farthest, `0A` nearest, `06` no-one delay, `02` status freq, `0C` distance freq, `0B` response speed |
| Read generic parameters     | `0x71`  | (`<2 B word>`) × N → ACK: (`<4 B value>`) × N                                                       |
| Write trigger threshold     | `0x72`  | (`<2 B gate idx 00..0F>  <4 B threshold>`) × 16                                                     |
| Read trigger threshold      | `0x73`  | (`<2 B gate idx>`) × 16 → ACK: (`<4 B threshold>`) × 16                                             |
| Write hold threshold        | `0x76`  | identical structure to `0x72`                                                                       |
| Read hold threshold         | `0x77`  | identical structure to `0x73`                                                                       |
| Auto-update threshold       | `0x09`  | `09 00  <2 B trigger factor>  <2 B retention factor>  <2 B scanning time>` → progress reported via `0x03` data frame |

---

## 6. Parameter model — what the radar exposes

| Concept                      | **LD2410 (base)**     | **LD2410C**           | **LD2410S**                          |
|------------------------------|-----------------------|-----------------------|--------------------------------------|
| Number of distance gates     | 9 (0 ~ 8)             | 9 (0 ~ 8) at 0.75 m **or** 16 at 0.2 m via `0xAA` | **16** (0 ~ 15) fixed             |
| Per-gate metric              | sensitivity (0 ~ 100) | sensitivity (0 ~ 100) | **trigger** + **hold** thresholds (separate)  |
| All-gates broadcast value    | gate-id `0xFFFF`      | gate-id `0xFFFF`      | one entry per gate (always 16 entries)        |
| Max moving / static gate     | independent (`0x0000` / `0x0001`)  | independent | "farthest gate" + "nearest gate" (`05` / `0A`) |
| No-one duration              | 0 ~ 65535 s (`0x0002`)| 0 ~ 65535 s           | 10 ~ 120 s (`06`)                    |
| Status reporting frequency   | not exposed           | not exposed           | 0.5 ~ 8 Hz, step 0.5 (`02`)          |
| Distance reporting frequency | not exposed           | not exposed           | 0.5 ~ 8 Hz, step 0.5 (`0C`)          |
| Response speed               | not exposed           | not exposed           | normal (5) / fast (10) (`0B`)        |
| Distance resolution          | fixed 0.75 m / gate   | switchable via `0xAA` (0.75 / 0.2 m) | implicit per gate width        |
| Bluetooth                    | ❌                    | ✅ (`0xA4`/`A8`/`A9`) | ❌                                   |
| MAC address                  | ❌                    | ✅ (`0xA5`)            | ❌                                   |
| Serial number                | ❌                    | ❌                    | ✅ (`0x10` / `0x11`)                 |
| Auto-threshold tuning        | ❌                    | ❌                    | ✅ (`0x09`)                          |

---

## 7. Factory defaults

| Item                              | **base**    | **C**       | **S** (firmware)                      |
|-----------------------------------|-------------|-------------|---------------------------------------|
| Max moving gate                   | 8           | 8           | depends on configured "farthest gate" |
| Max static gate                   | 8           | 8           | n/a                                   |
| No-one duration                   | 5 s         | 5 s         | n/a (delay configurable, range 10 ~ 120 s) |
| Baud rate                         | 57600       | 256000      | 115200                                |
| Output mode                       | basic       | basic       | minimal (`6E … 62`)                   |
| Distance resolution               | 0.75 m      | 0.75 m      | gate-implicit                         |
| Bluetooth                         | n/a         | on          | n/a                                   |
| Motion sens. gate 0               | 50          | 50          | (replaced by trigger threshold model) |
| Motion sens. gate 1               | 50          | 50          | "                                     |
| Motion sens. gate 2               | 40          | 40          | "                                     |
| Motion sens. gate 3               | 30          | 30          | "                                     |
| Motion sens. gate 4               | 20          | 20          | "                                     |
| Motion sens. gates 5 ~ 8          | 15          | 15          | "                                     |
| Static sens. gates 0 ~ 1          | not settable| not settable| "                                     |
| Static sens. gate 2 ~ 3           | 40          | 40          | "                                     |
| Static sens. gate 4 ~ 5           | 30          | 30          | "                                     |
| Static sens. gates 6 ~ 8          | 20          | 20          | "                                     |

---

## 8. Practical compatibility notes for `ld2410.cpp`

1. **`requestFirmwareVersion()`** uses opcode `0x00A0`. On S the call will time out with no ACK — the S opcode is `0x0000`. For multi-variant support the library would need a probe (try `0x00A0`, on timeout try `0x0000`).

2. **Engineering-mode toggles (`0x0062` / `0x0063`)** are silently ignored by S. The library currently wraps these in enter/leave configuration (PR #7 fix on the fork). On S this will succeed at the configuration layer but the toggles do nothing — the per-gate energies are already inline once the output mode is `standard` (`0x007A` with `00 00 01 00 00 00`).

3. **Frame parser**: the existing length-driven state machine (PR #6 on the fork) reads exactly `intra_len + 10` bytes, so it will tolerate the extended S "standard" frame as long as the per-gate accessors interpret offsets `19..27` and `28..36` correctly. **Currently those offsets assume the base/C engineering-frame layout (Table 14)**, which is structurally similar to the S standard frame — but the S frame fixes 16 gates, so any reader that loops to `N` derived from the current configuration must be re-pointed to a fixed 16-gate constant, or it will under-read.

4. **Stationary distance width**: the C variant uses **2 bytes** while the base PDF says 1 byte. The library implements 2 bytes (PR #2 + tests aligned with the C layout) — agrees with C and with most observed base firmwares.

5. **Default baud**: the existing `examples/basicSensor.ino` calls `RADAR_SERIAL.begin(256000, …)` which works for **C** and for base **only after** rebauding via `0x00A1` (the base default is 57600). For S, `115200` is required. There is no existing example for S; opening one would need new wiring (3.3 V supply, J2 layout, OT2 = presence flag instead of OUT-on-pin-1-or-3).

6. **`requestRestart()`** wraps the radar reboot blackout with `vTaskSuspend` — this only matters on the C/base because S has no `0x00A3`. Calling `requestRestart()` on S is a no-op that will time out.

7. **Bluetooth + distance resolution helpers** (regression noted in upstream issue #39: "readresolution / setresolution / disablebluetooth / enablebluetooth / getMAC" present in v0.1.3 but not in v0.1.4) are **C-only** at the protocol level. Re-adding them is meaningful for the C; on base they would always fail with no ACK; on S they are not part of the protocol.

---

## 9. One-line summary

- **base ↔ C** are the same protocol family with **C** adding six commands (BT, MAC, distance resolution) and a different default baud + pinout.
- **S** is a **separate protocol** that happens to share only the configuration-mode envelope (`0xFF` / `0xFE`) with base/C. Treating S as "another LD2410" is what causes the silent failures reported in upstream issues #21 (LD2410S/2420 misidentification) and #36 (LD2410S support request).
