# LD2410 / LD2410B / LD2410C / LD2410S — Byte-level protocol comparison

A side-by-side mapping of every byte sequence in the four HLK radar variants. Built directly from:

- `HLK-LD2410_protocol.md` — base, V1.02 / V1.03 (2022-12 / 2023-03)
- `HLK-LD2410B_protocol.md` — V1.06 (2023-02-20)
- `HLK-LD2410C_protocol.md` — V1.00 (2022-11-07)
- `HLK-LD2410S_protocol.md` — V1.00 (2024-08-23)

Legend: ✅ supported · ❌ not supported · 🟡 same opcode but different semantics or value range · ⚠️ structurally different.

---

## 0. Hardware & UART defaults

|                         | **LD2410 (base)**       | **LD2410B**             | **LD2410C**             | **LD2410S**                         |
|-------------------------|-------------------------|-------------------------|-------------------------|-------------------------------------|
| Supply voltage          | 5 V                     | 5 V                     | 5 V                     | **3.3 V** (3.0 ~ 3.6 V)             |
| Min current             | > 200 mA                | > 200 mA                | > 200 mA                | ultra-low power (battery target)    |
| Pin count (data header) | 5 (J0)                  | 5                       | 5                       | 5 (J2) + 4 (J1 = SWD debug)         |
| **Default baud**        | **57600** (V1.03)       | **256000**              | **256000**              | **115200**                          |
| Frame format            | 8N1                     | 8N1                     | 8N1                     | 8N1                                 |
| OUT pin position        | **pin 1**               | **pin 1**               | **pin 3**               | **pin 5 (J2 — OT2)**                |
| OUT pin level           | 3.3 V                   | 3.3 V                   | 3.3 V                   | 0 ~ 3.3 V                           |
| On-board photodiode     | —                       | ✅ (light-sense value in eng. frame; can gate OUT via `0xAD`) | —                       | —                                   |
| Auxiliary OUT pins      | —                       | —                       | —                       | OT1 = UART_Tx, OT2 = presence flag  |

### 0.1 Pin pinout side-by-side

| Pin | **LD2410 (base)** | **LD2410B**       | **LD2410C**   | **LD2410S (J2)**         |
|-----|-------------------|-------------------|---------------|--------------------------|
| 1   | OUT (status)      | OUT (status)      | UART_Tx       | 3V3 (power)              |
| 2   | UART_Tx           | UART_Tx           | UART_Rx       | GND                      |
| 3   | UART_Rx           | UART_Rx           | OUT (status)  | OT1 (UART_Tx)            |
| 4   | GND               | GND               | GND           | RX (UART_Rx)             |
| 5   | VCC (5 V)         | VCC (5 V)         | VCC (5 V)     | OT2 (presence high/low)  |

> **Wiring danger:** the base/B and C variants use **opposite** OUT-pin layouts (1 vs 3) on a 5-pin header. **B follows the base layout** (OUT on pin 1); C moves OUT to pin 3. A board labelled "LD2410" can be any of them: check before connecting, or you'll feed 5 V into a 3.3 V data pin.

---

## 1. Command-frame envelope (constant across all four variants)

```text
┌────────────────────┬──────────┬───────────────────────────┬───────────────────┐
│ Frame header       │ Length   │ Intra-frame data          │ Frame end (MFR)   │
│ FD FC FB FA        │ 2 bytes  │ command word + value      │ 04 03 02 01       │
└────────────────────┴──────────┴───────────────────────────┴───────────────────┘
```

ACK reply: same envelope, with the command word in the intra-frame data set to `<sent> | 0x0100` (so `0x00A0` send → `0x01A0` ACK, transmitted little-endian as `A0 01`).

This is the only part of the protocol shared identically by all four chips (the rest of the shared core covers base + B + C; S diverges).

---

## 2. Data-frame envelope (sensor → host)

|                                 | **LD2410 (base)** | **LD2410B**   | **LD2410C**   | **LD2410S**                                |
|---------------------------------|-------------------|---------------|---------------|--------------------------------------------|
| Header (standard / engineering) | `F4 F3 F2 F1`     | `F4 F3 F2 F1` | `F4 F3 F2 F1` | `F4 F3 F2 F1`                              |
| Trailer (standard / engineering)| `F8 F7 F6 F5`     | `F8 F7 F6 F5` | `F8 F7 F6 F5` | `F8 F7 F6 F5`                              |
| **"Minimal" frame**             | ❌                | ❌            | ❌            | ✅ `6E … 62` (head/end, single-byte each)  |
| Intra-frame head byte           | `0xAA`            | `0xAA`        | `0xAA`        | (none — minimal) / `0xAA` for standard     |
| Intra-frame tail byte           | `0x55 0x00`       | `0x55 0x00`   | `0x55 0x00`   | (none for minimal) / `0x55 0x00` for std   |

### 2.1 Data-type byte (right after the length field)

| Value | **LD2410 (base)**   | **LD2410B**         | **LD2410C**         | **LD2410S**                                          |
|-------|---------------------|---------------------|---------------------|------------------------------------------------------|
| `0x01`| Engineering mode    | Engineering mode (trailer bytes specialised — see §2.4) | Engineering mode    | **Standard data** (always includes per-gate energies)|
| `0x02`| Basic target data   | Basic target data   | Basic target data   | ❌                                                   |
| `0x03`| ❌                  | ❌                  | ❌                  | **Auto-threshold progress** (only while running)     |

> The S variant **inverts the meaning of `0x01`**: on base/C it means "engineering frame appended"; on S it means "standard standard" (which on S already carries the per-gate block — there is no separate engineering mode). A parser written for base/C will mis-decode S frames if reused as-is.

### 2.2 Standard / basic target frame layout

Field width per byte, in transmission order, after the data-type byte and intra-frame head `0xAA`:

| Field                          | **LD2410 (base)** | **LD2410B** | **LD2410C** | **LD2410S** (standard mode) |
|--------------------------------|-------------------|-------------|-------------|-----------------------------|
| Target state                   | 1 B               | 1 B         | 1 B         | 1 B (different code map)    |
| Movement distance              | 2 B (cm)          | 2 B (cm)    | 2 B (cm)    | —                           |
| Movement energy                | 1 B               | 1 B         | 1 B         | —                           |
| Stationary distance            | **1 B** (cm) ⚠️   | 2 B (cm)    | **2 B** (cm)| —                           |
| Stationary energy              | 1 B               | 1 B         | 1 B         | —                           |
| Detection distance             | **1 B** (cm) ⚠️   | 2 B (cm)    | **2 B** (cm)| 2 B (cm) — "object distance"|
| Reserved bits                  | —                 | —           | —           | 2 B                         |
| Per-gate energies block        | (only in eng. frame) | (only in eng. frame; trailer = photosens. + OUT) | (only in eng. frame) | **64 B inline** |
| Tail                           | `55 00`           | `55 00`     | `55 00`     | `55 00`                     |

> The base PDF marks stationary distance and detection distance as 1 B each, while the C PDF marks them as 2 B. In practice, **most real-world LD2410 / LD2410B firmwares emit 2 B fields** — the base PDF is widely considered to under-document the field width. The library assumes 2 B.

### 2.3 Target-state code map

| Code    | **base / B / C**                      | **S**                                |
|---------|---------------------------------------|--------------------------------------|
| `0x00`  | No target                             | "no one" (low nibble 0)              |
| `0x01`  | Moving target                         | "no one" (low nibble 1)              |
| `0x02`  | Stationary target                     | "someone present"                    |
| `0x03`  | Moving + stationary                   | "someone present"                    |

> S collapses the four-state map into "no one (0/1) vs present (2/3)". A boolean-only consumer can read either family, but anything that distinguishes moving from stationary needs different handling on S.

### 2.4 Engineering / per-gate energy block

|                                | **LD2410 (base)**       | **LD2410B**             | **LD2410C**             | **LD2410S**                       |
|--------------------------------|-------------------------|-------------------------|-------------------------|-----------------------------------|
| Activation                     | command `0x0062`        | command `0x0062`        | command `0x0062`        | always-on in standard mode        |
| Number of gates exposed        | configured `N+1` (≤ 9)  | configured `N+1` (≤ 9)  | configured `N+1` (≤ 9)  | fixed **16 gates** (0 ~ 15)       |
| Bytes per gate (motion + static)| 2 B/gate (1+1)         | 2 B/gate (1+1)          | 2 B/gate (1+1)          | 2 B/gate (1+1) — 32 B total       |
| "Reserved / extra info" tail   | `M` bytes (variable, undocumented) | **2 fixed, specified bytes**: 1 B photosensitivity (0~255) + 1 B OUT pin state (HLK §2.3.2 Table 15) — occupy the SAME wire slot base/C document as `M` reserved | `M` bytes (variable, undocumented) | included in 64 B inline block     |
| Block length                   | dynamic (≈ 18 ~ 22 B)   | dynamic (≈ 18 ~ 22 B) — **same length as base/C**; the B's trailer specialises bytes, it does not add new ones | dynamic (≈ 18 ~ 22 B)   | fixed **64 B**                    |

---

## 3. Command opcode matrix

Opcodes are 16-bit little-endian. The "Word" column shows the byte sequence as transmitted (LE).

| Word    | Command name                            | **base**                 | **B**                    | **C**                    | **S**                                                  |
|---------|-----------------------------------------|--------------------------|--------------------------|--------------------------|--------------------------------------------------------|
| `00 00` | Read firmware version                   | ❌                       | ❌                       | ❌                       | ✅ §2.2.2                                              |
| `09 00` | Auto-update threshold                   | ❌                       | ❌                       | ❌                       | ✅ §2.2.9 (trigger/retention/scan-time)                |
| `10 00` | Write serial number                     | ❌                       | ❌                       | ❌                       | ✅ §2.2.5 (8-byte SN)                                  |
| `11 00` | Read serial number                      | ❌                       | ❌                       | ❌                       | ✅ §2.2.6                                              |
| `60 00` | Set max gate + unmanned duration        | ✅ §2.2.3                | ✅ §2.2.3                | ✅ §2.2.3                | ❌ (replaced by `0x0070`)                              |
| `61 00` | Read parameters                         | ✅ §2.2.4                | ✅ §2.2.4                | ✅ §2.2.4                | ❌ (replaced by `0x0071`)                              |
| `62 00` | Enable engineering mode                 | ✅ §2.2.5                | ✅ §2.2.5                | ✅ §2.2.5                | ❌ (no separate mode)                                  |
| `63 00` | Close engineering mode                  | ✅ §2.2.6                | ✅ §2.2.6                | ✅ §2.2.6                | ❌                                                     |
| `64 00` | Range-gate sensitivity                  | ✅ §2.2.7 (single sens.) | ✅ §2.2.7 (single sens.) | ✅ §2.2.7 (single sens.) | ❌ (split into `0x0072` + `0x0076`)                    |
| `70 00` | Write generic parameters                | ❌                       | ❌                       | ❌                       | ✅ §2.2.7 (6 parameter words, see Table 2-2)           |
| `71 00` | Read generic parameters                 | ❌                       | ❌                       | ❌                       | ✅ §2.2.8                                              |
| `72 00` | Write trigger threshold (per gate)      | ❌                       | ❌                       | ❌                       | ✅ §2.2.10                                             |
| `73 00` | Read trigger threshold                  | ❌                       | ❌                       | ❌                       | ✅ §2.2.11                                             |
| `76 00` | Write hold threshold (per gate)         | ❌                       | ❌                       | ❌                       | ✅ §2.2.12                                             |
| `77 00` | Read hold threshold                     | ❌                       | ❌                       | ❌                       | ✅ §2.2.13                                             |
| `7A 00` | Switch output mode (minimal / standard) | ❌                       | ❌                       | ❌                       | ✅ §2.2.1                                              |
| `A0 00` | Read firmware version                   | ✅ §2.2.8                | ✅ §2.2.8                | ✅ §2.2.8                | ❌ (S uses `0x0000`)                                   |
| `A1 00` | Set serial port baud rate               | ✅ §2.2.9 (8 indices)    | ✅ §2.2.9 (8 indices)    | ✅ §2.2.9 (8 indices)    | ❌ (115200 fixed)                                      |
| `A2 00` | Factory reset                           | ✅ §2.2.10               | ✅ §2.2.10               | ✅ §2.2.10               | ❌                                                     |
| `A3 00` | Restart module                          | ✅ §2.2.11               | ✅ §2.2.11               | ✅ §2.2.11               | ❌                                                     |
| `A4 00` | Bluetooth on/off                        | ❌                       | ✅ §2.2.12               | ✅ §2.2.12               | ❌                                                     |
| `A5 00` | Get MAC address                         | ❌                       | ✅ §2.2.13               | ✅ §2.2.13               | ❌                                                     |
| `A8 00` | Obtain Bluetooth permissions            | ❌                       | ✅ §2.2.14               | ✅ §2.2.14               | ❌                                                     |
| `A9 00` | Set Bluetooth password                  | ❌                       | ✅ §2.2.15               | ✅ §2.2.15               | ❌                                                     |
| `AA 00` | Distance resolution setting             | ❌                       | ✅ §2.2.16 (0.75 / 0.2 m)| ✅ §2.2.16 (0.75 / 0.2 m)| ❌                                                     |
| `AB 00` | Query distance resolution               | ❌                       | ✅ §2.2.17               | ✅ §2.2.17               | ❌                                                     |
| `AD 00` | Auxiliary (light-sense) control set     | ❌                       | ✅ §2.2.18 (mode + threshold + OUT default) | ❌      | ❌                                                     |
| `AE 00` | Query auxiliary (light-sense) control   | ❌                       | ✅ §2.2.19               | ❌                       | ❌                                                     |
| `FE 00` | End configuration                       | ✅                       | ✅                       | ✅                       | ✅                                                     |
| `FF 00` | Enable configuration                    | ✅                       | ✅                       | ✅                       | ✅                                                     |

### 3.1 Visual partition

```text
opcodes shared by all four:     FE 00, FF 00                         (enter / leave config)
opcodes shared by base + B + C: 60 00, 61 00, 62 00, 63 00, 64 00,
                                A0 00, A1 00, A2 00, A3 00
opcodes shared by B + C:        A4 00, A5 00, A8 00, A9 00,
                                AA 00, AB 00                         (Bluetooth + distance resolution)
opcodes only on B:              AD 00, AE 00                         (light-sense auxiliary control:
                                                                      set/query mode + threshold +
                                                                      OUT default level)
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
| B       | `FD FC FB FA  04 00  FF 00  01 00  04 03 02 01`      | `FD FC FB FA  08 00  FF 01  00 00  01 00  40 00  04 03 02 01`                         |
| C       | `FD FC FB FA  04 00  FF 00  01 00  04 03 02 01`      | `FD FC FB FA  08 00  FF 01  00 00  01 00  40 00  04 03 02 01`                         |
| S       | `FD FC FB FA  04 00  FF 00  01 00  04 03 02 01`      | `FD FC FB FA  08 00  FF 01  00 00  03 00  80 00  04 03 02 01`  ← protocol = 3, buf 128 |

**Differences:** payload bytes are identical, but the S variant returns a **different protocol version** (`0x0003`) and **different buffer size** (`0x0080` = 128) inside the ACK. Parsers that only check the trailing magic + the success byte work everywhere; parsers that hardcode protocol/buffer values to `01 00 / 40 00` (as on base / B / C) will mis-flag the S as "wrong protocol".

### 4.2 End configuration (`0xFE`) — fully identical on all four

`FD FC FB FA  02 00  FE 00  04 03 02 01` → `FD FC FB FA  04 00  FE 01  00 00  04 03 02 01`

### 4.3 Engineering mode toggles (`0x62` / `0x63`)

| Variant | `enableEngineering` | `closeEngineering` |
|---------|---------------------|--------------------|
| base    | `…  02 00  62 00  …` → `…  04 00  62 01  00 00  …` | `…  02 00  63 00  …` → `…  04 00  63 01  00 00  …` |
| B       | identical to base   | identical to base — but resulting eng. frames carry 2 extra trailer bytes (see §2.4) |
| C       | identical to base   | identical to base  |
| S       | ❌ (`0x62` / `0x63` not part of the S protocol — the engineering data is folded into the always-on "standard" frame; toggle the format with `0x7A` instead) |

### 4.4 Restart (`0xA3`)

| Variant | Send                                       | ACK (success)                                       |
|---------|--------------------------------------------|-----------------------------------------------------|
| base    | `FD FC FB FA  02 00  A3 00  04 03 02 01`   | `FD FC FB FA  04 00  A3 01  00 00  04 03 02 01`     |
| B       | identical                                  | identical                                           |
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

### 5.3 LD2410B-only — Auxiliary (light-sense) control

The B is the only variant in this family with an on-board photodiode. The light-sense reading is exposed two ways:

1. **Passive read-out in the engineering frame** — two extra bytes appended after the per-gate energy block (HLK §2.3.2 Table 15):
   - **Photosensitivity detection value** (1 B, range 0 ~ 255)
   - **OUT pin output state** (1 B, `0 = no one`, `1 = someone`)
2. **Active gating** — when the auxiliary control function is enabled, the OUT pin transitions from "unoccupied" to "occupied" require **both** the radar to detect presence **and** the light-sense condition (less-than or greater-than the configured threshold) to be met. The transition back to "unoccupied" only requires the radar to lose presence.

The active gating is configured with the `0xAD` / `0xAE` command pair (HLK §2.2.18 / §2.2.19):

| Function                              | Word    | Send (key bytes)                                          | ACK                                       |
|---------------------------------------|---------|-----------------------------------------------------------|-------------------------------------------|
| Set aux control — off                 | `0xAD`  | `AD 00  00 <thr> <out> 00`                                | `AD 01  00 00`                            |
| Set aux control — gate when light < thr | `0xAD`| `AD 00  01 <thr> <out> 00`  (factory thr = 0x80)          | `AD 01  00 00`                            |
| Set aux control — gate when light > thr | `0xAD`| `AD 00  02 <thr> <out> 00`                                | `AD 01  00 00`                            |
| Query aux control                     | `0xAE`  | `AE 00`                                                   | `AE 01  00 00  <mode> <thr> <out> 00`     |

**Field map of the 4-byte `0xAD` value (and the same 4 bytes returned by `0xAE`):**

| Offset | Field                          | Range / values                                                                                          |
|--------|--------------------------------|---------------------------------------------------------------------------------------------------------|
| 0      | Mode                           | `0x00` = off (light sense doesn't affect OUT); `0x01` = gate when value < threshold; `0x02` = gate when value > threshold |
| 1      | Light-sense threshold          | `0x00 ~ 0xFF` (default `0x80`)                                                                          |
| 2      | OUT pin default level          | `0x00` = LOW when idle / HIGH on trigger (factory default); `0x01` = HIGH when idle / LOW on trigger    |
| 3      | Reserved                       | `0x00` (PDF example always sends 0; meaning unspecified)                                                |

Wire-byte example (HLK §2.2.18, set: "gate when light value < 0x60, OUT default level low"):

`FD FC FB FA  06 00  AD 00  01 60 00 00  04 03 02 01` → `FD FC FB FA  04 00  AD 01  00 00  04 03 02 01`

---

## 6. Parameter model — what the radar exposes

| Concept                      | **LD2410 (base)**     | **LD2410B**           | **LD2410C**           | **LD2410S**                          |
|------------------------------|-----------------------|-----------------------|-----------------------|--------------------------------------|
| Number of distance gates     | 9 (0 ~ 8)             | 9 (0 ~ 8); per-gate width switchable 0.75 m ↔ 0.20 m via `0xAA` (count unchanged) | 9 (0 ~ 8); per-gate width switchable 0.75 m ↔ 0.20 m via `0xAA` (count unchanged) | **16** (0 ~ 15) fixed             |
| Per-gate metric              | sensitivity (0 ~ 100) | sensitivity (0 ~ 100) | sensitivity (0 ~ 100) | **trigger** + **hold** thresholds (separate)  |
| All-gates broadcast value    | gate-id `0xFFFF`      | gate-id `0xFFFF`      | gate-id `0xFFFF`      | one entry per gate (always 16 entries)        |
| Max moving / static gate     | independent (`0x0000` / `0x0001`) | independent | independent | "farthest gate" + "nearest gate" (`05` / `0A`) |
| No-one duration              | 0 ~ 65535 s (`0x0002`)| 0 ~ 65535 s           | 0 ~ 65535 s           | 10 ~ 120 s (`06`)                    |
| Status reporting frequency   | not exposed           | not exposed           | not exposed           | 0.5 ~ 8 Hz, step 0.5 (`02`)          |
| Distance reporting frequency | not exposed           | not exposed           | not exposed           | 0.5 ~ 8 Hz, step 0.5 (`0C`)          |
| Response speed               | not exposed           | not exposed           | not exposed           | normal (5) / fast (10) (`0B`)        |
| Distance resolution          | fixed 0.75 m / gate   | switchable via `0xAA` (0.75 / 0.2 m) | switchable via `0xAA` (0.75 / 0.2 m) | implicit per gate width        |
| Bluetooth                    | ❌                    | ✅ (`0xA4`/`A8`/`A9`) | ✅ (`0xA4`/`A8`/`A9`) | ❌                                   |
| MAC address                  | ❌                    | ✅ (`0xA5`)            | ✅ (`0xA5`)            | ❌                                   |
| Light-sense (photodiode)     | ❌                    | ✅ (`0xAD`/`0xAE` + eng-frame trailer) | ❌                    | ❌                                   |
| OUT-pin default-level config | ❌                    | ✅ (low-active / high-active, via `0xAD` byte 2) | ❌                    | ❌                                   |
| Serial number                | ❌                    | ❌                    | ❌                    | ✅ (`0x10` / `0x11`)                 |
| Auto-threshold tuning        | ❌                    | ❌                    | ❌                    | ✅ (`0x09`)                          |

---

## 7. Factory defaults

| Item                              | **base**    | **B**       | **C**       | **S** (firmware)                      |
|-----------------------------------|-------------|-------------|-------------|---------------------------------------|
| Max moving gate                   | 8           | 8           | 8           | depends on configured "farthest gate" |
| Max static gate                   | 8           | 8           | 8           | n/a                                   |
| No-one duration                   | 5 s         | 5 s         | 5 s         | n/a (delay configurable, range 10 ~ 120 s) |
| Baud rate                         | 57600       | 256000      | 256000      | 115200                                |
| Output mode                       | basic       | basic       | basic       | minimal (`6E … 62`)                   |
| Distance resolution               | 0.75 m      | 0.75 m      | 0.75 m      | gate-implicit                         |
| Bluetooth                         | n/a         | on          | on          | n/a                                   |
| Light-sense aux control           | n/a         | off (`0xAD` mode byte = 0x00; thr 0x80) | n/a         | n/a                                   |
| OUT-pin default level             | low         | low (`0xAD` byte 2 = 0x00) | low         | n/a                                   |
| Motion sens. gate 0               | 50          | 50          | 50          | (replaced by trigger threshold model) |
| Motion sens. gate 1               | 50          | 50          | 50          | "                                     |
| Motion sens. gate 2               | 40          | 40          | 40          | "                                     |
| Motion sens. gate 3               | 30          | 30          | 30          | "                                     |
| Motion sens. gate 4               | 20          | 20          | 20          | "                                     |
| Motion sens. gates 5 ~ 8          | 15          | 15          | 15          | "                                     |
| Static sens. gates 0 ~ 1          | not settable| not settable| not settable| "                                     |
| Static sens. gate 2 ~ 3           | 40          | 40          | 40          | "                                     |
| Static sens. gate 4 ~ 5           | 30          | 30          | 30          | "                                     |
| Static sens. gates 6 ~ 8          | 20          | 20          | 20          | "                                     |

---

## 8. How the library handles the differences

> ⚠️ **Historical note.** Earlier revisions of this section described
> the library *before* the variant-aware refactor (when `requestFirmwareVersion`
> on S would time out, when the parser hard-coded base/C engineering
> offsets, and when several C-only commands had regressed out of v0.1.4).
> Those issues are no longer current. The list below describes how the
> library handles the documented protocol differences **today**.

1. **`requestFirmwareVersion()`** sends opcode `0xA0` on base/C and `0x00` on S. The ACK envelope is variant-aware: 12-byte intra on base/C with a 4-byte bugfix at `[14..17]`, 8-byte intra on S where the 16-bit major/minor/patch fields are stored as their LE low byte into the existing `firmware_*` fields.

2. **Engineering-mode toggles (`0x62` / `0x63`)** are gated behind `LD2410_HAS_ENGINEERING_MODE`, defined for base/C only. On S the per-gate energies arrive in the standard frame automatically; calling the toggles would be a compile error.

3. **Frame parser**: the length-driven state machine reads `intra_len + 10` bytes regardless of variant. Per-gate decoding uses `LD2410_GATE_COUNT` (9 on base/C, 16 on S) and per-variant offsets — base/C `[19..27]` motion + `[28..36]` stationary, S `[12..27]` motion + `[12+GATE_COUNT..]` stationary. The S layout is **UNVERIFIED on hardware** because the V1.00 PDF specifies the 64-byte block size but not its per-gate breakdown.

4. **Stationary distance width**: the C variant uses **2 bytes** while the base PDF says 1 byte. The library implements 2 bytes — agrees with C and with most observed base firmwares.

5. **Default baud** resolves at compile time via `LD2410_DEFAULT_BAUD` (57600 base / 256000 C / 115200 S). All `examples/*` sketches use this constant — no hard-coded value to mis-edit.

6. **`requestRestart()`** wraps the radar reboot blackout with `vTaskSuspend` on ESP32 (so `autoReadTask` is paused for the ~800 ms boot window). The opcode (`0xA3`) is gated behind `LD2410_HAS_RESTART`, defined for base/C only — calling it on S would be a compile error.

7. **Bluetooth + distance resolution helpers** (the regression noted in upstream issue #39) are gated behind `LD2410_HAS_BLUETOOTH` / `LD2410_HAS_DISTANCE_RESOLUTION`, both currently C-only. Building for `LD2410_VARIANT_C` exposes them; on base / S they are hidden and any usage produces a clean compile error pointing at the missing flag.

8. **LD2410B support — partial via the C build, no first-class variant yet.** The library does not define `LD2410_VARIANT_B` / `LD2410_HAS_AUX_CONTROL`, and no variant header exists at `src/ld2410_variants/ld2410_b.h`. A user who needs to drive a B board today should build with `LD2410_VARIANT_C`: the entire shared command set works because B uses identical opcodes for everything from `0x60`..`0xAB`. What is missing in that fallback:
   - the `0xAD` / `0xAE` light-sense auxiliary control commands (no `setAuxiliaryControl()` / `requestAuxiliaryControl()` methods exist);
   - the two trailing bytes of the B's engineering-mode frame (photosensitivity + OUT pin state). These occupy the SAME wire slot that base/C document as variable "M reserved", so the length-driven parser already absorbs them correctly — they are just silently dropped instead of exposed via public fields.

   The full first-class variant additions are tracked in [`method-coverage.md`](method-coverage.md#ld2410b) Table 3.

---

## 9. One-line summary

- **base ↔ B ↔ C** are the same protocol family. **B** and **C** both add the six BT/MAC/distance-resolution commands on top of base and share the same default baud (256000); **B** further adds the `0xAD`/`0xAE` light-sense auxiliary control pair and 2 trailing bytes in the engineering frame, while keeping the **base** pinout (OUT on pin 1) where C uses pin 3.
- **S** is a **separate protocol** that happens to share only the configuration-mode envelope (`0xFF` / `0xFE`) with base/B/C. Treating S as "another LD2410" is what causes the silent failures reported in upstream issues #21 (LD2410S/2420 misidentification) and #36 (LD2410S support request).
