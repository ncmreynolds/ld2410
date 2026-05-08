# HLK-LD2410S
## Low Power Human Presence Sensor Module — Serial Communication Protocol

**Manufacturer:** Shenzhen Hi-Link Electronic Co., Ltd
**Version:** V1.00
**Revised date:** 2024-08-23
**Copyright:** © Shenzhen Hi-Link Electronic Co., Ltd

---

## Catalog

1. [Hardware description](#1-hardware-description)
   - 1.1 [Hardware LD2410S](#11-hardware-ld2410s)
2. [Communication protocol](#2-communication-protocol)
   - 2.1 [Format of reported data](#21-format-of-reported-data)
   - 2.2 [Send commands and ACK](#22-send-commands-and-ack)
     - 2.2.1 [Switch the module output mode](#221-switch-the-module-output-mode)
     - 2.2.2 [Read the firmware version command](#222-read-the-firmware-version-command)
     - 2.2.3 [Enable configuration command](#223-enable-configuration-command)
     - 2.2.4 [End configuration command](#224-end-configuration-command)
     - 2.2.5 [Write serial number command](#225-write-serial-number-command)
     - 2.2.6 [Read serial number command](#226-read-serial-number-command)
     - 2.2.7 [Write generic parameter commands](#227-write-generic-parameter-commands)
     - 2.2.8 [Read common parameter commands](#228-read-common-parameter-commands)
     - 2.2.9 [Automatically update the threshold command](#229-automatically-update-the-threshold-command)
     - 2.2.10 [Write the trigger threshold parameter command](#2210-write-the-trigger-threshold-parameter-command)
     - 2.2.11 [Read trigger threshold parameter command](#2211-read-trigger-threshold-parameter-command)
     - 2.2.12 [Write the hold threshold command](#2212-write-the-hold-threshold-command)
     - 2.2.13 [Read the hold threshold parameter command](#2213-read-the-hold-threshold-parameter-command)
3. [Version history](#3-version-history)
4. [Technical support and contact information](#4-technical-support-and-contact-information)

---

## Chart Index

- Table 1-1 — J1 pin description
- Table 1-2 — J2 pin description
- Table 2-1 — Reported data formats (minimal / standard / automatic threshold progress)
- Table 2-2 — Definition of parameter word and value range of common parameters

---

# 1 Hardware description

## 1.1 Hardware LD2410S

The hardware LD2410S has two pin headers on the board:

- **J2** — five pin holes (factory-supplied pins) used for power supply and communication.
- **J1** — SWD interface for burning/debugging the MCU program.

**Table 1-1 — J1 pin description**

| Pin    | Name | Function                  | Remark             |
|--------|------|---------------------------|--------------------|
| J1Pin1 | GND  | Grounding                 | —                  |
| J1Pin2 | DIO  | SWD interface data line   | 0 ~ 3.3 V          |
| J1Pin3 | CLK  | SWD interface clock line  | 0 ~ 3.3 V          |
| J1Pin4 | 3V3  | Power input               | 3.0 ~ 3.6 V, typ. 3.3 V |

**Table 1-2 — J2 pin description**

| Pin    | Name | Function                                                                                       | Remark                  |
|--------|------|------------------------------------------------------------------------------------------------|-------------------------|
| J2Pin1 | 3V3  | Power input                                                                                    | 3.0 ~ 3.6 V, typ. 3.3 V |
| J2Pin2 | GND  | Grounding                                                                                      | —                       |
| J2Pin3 | OT1  | UART Tx                                                                                        | 0 ~ 3.3 V               |
| J2Pin4 | RX   | UART Rx                                                                                        | 0 ~ 3.3 V               |
| J2Pin5 | OT2  | IO used to report the detection status: high = manned, low = unmanned                          | 0 ~ 3.3 V               |

---

# 2 Communication protocol

This communication protocol is mainly used by users who need to do secondary development without visual tools.

The HLK-LD2410S is a battery-powered, ultra-low-power millimetre-wave human presence sensor that communicates over a TTL-level serial port. Sensor data output and parameter configuration commands all use this protocol.

**Default UART parameters:** 115200 bps, 8 data bits, 1 stop bit, no parity.

**Configuration workflow:**

1. Enter the command mode (Enable configuration command, §2.2.3).
2. Issue *set* / *read* parameter commands.
3. Exit the command mode (End configuration command, §2.2.4).

> **Endianness:** the LD2410S uses **little-endian** ordering. Hex values shown in the tables below are bytes in transmission order.

## 2.1 Format of reported data

The LD2410S supports three reported-data formats. The default is the minimal format. The standard format is used by the upper-computer (PC) tool. The automatic-threshold-progress format is emitted only while automatic threshold generation is in progress.

**Table 2-1 — Reported data formats**

| Format                            | Frame head   | Length     | Data type | Target state                                  | Object distance | Reserved | Per-gate energy | Frame end     |
|-----------------------------------|--------------|------------|-----------|-----------------------------------------------|-----------------|----------|-----------------|---------------|
| **Minimal data**                  | `6E`         | —          | —         | 1 byte (0/1 = no one; 2/3 = someone)          | 2 bytes (cm)    | —        | —               | `62`          |
| **Standard data**                 | `F4 F3 F2 F1`| 2 bytes    | `0x01`    | 1 byte (0/1 = no one; 2/3 = someone)          | 2 bytes (cm)    | 2 bytes  | 64 bytes        | `F8 F7 F6 F5` |
| **Automatic threshold progress**  | `F4 F3 F2 F1`| 2 bytes    | `0x03`    | —                                             | —               | —        | 2 bytes (progress × 100) | `F8 F7 F6 F5` |

## 2.2 Send commands and ACK

### 2.2.1 Switch the module output mode

Switches between minimal and standard reporting formats.

- **Command word:** `0x007A`
- **Command value:**
  - `00 00 00 01 00 00` — standard data output mode
  - `00 00 00 00 00 00` — minimal data output mode
- **Return value:** `00 00` on success.

**Send (standard mode):**

| Frame head      | Length  | Command word | Parameter values        | Frame end     |
|-----------------|---------|--------------|-------------------------|---------------|
| `FD FC FB FA`   | `08 00` | `7A 00`      | `00 00 01 00 00 00`     | `04 03 02 01` |

**ACK (success):**

| Frame head      | Length  | Command word | Parameter values | Frame end     |
|-----------------|---------|--------------|------------------|---------------|
| `FD FC FB FA`   | `04 00` | `7A 01`      | `00 00`          | `04 03 02 01` |

### 2.2.2 Read the firmware version command

Reads the sensor firmware version.

- **Command word:** `0x0000`
- **Command value:** none
- **Return value:** 2 bytes major + 2 bytes minor + 2 bytes patch.

**Send:**

| Frame head      | Length  | Command word | Frame end     |
|-----------------|---------|--------------|---------------|
| `FD FC FB FA`   | `02 00` | `00 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head      | Length  | Command word | Major  | Minor  | Patch  | Frame end     |
|-----------------|---------|--------------|--------|--------|--------|---------------|
| `FD FC FB FA`   | `08 00` | `00 01`      | `0x100`| `0x00` | `0x00` | `04 03 02 01` |

> Note: the upper byte of the major value carries the firmware-type identifier; in this example `1` denotes the firmware family.

### 2.2.3 Enable configuration command

Any other command can only be executed after this command has been issued.

- **Command word:** `0x00FF`
- **Command value:** `0x0001`
- **Return value:** 2-byte protocol version (`0x0001`) + 2-byte buffer size.

**Send:**

| Frame head      | Length  | Command word | Command value | Frame end     |
|-----------------|---------|--------------|---------------|---------------|
| `FD FC FB FA`   | `04 00` | `FF 00`      | `01 00`       | `04 03 02 01` |

**ACK (success):**

| Frame head      | Length  | Command word | Enable | Protocol version | Buffer size | Frame end     |
|-----------------|---------|--------------|--------|------------------|-------------|---------------|
| `FD FC FB FA`   | `08 00` | `FF 01`      | `00 00`| `03 00`          | `80 00`     | `04 03 02 01` |

### 2.2.4 End configuration command

Stops parameter configuration mode and resumes normal operation. To run any other command afterwards, send the *enable configuration* command first.

- **Command word:** `0x00FE`
- **Command value:** none
- **Return value:** 2-byte ACK status (`0` success, `1` failure).

**Send:**

| Frame head      | Length  | Command word | Frame end     |
|-----------------|---------|--------------|---------------|
| `FD FC FB FA`   | `02 00` | `FE 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head      | Length  | Command word | ACK    | Frame end     |
|-----------------|---------|--------------|--------|---------------|
| `FD FC FB FA`   | `04 00` | `FE 01`      | `00 00`| `04 03 02 01` |

### 2.2.5 Write serial number command

Writes the sensor serial number.

- **Command word:** `0x0010`
- **Command value:** 2-byte SN length + 8-byte SN.
- **Return value:** 2-byte ACK status (`0` success, `1` failure).

**Send (example — SN = `12345678`):**

| Frame head      | Length  | Command word | SN length | Serial number             | Frame end     |
|-----------------|---------|--------------|-----------|---------------------------|---------------|
| `FD FC FB FA`   | `0C 00` | `10 00`      | `08 00`   | `31 32 33 34 35 36 37 38` | `04 03 02 01` |

**ACK (success):**

| Frame head      | Length  | Command word | ACK    | Frame end     |
|-----------------|---------|--------------|--------|---------------|
| `FD FC FB FA`   | `04 00` | `10 01`      | `00 00`| `04 03 02 01` |

### 2.2.6 Read serial number command

Reads the sensor serial number.

- **Command word:** `0x0011`
- **Command value:** none
- **Return value:** 2-byte ACK status + 2-byte SN length + 8-byte SN.

**Send:**

| Frame head      | Length  | Command word | Frame end     |
|-----------------|---------|--------------|---------------|
| `FD FC FB FA`   | `02 00` | `11 00`      | `04 03 02 01` |

**ACK (success — SN = `12345678`):**

| Frame head      | Length  | Command word | ACK    | SN length | Serial number             | Frame end     |
|-----------------|---------|--------------|--------|-----------|---------------------------|---------------|
| `FD FC FB FA`   | `0E 00` | `11 01`      | `00 00`| `08 00`   | `31 32 33 34 35 36 37 38` | `04 03 02 01` |

### 2.2.7 Write generic parameter commands

Sets the sensor's general parameters.

- **Command word:** `0x0070`
- **Command value:** *(2-byte parameter word + 4-byte parameter value)* × N
- **Return value:** 2-byte ACK status.

**Send (example):**
- Detect farthest gate = 12, detect nearest gate = 0, no-one delay = 40 s, status reporting = 0.5 Hz, distance reporting = 0.5 Hz, response speed = normal.

| Frame head    | Length  | Command word | Farthest gate (`05`) | Nearest gate (`0A`) | Unattended delay (`06`) | Status report (`02`) | Distance report (`0C`) | Response speed (`0B`) | Frame end     |
|---------------|---------|--------------|----------------------|---------------------|-------------------------|----------------------|------------------------|-----------------------|---------------|
| `FD FC FB FA` | `26 00` | `70 00`      | `05 00 0C 00 00 00`  | `0A 00 00 00 00 00` | `06 00 28 00 00 00`     | `02 00 05 00 00 00`  | `0C 00 05 00 00 00`    | `0B 00 05 00 00 00`   | `04 03 02 01` |

**ACK (success):**

| Frame head      | Length  | Command word | ACK    | Frame end     |
|-----------------|---------|--------------|--------|---------------|
| `FD FC FB FA`   | `04 00` | `70 01`      | `00 00`| `04 03 02 01` |

**Table 2-2 — Common parameter words and value ranges**

| Parameter name             | Parameter word | Value range          | Unit |
|----------------------------|----------------|----------------------|------|
| Detect farthest gate       | `05`           | 1 ~ 16               | —    |
| Detect nearest gate        | `0A`           | 0 ~ 16               | —    |
| Unmanned delay time        | `06`           | 10 ~ 120             | s    |
| Status reporting frequency | `02`           | 0.5 ~ 8 (step 0.5)   | Hz   |
| Distance reporting frequency | `0C`         | 0.5 ~ 8 (step 0.5)   | Hz   |
| Response speed             | `0B`           | 5 (normal) / 10 (fast)| —   |

### 2.2.8 Read common parameter commands

Reads configuration parameters.

- **Command word:** `0x0071`
- **Command value:** *(2-byte parameter word)* × N
- **Return value:** *(4-byte parameter value)* × N

**Send:**

| Frame head    | Length  | Command word | `05 00` | `0A 00` | `06 00` | `02 00` | `0C 00` | `0B 00` | Frame end     |
|---------------|---------|--------------|---------|---------|---------|---------|---------|---------|---------------|
| `FD FC FB FA` | `0E 00` | `71 00`      |         |         |         |         |         |         | `04 03 02 01` |

**ACK (success — same parameters as the §2.2.7 example):**

| Frame head    | Length  | Command word | ACK     | Farthest    | Nearest     | Delay (s)   | Status freq | Distance freq | Resp. speed  | Frame end     |
|---------------|---------|--------------|---------|-------------|-------------|-------------|-------------|---------------|--------------|---------------|
| `FD FC FB FA` | `1A 00` | `71 01`      | `00 00` | `0C 00 00 00` | `00 00 00 00` | `28 00 00 00` | `05 00 00 00` | `05 00 00 00`   | `05 00 00 00`| `04 03 02 01` |

### 2.2.9 Automatically update the threshold command

Triggers automatic threshold update.

- **Command word:** `0x0009`
- **Command value:**
  - `00 02` — trigger factor
  - `00 01` — retention factor
  - `00 78` — scanning time
- **Return value:** 2 bytes (progress × 100), reported via the *automatic-threshold-progress* data frame.

**Send:**

| Frame head      | Length  | Command word | Trigger factor | Retention factor | Scanning time | Frame end     |
|-----------------|---------|--------------|----------------|------------------|---------------|---------------|
| `FD FC FB FA`   | `08 00` | `09 00`      | `02 00`        | `01 00`          | `78 00`       | `04 03 02 01` |

**Progress data frame (emitted while threshold update is running):**

| Frame head      | Length   | Data type | Threshold progress (×100)         | Frame end     |
|-----------------|----------|-----------|-----------------------------------|---------------|
| `F4 F3 F2 F1`   | 2 bytes  | `0x03`    | 2 bytes                           | `F8 F7 F6 F5` |

### 2.2.10 Write the trigger threshold parameter command

Sets the **trigger** threshold for distance gates 0 to 15.

- **Command word:** `0x0072`
- **Command value:** *(2-byte parameter word + 4-byte parameter value)* × N
- **Return value:** 2-byte ACK status.

**Send (example — trigger thresholds `[50, 46, 34, 32, 32, 32, 32, 32, 50, 46, 34, 32, 32, 32, 32, 32]` for gates 0..15):**

For each gate `g`, the entry is `gg 00  TT 00 00 00` where `gg` is the gate index and `TT` is the threshold byte.

| Gate | Word | Threshold (decimal → hex) | Encoded entry          |
|------|------|---------------------------|------------------------|
| 0    | `00` | 50 → `32`                 | `00 00  32 00 00 00`   |
| 1    | `01` | 46 → `2E`                 | `01 00  2E 00 00 00`   |
| 2    | `02` | 34 → `22`                 | `02 00  22 00 00 00`   |
| 3    | `03` | 32 → `20`                 | `03 00  20 00 00 00`   |
| 4    | `04` | 32 → `20`                 | `04 00  20 00 00 00`   |
| 5    | `05` | 32 → `20`                 | `05 00  20 00 00 00`   |
| 6    | `06` | 32 → `20`                 | `06 00  20 00 00 00`   |
| 7    | `07` | 32 → `20`                 | `07 00  20 00 00 00`   |
| 8    | `08` | 50 → `32`                 | `08 00  32 00 00 00`   |
| 9    | `09` | 46 → `2E`                 | `09 00  2E 00 00 00`   |
| 10   | `0A` | 34 → `22`                 | `0A 00  22 00 00 00`   |
| 11   | `0B` | 32 → `20`                 | `0B 00  20 00 00 00`   |
| 12   | `0C` | 32 → `20`                 | `0C 00  20 00 00 00`   |
| 13   | `0D` | 32 → `20`                 | `0D 00  20 00 00 00`   |
| 14   | `0E` | 32 → `20`                 | `0E 00  20 00 00 00`   |
| 15   | `0F` | 32 → `20`                 | `0F 00  20 00 00 00`   |

Send frame (length `0x62 = 98` bytes of intra-frame data):

`FD FC FB FA  62 00  72 00  <16 entries from the table above>  04 03 02 01`

**ACK (success):**

| Frame head      | Length  | Command word | ACK    | Frame end     |
|-----------------|---------|--------------|--------|---------------|
| `FD FC FB FA`   | `04 00` | `72 01`      | `00 00`| `04 03 02 01` |

### 2.2.11 Read trigger threshold parameter command

Reads the trigger thresholds for gates 0..15.

- **Command word:** `0x0073`
- **Command value:** *(2-byte parameter word)* × N
- **Return value:** *(4-byte parameter value)* × N

**Send (request all 16 gates):**

`FD FC FB FA  22 00  73 00  00 00 01 00 02 00 03 00 04 00 05 00 06 00 07 00 08 00 09 00 0A 00 0B 00 0C 00 0D 00 0E 00 0F 00  04 03 02 01`

**ACK (success — same thresholds as in §2.2.10):**

`FD FC FB FA  44 00  73 01  00 00  32 00 00 00  2E 00 00 00  22 00 00 00  20 00 00 00  20 00 00 00  20 00 00 00  20 00 00 00  20 00 00 00  32 00 00 00  2E 00 00 00  22 00 00 00  20 00 00 00  20 00 00 00  20 00 00 00  20 00 00 00  20 00 00 00  04 03 02 01`

### 2.2.12 Write the hold threshold command

Sets the **hold** threshold for distance gates 0 to 15.

- **Command word:** `0x0076`
- **Command value:** *(2-byte parameter word + 4-byte parameter value)* × N
- **Return value:** 2-byte ACK status.

**Send (example — hold thresholds `[15, 15, 15, 15, 15, 15, 15, 15, 9, 9, 9, 9, 9, 9, 9, 9]` for gates 0..15):**

| Gate | Word | Threshold | Encoded entry          |
|------|------|-----------|------------------------|
| 0    | `00` | 15 → `0F` | `00 00  0F 00 00 00`   |
| 1    | `01` | 15 → `0F` | `01 00  0F 00 00 00`   |
| 2    | `02` | 15 → `0F` | `02 00  0F 00 00 00`   |
| 3    | `03` | 15 → `0F` | `03 00  0F 00 00 00`   |
| 4    | `04` | 15 → `0F` | `04 00  0F 00 00 00`   |
| 5    | `05` | 15 → `0F` | `05 00  0F 00 00 00`   |
| 6    | `06` | 15 → `0F` | `06 00  0F 00 00 00`   |
| 7    | `07` | 15 → `0F` | `07 00  0F 00 00 00`   |
| 8    | `08` |  9 → `09` | `08 00  09 00 00 00`   |
| 9    | `09` |  9 → `09` | `09 00  09 00 00 00`   |
| 10   | `0A` |  9 → `09` | `0A 00  09 00 00 00`   |
| 11   | `0B` |  9 → `09` | `0B 00  09 00 00 00`   |
| 12   | `0C` |  9 → `09` | `0C 00  09 00 00 00`   |
| 13   | `0D` |  9 → `09` | `0D 00  09 00 00 00`   |
| 14   | `0E` |  9 → `09` | `0E 00  09 00 00 00`   |
| 15   | `0F` |  9 → `09` | `0F 00  09 00 00 00`   |

Send frame: `FD FC FB FA  62 00  76 00  <16 entries>  04 03 02 01`

**ACK (success):**

| Frame head      | Length  | Command word | ACK    | Frame end     |
|-----------------|---------|--------------|--------|---------------|
| `FD FC FB FA`   | `04 00` | `76 01`      | `00 00`| `04 03 02 01` |

### 2.2.13 Read the hold threshold parameter command

Reads the hold thresholds for gates 0..15.

- **Command word:** `0x0077`
- **Command value:** *(2-byte parameter word)* × N
- **Return value:** *(4-byte parameter value)* × N

**Send (request all 16 gates):**

`FD FC FB FA  22 00  77 00  00 00 01 00 02 00 03 00 04 00 05 00 06 00 07 00 08 00 09 00 0A 00 0B 00 0C 00 0D 00 0E 00 0F 00  04 03 02 01`

**ACK (success — same thresholds as §2.2.12):**

`FD FC FB FA  44 00  77 01  00 00  0F 00 00 00  0F 00 00 00  0F 00 00 00  0F 00 00 00  0F 00 00 00  0F 00 00 00  0F 00 00 00  0F 00 00 00  09 00 00 00  09 00 00 00  09 00 00 00  09 00 00 00  09 00 00 00  09 00 00 00  09 00 00 00  09 00 00 00  04 03 02 01`

---

# 3 Version history

| Version | Date       | Modification    |
|---------|------------|-----------------|
| 1.00    | 2024-08-23 | First edition   |

---

# 4 Technical support and contact information

**Shenzhen Hi-Link Electronic Co., Ltd**
Address: 17F Building E, Xinghe WORLD, Minzhi Street, Long Hua District, Shenzhen 518131
Phone: 0755-23152658 / 83575155
Email: sales@hlktech.com
Website: <https://www.hlktech.net/>
