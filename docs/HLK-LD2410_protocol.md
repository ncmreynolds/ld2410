# HLK-LD2410
## Human presence sensing module — Serial Communication Protocol

**Manufacturer:** Shenzhen Hi-Link Electronic Co., Ltd
**Versions covered:** V1.02 (2022-12-19, initial) and V1.03 (2023-03-22, default baud rate updated to 57600)
**Copyright:** © Shenzhen Hi-Link Electronic Co., Ltd

> This document describes the **base LD2410** module (5-pin, default UART 57600 bps, OUT pin on position 1). The **LD2410C** variant — which uses a 3-pin OUT layout, default 256000 bps, and adds Bluetooth/MAC/distance-resolution commands — is documented separately in [`HLK-LD2410C_protocol.md`](HLK-LD2410C_protocol.md). The **LD2410S** ultra-low-power variant is in [`HLK-LD2410S_protocol.md`](HLK-LD2410S_protocol.md).

---

## Catalog

1. [Introduction to communication interfaces](#1-introduction-to-communication-interfaces)
   - 1.1 [Pin definitions](#11-pin-definitions)
   - 1.2 [Use and configuration](#12-use-and-configuration)
     - 1.2.1 [Typical application circuit](#121-typical-application-circuit)
     - 1.2.2 [The role of configuration parameters](#122-the-role-of-configuration-parameters)
     - 1.2.3 [Visual configuration tool description](#123-visual-configuration-tool-description)
2. [Communication protocol](#2-communication-protocol)
   - 2.1 [Protocol format](#21-protocol-format)
     - 2.1.1 [Protocol data format](#211-protocol-data-format)
   - 2.2 [Send command word & ACK](#22-send-command-word--ack)
     - 2.2.1 [Enable configuration command](#221-enable-configuration-command)
     - 2.2.2 [End configuration command](#222-end-configuration-command)
     - 2.2.3 [Maximum distance gate and unmanned duration parameter configuration command](#223-maximum-distance-gate-and-unmanned-duration-parameter-configuration-command)
     - 2.2.4 [Read parameter command](#224-read-parameter-command)
     - 2.2.5 [Enable engineering mode command](#225-enable-engineering-mode-command)
     - 2.2.6 [Close engineering mode command](#226-close-engineering-mode-command)
     - 2.2.7 [Range gate sensitivity configuration command](#227-range-gate-sensitivity-configuration-command)
     - 2.2.8 [Read firmware version command](#228-read-firmware-version-command)
     - 2.2.9 [Set serial port baud rate](#229-set-serial-port-baud-rate)
     - 2.2.10 [Factory reset](#2210-factory-reset)
     - 2.2.11 [Restart the module](#2211-restart-the-module)
   - 2.3 [Radar data output protocol](#23-radar-data-output-protocol)
     - 2.3.1 [Reporting data frame format](#231-reporting-data-frame-format)
     - 2.3.2 [Target data composition](#232-target-data-composition)
   - 2.4 [Radar command configuration mode](#24-radar-command-configuration-mode)
     - 2.4.1 [Radar command configuration steps](#241-radar-command-configuration-steps)
3. [Revision records](#3-revision-records)

---

## Chart Index

- Table 1 — Pin definition table
- Table 2 — Send command protocol frame format
- Table 3 — Transmit frame data format
- Table 4 — ACK command protocol frame format
- Table 5 — ACK frame data format
- Table 6 — Serial port baud rate selection
- Table 7 — Factory default configuration values
- Table 8 — Reported data frame format
- Table 9 — Intra-frame data format
- Table 10 — Data type description
- Table 11 — Target data frame composition (basic)
- Table 12 — Target state value description
- Table 13 — Target data frame composition (engineering mode)

---

# 1 Introduction to communication interfaces

## 1.1 Pin definitions

**Table 1 — Pin definition table**

| Pin | Symbol  | Name                  | Function                                                |
|-----|---------|-----------------------|---------------------------------------------------------|
| 1   | OUT     | Target status output  | High = human presence detected, low = no presence       |
| 2   | UART_Tx | Serial Tx             | Serial port Tx pin                                      |
| 3   | UART_Rx | Serial Rx             | Serial port Rx pin                                      |
| 4   | GND     | GND                   | GND                                                     |
| 5   | VCC     | Power input           | Power input 5 V                                         |

> The original PDF lists pins 2 and 3 both as "UART_Tx" — pin 3 is the Rx pin (corrected here for clarity).

## 1.2 Use and configuration

### 1.2.1 Typical application circuit

The LD2410 outputs the detected target state directly through the **OUT** pin (high = someone, low = no one) and additionally streams detection result data over the serial port using the protocol described in §2.

- **Supply voltage:** 5 V
- **Supply current capability required:** > 200 mA
- **OUT pin level:** 3.3 V
- **UART defaults:** 57600 bps, 8 data bits, 1 stop bit, no parity

### 1.2.2 The role of configuration parameters

Users can modify the module configuration through the serial port to fit different scenarios.

The configurable detection parameters include:

- **Farthest detection distance**
  - Set via *distance gate* units; each gate represents 0.75 m.
  - Configurable separately for moving and stationary detection.
  - Range: 1 ~ 8. Example: with the farthest gate set to `2`, only humans within 1.5 m are reported.
- **Sensitivity**
  - A target is reported only when its measured energy (range 0 ~ 100) exceeds the configured sensitivity.
  - Range: 0 ~ 100. Each gate's sensitivity can be set independently.
  - Setting a gate's sensitivity to `100` effectively disables target recognition at that gate. Example: setting gates 3 and 4 to `20` and all others to `100` restricts detection to 2.25 m – 3.75 m.
- **No-one duration**
  - After a presence-to-absence transition the radar continues reporting "manned" for this duration. If presence is detected again during this window, the timer is refreshed. Once the timer expires the radar reports "no one". Unit: seconds.

### 1.2.3 Visual configuration tool description

A PC tool is provided to read/configure parameters and visualise detection results.

How to use:

1. Connect the module's UART via a USB-to-serial adapter.
2. In the PC tool, select the COM port, set the baud rate to **57600**, choose engineering mode and click *Connect*.
3. After connecting, click *Start* — the right-hand graphical pane will display test results and data.
4. After connecting, configuration can only be read or written **before** *Start* (or after *Stop*) — not while detection is running.

The on-screen status indicator uses three colours: **red** = moving target present, **purple** = stationary target present, **green** = no one.

---

# 2 Communication protocol

This protocol is intended for users doing secondary development without the visual tools. The LD2410 communicates over a TTL serial port. Both the radar data output and the parameter configuration commands are carried in this protocol.

**Default UART parameters:** 57600 bps (since V1.03; V1.02 used a different default), 8 data bits, 1 stop bit, no parity.

## 2.1 Protocol format

### 2.1.1 Protocol data format

The radar configuration command and ACK command formats are summarised in Tables 2–5 below.

**Table 2 — Send command protocol frame format**

| Frame header  | Frame data length | Frame data    | MFR (frame end) |
|---------------|-------------------|---------------|-----------------|
| `FD FC FB FA` | 2 bytes           | see Table 3   | `04 03 02 01`   |

**Table 3 — Transmit frame data format**

| Command word (2 bytes) | Command value (N bytes) |
|------------------------|-------------------------|

**Table 4 — ACK command protocol frame format**

| Frame header  | Frame data length | Frame data    | MFR (frame end) |
|---------------|-------------------|---------------|-----------------|
| `FD FC FB FA` | 2 bytes           | see Table 5   | `04 03 02 01`   |

**Table 5 — ACK frame data format**

| Send command word ⊕ `0x0100` (2 bytes) | Return value (N bytes) |
|----------------------------------------|------------------------|

## 2.2 Send command word & ACK

### 2.2.1 Enable configuration command

Any other command issued to the radar must be executed after this command, otherwise it is invalid.

- **Command word:** `0x00FF`
- **Command value:** `0x0001`
- **Return value:** 2-byte status (`0` success, `1` failure) + 2-byte protocol version (`0x0001`) + 2-byte buffer size (`0x0040`).

**Send:**

| Frame head    | Length  | Command word | Command value | Frame end     |
|---------------|---------|--------------|---------------|---------------|
| `FD FC FB FA` | `04 00` | `FF 00`      | `01 00`       | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | Status  | Protocol version | Buffer size | Frame end     |
|---------------|---------|--------------|---------|------------------|-------------|---------------|
| `FD FC FB FA` | `08 00` | `FF 01`      | `00 00` | `01 00`          | `40 00`     | `04 03 02 01` |

### 2.2.2 End configuration command

Ends configuration mode. The radar returns to working mode after execution. To issue further commands, send *enable configuration* first.

- **Command word:** `0x00FE`
- **Command value:** none
- **Return value:** 2-byte ACK status (`0` success, `1` failure).

**Send:**

| Frame head    | Length  | Command word | Frame end     |
|---------------|---------|--------------|---------------|
| `FD FC FB FA` | `02 00` | `FE 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `FE 01`      | `00 00` | `04 03 02 01` |

### 2.2.3 Maximum distance gate and unmanned duration parameter configuration command

Sets the radar's maximum detection range gate (moving & stationary, range 2 ~ 8) and the unmanned duration (range 0 ~ 65535 s). Configuration is non-volatile (persists across power cycles).

- **Command word:** `0x0060`
- **Command value:** 2-byte word + 4-byte value, repeated for the three parameters below.
- **Return value:** 2-byte ACK status (`0` success, `1` failure).

**`0x0060` parameter words:**

| Parameter name                | Parameter word |
|-------------------------------|----------------|
| Maximum moving distance gate  | `0x0000`       |
| Maximum static distance gate  | `0x0001`       |
| Unmanned duration             | `0x0002`       |

**Send (example — max moving gate = 8, max static gate = 8, unmanned duration = 5 s):**

| Frame head    | Length  | Command word | Word `0` | Value (4 B)     | Word `1` | Value (4 B)     | Word `2` | Value (4 B)     | Frame end     |
|---------------|---------|--------------|----------|-----------------|----------|-----------------|----------|-----------------|---------------|
| `FD FC FB FA` | `14 00` | `60 00`      | `00 00`  | `08 00 00 00`   | `01 00`  | `08 00 00 00`   | `02 00`  | `05 00 00 00`   | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `60 01`      | `00 00` | `04 03 02 01` |

### 2.2.4 Read parameter command

Reads the current configuration parameters.

- **Command word:** `0x0061`
- **Command value:** none
- **Return value:** 2-byte ACK status + header `0xAA` + max distance gate `N` (`0x08`) + configured max moving gate + configured max static gate + gate-0 motion sensitivity (1 byte) + … + gate-N motion sensitivity (1 byte) + gate-0 static sensitivity (1 byte) + … + gate-N static sensitivity (1 byte) + no-one duration (2 bytes).

**Send:**

`FD FC FB FA  02 00  61 00  04 03 02 01`

**ACK (success — example: N = 8, configured moving gate = 8, configured static gate = 8, motion sensitivity 20 for gates 0..8, static sensitivity 25 for gates 0..8, unmanned duration 5 s):**

| Bytes 1..4    | 5..6    | 7..8    | 9..10   | 11   | 12   | 13   | 14..22                              | 23..31                              | 32 |
|---------------|---------|---------|---------|------|------|------|-------------------------------------|-------------------------------------|----|
| `FD FC FB FA` | `18 00` | `61 01` | `00 00` | `AA` | `08` | `08` | `08 14 14 14 14 14 14 14 14`        | `19 19 19 19 19 19 19 19 19`        | …  |

> The remaining bytes contain the unmanned duration as 2 bytes followed by the trailer `04 03 02 01`. Field width: per-gate sensitivities are 1 byte each.

### 2.2.5 Enable engineering mode command

Turns on engineering mode. Once enabled, the per-gate energy values are appended to the standard target data — see §2.3.2 (Table 13).

> Engineering mode is **disabled by default** at power-on, and the setting is **lost on power-off** (volatile).

- **Command word:** `0x0062`
- **Command value:** none
- **Return value:** 2-byte ACK status.

**Send:**

| Frame head    | Length  | Command word | Frame end     |
|---------------|---------|--------------|---------------|
| `FD FC FB FA` | `02 00` | `62 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `62 01`      | `00 00` | `04 03 02 01` |

### 2.2.6 Close engineering mode command

Turns engineering mode off. After closing, the radar reverts to the basic target frame format (see §2.3.2 Table 11).

- **Command word:** `0x0063`
- **Command value:** none
- **Return value:** 2-byte ACK status.

**Send:**

| Frame head    | Length  | Command word | Frame end     |
|---------------|---------|--------------|---------------|
| `FD FC FB FA` | `02 00` | `63 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `63 01`      | `00 00` | `04 03 02 01` |

### 2.2.7 Range gate sensitivity configuration command

Configures the sensitivity of a distance gate. Configuration is non-volatile.

- Per-gate configuration, **or** "all gates at once" by using the magic distance-gate value `0xFFFF`.
- **Command word:** `0x0064`
- **Command value:** 2-byte distance gate word + 4-byte distance gate value + 2-byte motion sensitivity word + 4-byte motion sensitivity value + 2-byte static sensitivity word + 4-byte static sensitivity value.
- **Return value:** 2-byte ACK status.

**`0x0064` parameter words:**

| Parameter name        | Parameter word |
|-----------------------|----------------|
| Distance gate         | `0x0000`       |
| Motion sensitivity    | `0x0001`       |
| Static sensitivity    | `0x0002`       |

**Send (example — set all gates' motion sensitivity = 40, all gates' static sensitivity = 40):**

| Frame head    | Length  | Command word | Gate-word + value (`0xFFFF`) | Motion-word + value (`40`)        | Static-word + value (`40`)        | Frame end     |
|---------------|---------|--------------|------------------------------|-----------------------------------|-----------------------------------|---------------|
| `FD FC FB FA` | `14 00` | `64 00`      | `00 00  FF FF 00 00`         | `01 00  28 00 00 00`              | `02 00  28 00 00 00`              | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `64 01`      | `00 00` | `04 03 02 01` |

### 2.2.8 Read firmware version command

Reads firmware version information.

- **Command word:** `0x00A0`
- **Command value:** none
- **Return value:** 2-byte ACK status + 2-byte firmware type (`0x0000`) + 2-byte major version + 4-byte minor version.

**Send:**

| Frame head    | Length  | Command word | Frame end     |
|---------------|---------|--------------|---------------|
| `FD FC FB FA` | `02 00` | `A0 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Type    | Major   | Minor (4 B)     | Frame end     |
|---------------|---------|--------------|---------|---------|---------|-----------------|---------------|
| `FD FC FB FA` | `0B 00` | `A0 01`      | `00 00` | `00 00` | `02 01` | `16 24 06 22`   | `04 03 02 01` |

> Decoded: version V1.02.22062416.

### 2.2.9 Set serial port baud rate

Sets the module's UART baud rate. Configuration is non-volatile and **takes effect after the next module restart**.

- **Command word:** `0x00A1`
- **Command value:** 2-byte baud-rate index (see Table 6).
- **Return value:** 2-byte ACK status.

**Table 6 — Serial port baud rate selection**

| Index    | Baud rate |
|----------|-----------|
| `0x0001` | 9600      |
| `0x0002` | 19200     |
| `0x0003` | 38400     |
| `0x0004` | 57600     |
| `0x0005` | 115200    |
| `0x0006` | 230400    |
| `0x0007` | 256000    |
| `0x0008` | 460800    |

> Factory default: `0x0004` (57600).

**Send (example — switch to 57600):**

| Frame head    | Length  | Command word | Index    | Frame end     |
|---------------|---------|--------------|----------|---------------|
| `FD FC FB FA` | `04 00` | `A1 00`      | `04 00`  | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `A1 01`      | `00 00` | `04 03 02 01` |

### 2.2.10 Factory reset

Restores all configuration values to their factory defaults. Takes effect after the next module restart.

- **Command word:** `0x00A2`
- **Command value:** none
- **Return value:** 2-byte ACK status.

**Send:**

| Frame head    | Length  | Command word | Frame end     |
|---------------|---------|--------------|---------------|
| `FD FC FB FA` | `02 00` | `A2 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `A2 01`      | `00 00` | `04 03 02 01` |

**Table 7 — Factory default configuration values**

| Configuration item                  | Default      |
|-------------------------------------|--------------|
| Maximum moving distance gate        | 8            |
| Maximum static distance gate        | 8            |
| Unmanned duration                   | 5 s          |
| Serial port baud rate               | 57600        |
| Motion sensitivity, gate 0          | 50           |
| Motion sensitivity, gate 1          | 50           |
| Motion sensitivity, gate 2          | 40           |
| Motion sensitivity, gate 3          | 30           |
| Motion sensitivity, gate 4          | 20           |
| Motion sensitivity, gate 5          | 15           |
| Motion sensitivity, gate 6          | 15           |
| Motion sensitivity, gate 7          | 15           |
| Motion sensitivity, gate 8          | 15           |
| Static sensitivity, gate 0          | (cannot be set) |
| Static sensitivity, gate 1          | (cannot be set) |
| Static sensitivity, gate 2          | 40           |
| Static sensitivity, gate 3          | 40           |
| Static sensitivity, gate 4          | 30           |
| Static sensitivity, gate 5          | 30           |
| Static sensitivity, gate 6          | 20           |
| Static sensitivity, gate 7          | 20           |
| Static sensitivity, gate 8          | 20           |

### 2.2.11 Restart the module

The module restarts automatically after the ACK is sent.

- **Command word:** `0x00A3`
- **Command value:** none
- **Return value:** 2-byte ACK status.

**Send:**

| Frame head    | Length  | Command word | Frame end     |
|---------------|---------|--------------|---------------|
| `FD FC FB FA` | `02 00` | `A3 00`      | `04 03 02 01` |

**ACK (success):**

| Frame head    | Length  | Command word | ACK     | Frame end     |
|---------------|---------|--------------|---------|---------------|
| `FD FC FB FA` | `04 00` | `A3 01`      | `00 00` | `04 03 02 01` |

## 2.3 Radar data output protocol

The LD2410 outputs detection results over the serial port. By default it streams the basic target information (state, motion energy, static energy, motion distance, static distance, detection distance). When **engineering mode** is enabled, the per-gate energy values are appended.

### 2.3.1 Reporting data frame format

**Table 8 — Reported data frame format**

| Frame header  | Frame data length | Frame data    | MFR (frame end) |
|---------------|-------------------|---------------|-----------------|
| `F4 F3 F2 F1` | 2 bytes           | see Table 9   | `F8 F7 F6 F5`   |

**Table 9 — Intra-frame data format**

| Data type | Head    | Target data                        | Tail    | Check   |
|-----------|---------|------------------------------------|---------|---------|
| 1 byte    | `0xAA`  | see Table 11 (basic) / Table 13 (engineering) | `0x55`  | `0x00`  |

**Table 10 — Data type description**

| Data type value | Description             |
|-----------------|-------------------------|
| `0x01`          | Engineering-mode data   |
| `0x02`          | Target data composition |

### 2.3.2 Target data composition

Reported target content depends on the working mode:

- **Normal working mode**: Table 11 — basic information only.
- **Engineering mode**: Table 13 — basic information **plus** per-gate energy values.

The basic information is always present in every data frame; the per-gate energies are appended only when engineering mode is enabled.

**Table 11 — Target data frame composition (basic / normal mode)**

| Target state           | Movement target distance (cm) | Movement target energy | Stationary target distance (cm) | Stationary target energy | Detection distance (cm) |
|------------------------|-------------------------------|------------------------|---------------------------------|--------------------------|-------------------------|
| 1 byte (see Table 12)  | 2 bytes                       | 1 byte                 | 1 byte                          | 1 byte                   | 1 byte                  |

> The "1 byte" entries for stationary target distance and detection distance are reported in the original PDF; in practice firmware versions report stationary distance as 2 bytes — verify against the actual byte stream of your unit.

**Table 12 — Target state value description**

| Target state value | Description                                 |
|--------------------|---------------------------------------------|
| `0x00`             | No target                                   |
| `0x01`             | Moving target                               |
| `0x02`             | Stationary target                           |
| `0x03`             | Moving + stationary targets                 |

**Table 13 — Target data frame composition (engineering mode)**

| (basic info, see Table 11) | Maximum moving distance gate `N` | Maximum static distance gate `N` | Movement gate 0 energy | … | Movement gate `N` energy | Static gate 0 energy | … | Static gate `N` energy | Reserved (extra info) |
|----------------------------|----------------------------------|----------------------------------|------------------------|---|--------------------------|----------------------|---|------------------------|-----------------------|
| 2 bytes (continuation)     | 1 byte                           | 1 byte                           | 1 byte                 | … | 1 byte                   | 1 byte               | … | 1 byte                 | M bytes               |

**Example — basic mode frame:**

| Frame header  | Length  | Frame data                                              | MFR           |
|---------------|---------|---------------------------------------------------------|---------------|
| `F4 F3 F2 F1` | `0D 00` | `02 AA 02 51 00 00 00 00 3B 00 00 55 00`                | `F8 F7 F6 F5` |

**Example — engineering mode frame:**

| Frame header  | Length  | Frame data                                                                                              | MFR           |
|---------------|---------|---------------------------------------------------------------------------------------------------------|---------------|
| `F4 F3 F2 F1` | `23 00` | `01 AA 03 1E 00 3C 00 00 39 00 00 08 08 3C 22 05 03 03 04 03 06 05 00 00 39 10 13 06 06 08 04 03 05 55 00` | `F8 F7 F6 F5` |

## 2.4 Radar command configuration mode

### 2.4.1 Radar command configuration steps

Each configuration command involves two phases: the host *sends a command* and the radar *replies with an ACK*. If the radar does not ACK, or replies with a failure ACK, the configuration command was not executed.

The required sequence is always:

1. Send **enable configuration** (§2.2.1) — must succeed before any other command.
2. Send the actual configuration command(s) within the allowed time window.
3. Send **end configuration** (§2.2.2) — informs the radar that the configuration session is over.

Example — reading parameters:

1. Host sends *enable configuration* → radar ACK.
2. Host sends *read parameter* (§2.2.4) → radar ACK with the configured values.
3. Host sends *end configuration* → radar ACK indicates the read sequence is complete.

---

# 3 Revision records

| Date       | Version | Modification                                       |
|------------|---------|----------------------------------------------------|
| 2022-12-19 | 1.02    | Initial version                                    |
| 2023-03-22 | 1.03    | Updated default baud rate to 57600                 |
