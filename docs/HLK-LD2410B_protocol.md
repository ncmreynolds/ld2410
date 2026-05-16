# HLK-LD2410B
## Human presence sensing module — serial communication protocol

**Manufacturer:** Shenzhen Hi-Link Electronic Co., Ltd
**Version:** V1.06
**Modify date:** 2023-02-20
**Copyright:** © Shenzhen Hi-Link Electronic Co., Ltd

> This document describes the **LD2410B** module. The B variant uses the **5-pin layout with OUT on pin 1** (same as the base LD2410), runs at the **256000 bps** default baud rate (same as the LD2410C), and adds Bluetooth, MAC, distance-resolution and **light-sensor / auxiliary control** commands on top of the base feature set. The original LD2410 spec is in [`HLK-LD2410_protocol.md`](HLK-LD2410_protocol.md), the LD2410C (3-pin OUT layout, 256000 bps, no light sensor) in [`HLK-LD2410C_protocol.md`](HLK-LD2410C_protocol.md), and the LD2410S ultra-low-power variant in [`HLK-LD2410S_protocol.md`](HLK-LD2410S_protocol.md).

---

## Catalog

1. [Communication interface introduction](#1-communication-interface-introduction)
   - 1.1 [Pin definition](#11-pin-definition)
   - 1.2 [Use and configuration](#12-use-and-configuration)
     - 1.2.1 [Typical application circuits](#121-typical-application-circuits)
     - 1.2.2 [The role of configuration parameters](#122-the-role-of-configuration-parameters)
     - 1.2.3 [Visual configuration tool description](#123-visual-configuration-tool-description)
2. [Communication protocols](#2-communication-protocols)
   - 2.1 [Protocol format](#21-protocol-format)
     - 2.1.1 [Protocol data format](#211-protocol-data-format)
     - 2.1.2 [Command protocol frame format](#212-command-protocol-frame-format)
   - 2.2 [Send command with ACK](#22-send-command-with-ack)
     - 2.2.1 [Enabling configuration commands](#221-enabling-configuration-commands)
     - 2.2.2 [End configuration command](#222-end-configuration-command)
     - 2.2.3 [Maximum distance gate and unoccupied duration parameters configuration command](#223-maximum-distance-gate-and-unoccupied-duration-parameters-configuration-command)
     - 2.2.4 [Read parameter command](#224-read-parameter-command)
     - 2.2.5 [Enabling engineering mode command](#225-enabling-engineering-mode-command)
     - 2.2.6 [Close project mode command](#226-close-project-mode-command)
     - 2.2.7 [Distance gate sensitivity configuration command](#227-distance-gate-sensitivity-configuration-command)
     - 2.2.8 [Read firmware version command](#228-read-firmware-version-command)
     - 2.2.9 [Set serial port baud rate](#229-set-serial-port-baud-rate)
     - 2.2.10 [Restore factory settings](#2210-restore-factory-settings)
     - 2.2.11 [Restart module](#2211-restart-module)
     - 2.2.12 [Bluetooth settings](#2212-bluetooth-settings)
     - 2.2.13 [Get mac address](#2213-get-mac-address)
     - 2.2.14 [Obtaining bluetooth permissions](#2214-obtaining-bluetooth-permissions)
     - 2.2.15 [Setting Bluetooth password](#2215-setting-bluetooth-password)
     - 2.2.16 [Distance resolution setting](#2216-distance-resolution-setting)
     - 2.2.17 [Query distance resolution setting](#2217-query-distance-resolution-setting)
     - 2.2.18 [Auxiliary control function setting](#2218-auxiliary-control-function-setting)
     - 2.2.19 [Query auxiliary control function configuration](#2219-query-auxiliary-control-function-configuration)
   - 2.3 [Radar data output protocol](#23-radar-data-output-protocol)
     - 2.3.1 [Reported data frame format](#231-reported-data-frame-format)
     - 2.3.2 [Target data composition](#232-target-data-composition)
   - 2.4 [Radar command configuration method](#24-radar-command-configuration-method)
     - 2.4.1 [Radar command configuration steps](#241-radar-command-configuration-steps)
3. [Revision records](#3-revision-records)
4. [Technical support and contact information](#4-technical-support-and-contact-information)

---

## Chart Index

- Table 1 — Pin definition table
- Table 2 — Send command protocol frame format
- Table 3 — Data format in the sending frame
- Table 4 — ACK command protocol frame format
- Table 5 — ACK intra-frame data format
- Table 6 — Serial port baud rate selection
- Table 7 — Factory default configuration values
- Table 8 — Distance resolution selection
- Table 9 — Command values for auxiliary control function settings
- Table 10 — Reported data frame format
- Table 11 — Intra-frame data frame format
- Table 12 — Data type description
- Table 13 — Target basic information data composition
- Table 14 — Target state value description
- Table 15 — Engineering model target data composition

---

# 1 Communication interface introduction

## 1.1 Pin definition

**Table 1 — Pin definition table**

| Pin | Symbol  | Name                 | Function                                                                       |
|-----|---------|----------------------|--------------------------------------------------------------------------------|
| 1   | OUT     | Target state output  | Human presence detected: output high level / No human presence: output low level |
| 2   | UART_Tx | Serial Tx            | Serial port Tx pin                                                             |
| 3   | UART_Rx | Serial Rx            | Serial port Rx pin                                                             |
| 4   | GND     | Power Ground         | Power Ground                                                                   |
| 5   | VCC     | Power input          | Power supply input 5 V                                                         |

## 1.2 Use and configuration

### 1.2.1 Typical application circuits

LD2410B module directly outputs the detected target state through an IO pin (someone high, no one low), and also outputs the detection result data over the serial port according to the prescribed protocol. The serial output data contains the target state and distance auxiliary information; users can adapt usage to their specific application scenarios.

The module power supply voltage is 5 V and the input power supply capacity is required to be greater than 200 mA.

The module IO output level is 3.3 V. The default baud rate of the serial port is 256000, with 1 stop bit and no parity bit.

### 1.2.2 The role of configuration parameters

Users can modify the configuration parameters of the module through the serial port of the LD2410B to adapt to different application requirements.

The configurable radar detection parameters include the following:

**● Maximum detection distance**

Set the farthest detectable distance: only human targets that appear within this farthest distance will be detected and the result will be output.

Set up in units of distance gate, maximum 8 distance gates, configurable distance resolution (0.2 m or 0.75 m per distance gate).

Including motion detection of the farthest distance gate and stationary detection of the farthest distance gate, which can be set in the range of 1 to 8. For example, with the farthest distance gate set to 2 and the distance resolution at 0.75 m, only the presence of a human body within 1.5 m will be effectively detected and the result reported.

**● Sensitivity**

The presence of a target is determined when the detected target energy value (range 0 to 100) is greater than the sensitivity value, otherwise it is ignored.

Sensitivity values can be set in the range of 0 to 100. Each distance gate can be set independently, allowing precise adjustment for different distance ranges, local fine detection or filtering of specific areas of interference.

In addition, if the sensitivity of a given distance gate is set to 100, no target will be identified under that distance gate. For example, with distance gate 3 and distance gate 4 sensitivity set to 20, all other distance gates set to 100, and distance resolution at 0.75 m, only humans in the range 2.25 m to 3.75 m from the module will be detected.

**● No-one duration**

When the radar transitions from "occupied" to "unoccupied", it will continue to report "occupied" for a period of time. If the radar detection range remains unoccupied throughout this period, the radar will then report unoccupied; if the radar detects someone in this time period, the timer is refreshed. Unit: seconds. Equivalent to a no-one delay time — after the person leaves, the output state stays "occupied" until the no-one condition persists for longer than this duration.

### 1.2.3 Visual configuration tool description

To facilitate quick and efficient testing and configuration, the manufacturer provides a PC-side configuration tool. Users can use this tool software to connect to the module's serial port, read and configure the module's parameters, and also receive the detection results data reported by the module with a real-time visual display.

**Usage of the Uplink tool:**

1. Properly connect the module serial port using a USB-to-serial tool.
2. Select the corresponding serial port number in the upper-computer tool, set baud rate 256000, select project mode and click "Connect Device".
3. After successful connection, click the "Start" button; the test results and data will be displayed on the right graphical interface.
4. After connection, when the start button is not clicked (or after pressing Stop), the mode parameter information can be read or set.

**Note:** Parameters cannot be read or configured after clicking Start; configuration is only possible after stopping.

In the visualization tool, the round ball is an indication of the target status output: **red** means a moving target, **purple** means a stationary target, **green** means no one.

---

# 2 Communication protocols

This communication protocol is mainly aimed at users who need to do secondary development without the visualization tool. The data output and parameter configuration commands of the radar are carried out under this protocol. The default baud rate of the radar serial port is 256000, 1 stop bit, no parity bit.

## 2.1 Protocol format

### 2.1.1 Protocol data format

The LD2410B uses **little-endian** format for serial data communication, and all data in the following tables are in hexadecimal.

### 2.1.2 Command protocol frame format

The protocol-defined radar configuration command and ACK command formats are shown in Table 2 through Table 5.

**Table 2 — Send command protocol frame format**

| Frame header | Intra-frame data length | Intra-frame data | End of frame |
|--------------|-------------------------|------------------|--------------|
| FD FC FB FA  | 2 bytes                 | See Table 3      | 04 03 02 01  |

**Table 3 — Data format in the sending frame**

| Command word (2 bytes) | Command value (N bytes) |
|------------------------|-------------------------|

**Table 4 — ACK command protocol frame format**

| Frame header | Intra-frame data length | Intra-frame data | End of frame |
|--------------|-------------------------|------------------|--------------|
| FD FC FB FA  | 2 bytes                 | See Table 5      | 04 03 02 01  |

**Table 5 — ACK intra-frame data format**

| Send command word \| 0x0100 (2 bytes) | Return value (N bytes) |
|----------------------------------------|------------------------|

## 2.2 Send command with ACK

### 2.2.1 Enabling configuration commands

Any other command issued to the radar must be executed after this command is sent, otherwise it is invalid.

- **Command word:** 0x00FF
- **Command value:** 0x0001
- **Return value:** 2 bytes ACK status (0 success, 1 failure) + 2 bytes protocol version (0x0001) + 2 bytes buffer size (0x0040)

**Send data:**

| FD FC FB FA | 04 00 | FF 00 | 01 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 08 00 | FF 01 | 00 00 | 01 00 | 40 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------|-------|-------------|

### 2.2.2 End configuration command

End the configuration command — after execution the radar resumes its working mode. If other commands need to be issued again, the enable-configuration command must be sent first.

- **Command word:** 0x00FE
- **Command value:** None
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:**

| FD FC FB FA | 02 00 | FE 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | FE 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.3 Maximum distance gate and unoccupied duration parameters configuration command

This command sets the maximum radar detection distance gate (motion & stationary) (configuration range 2~8) and the unmanned duration parameter (configuration range 0~65535 seconds). See the parameter-word table below. This configuration value is not lost when power is dropped.

- **Command word:** 0x0060
- **Command value:** 2-byte maximum motion distance gate word + 4-byte maximum motion distance gate parameter + 2-byte maximum standstill distance gate word + 4-byte maximum standstill distance gate parameter + 2-byte unoccupied duration word + 4-byte unoccupied duration parameter
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**0x0060 protocol parameter word:**

| Parameter name                 | Parameter word |
|--------------------------------|----------------|
| Maximum movement distance door | 0x0000         |
| Maximum resting distance door  | 0x0001         |
| No one duration                | 0x0002         |

**Send data:** maximum distance door 8 (motion & stationary), no-one duration 5 seconds

| FD FC FB FA | 14 00 | 60 00 | 00 00 | 08 00 00 00 | 01 00 | 08 00 00 00 | 02 00 | 05 00 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|-------|-------------|-------|-------------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | 60 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.4 Read parameter command

This command allows you to read the current configuration parameters of the radar.

- **Command word:** 0x0061
- **Command value:** None
- **Return value:** 2 bytes ACK status (0 success, 1 failure) + header (0xAA) + max distance gate N (0x08) + configured max motion distance gate + configured max rest distance gate + distance gate 0 motion sensitivity (1 byte) + … + distance gate N motion sensitivity (1 byte) + distance gate 0 stationary sensitivity (1 byte) + … + distance gate N stationary sensitivity (1 byte) + unoccupied duration (2 bytes)

**Send data:**

| FD FC FB FA | 02 00 | 61 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK** (success, maximum distance gate 8, configured motion distance gate 8, stationary distance gate 8, 0~8 motion sensitivity 20, 0~8 stationary sensitivity 25, unoccupied duration 5 seconds):

| Bytes       | Value                |
|-------------|----------------------|
| Byte 1–4    | FD FC FB FA          |
| Byte 5, 6   | 1C 00                |
| Byte 7, 8   | 61 01                |
| Byte 9, 10  | 00 00                |
| Byte 11     | AA                   |
| Byte 12     | 08                   |
| Byte 13     | 08                   |
| Byte 14     | 08                   |
| Byte 15     | 14                   |
| Byte 16     | 14                   |
| Byte 17     | 14                   |
| Byte 18     | 14                   |
| Byte 19     | 14                   |
| Byte 20     | 14                   |
| Byte 21     | 14                   |
| Byte 22     | 14                   |
| Byte 23     | 14                   |
| Byte 24     | 19                   |
| Byte 25     | 19                   |
| Byte 26     | 19                   |
| Byte 27     | 19                   |
| Byte 28     | 19                   |
| Byte 29     | 19                   |
| Byte 30     | 19                   |
| Byte 31     | 19                   |
| Byte 32     | 19                   |
| Byte 33, 34 | 05 00                |
| Byte 35–38  | 04 03 02 01          |

### 2.2.5 Enabling engineering mode command

This command opens the radar engineering mode. When engineering mode is on, each distance-gate energy value is added to the radar report data — see [2.3.2 Target Data Composition](#232-target-data-composition) for the detailed format. Engineering mode is off by default after the module is powered on, and this configuration value is lost when power is lost.

- **Command word:** 0x0062
- **Command value:** None
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:**

| FD FC FB FA | 02 00 | 62 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | 62 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.6 Close project mode command

This command turns off the radar engineering mode. After it is turned off, see [2.3.2 Target Data Composition](#232-target-data-composition) for the radar report data format.

- **Command word:** 0x0063
- **Command value:** None
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:**

| FD FC FB FA | 02 00 | 63 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | 63 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.7 Distance gate sensitivity configuration command

This command configures the sensitivity of the distance gate; the configured value is not lost when power is dropped. It supports configuring each distance gate individually, or configuring all distance gates to a uniform value at the same time. If all distance gates are to be set to the same value, the distance gate value should be set to 0xFFFF.

- **Command word:** 0x0064
- **Command value:** 2-byte distance gate word + 4-byte distance gate value + 2-byte motion sensitivity word + 4-byte motion sensitivity value + 2-byte standstill sensitivity word + 4-byte standstill sensitivity value
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**0x0064 protocol parameter word:**

| Parameter name             | Parameter word |
|----------------------------|----------------|
| Distance door              | 0x0000         |
| Movement sensitivity word  | 0x0001         |
| Static sensitivity word    | 0x0002         |

**Send data:** configure distance gate 3, motion sensitivity 40, stationary sensitivity 40

| FD FC FB FA | 14 00 | 64 00 | 00 00 | 03 00 00 00 | 01 00 | 28 00 00 00 | 02 00 | 28 00 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|-------|-------------|-------|-------------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | 64 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Send data:** configure motion sensitivity 40 for all distance gates, stationary sensitivity 40

| FD FC FB FA | 14 00 | 64 00 | 00 00 | FF FF 00 00 | 01 00 | 28 00 00 00 | 02 00 | 28 00 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|-------|-------------|-------|-------------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | 64 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.8 Read firmware version command

This command reads the radar firmware version information.

- **Command word:** 0x00A0
- **Command value:** None
- **Return value:** 2 bytes ACK status (0 success, 1 failure) + 2 bytes firmware type (0x0001) + 2 bytes major version number + 4 bytes minor version number

**Send data:**

| FD FC FB FA | 02 00 | A0 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 0C 00 | A0 01 | 00 00 | 00 01 | 02 01 | 16 24 06 22 | 04 03 02 01 |
|-------------|-------|-------|-------|-------|-------|-------------|-------------|

The corresponding version number is V1.02.22062416.

### 2.2.9 Set serial port baud rate

This command is used to set the baud rate of the serial port of the module. The configured value is not lost when power is lost, and takes effect after restarting the module.

- **Command word:** 0x00A1
- **Command value:** 2-byte baud rate selection index
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Table 6 — Serial port baud rate selection**

| Baud rate selection index value | Baud rate |
|---------------------------------|-----------|
| 0x0001                          | 9600      |
| 0x0002                          | 19200     |
| 0x0003                          | 38400     |
| 0x0004                          | 57600     |
| 0x0005                          | 115200    |
| 0x0006                          | 230400    |
| 0x0007                          | 256000    |
| 0x0008                          | 460800    |

The factory default value is 0x0007, which is 256000.

**Send data:**

| FD FC FB FA | 04 00 | A1 00 | 07 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | A1 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.10 Restore factory settings

This command is used to restore all the configuration values to their non-factory values; the configuration values take effect after rebooting the module.

- **Command word:** 0x00A2
- **Command value:** None
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:**

| FD FC FB FA | 02 00 | A2 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | A2 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

The factory default configuration values are as follows:

**Table 7 — Factory default configuration values**

| Configuration items            | Default value |
|--------------------------------|---------------|
| Maximum movement distance door | 8             |
| Maximum resting distance door  | 8             |
| No one duration                | 5             |
| Serial port baud rate          | 256000        |
| Distance resolution            | 0.75 m        |

| Configuration items                              | Default value     | Configuration items                                | Default value      |
|--------------------------------------------------|-------------------|----------------------------------------------------|--------------------|
| Motion sensitivity at distance gate 0            | 50                | Stationary sensitivity at distance gate 0          | – (not settable)   |
| Motion sensitivity at distance gate 1            | 50                | Stationary sensitivity at distance gate 1          | – (not settable)   |
| Motion sensitivity at distance gate 2            | 40                | Stationary sensitivity at distance gate 2          | 40                 |
| Motion sensitivity at distance gate 3            | 30                | Stationary sensitivity at distance gate 3          | 40                 |
| Motion sensitivity at distance gate 4            | 20                | Stationary sensitivity at distance gate 4          | 30                 |
| Motion sensitivity at distance gate 5            | 15                | Stationary sensitivity at distance gate 5          | 30                 |
| Motion sensitivity at distance gate 6            | 15                | Stationary sensitivity at distance gate 6          | 20                 |
| Motion sensitivity at distance gate 7            | 15                | Stationary sensitivity at distance gate 7          | 20                 |
| Motion sensitivity at distance gate 8            | 15                | Stationary sensitivity at distance gate 8          | 20                 |

### 2.2.11 Restart module

The module receives this command and will automatically restart after the answer is sent.

- **Command word:** 0x00A3
- **Command value:** None
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:**

| FD FC FB FA | 02 00 | A3 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | A3 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.12 Bluetooth settings

This command is used to control Bluetooth on or off. The Bluetooth function of the module is on by default. The configuration value is not lost when power is lost, and takes effect after restarting the module.

- **Command word:** 0x00A4
- **Command value:** 0x0100 Turn on bluetooth / 0x0000 Turn off bluetooth
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:** (Turn on bluetooth)

| FD FC FB FA | 04 00 | A4 00 | 01 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | A4 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.13 Get mac address

This command is used to query the MAC address.

- **Command word:** 0x00A5
- **Command value:** 0x0001
- **Return value:** 2 bytes ACK status (0 success, 1 failure) + 1 byte fixed type (0x00) + 3 bytes MAC address (address is in big-endian order)

> Note: the documented payload returns 6 bytes of MAC address (the example below shows a 6-byte address).

**Send data:**

| FD FC FB FA | 04 00 | A5 00 | 01 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 0A 00 | A5 01 | 00 00 | 8F 27 | 2E B8 | 0F 65 | 04 03 02 01 |
|-------------|-------|-------|-------|-------|-------|-------|-------------|

The MAC address queried is: `8F 27 2E B8 0F 65`.

### 2.2.14 Obtaining bluetooth permissions

This command is used to get Bluetooth permission. After acquiring permission successfully, the APP can read device information and debug parameters via Bluetooth.

- **Command word:** 0x00A8
- **Command value:** 6 bytes of password value (every 2 bytes in little-endian order)
- **Return value:** 2-byte ACK status (0 success, 1 failure)

The default password is `"HiLink"`, which corresponds to 0x4869 (Hi) 0x4c69 (Li) 0x6e6b (nk).

**Send data:**

| FD FC FB FA | 08 00 | A8 00 | 48 69 | 4c 69 | 6e 6b | 48 69 | 04 03 02 01 |
|-------------|-------|-------|-------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | A8 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Note:** This response is only answered over Bluetooth, not on the serial port.

### 2.2.15 Setting Bluetooth password

This command is used to set the password for Bluetooth control. The configuration value is not lost when power is lost, and takes effect after restarting the module.

- **Command word:** 0x00A9
- **Command value:** 6 bytes of password value (each byte in little-endian order)
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Send data:**

| FD FC FB FA | 08 00 | A9 00 | 48 69 | 4c 69 | 6e 6b | 04 03 02 01 |
|-------------|-------|-------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | A9 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.16 Distance resolution setting

Set the distance resolution of the module, i.e. how far each distance gate represents. The configuration value is not lost when power is lost, and takes effect after restarting the module. Can be configured to 0.75 m or 0.2 m per distance gate; the maximum number of distance gates supported is 8.

- **Command word:** 0x00AA
- **Command value:** 2 bytes of distance resolution selection index
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Table 8 — Distance resolution selection**

| Distance resolution selection index value | Distance resolution (distance represented by each distance gate) |
|-------------------------------------------|------------------------------------------------------------------|
| 0x0000                                    | 0.75 m                                                           |
| 0x0001                                    | 0.2 m                                                            |

Factory default value is 0x0000, which is 0.75 m.

> Note: the original document text states "Factory default value is 0x0001, i.e. 0.75m", which is inconsistent with Table 8. Per the table mapping above, 0x0000 corresponds to 0.75 m and 0x0001 to 0.2 m — Table 7 (factory defaults) lists 0.75 m, so the correct factory default index is 0x0000.

**Send data:**

| FD FC FB FA | 04 00 | AA 00 | 01 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | AA 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.17 Query distance resolution setting

Query the module's current distance resolution setting, i.e. how far each distance gate represents.

- **Command word:** 0x00AB
- **Command value:** None
- **Return value:** 2 bytes ACK status (0 success, 1 failure) + 2 bytes distance resolution selection index

Return value definition is the same as Table 8 — Distance resolution selection.

**Send data:**

| FD FC FB FA | 02 00 | AB 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 06 00 | AB 01 | 00 00 | 01 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------|-------------|

Represents the currently set distance resolution of 0.2 m.

### 2.2.18 Auxiliary control function setting

This module comes with a photodiode whose detected light-sense value is included in the engineering-mode report (see [2.3.2 Target Data Composition](#232-target-data-composition)). The user can also configure the **light-sense auxiliary control function**; when this function is enabled, the OUT pin output is gated by **both** the radar detection result and the light-sense control logic:

- The OUT pin transitions from "unoccupied" to "occupied" only when: the radar detects "occupied" **and** the light-sensing auxiliary control logic condition is met.
- The OUT pin transitions from "occupied" to "unoccupied" requires only: the radar detects "unoccupied".

The light-sensing control logic can be configured to trigger either when the detected light-sensing value is **less than** the set threshold, or when it is **greater than** the set threshold. The default output level of the OUT pin can also be configured.

- **Command word:** 0x00AD
- **Command value:** 4-byte configuration value
- **Return value:** 2-byte ACK status (0 success, 1 failure)

**Table 9 — Command values for auxiliary control function settings**

**First byte — light-sense auxiliary control mode:**

| First byte | Description                                                                                                                                                       |
|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0x00       | Turn off the light-sense auxiliary control function; OUT pin output is not affected by light sense.                                                               |
| 0x01       | Enable the light-sense auxiliary control function: condition is met when the detected light-sense value is **less than** the threshold set by the second byte (0x00–0xFF). |
| 0x02       | Enable the light-sense auxiliary control function: condition is met when the detected light-sense value is **greater than** the threshold set by the second byte (0x00–0xFF). |

The factory default value of the first byte is 0x00, i.e. the light-sense auxiliary control function is turned off.

**Second byte — light-sense threshold:**

| Second byte | Description                                                                       |
|-------------|-----------------------------------------------------------------------------------|
| 0x00–0xFF   | The light sensitivity threshold to be set (range 0 to 255); default is 0x80.      |

**Third byte — OUT pin default level:**

| Third byte | Description                                                                                                  |
|------------|--------------------------------------------------------------------------------------------------------------|
| 0x00       | OUT pin is low by default: output low when no target triggers; output high when a target triggers.           |
| 0x01       | OUT pin is high by default: output high when no target triggers; output low when a target triggers.          |

The default value is 0x00, i.e. the OUT pin is low by default.

**Send data:** condition met when detected light-sense value is less than the set threshold; light-sense threshold set to 0x60; OUT default level low.

| FD FC FB FA | 06 00 | AD 00 | 01 60 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 04 00 | AD 01 | 00 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|

### 2.2.19 Query auxiliary control function configuration

Query the current auxiliary control configuration value of the module.

- **Command word:** 0x00AE
- **Command value:** None
- **Return value:** 2 bytes ACK status (0 success, 1 failure) + 4 bytes configuration value

Configuration value definition is the same as Table 9 — Command values for auxiliary control function settings.

**Send data:**

| FD FC FB FA | 02 00 | AE 00 | 04 03 02 01 |
|-------------|-------|-------|-------------|

**Radar ACK (success):**

| FD FC FB FA | 08 00 | AE 01 | 00 00 | 01 60 01 00 | 04 03 02 01 |
|-------------|-------|-------|-------|-------------|-------------|

Indicates the current setting is: auxiliary control condition is met when the detected light-sense value is less than the set threshold; light-sense threshold set to 0x60; default level of OUT is high.

## 2.3 Radar data output protocol

LD2410B outputs radar detection results through the serial port. By default it outputs basic target information, including target status, motion energy value, stationary energy value, motion distance and stationary distance. If the radar is configured for engineering mode, it additionally outputs the energy value for each distance gate (motion & stationary), the photosensitivity detection value, and the OUT pin output state. Radar data is output in the specified frame format.

### 2.3.1 Reported data frame format

The format of the radar uplink message frames defined by the protocol is shown in Tables 10 and 11. The definition of the upload data type in normal operation mode and engineering mode is shown in Table 12.

**Table 10 — Reported data frame format**

| Frame header | Length of data in the frame | Intra-frame data | End of frame |
|--------------|-----------------------------|------------------|--------------|
| F4 F3 F2 F1  | 2 bytes                     | See Table 11     | F8 F7 F6 F5  |

**Table 11 — Intra-frame data frame format**

| Data type             | Head | Target data            | Tail | Calibration |
|-----------------------|------|------------------------|------|-------------|
| 1 byte (See Table 12) | 0xAA | See Table 13, Table 15 | 0x55 | 0x00        |

**Table 12 — Data type description**

| Data type value | Description                   |
|-----------------|-------------------------------|
| 0x01            | Engineering mode data         |
| 0x02            | Target basic information data |

### 2.3.2 Target data composition

The content of the target data reported by the radar changes depending on the operating mode. In normal operation mode, the radar outputs the basic target information data by default; when configured to engineering mode, the radar adds per-distance-gate energy value information, the photosensitivity detection value and the OUT pin output state after the basic target information. The basic target information is always output in the radar report, while the engineering-mode extra data is only output once engineering mode has been enabled by command.

The composition of the target data reported by the radar in normal operation mode is shown in Table 13, with target state values defined in Table 14. The composition of target data frames in engineering mode is shown in Table 15, with additional data appended after the basic target information.

**Table 13 — Target basic information data composition**

| Target status         | Movement target distance (cm) | Movement target energy value | Distance to stationary target (cm) | Stationary target energy value | Detection distance (cm) |
|-----------------------|-------------------------------|------------------------------|-------------------------------------|--------------------------------|-------------------------|
| 1 byte (See Table 14) | 2 bytes                       | 1 byte                       | 2 bytes                             | 1 byte                         | 2 bytes                 |

**Table 14 — Target state value description**

| Target state value | Description                  |
|--------------------|------------------------------|
| 0x00               | No target                    |
| 0x01               | Movement target              |
| 0x02               | Stationary target            |
| 0x03               | Movement & stationary target |

**Table 15 — Engineering model target data composition**

Add the following data after the basic target information data in Table 13:

| Field                                       | Size   |
|---------------------------------------------|--------|
| Maximum movement distance gate N            | 1 byte |
| Maximum resting distance gate N             | 1 byte |
| Movement distance gate 0 energy value       | 1 byte |
| …                                           | …      |
| Movement distance gate N energy value       | 1 byte |
| Stationary distance gate 0 energy value     | 1 byte |
| …                                           | …      |
| Stationary distance gate N energy value     | 1 byte |
| Photosensitivity detection value            | 1 byte |
| OUT pin output state                        | 1 byte |

> Photodetection value range: 0 to 255. OUT pin output state: 0 = no one, 1 = someone.

**Example of reported data:**

Data reported in normal operating mode:

| Frame header | Length of data in frame | Intra-frame data                          | End of frame |
|--------------|-------------------------|-------------------------------------------|--------------|
| F4 F3 F2 F1  | 0D 00                   | 02 AA 02 51 00 00 00 00 3B 00 00 55 00    | F8 F7 F6 F5  |

Data reported in engineering mode:

| Frame header | Length of data in frame | Intra-frame data                                                                                                       | End of frame |
|--------------|-------------------------|------------------------------------------------------------------------------------------------------------------------|--------------|
| F4 F3 F2 F1  | 23 00                   | 01 AA 03 1E 00 3C 00 00 39 00 00 08 08 3C 22 05 03 03 04 03 06 05 00 00 39 10 13 06 06 08 04 60 01 55 00               | F8 F7 F6 F5  |

## 2.4 Radar command configuration method

### 2.4.1 Radar command configuration steps

The process of executing a configuration command by the LD2410B radar consists of two parts: the upper computer "sends the command" and the radar "replies with the command ACK". If the radar does not reply with ACK, or replies with a failure ACK, the configuration command has failed.

As mentioned earlier, before sending any other commands to the radar, the developer must send the "enable configuration" command and then send the configuration command within the specified time. After the configuration commands are issued, send the "end configuration" command to inform the radar that the configuration is finished.

For example, to read the radar configuration parameters: first the upper computer sends the "enable configuration" command; after receiving a successful radar ACK, it sends the "read parameters" command; after receiving a successful radar ACK, it finally sends the "end configuration" command. When that ACK is successful, the complete read-parameters action is finished.

The radar command configuration flow is as follows:

```
            ┌───────┐
            │ Start │
            └───┬───┘
                │
                ▼
   Send the "enable command configuration" command
                │
                ▼
        Received radar ACK success ── NO ──┐
                │ YES                       │
                ▼                           │
   Send command (Multiple bars available)   │
                │                           │
                ▼                           │
        Received radar ACK success ── NO ──┤
                │ YES                       │
                ▼                           │
       Send "End command configuration"     │
                │                           │
                ▼                           │
        Received radar ACK success ── NO ──┤
                │ YES                       ▼
                ▼                       ┌───────┐
            ┌───────┐                   │  End  │
            │  End  │ ◄─────────────────┤       │
            └───────┘                   └───────┘
```

*Figure — Radar command configuration process*

---

# 3 Revision records

| Date       | Version | Modified content                                                                |
|------------|---------|---------------------------------------------------------------------------------|
| 2022-06-24 | 1.01    | Initial version.                                                                |
| 2022-07-01 | 1.02    | Fix some error descriptions; add reboot and restore-factory commands.           |
| 2022-07-19 | 1.03    | Fix the length value of some command instances.                                 |
| 2022-08-26 | 1.04    | Add distance-resolution configuration command description.                      |
| 2022-09-20 | 1.05    | Add Bluetooth part of the protocol.                                             |
| 2023-02-20 | 1.06    | Add light-sensor value output description; add auxiliary control function setting command. |

---

# 4 Technical support and contact information

**Shenzhen Hi-Link Electronic Co., Ltd**

- **Address:** 1705, 17/F, Building E, XingheWORLD, Minle Community, Minzhi Street, Longhua District, Shenzhen, China
- **Tel:** 0755-23152658 / 83575155
- **Website:** [www.hlktech.com](http://www.hlktech.com)
