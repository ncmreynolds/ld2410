# 05 — LD2410 (base) API — extensions over Core

Methods exposed on the original LD2410, LD2410B **and** LD2410C.
(The LD2410S has its own configuration model — see
[`07-api-ld2410s.md`](07-api-ld2410s.md).)

These are the configuration commands: max distance / sensitivity /
idle delay, gate sensitivity, current-config readback, engineering
mode toggle, restart, factory reset, firmware version. They share
the same envelope (`02 00 OP 00`) and the same configuration-window
discipline — the library wraps each call in `enter_configuration_mode_`
+ `leave_configuration_mode_` for you.

For the methods that work on **all four** variants (begin/read/
getters/snapshots) see [`04-api-core.md`](04-api-core.md). For the
LD2410B/C extensions (Bluetooth, MAC, baud rate, distance resolution
— shared between B and C — plus B-only auxiliary light-sense
control) see [`06-api-ld2410c.md`](06-api-ld2410c.md).

---

## Configuration writes

### `bool setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer)`

Set the maximum gate **value** for moving and stationary detection
(each in `1..LD2410_GATE_COUNT-1`, i.e. `1..8` on base/C — the radar
**rejects 0 with an ACK failure**, see the comment in
`src/ld2410_variants/ld2410_base.h`) and the inactivity timer in
seconds (after which the radar reports "no presence" once both tracks
go quiet).

Note: this is a *value range* (1..8), not a *gate-index range* (0..8).
The configured value caps which gate IDs the radar will report on, but
the radar's per-gate reporting itself indexes from gate 0.

Non-volatile — persists across reboots. Returns `true` on ACK.

```cpp
// 1. Standard config: full range, 5 s timeout
radar.setMaxValues(8, 8, 5);

// 2. Restrict to close range only (max gate 3 = gates 0..3)
radar.setMaxValues(3, 3, 2);

// 3. Long timeout for "should-stay-on" applications
radar.setMaxValues(8, 8, 60);                    // 60 s before "no presence"
```

Example: [`../examples/setupSensor/setupSensor.ino`](../examples/setupSensor/setupSensor.ino).

### `bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary)`

Set per-gate energy thresholds (0..100). Below the threshold, the
radar treats the gate as quiet. Lower = more sensitive (more false
positives), higher = stricter.

Non-volatile. Apply per gate; there's no "all gates at once" opcode
(use the helper pattern below).

```cpp
// 1. Standard mid-sensitivity preset (factory ships at ~40 / 40)
radar.setGateSensitivityThreshold(4, 40, 40);

// 2. Loop to apply same preset to all 9 gates (base/C)
for (uint8_t g = 0; g < 9; g++) radar.setGateSensitivityThreshold(g, 50, 50);

// 3. Per-gate tuning: tight on close gates, loose on far gates
const uint8_t presets[9][2] = {
  {30,30},{35,35},{40,40},{50,50},{60,60},{70,70},{75,75},{80,80},{85,85}
};
for (uint8_t g = 0; g < 9; g++)
  radar.setGateSensitivityThreshold(g, presets[g][0], presets[g][1]);
```

Example: [`../examples/setupSensor/setupSensor.ino`](../examples/setupSensor/setupSensor.ino).

---

## Configuration readback

### `bool requestCurrentConfiguration()`

Issue opcode 0x61 and populate these public fields from the ACK:

| Field | Meaning |
|---|---|
| `max_gate` | Hardware max gate (always 8 on base/C) |
| `max_moving_gate` | Current moving max (set by `setMaxValues`) |
| `max_stationary_gate` | Current stationary max |
| `motion_sensitivity[9]` | Per-gate moving threshold |
| `stationary_sensitivity[9]` | Per-gate stationary threshold |
| `sensor_idle_time` | Inactivity timer in seconds |

Returns `true` on ACK.

```cpp
// 1. Print everything
if (radar.requestCurrentConfiguration()) {
  Serial.print("max gates m/s: "); Serial.print(radar.max_moving_gate);
  Serial.print('/'); Serial.println(radar.max_stationary_gate);
  Serial.print("idle: "); Serial.print(radar.sensor_idle_time); Serial.println("s");
}

// 2. Snapshot before changing — typical save/modify/restore pattern
radar.requestCurrentConfiguration();
const uint8_t saved_idle = radar.sensor_idle_time;
radar.setMaxValues(8, 8, 30);
// ... do test ...
radar.setMaxValues(8, 8, saved_idle);

// 3. Verify a write took effect
radar.setMaxValues(6, 5, 3);
radar.requestCurrentConfiguration();
if (radar.max_moving_gate != 6) Serial.println("write rejected!");
```

---

## Engineering mode

### `bool requestStartEngineeringMode()` / `bool requestEndEngineeringMode()`

Toggle the per-gate energy data block on / off in the radar's stream.
Once enabled, every standard 0x01 frame includes a 9-byte motion +
9-byte stationary energy array. The library exposes these via
[`movingEnergyAtGate / stationaryEnergyAtGate / snapshotEngineering*`](04-api-core.md#engineering-mode--per-gate-energy).

Engineering mode is **volatile**: it resets across radar reboots.
Re-enable in `setup()` if you want it always on.

```cpp
// 1. Enable in setup, leave on forever
radar.requestStartEngineeringMode();

// 2. Toggle on demand for a tuning session
radar.requestStartEngineeringMode();
collectGateData();
radar.requestEndEngineeringMode();

// 3. Toggle from a serial-monitor command (interactive tuning)
if (Serial.available() && Serial.read() == 'e') {
  static bool eng = false;
  eng = !eng;
  if (eng) radar.requestStartEngineeringMode(); else radar.requestEndEngineeringMode();
}
```

Example: [`../examples/engineeringMode/engineeringMode.ino`](../examples/engineeringMode/engineeringMode.ino).

---

## Firmware version

### `bool requestFirmwareVersion()`

Issue the firmware-version query and populate three public fields:

| Field | Type | Notes |
|---|---|---|
| `firmware_major_version` | `uint8_t` | major component |
| `firmware_minor_version` | `uint8_t` | minor component |
| `firmware_bugfix_version` | `uint32_t` | bugfix on base/C (4 bytes); 16-bit zero-extended on S |

Returns `true` on ACK. Variant-aware: opcode is `0xA0` on base/C and
`0x00` on S, ACK length differs (12 vs 8 bytes), and field offsets
are decoded accordingly.

```cpp
// 1. Print on boot
radar.requestFirmwareVersion();
Serial.printf("v%u.%u (bugfix=%lx)\n",
              radar.firmware_major_version,
              radar.firmware_minor_version,
              (unsigned long)radar.firmware_bugfix_version);

// 2. Decision-tree based on FW
radar.requestFirmwareVersion();
if (radar.firmware_major_version >= 2) useNewFeature();

// 3. Health check by firmware probe
radar.requestFirmwareVersion();
if (radar.firmware_major_version == 0) Serial.println("radar not responding");
```

> The default `radar.begin(stream, /*waitForRadar=*/true)` call already
> issues `requestFirmwareVersion()` internally. You usually don't need
> to call it again unless you're verifying connectivity later.

---

## Lifecycle commands

### `bool requestRestart()`

Reboot the radar via opcode 0xA3. The library automatically suspends
`autoReadTask` (if running), drains the UART RX FIFO twice, waits
~800 ms for the boot blackout, clears the parser state, then resumes
the task. Returns `true` if the ACK was received before reboot.

```cpp
// 1. Apply non-volatile settings (e.g. baud rate, distance resolution)
radar.setBaudRate(LD2410_BAUD_INDEX_115200);
radar.requestRestart();
RADAR_SERIAL.begin(115200, SERIAL_8N1, RX, TX);   // re-open at the new baud

// 2. Recover from a hung state
if (!radar.isConnected()) { radar.requestRestart(); }

// 3. Periodic restart for long-running deployments (rare; usually unnecessary)
if (millis() - lastRestart > 24L * 3600L * 1000L) {
  radar.requestRestart();
  lastRestart = millis();
}
```

### `bool requestFactoryReset()`

Clear all configuration to factory defaults. **Destructive.** Requires
a `requestRestart()` afterwards to take effect for some fields.

```cpp
// 1. One-shot factory reset
radar.requestFactoryReset();
radar.requestRestart();

// 2. Reset only if a "known bad" state is detected
if (radar.requestCurrentConfiguration() && radar.sensor_idle_time > 600) {
  radar.requestFactoryReset();
  radar.requestRestart();
}

// 3. Bench-test pattern: factory-reset between test cases for a clean baseline
factoryResetBetweenTests();   // wraps requestFactoryReset + requestRestart
```

> ⚠️ Test sketches in `tests/hw/` deliberately skip
> `requestFactoryReset()` because it overwrites the maintainer's
> bench config. Be intentional about calling it.

---

## See also

- [`04-api-core.md`](04-api-core.md) — methods on all variants
- [`06-api-ld2410c.md`](06-api-ld2410c.md) — C-only extensions
- [`07-api-ld2410s.md`](07-api-ld2410s.md) — S has a different config model
- [`method-coverage.md`](method-coverage.md) — opcode reference
