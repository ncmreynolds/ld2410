# 04 — Core API (all variants)

Methods on `class ld2410` that work identically on **all four**
variants (LD2410 base, LD2410B, LD2410C, LD2410S). If you only need
presence, distance and per-gate energy, every method you'll call
lives here.

For variant-specific extensions see
[`05-api-ld2410-base.md`](05-api-ld2410-base.md),
[`06-api-ld2410c.md`](06-api-ld2410c.md),
[`07-api-ld2410s.md`](07-api-ld2410s.md).

---

## Lifecycle

### `bool begin(Stream& stream, bool waitForRadar = true)`

Bind the radar to a `Stream`-derived UART (typically `Serial1`,
`Serial2`, or a `SoftwareSerial` instance) and optionally block until
the first firmware ACK arrives (≤ 1 s timeout). On ESP32 it also
creates the internal command-mutex used by `autoReadTask`.

Returns `true` if `waitForRadar=false` (always) or if the radar
responded to a `requestFirmwareVersion` within 1 s. Returns `false`
on timeout.

```cpp
// 1. Default — block until firmware ACK or 1 s timeout
ld2410 radar;
RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RX, TX);
if (!radar.begin(RADAR_SERIAL)) {
  Serial.println("radar offline");
}

// 2. Non-blocking init — if the radar might be powered up later
radar.begin(RADAR_SERIAL, /*waitForRadar=*/false);

// 3. With debug stream attached BEFORE begin so the firmware probe is logged
radar.debug(Serial);
radar.begin(RADAR_SERIAL);
```

Example: every `examples/*` sketch uses this in `setup()`.

### `bool isConnected()`

Returns `true` if a frame was parsed in the last `radar_uart_timeout`
(default 500 ms). If the cached state is stale, attempts one
`read_frame_()` synchronously before reporting.

```cpp
// 1. Health check in loop
if (!radar.isConnected()) {
  Serial.println("LOSS OF SIGNAL");
}

// 2. Reconnection probe with backoff
static uint32_t nextProbe = 0;
if (millis() > nextProbe && !radar.isConnected()) {
  nextProbe = millis() + 5000;
  // ... try power-cycle
}

// 3. Use as a read-or-skip gate before snapshotTargetState()
if (radar.isConnected()) { LD2410TargetState s; radar.snapshotTargetState(s); use(s); }
```

### `void debug(Stream& stream)`

Attach a stream that the library writes diagnostic messages to. By
default no diagnostics are emitted (silent). Adding a stream **does
not** enable per-frame parse traces; for that, define
`LD2410_DEBUG_PARSE` or `LD2410_DEBUG_COMMANDS` at compile time
(see [`08-advanced.md`](08-advanced.md#debug-flags)).

```cpp
// 1. Common: log to the same monitor used by your sketch
radar.debug(Serial);

// 2. Log to a different UART (separate from the user-facing console)
radar.debug(Serial2);

// 3. Disable runtime diagnostics by passing a NULL-ish stream is NOT
//    supported — simply don't call debug() if you want silence.
```

---

## Reading frames

### `bool read()`

Drain pending bytes from the radar UART into the library's circular
buffer, attempt to parse one complete frame, and return `true` if a
new frame was parsed (or new bytes arrived). Non-blocking; safe to
call as often as you like in `loop()`.

```cpp
// 1. Standard polling loop
void loop() { radar.read(); doStuffWith(radar); }

// 2. Throttled read — only check radar at 20 Hz
static uint32_t next = 0;
if (millis() >= next) { next = millis() + 50; radar.read(); }

// 3. Drain-only sweep before issuing a synchronous command
while (radar.read()) { /* drain whatever is buffered */ }
```

Example: [`../examples/basicSensor/basicSensor.ino`](../examples/basicSensor/basicSensor.ino).

### `bool autoReadTask(uint32_t stack=4096, UBaseType_t priority=1, BaseType_t core=tskNO_AFFINITY)`

ESP32-only. Spawns a FreeRTOS task that drains UART + parses frames
in the background, so you never need to call `read()` from `loop()`.
Returns `true` if the task was created (or is already running).
On non-ESP32 platforms the method declaration is hidden behind
`#if defined(ESP32)` in `src/ld2410.h` — calling it produces a
**compile error** ("`autoReadTask` is not a member of `ld2410`").
Wrap your usage in `#if defined(ESP32)` (or check `defined(ESP32)`
at the top of your sketch) if you need to share code across cores.

```cpp
// 1. Default — let FreeRTOS pick a core
radar.autoReadTask();

// 2. Pin to core 0, priority 2 (priority 1 is the IDLE task on each core)
radar.autoReadTask(/*stack=*/4096, /*priority=*/2, /*core=*/0);

// 3. Bigger stack if your callbacks (e.g. via getters) chain into heavy code
radar.autoReadTask(/*stack=*/8192);
```

Example: [`../examples/autoReadTask/autoReadTask.ino`](../examples/autoReadTask/autoReadTask.ino).

### `void stopAutoReadTask()`

Idempotent. Tears down the FreeRTOS task. After return, you must
resume calling `read()` from `loop()` if you want fresh frames.

### `bool isAutoReadTaskRunning()`

Returns whether the background task is currently active. Useful when
your code is variant- or runtime-mode-aware. On non-ESP32 platforms,
always returns `false`.

---

## Target state — basic getters

These five getters report the latest values from a parsed frame. They
are non-blocking and never fail; if no frame has arrived yet they
return zeros.

> ⚠️ Each getter reads **one** field with no lock. Under
> `autoReadTask` on ESP32 dual-core, calling several getters in
> sequence can observe a torn snapshot. For coherent reads use
> [`snapshotTargetState()`](#snapshottargetstate) below.

### `bool presenceDetected()`

`true` if the radar reports any target (moving or stationary).

```cpp
if (radar.presenceDetected()) { triggerLight(); }
if (!radar.presenceDetected()) { startSleepTimer(); }
const bool occupied = radar.presenceDetected();
```

### `bool movingTargetDetected()`

`true` if a moving target with non-zero distance and energy is
present. Stricter than `presenceDetected()` — filters out the
stationary-only case.

### `uint16_t movingTargetDistance()`

Distance to the moving target, in cm. Returns 0 if no moving target.

### `uint8_t movingTargetEnergy()`

Energy of the moving target, **clamped to 0..100**. Useful as a
percent-style indicator.

### `bool stationaryTargetDetected()`

Mirror of `movingTargetDetected()` for the stationary track.

### `uint16_t stationaryTargetDistance()` / `uint8_t stationaryTargetEnergy()`

Mirror of the moving-target getters.

### `uint16_t detectionDistance()`

Distance reported by the radar's primary detector — the most
"trustworthy" single-number distance. Use this when you don't care
about the moving / stationary split.

```cpp
// 1. Single-number presence + distance
if (radar.presenceDetected()) Serial.println(radar.detectionDistance());

// 2. Compare against a threshold
if (radar.detectionDistance() < 100) closeRange();

// 3. Use only the detection distance, ignore the per-track values
const uint16_t d = radar.detectionDistance();
publish(d);
```

Example: [`../examples/basicSensor/basicSensor.ino`](../examples/basicSensor/basicSensor.ino).

---

## Target state — atomic snapshot

### `void snapshotTargetState(LD2410TargetState& out)`

Copy all six basic-target fields into `out` under one critical
section. Use this instead of calling the individual getters when
multiple fields must be read coherently — the only safe option under
`autoReadTask` on ESP32 dual-core.

```cpp
struct LD2410TargetState {
  uint8_t  target_type;
  uint16_t moving_distance;
  uint8_t  moving_energy;
  uint16_t stationary_distance;
  uint8_t  stationary_energy;
  uint16_t detection_distance;
};
```

```cpp
// 1. Race-safe replacement for the individual getters
LD2410TargetState s;
radar.snapshotTargetState(s);
if (s.target_type & 0x01) handleMoving(s.moving_distance, s.moving_energy);

// 2. Snapshot once, log many fields without re-acquiring the lock
LD2410TargetState s; radar.snapshotTargetState(s);
log("type=%u mov=%u/%u sta=%u/%u dd=%u",
    s.target_type, s.moving_distance, s.moving_energy,
    s.stationary_distance, s.stationary_energy, s.detection_distance);

// 3. Diff two snapshots to detect frame-to-frame change
static LD2410TargetState prev{};
LD2410TargetState curr; radar.snapshotTargetState(curr);
if (curr.detection_distance != prev.detection_distance) emitChange(prev, curr);
prev = curr;
```

Example: [`../examples/snapshotAtomic/snapshotAtomic.ino`](../examples/snapshotAtomic/snapshotAtomic.ino).

---

## Engineering mode — per-gate energy

These getters expose the per-gate energy block emitted by the radar
in engineering mode. On base/C, engineering mode is opt-in — see
[`requestStartEngineeringMode()` (05)](05-api-ld2410-base.md#requeststartengineeringmode). On S, the per-gate block is part of the standard
0x01 frame and is always parsed.

### `bool engineeringRetrieved()`

`true` if the library has parsed at least one frame containing a
per-gate energy block since `begin()`. Acts as a "data is present"
flag for the per-gate getters and snapshots.

### `uint8_t movingEnergyAtGate(uint8_t gate)` / `uint8_t stationaryEnergyAtGate(uint8_t gate)`

Energy for a single gate, by index `0..LD2410_GATE_COUNT-1`
(9 on base/C, 16 on S). Returns 0 for out-of-range indices.

```cpp
// 1. Print the closest 3 gates
for (uint8_t g = 0; g < 3; g++) Serial.println(radar.movingEnergyAtGate(g));

// 2. Find peak gate
uint8_t peak = 0;
for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++)
  if (radar.movingEnergyAtGate(g) > radar.movingEnergyAtGate(peak)) peak = g;

// 3. Threshold-based detection on a specific gate
if (radar.stationaryEnergyAtGate(4) > 50) maybeOccupied();
```

### `void snapshotEngineeringMotionEnergies(uint8_t out[LD2410_GATE_COUNT])` / `void snapshotEngineeringStationaryEnergies(uint8_t out[LD2410_GATE_COUNT])`

Atomic snapshot of the per-gate arrays under one critical section.
Race-safe under `autoReadTask`. The two methods can be called back
to back; each is one frame's worth of data.

```cpp
uint8_t mot[LD2410_GATE_COUNT], sta[LD2410_GATE_COUNT];
radar.snapshotEngineeringMotionEnergies(mot);
radar.snapshotEngineeringStationaryEnergies(sta);

// 1. CSV dump
for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
  Serial.print(mot[g]); Serial.print(',');
}
Serial.println();

// 2. Sum total energy across all gates
uint32_t total = 0;
for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) total += mot[g] + sta[g];

// 3. Compare snapshots to derive a "movement vs settled" classifier
uint8_t prev[LD2410_GATE_COUNT] = {};
uint8_t now[LD2410_GATE_COUNT];
radar.snapshotEngineeringMotionEnergies(now);
int delta = 0;
for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) delta += abs((int)now[g] - (int)prev[g]);
memcpy(prev, now, sizeof(prev));
```

Example: [`../examples/engineeringMode/engineeringMode.ino`](../examples/engineeringMode/engineeringMode.ino).

---

## Direct frame access (advanced)

### `FrameData getFrameData() const`

Returns the most recent valid frame as `{ const uint8_t* data; uint16_t length; }`.
Both fields are zero if no valid frame is available. Useful for
forwarding raw frames to another consumer (e.g. logging, offline
analysis, sniffing).

See [`08-advanced.md`](08-advanced.md#getframedata) for usage examples.

---

## Public fields

A few non-method fields are read directly:

| Field | Type | Set by | Notes |
|---|---|---|---|
| `firmware_major_version` | `uint8_t` | `requestFirmwareVersion()` | High byte of major.minor |
| `firmware_minor_version` | `uint8_t` | `requestFirmwareVersion()` | Low byte of major.minor |
| `firmware_bugfix_version` | `uint32_t` | `requestFirmwareVersion()` | 4-byte bugfix on base/C, 16-bit zero-extended on S |
| `mac_address[6]` | `uint8_t[]` | `requestMACAddress()` (LD2410C) | Big-endian wire order |
| `serial_number[8]` | `uint8_t[]` | `requestSerialNumber()` (LD2410S) | Wire order |
| (and several others — see `src/ld2410.h`) | | | |

The full list is in `src/ld2410.h` — these are intentionally public to
keep the API ergonomic for short sketches.

---

## See also

- [`05-api-ld2410-base.md`](05-api-ld2410-base.md) — base + C config commands
- [`06-api-ld2410c.md`](06-api-ld2410c.md) — C-only extensions
- [`07-api-ld2410s.md`](07-api-ld2410s.md) — S-only extensions
- [`08-advanced.md`](08-advanced.md) — `getFrameData`, debug flags
- [`method-coverage.md`](method-coverage.md) — opcode-by-opcode capability matrix
