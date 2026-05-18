# 03 — Runtime models: synchronous vs FreeRTOS background task

The library exposes two complementary patterns for getting frames out
of the radar's UART and into your program:

| Pattern | Where it works | Who drains UART | When to use it |
|---|---|---|---|
| **Synchronous** — call `radar.read()` from `loop()` | All platforms (AVR, ESP8266, ESP32, RP2040, ATmega32u4, …) | Your `loop()` | Default. Simple, portable, no concurrency. |
| **Asynchronous** — `radar.autoReadTask()` spawns a FreeRTOS task | **ESP32 only** (and any future FreeRTOS-on-Arduino port) | Background task on its own core | When your `loop()` does heavy work that can stall UART consumption. |

Both patterns expose the **same getter API**. You do not need to change
your code that calls `presenceDetected()` / `movingTargetDistance()` /
etc. when you switch between them.

## Pattern A — synchronous

This is the default. You call `radar.read()` whenever you can in
`loop()`, and the library drains all currently available UART bytes,
parses any complete frame, and returns. Then you read the latest
values via the getters.

```cpp
void loop() {
  radar.read();                                // drain + parse
  if (radar.presenceDetected()) {
    handlePresence(radar.detectionDistance());
  }
  delay(50);                                   // anything ≤ 100 ms is fine
}
```

**Constraints to be aware of:**

- The radar emits a frame every ~50 ms at default settings. If your
  `loop()` runs slower than ~10 Hz, you'll start losing frames as the
  on-chip UART RX FIFO overflows. The library's internal circular
  buffer is sized at 4× the max frame length (256 B on base/C, 384 B
  on S), so you have a small safety margin, but `loop()` cycles in
  the seconds range will drop data.
- `radar.read()` is safe to call as often as you like. Calls with no
  pending UART bytes return immediately (just two register reads).
- It is **not safe** to call `radar.read()` from an ISR. Use the
  background task instead, or set a flag and read in `loop()`.

Working example:
[`../examples/basicSensor/basicSensor.ino`](../examples/basicSensor/basicSensor.ino).

## Pattern B — FreeRTOS background task (ESP32)

On ESP32 you can offload UART draining + parsing to a FreeRTOS task
that runs on a separate core. The task pumps the radar continuously
and updates the library's internal field cache; your `loop()` just
reads the cache via the same getters.

```cpp
#include <ld2410.h>
ld2410 radar;

void setup() {
  Serial.begin(115200);
  Serial1.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  radar.begin(Serial1);

  // Defaults: stack=4096, priority=1, core=tskNO_AFFINITY (FreeRTOS picks).
  // Override if you need a specific core or priority.
  if (!radar.autoReadTask(/*stack=*/4096, /*priority=*/1, /*core=*/0)) {
    Serial.println("autoReadTask: FAIL");
    while (true) delay(1000);
  }
}

void loop() {
  // No radar.read() here — the task is doing it.
  if (radar.presenceDetected()) {
    handlePresence(radar.detectionDistance());
  }
  delay(500);                                  // your business logic, no UART concerns
}

void teardown() {
  radar.stopAutoReadTask();                    // optional, idempotent
}
```

Working example:
[`../examples/autoReadTask/autoReadTask.ino`](../examples/autoReadTask/autoReadTask.ino).

### When to pick task over polling

- **Your `loop()` blocks for hundreds of ms or more** (heavy WiFi
  transactions, slow display refresh, MQTT publish, …) and is dropping
  frames.
- **You need consistent latency** between a target appearing and your
  business logic noticing — the task wakes ~every 10 ms regardless of
  what `loop()` is doing.
- **You're on ESP32 anyway** and want the simpler "read like a sensor
  that auto-updates itself" mental model.

### When to stick with polling

- **You're on AVR / ESP8266 / RP2040** — there is no FreeRTOS
  multi-core scheduling on these. (Note: the task body uses
  `xTaskCreatePinnedToCore` which is ESP-IDF specific; building on
  non-ESP32 platforms with `autoReadTask()` calls would fail at link
  time, so the helper is `#ifdef`-gated to ESP32.)
- **Your `loop()` is already fast** (≤ 50 ms) — the task adds ~80 B
  RAM (FreeRTOS task control block) and one small stack, which may
  matter on tight builds.

## The dual-core race (and how the library solves it)

Pattern B has a subtle correctness issue that doesn't exist in pattern
A: on ESP32 the task runs on one core while `loop()` runs on the
other. Both can touch the field cache at any moment. If the task is
mid-write when `loop()` reads the getters one by one:

```cpp
// BAD pattern under autoReadTask:
bool present  = radar.presenceDetected();      // T0  — frame N
uint16_t mov  = radar.movingTargetDistance();  // T0+ε  — could be N+1
uint8_t  enrg = radar.movingTargetEnergy();    // T0+2ε  — could be N+1
```

You can observe an "impossible" combination like `target_type =
MOVING+STATIONARY` together with `moving_distance = 0`. The race is
rare (~1 / 1000 reads at 20 Hz frame rate) but real.

The fix is `snapshotTargetState()`, which copies all six fields under
one critical section:

```cpp
LD2410TargetState snap;
radar.snapshotTargetState(snap);     // atomic
if (snap.target_type & 0x01) {
  handleMoving(snap.moving_distance, snap.moving_energy);
}
```

The same pattern exists for the per-gate engineering arrays:

```cpp
uint8_t mot[LD2410_GATE_COUNT], sta[LD2410_GATE_COUNT];
radar.snapshotEngineeringMotionEnergies(mot);
radar.snapshotEngineeringStationaryEnergies(sta);
```

These methods are described in [`08-advanced.md`](08-advanced.md) and
the dedicated example
[`../examples/snapshotAtomic/snapshotAtomic.ino`](../examples/snapshotAtomic/snapshotAtomic.ino).

On non-ESP32 platforms the lock is a no-op (single-thread assumption)
and using the snapshot API costs nothing — it's still good practice
because the same code then ports cleanly to ESP32 if you ever need it.

## Switching between patterns at runtime

You can start and stop the task on the fly without re-initialising the
radar:

```cpp
radar.autoReadTask();          // task takes over UART draining
// ... user loop runs, no radar.read() needed
radar.stopAutoReadTask();      // task goes away
// ... back to calling radar.read() from loop()
```

[`isAutoReadTaskRunning()`](04-api-core.md#isautoreadtaskrunning) lets
you query the current state, useful for libraries that need to gate
behaviour on whether they're in async mode.

The `tests/hw/ld2410c_modes_switch` sketch (under `tests/hw/`, not in
`examples/` because it's primarily a hardware regression test) is a
useful reference for the transition order between the four
sync/async × normal/engineering states.
