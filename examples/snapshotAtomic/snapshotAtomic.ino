/*
 * snapshotAtomic — race-safe getter pattern for ESP32 dual-core.
 *
 * On ESP32 the autoReadTask() background task runs on one core (typically
 * core 0) and writes the parsed-frame fields under a portMUX critical
 * section. The user loop() runs on the other core. Calling the individual
 * getters (movingTargetDistance(), movingTargetEnergy(), ...) from the
 * user loop reads each field independently with no lock — so a concurrent
 * autoReadTask write can produce a torn snapshot where target_type already
 * reflects the new frame but distance/energy still hold the previous one.
 *
 * The race is rare (~1/1000 reads on a busy radar) but real, and shows up
 * as "impossible" combinations like target_type=BOTH with moving_distance=0.
 *
 * snapshotTargetState() copies all six basic-target fields into a struct
 * inside a single critical section, eliminating the inter-field race.
 * snapshotEngineeringMotionEnergies/StationaryEnergies do the same for
 * the per-gate arrays.
 *
 * This sketch demonstrates the correct pattern. ESP32 only — on
 * single-thread platforms (AVR, ESP8266) the lock is a no-op and the
 * race cannot occur, but using the snapshot API is still good practice
 * because the same code then ports cleanly to ESP32.
 */

#if !defined(ESP32)
  #error "snapshotAtomic example illustrates the dual-core race fix and is ESP32-only."
#endif

#ifdef ESP_IDF_VERSION_MAJOR
  #if CONFIG_IDF_TARGET_ESP32
    #define RADAR_RX_PIN 32
    #define RADAR_TX_PIN 33
  #elif CONFIG_IDF_TARGET_ESP32S2
    #define RADAR_RX_PIN 9
    #define RADAR_TX_PIN 8
  #elif CONFIG_IDF_TARGET_ESP32C3
    #define RADAR_RX_PIN 4
    #define RADAR_TX_PIN 5
  #else
    #error Target CONFIG_IDF_TARGET is not supported
  #endif
#else
  #define RADAR_RX_PIN 32
  #define RADAR_TX_PIN 33
#endif

#define MONITOR_SERIAL Serial
#define RADAR_SERIAL   Serial1

#include <ld2410.h>

ld2410 radar;
uint32_t lastPrint = 0;

void setup() {
  MONITOR_SERIAL.begin(115200);
  delay(500);
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(500);

  MONITOR_SERIAL.println(F("\nLD2410 snapshotAtomic example (ESP32 dual-core)"));
  if (!radar.begin(RADAR_SERIAL)) {
    MONITOR_SERIAL.println(F("radar.begin: FAIL"));
    while (true) delay(1000);
  }

  // Spawn the FreeRTOS task that drains UART continuously. Once started,
  // calling radar.read() from loop() is unnecessary (and may even race
  // for buffer access — leave it to the task).
  if (!radar.autoReadTask()) {
    MONITOR_SERIAL.println(F("autoReadTask: FAIL"));
    while (true) delay(1000);
  }
  MONITOR_SERIAL.println(F("autoReadTask: OK — reading from background core"));
}

void loop() {
  // No radar.read() here — the autoReadTask is doing it for us.
  // Just sleep + read the latest cached values atomically.

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    // -- Race-safe snapshot of basic target state ----------------------
    LD2410TargetState snap;
    radar.snapshotTargetState(snap);

    MONITOR_SERIAL.print(F("[atomic] type="));
    MONITOR_SERIAL.print(snap.target_type, BIN);
    MONITOR_SERIAL.print(F(" mov d/e="));
    MONITOR_SERIAL.print(snap.moving_distance);
    MONITOR_SERIAL.print('/');
    MONITOR_SERIAL.print(snap.moving_energy);
    MONITOR_SERIAL.print(F(" sta d/e="));
    MONITOR_SERIAL.print(snap.stationary_distance);
    MONITOR_SERIAL.print('/');
    MONITOR_SERIAL.print(snap.stationary_energy);
    MONITOR_SERIAL.print(F(" dd="));
    MONITOR_SERIAL.print(snap.detection_distance);
    MONITOR_SERIAL.println(F("cm"));

    // -- BAD pattern (commented) --------------------------------------
    // The block below ALSO works most of the time, but each getter call
    // is an independent read against the live fields. A frame update
    // landing between two getters can produce a torn snapshot:
    //
    //    bool present  = radar.presenceDetected();
    //    uint16_t mov  = radar.movingTargetDistance();   // could be old
    //    uint8_t  enrg = radar.movingTargetEnergy();     // could be new
    //    if (present && mov == 0 && enrg > 0) { /* impossible? not really */ }
    //
    // Replace with snapshotTargetState(snap) and read snap.* fields,
    // which are guaranteed coherent with each other.
  }
}
