/*
 * engineeringMode — request the LD2410 / LD2410C to emit per-gate
 * energy frames.
 *
 * On base/C, engineering mode is opt-in: opcode 0x62 enables a
 * 9-byte motion + 9-byte stationary energy block appended to each
 * basic frame, and 0x63 disables it again. This sketch:
 *   1. starts engineering mode via requestStartEngineeringMode()
 *   2. polls read() in loop()
 *   3. when engineeringRetrieved() flips true, snapshots the two
 *      per-gate arrays atomically and prints them as a CSV-like row
 *
 * The snapshotEngineering*() accessors copy the arrays under
 * portENTER_CRITICAL on ESP32; on single-thread platforms they
 * degenerate to plain memcpy. Either way they avoid torn reads from
 * autoReadTask in concurrent setups (see the snapshotAtomic example).
 *
 * Compatible with: LD2410 base, LD2410C only. The 0x62/0x63 toggles
 * do NOT exist on LD2410S — on S, the per-gate energies are part of
 * the standard 0x01 frame and are parsed automatically as long as
 * the output mode is "standard" (the default). For an LD2410S there
 * is nothing to enable; just call snapshotEngineering*() once
 * engineeringRetrieved() returns true.
 *
 * Pin map (same convention as basicSensor):
 *   ESP32     -> radar TX→GPIO 32, RX→GPIO 33
 *   ESP32-S2  -> radar TX→GPIO 9,  RX→GPIO 8
 *   ESP32-C3  -> radar TX→GPIO 4,  RX→GPIO 5
 *   RP2040    -> radar TX→GPIO 1,  RX→GPIO 0
 *   Leonardo  -> radar TX/RX hardware Serial1
 */

#if defined(ESP32)
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
#elif defined(__AVR_ATmega32U4__)
  #define MONITOR_SERIAL Serial
  #define RADAR_SERIAL   Serial1
#elif defined(ARDUINO_ARCH_RP2040)
  #define MONITOR_SERIAL Serial
  #define RADAR_SERIAL   Serial1
  #define RADAR_RX_PIN 1
  #define RADAR_TX_PIN 0
#endif

#include <ld2410.h>

ld2410 radar;
uint32_t lastPrint = 0;

void setup() {
  MONITOR_SERIAL.begin(115200);
  delay(500);
#if defined(ESP32)
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#elif defined(ARDUINO_ARCH_RP2040)
  RADAR_SERIAL.setRX(RADAR_RX_PIN);
  RADAR_SERIAL.setTX(RADAR_TX_PIN);
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD);
#else
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD);
#endif
  delay(500);

  MONITOR_SERIAL.println(F("\nLD2410 engineering-mode example"));
  if (!radar.begin(RADAR_SERIAL)) {
    MONITOR_SERIAL.println(F("radar.begin: FAIL — check wiring"));
    while (true) delay(1000);
  }
  MONITOR_SERIAL.println(F("radar.begin: OK"));

#if defined(LD2410_HAS_ENGINEERING_MODE)
  // base/C: engineering mode is volatile (resets when the radar reboots),
  // so we re-enable on every setup(). The command takes ~50 ms inside a
  // configuration window managed by the helper.
  if (radar.requestStartEngineeringMode()) {
    MONITOR_SERIAL.println(F("requestStartEngineeringMode: OK"));
  } else {
    MONITOR_SERIAL.println(F("requestStartEngineeringMode: FAIL"));
  }
#else
  // S: per-gate energies are part of the standard 0x01 frame; nothing
  // to enable. As long as the output mode is "standard" (the default
  // out-of-the-box) snapshotEngineering*() will be populated within
  // a few frames.
  MONITOR_SERIAL.println(F("LD2410S: per-gate data is in the standard frame, no toggle required."));
#endif

  MONITOR_SERIAL.print(F("Per-gate columns: "));
  for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
    MONITOR_SERIAL.print(F("g")); MONITOR_SERIAL.print(g);
    if (g + 1 < LD2410_GATE_COUNT) MONITOR_SERIAL.print(',');
  }
  MONITOR_SERIAL.println();
}

void loop() {
  radar.read();

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    if (!radar.engineeringRetrieved()) {
      MONITOR_SERIAL.println(F("(no engineering frame yet)"));
      return;
    }

    // Take an atomic snapshot before formatting — preserves coherency
    // even if a future change moves to the autoReadTask pattern.
    uint8_t mot[LD2410_GATE_COUNT], sta[LD2410_GATE_COUNT];
    radar.snapshotEngineeringMotionEnergies(mot);
    radar.snapshotEngineeringStationaryEnergies(sta);

    MONITOR_SERIAL.print(F("motion=["));
    for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
      MONITOR_SERIAL.print(mot[g]);
      if (g + 1 < LD2410_GATE_COUNT) MONITOR_SERIAL.print(',');
    }
    MONITOR_SERIAL.print(F("] stationary=["));
    for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
      MONITOR_SERIAL.print(sta[g]);
      if (g + 1 < LD2410_GATE_COUNT) MONITOR_SERIAL.print(',');
    }
    MONITOR_SERIAL.println(']');
  }
}
