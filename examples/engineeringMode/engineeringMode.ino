/*
 * engineeringMode — request the LD2410 to emit per-gate energy frames.
 *
 * In engineering mode the radar adds a 9-byte (base/C) or 16-byte (S)
 * per-gate energy block to each data frame. This sketch:
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
 * Compatible with: LD2410 base, LD2410C. Not applicable on LD2410S
 * (engineering data on S is delivered as part of the standard 0x01
 * frame and parsed automatically — no opcode toggle needed).
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

  // Engineering mode is volatile: it resets when the radar reboots,
  // so we re-enable on every setup(). The command takes ~50 ms inside
  // a configuration window managed by the helper.
  if (radar.requestStartEngineeringMode()) {
    MONITOR_SERIAL.println(F("requestStartEngineeringMode: OK"));
  } else {
    MONITOR_SERIAL.println(F("requestStartEngineeringMode: FAIL"));
  }

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
