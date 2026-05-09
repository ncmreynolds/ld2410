/*
 * distanceResolution — LD2410C-only command 0xAA / 0xAB.
 *
 * The LD2410C has two distance-quantisation modes:
 *   index 0x0000 -> 0.75 m per gate  (factory default, 9 gates × 0.75 = 6.75 m max)
 *   index 0x0001 -> 0.20 m per gate  (finer near-field, 9 × 0.20 = 1.80 m max)
 *
 * The setting is non-volatile and takes effect after radar.requestRestart().
 * Without restart the radar acknowledges the write but keeps reporting at
 * the old resolution.
 *
 * This sketch demonstrates the full flow:
 *   1. read current resolution
 *   2. flip it
 *   3. restart
 *   4. confirm the readback matches the new value
 *   5. flip back to the original (so the bench is left untouched)
 *
 * Compatible with: LD2410C only (opcode does not exist on base nor S).
 */

#define LD2410_VARIANT_C

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

static const __FlashStringHelper* resolutionLabel(uint16_t idx) {
  if (idx == LD2410_DISTANCE_RESOLUTION_075M) return F("0.75 m/gate (max 6.75 m)");
  if (idx == LD2410_DISTANCE_RESOLUTION_02M) return F("0.20 m/gate (max 1.80 m)");
  return F("(unknown)");
}

void setup() {
  MONITOR_SERIAL.begin(115200);
  delay(500);
#if defined(ESP32)
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#else
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD);
#endif
  delay(500);

  MONITOR_SERIAL.println(F("\nLD2410C distanceResolution example"));
  if (!radar.begin(RADAR_SERIAL)) {
    MONITOR_SERIAL.println(F("radar.begin: FAIL"));
    while (true) delay(1000);
  }

  // -- Read original resolution --------------------------------------
  if (!radar.requestDistanceResolution()) {
    MONITOR_SERIAL.println(F("requestDistanceResolution: FAIL"));
    return;
  }
  const uint16_t original = radar.distance_resolution;
  MONITOR_SERIAL.print(F("Original resolution: index="));
  MONITOR_SERIAL.print(original);
  MONITOR_SERIAL.print(F(" -> "));
  MONITOR_SERIAL.println(resolutionLabel(original));

  // -- Flip to the other resolution ----------------------------------
  const uint16_t flipped = (original == LD2410_DISTANCE_RESOLUTION_075M)
                            ? LD2410_DISTANCE_RESOLUTION_02M
                            : LD2410_DISTANCE_RESOLUTION_075M;
  MONITOR_SERIAL.print(F("Setting to: "));
  MONITOR_SERIAL.println(resolutionLabel(flipped));
  if (!radar.setDistanceResolution(flipped)) {
    MONITOR_SERIAL.println(F("setDistanceResolution(flipped): FAIL"));
    return;
  }

  // The new value is persisted in NVS but only takes effect after restart.
  // requestRestart() handles the ~800 ms reboot blackout cleanly,
  // including the in-flight UART drain that protects autoReadTask users.
  MONITOR_SERIAL.println(F("Restarting radar to apply..."));
  radar.requestRestart();

  // -- Verify readback ------------------------------------------------
  if (!radar.requestDistanceResolution()) {
    MONITOR_SERIAL.println(F("requestDistanceResolution after restart: FAIL"));
    return;
  }
  MONITOR_SERIAL.print(F("After restart: index="));
  MONITOR_SERIAL.print(radar.distance_resolution);
  MONITOR_SERIAL.print(F(" -> "));
  MONITOR_SERIAL.println(resolutionLabel(radar.distance_resolution));

  // -- Restore original (so the bench config isn't permanently altered) -
  MONITOR_SERIAL.print(F("Restoring original: "));
  MONITOR_SERIAL.println(resolutionLabel(original));
  radar.setDistanceResolution(original);
  radar.requestRestart();
  radar.requestDistanceResolution();
  MONITOR_SERIAL.print(F("Final readback: index="));
  MONITOR_SERIAL.print(radar.distance_resolution);
  MONITOR_SERIAL.print(F(" -> "));
  MONITOR_SERIAL.println(resolutionLabel(radar.distance_resolution));
  MONITOR_SERIAL.println(F("Done."));
}

void loop() {
  delay(1000);
}
