/*
 * Example sketch demonstrating the FreeRTOS task-based read pattern on ESP32.
 *
 * radar.autoReadTask() spawns a background task that drains the LD2410 UART
 * continuously, so the main loop() never has to call radar.read() and never
 * blocks on UART. The loop just consumes the latest cached values via the
 * same getters used by basicSensor.
 *
 * ESP32 only — autoReadTask uses xTaskCreatePinnedToCore which is not
 * available on ESP8266/AVR. For those boards use basicSensor.ino with the
 * polling read() call.
 *
 * Pin map (same as basicSensor):
 *   ESP32     -> LD2410 TX→GPIO 32, RX→GPIO 33
 *   ESP32-S2  -> LD2410 TX→GPIO 9,  RX→GPIO 8
 *   ESP32-C3  -> LD2410 TX→GPIO 4,  RX→GPIO 5
 */

#if !defined(ESP32)
  #error "autoReadTask example requires ESP32. On ESP8266/AVR use basicSensor.ino instead."
#endif

#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
  #if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
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
#else // ESP32 before IDF 4.0
  #define RADAR_RX_PIN 32
  #define RADAR_TX_PIN 33
#endif

#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1

#include <ld2410.h>

ld2410 radar;
uint32_t lastReading = 0;
uint32_t lastConfigPoll = 0;

void setup() {
  MONITOR_SERIAL.begin(115200);
  // radar.debug(MONITOR_SERIAL);  // Uncomment for verbose library debug output
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(500);

  MONITOR_SERIAL.print(F("\nConnect LD2410 radar TX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);
  MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));

  if (!radar.begin(RADAR_SERIAL)) {
    MONITOR_SERIAL.println(F("not connected"));
    while (true) {
      delay(1000);
    }
  }
  MONITOR_SERIAL.println(F("OK"));
  MONITOR_SERIAL.print(F("LD2410 firmware version: "));
  MONITOR_SERIAL.print(radar.firmware_major_version);
  MONITOR_SERIAL.print('.');
  MONITOR_SERIAL.print(radar.firmware_minor_version);
  MONITOR_SERIAL.print('.');
  MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);

  // Spawn the background reader task. Defaults: stack=4096, priority=1,
  // core=tskNO_AFFINITY (FreeRTOS picks a free core). Override the args if
  // your sketch needs to pin to a specific core or change priority.
  if (!radar.autoReadTask()) {
    MONITOR_SERIAL.println(F("autoReadTask: failed to create FreeRTOS task"));
    while (true) {
      delay(1000);
    }
  }
  MONITOR_SERIAL.println(F("autoReadTask: running in background"));
}

void loop() {
  // Note: no radar.read() here. The background task is updating the fields
  // for us — we just consume them.
  if (radar.isConnected() && millis() - lastReading > 1000) {
    lastReading = millis();
    if (radar.presenceDetected()) {
      if (radar.stationaryTargetDetected()) {
        MONITOR_SERIAL.print(F("Stationary target: "));
        MONITOR_SERIAL.print(radar.stationaryTargetDistance());
        MONITOR_SERIAL.print(F("cm energy:"));
        MONITOR_SERIAL.print(radar.stationaryTargetEnergy());
        MONITOR_SERIAL.print(' ');
      }
      if (radar.movingTargetDetected()) {
        MONITOR_SERIAL.print(F("Moving target: "));
        MONITOR_SERIAL.print(radar.movingTargetDistance());
        MONITOR_SERIAL.print(F("cm energy:"));
        MONITOR_SERIAL.print(radar.movingTargetEnergy());
      }
      MONITOR_SERIAL.println();
    } else {
      MONITOR_SERIAL.println(F("No target"));
    }
  }

  // Demonstration: every 30 s, while autoReadTask is busy reading frames,
  // issue a configuration request. The library serialises the command on
  // top of the running task — the task keeps draining UART, the command
  // path waits on cmd_ack_seq_ for the matching ACK, and no race occurs.
  if (radar.isAutoReadTaskRunning() && millis() - lastConfigPoll > 30000) {
    lastConfigPoll = millis();
    if (radar.requestCurrentConfiguration()) {
      MONITOR_SERIAL.print(F("[config] max gates: "));
      MONITOR_SERIAL.print(radar.max_moving_gate);
      MONITOR_SERIAL.print(F(" moving / "));
      MONITOR_SERIAL.print(radar.max_stationary_gate);
      MONITOR_SERIAL.print(F(" stationary; idle "));
      MONITOR_SERIAL.print(radar.sensor_idle_time);
      MONITOR_SERIAL.println('s');
    } else {
      MONITOR_SERIAL.println(F("[config] requestCurrentConfiguration failed"));
    }
  }

  // To stop the background task (e.g. before re-init or shutdown):
  //   radar.stopAutoReadTask();
}
