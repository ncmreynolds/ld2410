/*
 * Example sketch for the LD2410 on ESP8266.
 *
 * The ESP8266 only has 1.5 hardware UARTs (UART0 full-duplex on GPIO1/3,
 * UART1 TX-only on GPIO2). Swapping UART0 to GPIO13/15 collides with
 * strapping pin GPIO15 — the radar TX idles HIGH and would block boot.
 * This sketch therefore uses SoftwareSerial for the radar and keeps UART0
 * free for the serial monitor.
 *
 * Caveat: SoftwareSerial at 256000 baud (the radar's default) is at the
 * upper edge of EspSoftwareSerial's reliable range. Occasional frame drops
 * are expected, especially during dense detection bursts. For production
 * use prefer a board with two hardware UARTs (ESP32, Leonardo).
 *
 * Wiring (NodeMCU / Wemos D1 mini):
 *   LD2410 TX  -> ESP8266 GPIO14 (D5)
 *   LD2410 RX  -> ESP8266 GPIO12 (D6)
 *   LD2410 GND -> GND
 *   LD2410 VCC -> 5V
 *
 * Avoid GPIO0 / GPIO2 / GPIO15 (strapping pins) and GPIO1 / GPIO3
 * (UART0, used here by the serial monitor).
 */

#if !defined(ESP8266)
  #error "basicSensorEsp8266 targets ESP8266 only. Use basicSensor for ESP32/AVR."
#endif

#include <ld2410.h>
#include <SoftwareSerial.h>

#define RADAR_RX_PIN 14
#define RADAR_TX_PIN 12

ld2410 radar;
SoftwareSerial radarSerial(RADAR_RX_PIN, RADAR_TX_PIN);

uint32_t lastReading = 0;

void setup() {
  Serial.begin(115200);
  // radar.debug(Serial);  // Uncomment for verbose library debug output

  radarSerial.begin(LD2410_DEFAULT_BAUD);
  delay(500);

  Serial.print(F("\nConnect LD2410 radar TX to GPIO:"));
  Serial.println(RADAR_RX_PIN);
  Serial.print(F("Connect LD2410 radar RX to GPIO:"));
  Serial.println(RADAR_TX_PIN);
  Serial.print(F("LD2410 radar sensor initialising: "));

  if (radar.begin(radarSerial)) {
    Serial.println(F("OK"));
    Serial.print(F("LD2410 firmware version: "));
    Serial.print(radar.firmware_major_version);
    Serial.print('.');
    Serial.print(radar.firmware_minor_version);
    Serial.print('.');
    Serial.println(radar.firmware_bugfix_version, HEX);
  } else {
    Serial.println(F("not connected"));
  }
}

void loop() {
  radar.read();
  if (radar.isConnected() && millis() - lastReading > 1000) {
    lastReading = millis();
    if (radar.presenceDetected()) {
      if (radar.stationaryTargetDetected()) {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.stationaryTargetEnergy());
        Serial.print(' ');
      }
      if (radar.movingTargetDetected()) {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.movingTargetEnergy());
      }
      Serial.println();
    } else {
      Serial.println(F("No target"));
    }
  }
}
