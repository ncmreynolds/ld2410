/*
 * Example sketch for reporting on readings from the LD2410 using whatever settings are currently configured.
 * 
 * This has been tested on the following platforms...
 * 
 * On ESP32, connect the LD2410 to GPIO pins 32&33
 * On ESP32S2, connect the LD2410 to GPIO pins 8&9
 * On ESP32C3, connect the LD2410 to GPIO pins 4&5
 * On Arduino Leonardo or other ATmega32u4 board connect the LD2410 to GPIO pins TX & RX hardware serial
 * On AVR128DA32 (SpenceKonde DxCore), connect the LD2410 to USART1 = pins PC0 (TX) & PC1 (RX);
 *   the monitor uses Serial = USART0 on PORTA (PA0/PA1).
 * 
 * The serial configuration for other boards will vary and you'll need to assign them yourself
 * 
 * For ESP8266, see the basicSensorEsp8266 sibling example — it uses
 * SoftwareSerial because the ESP8266's hardware UARTs cannot host both the
 * radar and the serial monitor without colliding with strapping pins.
 *
 * For this sketch the board needs two usable hardware UARTs.
 * 
 */

#if defined(ESP32)
  #ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
    #if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
      #define MONITOR_SERIAL Serial
      #define RADAR_SERIAL Serial1
      #define RADAR_RX_PIN 32
      #define RADAR_TX_PIN 33
    #elif CONFIG_IDF_TARGET_ESP32S2
      #define MONITOR_SERIAL Serial
      #define RADAR_SERIAL Serial1
      #define RADAR_RX_PIN 9
      #define RADAR_TX_PIN 8
    #elif CONFIG_IDF_TARGET_ESP32C3
      #define MONITOR_SERIAL Serial
      #define RADAR_SERIAL Serial1
      #define RADAR_RX_PIN 4
      #define RADAR_TX_PIN 5
    #else 
      #error Target CONFIG_IDF_TARGET is not supported
    #endif
  #else // ESP32 Before IDF 4.0
    #define MONITOR_SERIAL Serial
    #define RADAR_SERIAL Serial1
    #define RADAR_RX_PIN 32
    #define RADAR_TX_PIN 33
  #endif
#elif defined(__AVR_ATmega32U4__)
  #define MONITOR_SERIAL Serial
  #define RADAR_SERIAL Serial1
  #define RADAR_RX_PIN 0
  #define RADAR_TX_PIN 1
#elif defined(ARDUINO_ARCH_RP2040)
  // Earle Philhower's arduino-pico core (rp2040:rp2040). Serial1 on
  // the Pico defaults to UART0 (GP0 TX / GP1 RX), so the radar pins
  // are 1 (RX) and 0 (TX) on the host side. Note the SDK accepts
  // RADAR_SERIAL.begin(baud) without a pin override; the user should
  // call setRX/setTX before begin if they need a different pinout.
  #define MONITOR_SERIAL Serial
  #define RADAR_SERIAL Serial1
  #define RADAR_RX_PIN 1
  #define RADAR_TX_PIN 0
#elif defined(DXCORE) || defined(__AVR_AVR128DA32__)
  // SpenceKonde DxCore (DxCore:megaavr). AVR128DA32 exposes three
  // hardware USARTs: Serial = USART0 (PORTA: PA0=TX, PA1=RX),
  // Serial1 = USART1 (PORTC: PC0=TX, PC1=RX), Serial2 = USART2
  // (PORTF: PF0=TX, PF1=RX). Use Serial for the monitor and Serial1
  // for the radar — the same convention as ATmega32U4 / RP2040.
  // RADAR_RX_PIN / _TX_PIN are HOST-side, so RADAR_RX_PIN is the
  // host's RX (PC1, wired to the radar's TX).
  #define MONITOR_SERIAL Serial
  #define RADAR_SERIAL Serial1
  #define RADAR_RX_PIN PIN_PC1
  #define RADAR_TX_PIN PIN_PC0
#endif

#include <ld2410.h>

ld2410 radar;

uint32_t lastReading = 0;
bool radarConnected = false;

void setup(void)
{
  MONITOR_SERIAL.begin(115200); //Feedback over Serial Monitor
  //radar.debug(MONITOR_SERIAL); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
  #if defined(ESP32)
    RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar
  #elif defined(__AVR_ATmega32U4__)
    RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD); //UART for monitoring the radar
  #elif defined(ARDUINO_ARCH_RP2040)
    // arduino-pico: setRX/setTX must come BEFORE begin(); the pin macros
    // at the top of this file resolve to GP1 (RX) and GP0 (TX) for UART0.
    RADAR_SERIAL.setRX(RADAR_RX_PIN);
    RADAR_SERIAL.setTX(RADAR_TX_PIN);
    RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD); //UART for monitoring the radar
  #elif defined(DXCORE) || defined(__AVR_AVR128DA32__)
    // DxCore: Serial1 is wired to USART1 on PORTC by default; no
    // per-call pin override needed (DxCore exposes setPins() but the
    // default mapping matches the macros above).
    RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD);
  #endif
  delay(500);
  MONITOR_SERIAL.print(F("\nConnect LD2410 radar TX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);
  MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));

  
  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println(F("OK"));
    MONITOR_SERIAL.print(F("LD2410 firmware version: "));
    MONITOR_SERIAL.print(radar.firmware_major_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.print(radar.firmware_minor_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    MONITOR_SERIAL.println(F("not connected"));
  }
}

void loop()
{
  radar.read();
  if(radar.isConnected() && millis() - lastReading > 1000)  //Report every 1000ms
  {
    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.stationaryTargetDetected())
      {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.stationaryTargetEnergy());
        Serial.print(' ');
      }
      if(radar.movingTargetDetected())
      {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.movingTargetEnergy());
      }
      Serial.println();
    }
    else
    {
      Serial.println(F("No target"));
    }
  }
}
