/*
 * Example sketch for reporting on readings from the LD2410 using whatever settings are currently configured.
 * 
 * The sketch assumes an ESP32 board with the LD2410 connected as Serial1 to pins 8 & 9, the serial configuration for other boards may vary
 * 
 */

#include <ld2410.h>

#define RXD2 16 // 8
#define TXD2 17 // 9

ld2410 radar;

uint32_t lastReading = 0;

uint32_t doConfig = 0;
uint32_t doEngineering = 0;

void setup(void)
{
  delay(1000);
  Serial.begin(115200); //Feedback over Serial Monitor
  delay(100);
  radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
  Serial2.begin (256000, SERIAL_8N1, RXD2, TXD2); //UART for monitoring the radar rx, tx
  delay(100);
  Serial.println(F("\nLD2410 radar sensor initialising: "));
  if(radar.begin(Serial2))
  {
    Serial.println(F("OK "));
    doConfig=millis() + 5000;
    doEngineering=millis() + 10000;
  }
  else
  {
    Serial.println(F(" not connected"));
  }  
}

void loop()
{
  radar.read();
  if(millis()==doConfig) {
    radar.requestCurrentConfiguration();
  }
  if(millis() == doEngineering) {
    radar.requestStartEngineeringMode();
  }
  if(radar.isConnected() && millis() - lastReading > 2000)  //Report every 1000ms
  {
    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.stationaryTargetDistance())
      {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.println(radar.stationaryTargetEnergy());
      }
      if(radar.movingTargetDistance())
      {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.println(radar.movingTargetEnergy());
      }
    }
    else
    {
      Serial.println(F("No target"));
    }
  }
}
