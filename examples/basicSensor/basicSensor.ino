/*
 * Example scketh for reporting on readings from the LD2410 using whatever settings are currently configured.
 * 
 * The sketch assumes an ESP32 board with the LD2410 connected as Serial1 to pins 8 & 9, the serial configuration for other boards may vary
 * 
 */

#include <ld2410.h>

ld2410 radar;

void setup(void)
{
  Serial.begin(115200); //Feedback over Serial Monitor
  //radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor
  Serial1.begin (256000, SERIAL_8N1, 9, 8); //UART for monitoring the radar
  delay(500);
  Serial.println(F("\nLD2410 radar sensor initialising: "));
  if(radar.begin(Serial1))
  {
    Serial.println(F("OK"));
  }
  else
  {
    Serial.println(F("not connected"));
  }
}

void loop()
{
  if(radar.read())  //Some data has been received from the radar
  {
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
  }
}
