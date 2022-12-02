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
    delay(0);
    doEngineering=millis() + 60000;
  }
  else
  {
    Serial.println(F(" not connected"));
  }  
}

void loop()
{
  radar.ld2410_loop();
  if(millis() == doEngineering) {
    radar.requestStartEngineeringMode();
  }
  if(radar.isConnected() && millis() - lastReading > 2000)  //Report every 1000ms
  {
    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.isStationary())
      {
        Serial.printf("Stationary target: %03dcm  energy: %03d  Detection distance: %03dcm\n", radar.stationaryTargetDistance(), radar.stationaryTargetEnergy(), radar.detectionDistance());
        if(radar.isEngineeringMode()){
          Serial.printf("Moving Gate:%d, Static Gate:%d\n",radar.engMaxMovingDistanceGate(), radar.engMaxStaticDistanceGate());
          for(int x = 0; x < LD2410_MAX_GATES; ++x) {
            Serial.printf("Gate:%d, Movement Energy: %03d, Static Energy:%03d\n", x, radar.engMovingDistanceGateEnergy(x), radar.engStaticDistanceGateEnergy(x));
          }
          Serial.println("");
        }
      }
      if(radar.isMoving())
      {
        Serial.printf("Moving target: %03dcm  energy: %03d  Detection distance: %03dcm\n", radar.movingTargetDistance(), radar.movingTargetEnergy(), radar.detectionDistance());
        if(radar.isEngineeringMode()){
          Serial.printf("Moving Gate:%d, Static Gate:%d\n",radar.engMaxMovingDistanceGate(), radar.engMaxStaticDistanceGate());
          for(int x = 0; x < LD2410_MAX_GATES; ++x) {
            Serial.printf("Gate:%d, Movement Energy: %03d, Static Energy:%03d\n", x, radar.engMovingDistanceGateEnergy(x), radar.engStaticDistanceGateEnergy(x));
          }
        }
        Serial.println("");
      }
    }
    else
    {
      Serial.println(F("No target"));
    }
  }
}
