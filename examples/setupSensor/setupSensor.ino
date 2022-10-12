/*
 * Example sketch to show using configuration commands on the LD2410.
 * 
 * The sketch assumes an ESP32 board with the LD2410 connected as Serial1 to pins 8 & 9, the serial configuration for other boards may vary
 * 
 */

#include <ld2410.h>

ld2410 radar;
bool engineeringMode = false;
String command;

void setup(void)
{
  Serial.begin(115200); //Feedback over Serial Monitor
  delay(500); //Give a while for Serial Monitor to wake up
  radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor
  Serial1.begin (256000, SERIAL_8N1, 9, 8); //UART for monitoring the radar
  delay(500);
  Serial.print(F("\nLD2410 radar sensor initialising: "));
  if(radar.begin(Serial1))
  {
    Serial.println(F("OK"));
  }
  else
  {
    Serial.println(F("not connected"));
  }
  Serial.println(F("Supported commands\nread: read current values from the sensor\nreadconfig: read the configuration from the sensor\nenableengineeringmode: enable engineering mode\ndisableengineeringmode: disable engineering mode\nrestart: restart the sensor\nversion: read firmware version\nfactoryreset: factory reset the sensor\n"));
}

void loop()
{
  radar.read(); //Always read frames from the sensor
  if(Serial.available())
  {
    char typedCharacter = Serial.read();
    if(typedCharacter == '\r' || typedCharacter == '\n')
    {
      command.trim();
      if(command.equals("read"))
      {
        command = "";
        Serial.print(F("Reading from sensor: "));
        if(radar.isConnected())
        {
          Serial.println(F("OK"));
          if(radar.presenceDetected())
          {
            if(radar.stationaryTargetDetected())
            {
              Serial.print(F("Stationary target: "));
              Serial.print(radar.stationaryTargetDistance());
              Serial.print(F("cm energy: "));
              Serial.println(radar.stationaryTargetEnergy());
            }
            if(radar.movingTargetDetected())
            {
              Serial.print(F("Moving target: "));
              Serial.print(radar.movingTargetDistance());
              Serial.print(F("cm energy: "));
              Serial.println(radar.movingTargetEnergy());
            }
          }
          else
          {
            Serial.println(F("nothing detected"));
          }
        }
        else
        {
          Serial.println(F("failed to read"));
        }
      }
      else if(command.equals("readconfig"))
      {
        command = "";
        Serial.print(F("Reading configuration from sensor: "));
        if(radar.requestCurrentConfiguration())
        {
          Serial.println(F("OK"));
          //Serial.print(F("v"));
          //Serial.print(radar.firmware_major_version);
          //Serial.print('.');
          //Serial.print(radar.firmware_minor_version);
          //Serial.print('.');
          //Serial.println(radar.firmware_bugfix_version);
        }
        else
        {
          Serial.println(F("Failed"));
        }
      }
      else if(command.equals("enableengineeringmode"))
      {
        command = "";
        Serial.print(F("Enabling engineering mode: "));
        if(radar.requestStartEngineeringMode())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("disableengineeringmode"))
      {
        command = "";
        Serial.print(F("Disabling engineering mode: "));
        if(radar.requestEndEngineeringMode())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("restart"))
      {
        command = "";
        Serial.print(F("Restarting sensor: "));
        if(radar.requestRestart())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("version"))
      {
        command = "";
        Serial.print(F("Requesting firmware version: "));
        if(radar.requestFirmwareVersion())
        {
          Serial.print(F("v"));
          Serial.print(radar.firmware_major_version);
          Serial.print('.');
          Serial.print(radar.firmware_minor_version);
          Serial.print('.');
          Serial.println(radar.firmware_bugfix_version);
        }
        else
        {
          Serial.println(F("Failed"));
        }
      }
      else if(command.equals("factoryreset"))
      {
        command = "";
        Serial.print(F("Factory resetting sensor: "));
        if(radar.requestFactoryReset())
        {
          Serial.println(F("OK, now restart sensor to take effect"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else
      {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        command = "";
      }
    }
    else
    {
      command += typedCharacter;
    }
  }
  /*
  if()  //Some data has been received from the radar
  {
    if(radar.presenceDetected())
    {
      Serial.print(F("Stationary target: "));
      Serial.println(radar.stationaryTargetDistance());
      Serial.print(F("Moving target: "));
      Serial.println(radar.movingTargetDistance());
    }
  }
  */
}
