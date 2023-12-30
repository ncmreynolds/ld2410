/*
 * Example sketch for reporting individual gate values using engineering mode. 
 */

#if defined(ESP32)
#ifdef ESP_IDF_VERSION_MAJOR  // IDF 4+
#if CONFIG_IDF_TARGET_ESP32   // ESP32/PICO-D4
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
#else  // ESP32 Before IDF 4.0
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
#endif

#include <ld2410.h>

ld2410 radar;

uint32_t lastReading = 0;
bool radarConnected = false;

// Function to print an array
void printArray(const uint8_t arr[], size_t size) {
  MONITOR_SERIAL.print("[ ");
  for (size_t i = 0; i < size / sizeof(uint8_t); i++) {
    MONITOR_SERIAL.print(arr[i]);
    MONITOR_SERIAL.print(",");
  }
  MONITOR_SERIAL.println("]");
}

void setup(void) {
  MONITOR_SERIAL.begin(115200);  //Feedback over Serial Monitor
//radar.debug(MONITOR_SERIAL); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
#if defined(ESP32)
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);  //UART for monitoring the radar
#elif defined(__AVR_ATmega32U4__)
  RADAR_SERIAL.begin(256000);  //UART for monitoring the radar
#endif
  delay(500);
  MONITOR_SERIAL.print(F("\nConnect LD2410 radar TX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);
  MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));
  if (radar.begin(RADAR_SERIAL) && radar.requestStartEngineeringMode()) {
    MONITOR_SERIAL.println(F("OK"));
  } else {
    MONITOR_SERIAL.println(F("not connected"));
  }
}

void loop() {
  radar.read();
  if (radar.isConnected() && radar.isEngineeringMode() && millis() - lastReading > 1000)  //Report every 1000ms
  {
    lastReading = millis();
    MONITOR_SERIAL.print(F("Moving gates: "));
    printArray(radar.engineering_moving_target_energy, (radar.engineering_max_gates_moving_distance));

    MONITOR_SERIAL.print(F("Stationary gates: "));
    printArray(radar.engineering_stationary_target_energy, (radar.engineering_max_gates_stationary_distance));
  }
}
