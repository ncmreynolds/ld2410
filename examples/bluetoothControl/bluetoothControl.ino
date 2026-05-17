/*
 * bluetoothControl — LD2410C-only commands for Bluetooth + MAC.
 *
 * Demonstrates the four BT-related opcodes that exist on LD2410C
 * (and not on the original LD2410 nor on LD2410S):
 *   0xA4 setBluetooth(on/off)            non-volatile, takes effect after restart
 *   0xA5 requestMACAddress()             populates radar.mac_address[6]
 *   0xA8 obtainBluetoothPermissions(pwd) NOTE: ACK delivered over BLE only
 *   0xA9 setBluetoothPassword(pwd)       4-byte ACK on UART
 *
 * The setBluetoothPassword() call below is COMMENTED by default
 * because it changes a value the user may rely on for BLE pairing.
 * Uncomment only after you understand the implication.
 *
 * Compatible with: LD2410C only. The BT opcodes do not exist on
 * the original LD2410 nor on LD2410S. This sketch uses the
 * <ld2410c.h> entry header to pin the variant — see the include
 * directive below. For PlatformIO / arduino-cli builds you can
 * equivalently pass -DLD2410_VARIANT_C as a build flag and use
 * <ld2410.h>; both routes are supported.
 */

// (LD2410C variant is selected by including <ld2410c.h> below, instead
// of the variant-neutral <ld2410.h>. This works on every build system
// including the Arduino IDE GUI, which cannot pass per-sketch -D flags.)

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

#include <ld2410c.h>     // entry header pins the variant for this TU

ld2410 radar;

static void printMac(const uint8_t mac[6]) {
  for (uint8_t i = 0; i < 6; i++) {
    if (mac[i] < 0x10) MONITOR_SERIAL.print('0');
    MONITOR_SERIAL.print(mac[i], HEX);
    if (i + 1 < 6) MONITOR_SERIAL.print(':');
  }
}

void setup() {
  MONITOR_SERIAL.begin(115200);
  delay(500);
#if defined(ESP32)
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#elif defined(ARDUINO_ARCH_RP2040)
  RADAR_SERIAL.setRX(RADAR_RX_PIN);
  RADAR_SERIAL.setTX(RADAR_TX_PIN);
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD);
#else
  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD);
#endif
  delay(500);

  MONITOR_SERIAL.println(F("\nLD2410C bluetoothControl example"));
  if (!radar.begin(RADAR_SERIAL)) {
    MONITOR_SERIAL.println(F("radar.begin: FAIL — check wiring"));
    while (true) delay(1000);
  }
  MONITOR_SERIAL.println(F("radar.begin: OK"));

  // -- Read MAC -------------------------------------------------------
  if (radar.requestMACAddress()) {
    MONITOR_SERIAL.print(F("MAC: "));
    printMac(radar.mac_address);
    MONITOR_SERIAL.println();
  } else {
    MONITOR_SERIAL.println(F("requestMACAddress: FAIL"));
  }

  // -- Disable Bluetooth, then re-enable -----------------------------
  // setBluetooth() is non-volatile: it persists across reboots and is
  // applied only after a power cycle / requestRestart(). For testing
  // purposes we toggle and restore in one go.
  MONITOR_SERIAL.print(F("setBluetooth(false): "));
  MONITOR_SERIAL.println(radar.setBluetooth(false) ? F("OK") : F("FAIL"));

  delay(50);
  MONITOR_SERIAL.print(F("setBluetooth(true) [restore]: "));
  MONITOR_SERIAL.println(radar.setBluetooth(true) ? F("OK") : F("FAIL"));

  // -- Bluetooth password (commented by default) ---------------------
  // Uncomment to set a custom 6-byte BLE pairing password. The default
  // factory value is "HiLink"; changing it requires you to remember the
  // new value to re-pair from a phone afterwards.
  //
  // uint8_t pwd[LD2410_BLUETOOTH_PASSWORD_LENGTH] = {'M','y','C','o','d','e'};
  // MONITOR_SERIAL.print(F("setBluetoothPassword: "));
  // MONITOR_SERIAL.println(radar.setBluetoothPassword(pwd) ? F("OK") : F("FAIL"));

  // -- Permissions opcode (rarely useful from UART) ------------------
  // obtainBluetoothPermissions() ACK is delivered ONLY over BLE per
  // the HLK protocol PDF — most firmwares return false on UART. We
  // call it for completeness; do not treat false as a hard error.
  uint8_t defaultPwd[LD2410_BLUETOOTH_PASSWORD_LENGTH] = {'H','i','L','i','n','k'};
  MONITOR_SERIAL.print(F("obtainBluetoothPermissions(default): "));
  MONITOR_SERIAL.println(radar.obtainBluetoothPermissions(defaultPwd) ? F("OK") : F("FAIL (BLE-only ACK, expected)"));
}

void loop() {
  // Nothing to poll — this sketch is one-shot config + readback.
  delay(1000);
}
