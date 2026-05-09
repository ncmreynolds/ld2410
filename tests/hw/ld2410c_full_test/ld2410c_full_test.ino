// =====================================================================
// ld2410c_full_test — exercise every public method of `class ld2410`
// against a real LD2410C connected to an ESP32 (Serial2 by default).
//
// Phases (in order):
//   1. Connection sanity                — begin + isConnected + first frame
//   2. Read commands (synchronous)      — firmware, config, MAC, distance res
//   3. Safe write/restore (synchronous) — setMaxValues, setGateSensitivity,
//                                         setDistanceResolution, setBluetooth
//   4. setBaudRate roundtrip            — switch to 115200, check, restore
//   5. autoReadTask (asynchronous)      — engineering mode, frame liveness,
//                                         start/stop predicate
//
// Each step prints `[PASS]` or `[FAIL]` followed by a short note. A final
// summary line reports counts.
//
// Build & upload (from repo root):
//   bash tests/hw/run.sh /dev/ttyUSB0
//
// or manually:
//   arduino-cli compile --fqbn esp32:esp32:esp32 \
//     --library . \
//     --build-property "compiler.cpp.extra_flags=-DLD2410_VARIANT_C" \
//     tests/hw/ld2410c_full_test
//   arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 \
//     tests/hw/ld2410c_full_test
//   arduino-cli monitor --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 \
//     -c baudrate=115200
// =====================================================================

#include <Arduino.h>
#define LD2410_VARIANT_C
#include <ld2410.h>

// ---- Pinout (matches user setup: ESP32 WROOM, UART2 GPIO16/17) -------
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL   Serial2
#define RADAR_RX_PIN   16   // ESP32 RX  ← radar TX
#define RADAR_TX_PIN   17   // ESP32 TX  → radar RX

ld2410 radar;
uint16_t pass_count = 0;
uint16_t fail_count = 0;

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------
static void report(const char* name, bool ok) {
  MONITOR_SERIAL.print(ok ? F("[PASS] ") : F("[FAIL] "));
  MONITOR_SERIAL.println(name);
  if (ok) pass_count++; else fail_count++;
}
static void report(const char* name, bool ok, const String& extra) {
  MONITOR_SERIAL.print(ok ? F("[PASS] ") : F("[FAIL] "));
  MONITOR_SERIAL.print(name);
  MONITOR_SERIAL.print(F("  "));
  MONITOR_SERIAL.println(extra);
  if (ok) pass_count++; else fail_count++;
}

static void section(const char* title) {
  MONITOR_SERIAL.println();
  MONITOR_SERIAL.print(F("=== "));
  MONITOR_SERIAL.print(title);
  MONITOR_SERIAL.println(F(" ==="));
}

// Drain the parser for a fixed number of milliseconds (used between
// async writes and the next read).
static void pump(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { radar.read(); delay(1); }
}

// Reopen the host UART at a new baud and re-run radar.begin().
// Required after setBaudRate + restart (the radar is now talking at
// the new rate and the library's internal state must be primed again).
static void reopen_uart(uint32_t baud) {
  RADAR_SERIAL.end();
  delay(50);
  RADAR_SERIAL.begin(baud, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(50);
  radar.begin(RADAR_SERIAL, /*waitForRadar=*/false);
  pump(500);   // give the radar time to emit its first post-reopen frame
}

// ---------------------------------------------------------------------
// Saved state — restored at the end so the radar is left in factory-
// equivalent condition.
// ---------------------------------------------------------------------
struct SavedState {
  uint8_t  max_moving_gate;
  uint8_t  max_stationary_gate;
  uint16_t sensor_idle_time;
  uint8_t  motion_sens_g4;
  uint8_t  stationary_sens_g4;
  uint16_t distance_resolution;
  bool     valid;
} saved = { 0, 0, 0, 0, 0, 0xFFFF, false };

static bool snapshot_state() {
  if (!radar.requestCurrentConfiguration()) return false;
  saved.max_moving_gate     = radar.max_moving_gate;
  saved.max_stationary_gate = radar.max_stationary_gate;
  saved.sensor_idle_time    = radar.sensor_idle_time;
  saved.motion_sens_g4      = radar.motion_sensitivity[4];
  saved.stationary_sens_g4  = radar.stationary_sensitivity[4];
  if (!radar.requestDistanceResolution()) return false;
  saved.distance_resolution = radar.distance_resolution;
  saved.valid = true;
  return true;
}

static bool restore_state() {
  if (!saved.valid) return false;
  bool ok = true;
  ok &= radar.setMaxValues(saved.max_moving_gate, saved.max_stationary_gate, saved.sensor_idle_time);
  ok &= radar.setGateSensitivityThreshold(4, saved.motion_sens_g4, saved.stationary_sens_g4);
  ok &= radar.setDistanceResolution(saved.distance_resolution);
  return ok;
}

// =====================================================================
// PHASE 1 — connection sanity
// =====================================================================
static void phase_1_connection() {
  section("Phase 1 — connection sanity");

  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  bool begin_ok = radar.begin(RADAR_SERIAL, /*waitForRadar=*/false);
  report("begin(Serial2)", begin_ok);

  // Pump for up to 2 s waiting for the first valid frame.
  uint32_t t0 = millis();
  bool any_frame = false;
  while (millis() - t0 < 2000) {
    if (radar.read()) { any_frame = true; break; }
    delay(2);
  }
  report("first data frame parsed", any_frame);

  pump(500);
  report("isConnected()", radar.isConnected());
}

// =====================================================================
// PHASE 2 — read commands
// =====================================================================
static void phase_2_reads() {
  section("Phase 2 — read commands (synchronous)");

  bool fw = radar.requestFirmwareVersion();
  String fw_extra = String("v") + radar.firmware_major_version + "." +
                    radar.firmware_minor_version + "." +
                    String(radar.firmware_bugfix_version, HEX);
  report("requestFirmwareVersion", fw && radar.firmware_major_version != 0, fw_extra);

  bool cfg = radar.requestCurrentConfiguration();
  String cfg_extra = String("max_gate=") + radar.max_gate +
                     " mov=" + radar.max_moving_gate +
                     " sta=" + radar.max_stationary_gate +
                     " idle=" + radar.sensor_idle_time + "s";
  report("requestCurrentConfiguration", cfg && radar.max_gate > 0, cfg_extra);

  bool mac = radar.requestMACAddress();
  char mac_buf[24];
  snprintf(mac_buf, sizeof(mac_buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           radar.mac_address[0], radar.mac_address[1], radar.mac_address[2],
           radar.mac_address[3], radar.mac_address[4], radar.mac_address[5]);
  bool mac_nonzero = mac && (radar.mac_address[0] | radar.mac_address[1] |
                              radar.mac_address[2] | radar.mac_address[3] |
                              radar.mac_address[4] | radar.mac_address[5]);
  report("requestMACAddress", mac_nonzero, String(mac_buf));

  bool dr = radar.requestDistanceResolution();
  String dr_extra;
  if (radar.distance_resolution == LD2410_DISTANCE_RESOLUTION_075M)      dr_extra = "0.75 m";
  else if (radar.distance_resolution == LD2410_DISTANCE_RESOLUTION_02M)  dr_extra = "0.20 m";
  else                                                                   dr_extra = String("idx=") + radar.distance_resolution;
  report("requestDistanceResolution", dr && radar.distance_resolution != 0xFFFF, dr_extra);
}

// =====================================================================
// PHASE 3 — safe writes (save → modify → verify → restore)
// =====================================================================
static void phase_3_writes() {
  section("Phase 3 — safe writes (save -> modify -> verify -> restore)");

  bool snap = snapshot_state();
  String snap_extra = String("mov=") + saved.max_moving_gate +
                      " sta=" + saved.max_stationary_gate +
                      " idle=" + saved.sensor_idle_time +
                      " g4_m/s=" + saved.motion_sens_g4 + "/" + saved.stationary_sens_g4 +
                      " dr=" + saved.distance_resolution;
  report("snapshot original state", snap, snap_extra);

  // ---- setMaxValues: change to (6, 5, 3 s), read back, restore ------
  bool smv = radar.setMaxValues(6, 5, 3);
  pump(100);
  bool smv_check = radar.requestCurrentConfiguration() &&
                   radar.max_moving_gate == 6 &&
                   radar.max_stationary_gate == 5 &&
                   radar.sensor_idle_time == 3;
  report("setMaxValues(6,5,3) + readback", smv && smv_check,
         String("got mov=") + radar.max_moving_gate +
         " sta=" + radar.max_stationary_gate +
         " idle=" + radar.sensor_idle_time);

  // ---- setGateSensitivityThreshold(gate=4, motion=75, stationary=40) -
  bool sgs = radar.setGateSensitivityThreshold(4, 75, 40);
  pump(100);
  bool sgs_check = radar.requestCurrentConfiguration() &&
                   radar.motion_sensitivity[4] == 75 &&
                   radar.stationary_sensitivity[4] == 40;
  report("setGateSensitivityThreshold(g4, 75, 40) + readback",
         sgs && sgs_check,
         String("got m=") + radar.motion_sensitivity[4] +
         " s=" + radar.stationary_sensitivity[4]);

  // ---- setBluetooth: toggle (no readback path; pure ACK check) ------
  bool bt_off = radar.setBluetooth(false);
  report("setBluetooth(false)", bt_off);
  bool bt_on  = radar.setBluetooth(true);
  report("setBluetooth(true) (restore default)", bt_on);

  // ---- setDistanceResolution: switch and switch back ---------------
  uint16_t inverse = (saved.distance_resolution == LD2410_DISTANCE_RESOLUTION_075M)
                       ? LD2410_DISTANCE_RESOLUTION_02M
                       : LD2410_DISTANCE_RESOLUTION_075M;
  bool sdr_set = radar.setDistanceResolution(inverse);
  report("setDistanceResolution(inverse)", sdr_set);
  bool sdr_restore = radar.setDistanceResolution(saved.distance_resolution);
  report("setDistanceResolution(restore)", sdr_restore);

  // ---- Restore everything else ------------------------------------
  bool restored = restore_state();
  pump(100);
  bool restore_verify = radar.requestCurrentConfiguration() &&
                        radar.max_moving_gate == saved.max_moving_gate &&
                        radar.max_stationary_gate == saved.max_stationary_gate &&
                        radar.sensor_idle_time == saved.sensor_idle_time &&
                        radar.motion_sensitivity[4] == saved.motion_sens_g4 &&
                        radar.stationary_sensitivity[4] == saved.stationary_sens_g4;
  report("restore original state + verify", restored && restore_verify);
}

// =====================================================================
// PHASE 4 — setBaudRate roundtrip (115200 → restore default 256000)
// =====================================================================
static void phase_4_baudrate() {
  section("Phase 4 — setBaudRate roundtrip");

  bool sb_115k = radar.setBaudRate(LD2410_BAUD_INDEX_115200);
  report("setBaudRate(115200)", sb_115k);

  // Restart so the new baud takes effect.
  bool restart_a = radar.requestRestart();
  report("requestRestart() after baud change", restart_a);
  delay(800);                          // radar boot blackout (mirrors libs')
  reopen_uart(115200);

  bool fw_at_115k = radar.requestFirmwareVersion();
  report("requestFirmwareVersion @ 115200", fw_at_115k && radar.firmware_major_version != 0);

  bool sb_back = radar.setBaudRate(LD2410_BAUD_INDEX_256000);
  report("setBaudRate(256000) (restore default)", sb_back);
  bool restart_b = radar.requestRestart();
  report("requestRestart() restoring baud", restart_b);
  delay(800);
  reopen_uart(LD2410_DEFAULT_BAUD);    // 256000 on C

  bool fw_at_default = radar.requestFirmwareVersion();
  report("requestFirmwareVersion @ default baud", fw_at_default && radar.firmware_major_version != 0);
}

// =====================================================================
// PHASE 5 — autoReadTask (asynchronous)
// =====================================================================
static void phase_5_autoreadtask() {
  section("Phase 5 — autoReadTask (asynchronous)");

#if defined(ESP32)
  bool task_started = radar.autoReadTask();
  report("autoReadTask() started", task_started);
  delay(50);
  report("isAutoReadTaskRunning() == true", radar.isAutoReadTaskRunning());

  // The task is now feeding the parser in background. Wait ~1 s and
  // confirm that frames are arriving (isConnected uses the timestamp
  // of the last parsed packet).
  delay(1500);
  report("isConnected() with task running", radar.isConnected());

  // Engineering mode while the task is running — this exercises the
  // CommandTransaction lock + the per-gate energy path on a populated
  // radar.
  bool eng_on = radar.requestStartEngineeringMode();
  report("requestStartEngineeringMode (with task)", eng_on);
  delay(800);
  bool eng_recv = radar.engineeringRetrieved();
  String eng_extra = String("g4 m/s = ") + radar.movingEnergyAtGate(4) +
                     "/" + radar.stationaryEnergyAtGate(4);
  report("engineeringRetrieved() + per-gate energy", eng_recv, eng_extra);
  bool eng_off = radar.requestEndEngineeringMode();
  report("requestEndEngineeringMode", eng_off);

  radar.stopAutoReadTask();
  delay(50);
  report("isAutoReadTaskRunning() == false (after stop)",
         !radar.isAutoReadTaskRunning());
#else
  report("autoReadTask (skipped: not ESP32)", true);
#endif
}

// =====================================================================
// Setup / loop
// =====================================================================
void setup() {
  MONITOR_SERIAL.begin(115200);
  delay(500);
  MONITOR_SERIAL.println();
  MONITOR_SERIAL.println(F("======================================================="));
  MONITOR_SERIAL.print  (F("  ld2410c_full_test — variant: "));
  MONITOR_SERIAL.println(F(LD2410_VARIANT_NAME));
  MONITOR_SERIAL.print  (F("  pins: RX="));
  MONITOR_SERIAL.print  (RADAR_RX_PIN);
  MONITOR_SERIAL.print  (F(" TX="));
  MONITOR_SERIAL.print  (RADAR_TX_PIN);
  MONITOR_SERIAL.print  (F("  default baud="));
  MONITOR_SERIAL.println(LD2410_DEFAULT_BAUD);
  MONITOR_SERIAL.println(F("======================================================="));

  phase_1_connection();
  phase_2_reads();
  phase_3_writes();
  phase_4_baudrate();
  phase_5_autoreadtask();

  MONITOR_SERIAL.println();
  MONITOR_SERIAL.println(F("======================================================="));
  MONITOR_SERIAL.print  (F("  RESULTS: "));
  MONITOR_SERIAL.print  (pass_count);
  MONITOR_SERIAL.print  (F(" pass / "));
  MONITOR_SERIAL.print  (fail_count);
  MONITOR_SERIAL.print  (F(" fail / "));
  MONITOR_SERIAL.print  (pass_count + fail_count);
  MONITOR_SERIAL.println(F(" total"));
  MONITOR_SERIAL.println(F("======================================================="));
}

void loop() {
  // Idle — keep the link alive so the user can poke around if desired.
  radar.read();
  delay(10);
}
