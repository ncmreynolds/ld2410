// =====================================================================
// ld2410c_modes_switch — interactive 4-mode switcher.
//
// Type a single character on the Serial Monitor to switch the
// reporting mode of the LD2410C live, without re-uploading:
//
//   1  →  SYNC | NORMAL    (radar.read() in loop, basic frames only)
//   2  →  SYNC | ENG       (radar.read() + engineering mode ON)
//   3  →  ASYNC| NORMAL    (autoReadTask, basic frames only)
//   4  →  ASYNC| ENG       (autoReadTask + engineering mode ON)
//   h  →  print help
//
// The current mode is shown in the [tag] prefix of every status line.
// Status is printed every 500 ms (= 2 Hz) regardless of mode.
//
// Build & upload (from repo root):
//   SKETCH=ld2410c_modes_switch bash tests/hw/run.sh
// or manually with arduino-cli (see comments in tests/hw/run.sh).
// =====================================================================

#include <Arduino.h>
#define LD2410_VARIANT_C
#include <ld2410.h>

// ---- Pinout (ESP32 WROOM, UART2 GPIO16/17) -------------------------
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL   Serial2
#define RADAR_RX_PIN   16
#define RADAR_TX_PIN   17

ld2410 radar;

// ---- Mode state ----------------------------------------------------
enum Mode : uint8_t { SYNC_NORMAL, SYNC_ENG, ASYNC_NORMAL, ASYNC_ENG };
static Mode current_mode = SYNC_NORMAL;

static const __FlashStringHelper* mode_tag(Mode m) {
  switch (m) {
    case SYNC_NORMAL:  return F("SYNC |NORMAL");
    case SYNC_ENG:     return F("SYNC |  ENG ");
    case ASYNC_NORMAL: return F("ASYNC|NORMAL");
    case ASYNC_ENG:    return F("ASYNC|  ENG ");
  }
  return F("?");
}

static bool is_async(Mode m)        { return (m == ASYNC_NORMAL || m == ASYNC_ENG); }
static bool is_engineering(Mode m)  { return (m == SYNC_ENG     || m == ASYNC_ENG); }

// ---- Help ----------------------------------------------------------
static void print_help() {
  MONITOR_SERIAL.println();
  MONITOR_SERIAL.println(F("------ commands ------"));
  MONITOR_SERIAL.println(F("  1   sync  | normal       (radar.read() loop, basic 0x02 frames)"));
  MONITOR_SERIAL.println(F("  2   sync  | engineering  (radar.read() loop, 0x01 frames + per-gate)"));
  MONITOR_SERIAL.println(F("  3   async | normal       (autoReadTask, basic 0x02 frames)"));
  MONITOR_SERIAL.println(F("  4   async | engineering  (autoReadTask, 0x01 frames + per-gate)"));
  MONITOR_SERIAL.println(F("  h   show this help"));
  MONITOR_SERIAL.println(F("Status line every 500 ms; current mode shown in the [tag] prefix."));
  MONITOR_SERIAL.println();
}

// ---- Mode transitions ---------------------------------------------
// Order of operations (matters!):
//   1. stop async task   if leaving async
//   2. end engineering   if leaving engineering
//   3. start engineering if entering engineering
//   4. start async task  if entering async
// This guarantees command-channel calls (request*/set*) happen at most
// against either pure-sync or task-running radar — never during a
// half-stopped task — and that engineering mode is set ON the radar
// BEFORE the background task starts consuming engineering frames.
static void apply_mode(Mode target) {
  if (target == current_mode) {
    MONITOR_SERIAL.print(F("(already in "));
    MONITOR_SERIAL.print(mode_tag(target));
    MONITOR_SERIAL.println(F(")"));
    return;
  }

  const bool was_async = is_async(current_mode);
  const bool was_eng   = is_engineering(current_mode);
  const bool to_async  = is_async(target);
  const bool to_eng    = is_engineering(target);

  if (was_async && !to_async) {
#if defined(ESP32)
    radar.stopAutoReadTask();
    delay(50);
    MONITOR_SERIAL.println(F("> autoReadTask stopped"));
#endif
  }

  if (was_eng && !to_eng) {
    bool ok = radar.requestEndEngineeringMode();
    MONITOR_SERIAL.print(F("> requestEndEngineeringMode: "));
    MONITOR_SERIAL.println(ok ? F("OK") : F("FAIL"));
  }

  if (!was_eng && to_eng) {
    bool ok = radar.requestStartEngineeringMode();
    MONITOR_SERIAL.print(F("> requestStartEngineeringMode: "));
    MONITOR_SERIAL.println(ok ? F("OK") : F("FAIL"));
  }

  if (!was_async && to_async) {
#if defined(ESP32)
    bool ok = radar.autoReadTask();
    MONITOR_SERIAL.print(F("> autoReadTask: "));
    MONITOR_SERIAL.println(ok ? F("OK") : F("FAIL"));
#else
    MONITOR_SERIAL.println(F("> autoReadTask not available (not ESP32)"));
#endif
  }

  current_mode = target;
  MONITOR_SERIAL.print(F(">>> now in: ["));
  MONITOR_SERIAL.print(mode_tag(target));
  MONITOR_SERIAL.println(F("]"));
}

// ---- Input handling -----------------------------------------------
static void handle_command() {
  while (MONITOR_SERIAL.available() > 0) {
    char c = MONITOR_SERIAL.read();
    switch (c) {
      case '1': apply_mode(SYNC_NORMAL);  break;
      case '2': apply_mode(SYNC_ENG);     break;
      case '3': apply_mode(ASYNC_NORMAL); break;
      case '4': apply_mode(ASYNC_ENG);    break;
      case 'h': case 'H': case '?': print_help(); break;
      case '\r': case '\n': case ' ': /* ignore line endings + space */ break;
      default:
        MONITOR_SERIAL.print(F("(unknown cmd: '"));
        MONITOR_SERIAL.print(c);
        MONITOR_SERIAL.println(F("' — type 'h' for help)"));
        break;
    }
  }
}

// ---- Status print --------------------------------------------------
static void print_status() {
  MONITOR_SERIAL.print('[');
  MONITOR_SERIAL.print(mode_tag(current_mode));
  MONITOR_SERIAL.print(F("]  P:"));
  MONITOR_SERIAL.print(radar.presenceDetected()         ? 'Y' : 'N');
  MONITOR_SERIAL.print(F(" M:"));
  MONITOR_SERIAL.print(radar.movingTargetDetected()     ? 'Y' : 'N');
  MONITOR_SERIAL.print(F(" S:"));
  MONITOR_SERIAL.print(radar.stationaryTargetDetected() ? 'Y' : 'N');
  MONITOR_SERIAL.print(F("  dd="));
  MONITOR_SERIAL.print(radar.detectionDistance());
  MONITOR_SERIAL.print(F("cm  mov(d/e)="));
  MONITOR_SERIAL.print(radar.movingTargetDistance());
  MONITOR_SERIAL.print('/');
  MONITOR_SERIAL.print(radar.movingTargetEnergy());
  MONITOR_SERIAL.print(F("  sta(d/e)="));
  MONITOR_SERIAL.print(radar.stationaryTargetDistance());
  MONITOR_SERIAL.print('/');
  MONITOR_SERIAL.print(radar.stationaryTargetEnergy());

  if (is_engineering(current_mode)) {
    if (radar.engineeringRetrieved()) {
      // Take an atomic snapshot of both per-gate arrays before
      // formatting. In SYNC mode this is unnecessary (single thread,
      // no race), but doing it unconditionally keeps the print path
      // identical between modes — which is exactly what we need to
      // tell whether a misformatted line is a Serial.print artifact
      // or something else. If the snapshot version still shows
      // garbled lines the bug is NOT in the print interleaving.
      uint8_t mot[LD2410_GATE_COUNT], sta[LD2410_GATE_COUNT];
      radar.snapshotEngineeringMotionEnergies(mot);
      radar.snapshotEngineeringStationaryEnergies(sta);

      MONITOR_SERIAL.print(F("\n  m=["));
      for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
        if (g) MONITOR_SERIAL.print(',');
        MONITOR_SERIAL.print(mot[g]);
      }
      MONITOR_SERIAL.print(F("]  s=["));
      for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
        if (g) MONITOR_SERIAL.print(',');
        MONITOR_SERIAL.print(sta[g]);
      }
      MONITOR_SERIAL.print(']');
    } else {
      MONITOR_SERIAL.print(F("  (no engineering frame parsed yet)"));
    }
  }
  MONITOR_SERIAL.println();
}

// =====================================================================
void setup() {
  MONITOR_SERIAL.begin(115200);
  delay(500);
  MONITOR_SERIAL.println();
  MONITOR_SERIAL.println(F("========================================================"));
  MONITOR_SERIAL.print  (F("  ld2410c_modes_switch  —  variant: "));
  MONITOR_SERIAL.println(F(LD2410_VARIANT_NAME));
  MONITOR_SERIAL.print  (F("  pins: RX="));
  MONITOR_SERIAL.print  (RADAR_RX_PIN);
  MONITOR_SERIAL.print  (F(" TX="));
  MONITOR_SERIAL.print  (RADAR_TX_PIN);
  MONITOR_SERIAL.print  (F("   baud="));
  MONITOR_SERIAL.println(LD2410_DEFAULT_BAUD);
  MONITOR_SERIAL.println(F("========================================================"));

  RADAR_SERIAL.begin(LD2410_DEFAULT_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  bool ok = radar.begin(RADAR_SERIAL, /*waitForRadar=*/false);
  MONITOR_SERIAL.print(F("radar.begin: "));
  MONITOR_SERIAL.println(ok ? F("OK") : F("FAIL"));

  print_help();
  MONITOR_SERIAL.print(F(">>> starting in: ["));
  MONITOR_SERIAL.print(mode_tag(current_mode));
  MONITOR_SERIAL.println(F("]"));
}

void loop() {
  handle_command();

  // In sync modes WE drain the parser; in async modes the FreeRTOS
  // task does it for us.
  if (!is_async(current_mode)) {
    radar.read();
  }

  static uint32_t last_print = 0;
  const uint32_t now = millis();
  if (now - last_print >= 500) {
    last_print = now;
    print_status();
  }
}
