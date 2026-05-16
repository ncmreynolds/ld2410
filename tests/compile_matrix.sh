#!/usr/bin/env bash
# End-to-end compile matrix for the ld2410 library.
#
# Iterates the cross-product of (board × variant) and runs arduino-cli
# compile on the appropriate example sketch for each cell. Reports a
# pass/fail line per cell, and exits non-zero if any cell failed.
#
# Cells:
#   esp32     default / -DLD2410_VARIANT_B / -DLD2410_VARIANT_C / -DLD2410_VARIANT_S   → basicSensor
#   esp8266   default / -DLD2410_VARIANT_B / -DLD2410_VARIANT_C / -DLD2410_VARIANT_S   → basicSensorEsp8266
#   rp2040    default / -DLD2410_VARIANT_B / -DLD2410_VARIANT_C / -DLD2410_VARIANT_S   → basicSensor
#   avr128da32 default / -DLD2410_VARIANT_B / -DLD2410_VARIANT_C / -DLD2410_VARIANT_S  → basicSensor
#       (SpenceKonde DxCore, 8-bit AVR Dx-series — the only 8-bit target in the matrix;
#        validates that the library still compiles where int is 16 bits.)
#
# Usage:
#   bash tests/compile_matrix.sh        # full matrix
#   FQBN_ESP32=...  bash tests/...      # override board fqbns
#
# Requires arduino-cli on PATH and the three core packages
# (esp32:esp32, esp8266:esp8266, rp2040:rp2040) installed.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

FQBN_ESP32="${FQBN_ESP32:-esp32:esp32:esp32}"
FQBN_ESP8266="${FQBN_ESP8266:-esp8266:esp8266:nodemcuv2}"
FQBN_RP2040="${FQBN_RP2040:-rp2040:rp2040:rpipico}"
FQBN_AVRDA="${FQBN_AVRDA:-DxCore:megaavr:avrda:chip=avr128da32}"

EX_BASIC="$ROOT/examples/basicSensor/basicSensor.ino"
EX_8266="$ROOT/examples/basicSensorEsp8266/basicSensorEsp8266.ino"

PASS=0
FAIL=0
FAILED_CELLS=()

run_cell() {
  local label="$1"; local fqbn="$2"; local sketch="$3"; local extra_flag="$4"
  local extra_args=()
  if [[ -n "$extra_flag" ]]; then
    extra_args+=( --build-property "compiler.cpp.extra_flags=$extra_flag" )
  fi
  printf '  %-40s ... ' "$label"
  if arduino-cli compile --fqbn "$fqbn" --library "$ROOT" "${extra_args[@]}" "$sketch" >/dev/null 2>&1; then
    echo "ok"
    PASS=$((PASS + 1))
  else
    echo "FAIL"
    FAIL=$((FAIL + 1))
    FAILED_CELLS+=("$label")
  fi
}

echo "=== arduino-cli compile matrix ==="

echo "[esp32 / $FQBN_ESP32]"
run_cell "esp32 default (BASE)"           "$FQBN_ESP32" "$EX_BASIC" ""
run_cell "esp32 -DLD2410_VARIANT_B"       "$FQBN_ESP32" "$EX_BASIC" "-DLD2410_VARIANT_B"
run_cell "esp32 -DLD2410_VARIANT_C"       "$FQBN_ESP32" "$EX_BASIC" "-DLD2410_VARIANT_C"
run_cell "esp32 -DLD2410_VARIANT_S"       "$FQBN_ESP32" "$EX_BASIC" "-DLD2410_VARIANT_S"

echo "[esp8266 / $FQBN_ESP8266]"
run_cell "esp8266 default (BASE)"         "$FQBN_ESP8266" "$EX_8266" ""
run_cell "esp8266 -DLD2410_VARIANT_B"     "$FQBN_ESP8266" "$EX_8266" "-DLD2410_VARIANT_B"
run_cell "esp8266 -DLD2410_VARIANT_C"     "$FQBN_ESP8266" "$EX_8266" "-DLD2410_VARIANT_C"
run_cell "esp8266 -DLD2410_VARIANT_S"     "$FQBN_ESP8266" "$EX_8266" "-DLD2410_VARIANT_S"

echo "[rp2040 / $FQBN_RP2040]"
run_cell "rp2040 default (BASE)"          "$FQBN_RP2040" "$EX_BASIC" ""
run_cell "rp2040 -DLD2410_VARIANT_B"      "$FQBN_RP2040" "$EX_BASIC" "-DLD2410_VARIANT_B"
run_cell "rp2040 -DLD2410_VARIANT_C"      "$FQBN_RP2040" "$EX_BASIC" "-DLD2410_VARIANT_C"
run_cell "rp2040 -DLD2410_VARIANT_S"      "$FQBN_RP2040" "$EX_BASIC" "-DLD2410_VARIANT_S"

echo "[avr128da32 / $FQBN_AVRDA]"
run_cell "avr128da32 default (BASE)"      "$FQBN_AVRDA"  "$EX_BASIC" ""
run_cell "avr128da32 -DLD2410_VARIANT_B"  "$FQBN_AVRDA"  "$EX_BASIC" "-DLD2410_VARIANT_B"
run_cell "avr128da32 -DLD2410_VARIANT_C"  "$FQBN_AVRDA"  "$EX_BASIC" "-DLD2410_VARIANT_C"
run_cell "avr128da32 -DLD2410_VARIANT_S"  "$FQBN_AVRDA"  "$EX_BASIC" "-DLD2410_VARIANT_S"

echo
echo "=== matrix summary: $PASS pass, $FAIL fail ==="
if (( FAIL > 0 )); then
  for c in "${FAILED_CELLS[@]}"; do echo "  FAILED: $c"; done
  exit 1
fi
exit 0
