#!/usr/bin/env bash
# Compile + upload + monitor an LD2410 hardware test sketch.
#
# Usage:
#   bash tests/hw/run.sh [PORT] [FQBN]
#
# Defaults:
#   SKETCH = ld2410c_full_test       (override via env: SKETCH=...)
#   PORT   = /dev/ttyUSB0
#   FQBN   = esp32:esp32:esp32
#
# Examples:
#   bash tests/hw/run.sh                                  # run the full-API sweep
#   SKETCH=ld2410c_modes_switch bash tests/hw/run.sh      # run interactive mode-switcher
#   bash tests/hw/run.sh /dev/ttyUSB1                     # custom port, default sketch
#
# The monitor stays attached after upload; press Ctrl-C to detach.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

SKETCH_NAME="${SKETCH:-ld2410c_full_test}"
SKETCH_DIR="$HERE/$SKETCH_NAME"
if [[ ! -d "$SKETCH_DIR" ]]; then
  echo "ERROR: sketch dir not found: $SKETCH_DIR" >&2
  echo "Available sketches under tests/hw:" >&2
  # `|| true` prevents `set -euo pipefail` from masking the original
  # "sketch dir not found" error if grep happens to drop everything.
  (ls -1 "$HERE" | grep -v '^run' || true) >&2
  exit 1
fi

PORT="${1:-/dev/ttyUSB0}"
FQBN="${2:-esp32:esp32:esp32}"
# Default monitor baud is 115200 (matches ld2410c_full_test). The
# modes-switch sketch runs the monitor at 57600 instead — see the
# "MONITOR_BAUD" comment in that .ino for the rationale (CP2102
# clone clock skew). Caller can always override the per-sketch
# default by exporting MONITOR_BAUD before running this script.
if [[ -z "${MONITOR_BAUD:-}" ]]; then
  case "$SKETCH_NAME" in
    ld2410c_modes_switch) MONITOR_BAUD=57600 ;;
    *)                    MONITOR_BAUD=115200 ;;
  esac
fi

echo "=== $SKETCH_NAME ==="
echo "  port  : $PORT"
echo "  fqbn  : $FQBN"
echo "  sketch: $SKETCH_DIR"
echo

echo "[1/3] compile"
arduino-cli compile \
  --fqbn "$FQBN" \
  --library "$ROOT" \
  --build-property "compiler.cpp.extra_flags=-DLD2410_VARIANT_C" \
  "$SKETCH_DIR"

echo
echo "[2/3] upload"
arduino-cli upload \
  --fqbn "$FQBN" \
  --port "$PORT" \
  "$SKETCH_DIR"

echo
echo "[3/3] monitor (Ctrl-C to detach) — baud $MONITOR_BAUD"
arduino-cli monitor \
  --fqbn "$FQBN" \
  --port "$PORT" \
  --config "baudrate=$MONITOR_BAUD"
