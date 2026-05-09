#!/usr/bin/env bash
# Compile + upload + monitor the LD2410C hardware test sketch.
#
# Usage:
#   bash tests/hw/run.sh [PORT] [FQBN]
#
# Defaults:
#   PORT  = /dev/ttyUSB0
#   FQBN  = esp32:esp32:esp32
#
# The monitor stays attached after upload; press Ctrl-C to detach.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
SKETCH="$HERE/ld2410c_full_test"

PORT="${1:-/dev/ttyUSB0}"
FQBN="${2:-esp32:esp32:esp32}"

echo "=== ld2410c_full_test ==="
echo "  port  : $PORT"
echo "  fqbn  : $FQBN"
echo "  sketch: $SKETCH"
echo

echo "[1/3] compile"
arduino-cli compile \
  --fqbn "$FQBN" \
  --library "$ROOT" \
  --build-property "compiler.cpp.extra_flags=-DLD2410_VARIANT_C" \
  "$SKETCH"

echo
echo "[2/3] upload"
arduino-cli upload \
  --fqbn "$FQBN" \
  --port "$PORT" \
  "$SKETCH"

echo
echo "[3/3] monitor (Ctrl-C to detach)"
arduino-cli monitor \
  --fqbn "$FQBN" \
  --port "$PORT" \
  --config baudrate=115200
