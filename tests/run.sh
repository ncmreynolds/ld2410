#!/usr/bin/env bash
# Build and run the host-side parser unit tests.
# Usage: bash tests/run.sh   (from the repo root, or anywhere)
#
# Compiles the test suite THREE times:
#   - default mode (LD2410_VARIANT_BASE, the historical base/C target)
#   - -DLD2410_VARIANT_C  (exercises Bluetooth / MAC / distance-resolution
#     command paths, gated by LD2410_HAS_BLUETOOTH/_MAC_ADDRESS/
#     _DISTANCE_RESOLUTION which are only defined by ld2410_c.h)
#   - -DLD2410_VARIANT_S  (exercises S-only frame layouts)
# so that variant-specific paths in parse_data_frame_ and
# parse_command_frame_ are all exercised on the host.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

CXXFLAGS="-std=c++17 -Wall -Wextra -I$HERE -I$ROOT/src"
SOURCES="$HERE/test_parser.cpp $ROOT/src/ld2410.cpp"

echo "=== Building & running base/C test suite ==="
g++ $CXXFLAGS $SOURCES -o "$HERE/test_parser_basec"
"$HERE/test_parser_basec"

echo
echo "=== Building & running LD2410_VARIANT_C test suite ==="
g++ $CXXFLAGS -DLD2410_VARIANT_C $SOURCES -o "$HERE/test_parser_c"
"$HERE/test_parser_c"

echo
echo "=== Building & running LD2410_VARIANT_S test suite ==="
g++ $CXXFLAGS -DLD2410_VARIANT_S $SOURCES -o "$HERE/test_parser_s"
"$HERE/test_parser_s"
