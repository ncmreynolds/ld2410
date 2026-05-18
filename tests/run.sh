#!/usr/bin/env bash
# Build and run the host-side parser unit tests.
# Usage: bash tests/run.sh   (from the repo root, or anywhere)
#
# Compiles the test suite FOUR times, one per variant — 84 tests total
# (16 base + 27 B + 22 C + 19 S):
#   - default mode (LD2410_VARIANT_BASE, the historical base/C target)
#   - -DLD2410_VARIANT_B  (exercises B-only auxiliary-control commands
#     0xAD/0xAE + engineering-frame trailer extraction, on top of every
#     C-shared path; gated by LD2410_HAS_AUX_CONTROL defined in ld2410_b.h)
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
# Library is header-only as of the variant-abstraction refactor: the
# implementation lives in src/ld2410_impl.h, included from src/ld2410.h
# with every method marked `inline`. The only TU to compile is the test
# driver, which pulls in the impl via the include chain.
SOURCES="$HERE/test_parser.cpp"

echo "=== Building & running base/C test suite ==="
g++ $CXXFLAGS $SOURCES -o "$HERE/test_parser_basec"
"$HERE/test_parser_basec"

echo
echo "=== Building & running LD2410_VARIANT_B test suite ==="
g++ $CXXFLAGS -DLD2410_VARIANT_B $SOURCES -o "$HERE/test_parser_b"
"$HERE/test_parser_b"

echo
echo "=== Building & running LD2410_VARIANT_C test suite ==="
g++ $CXXFLAGS -DLD2410_VARIANT_C $SOURCES -o "$HERE/test_parser_c"
"$HERE/test_parser_c"

echo
echo "=== Building & running LD2410_VARIANT_S test suite ==="
g++ $CXXFLAGS -DLD2410_VARIANT_S $SOURCES -o "$HERE/test_parser_s"
"$HERE/test_parser_s"
