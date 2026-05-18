/*
 *  Variant entry header for the HLK LD2410S.
 *
 *  Use this in your sketch instead of <ld2410.h> to get LD2410S support
 *  out of the box on every build system (Arduino IDE GUI, arduino-cli,
 *  PlatformIO) without having to remember a -D build flag or a
 *  #define before the include.
 *
 *      #include <ld2410s.h>
 *      ld2410 radar;
 *
 *  Backward compat: defining LD2410_VARIANT_S yourself (e.g. via
 *  PlatformIO build_flags) then including <ld2410.h> still works
 *  identically — this header just sets the macro for you if it isn't
 *  already defined, then includes the shared implementation.
 *
 *  Note: LD2410S support is currently UNVERIFIED ON HARDWARE — see
 *  docs/02-variants.md and docs/07-api-ld2410s.md.
 */
#pragma once
#ifndef LD2410_VARIANT_S
#define LD2410_VARIANT_S
#endif
#include "ld2410.h"
