/*
 *  Variant entry header for the HLK LD2410C.
 *
 *  Use this in your sketch instead of <ld2410.h> to get LD2410C support
 *  out of the box on every build system (Arduino IDE GUI, arduino-cli,
 *  PlatformIO) without having to remember a -D build flag or a
 *  #define before the include.
 *
 *      #include <ld2410c.h>
 *      ld2410 radar;
 *
 *  Backward compat: defining LD2410_VARIANT_C yourself (e.g. via
 *  PlatformIO build_flags) then including <ld2410.h> still works
 *  identically — this header just sets the macro for you if it isn't
 *  already defined, then includes the shared implementation.
 */
#pragma once
#ifndef LD2410_VARIANT_C
#define LD2410_VARIANT_C
#endif
#include "ld2410.h"
