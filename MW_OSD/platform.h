#ifndef __PLATFORM_H
#define __PLATFORM_H

// we're going to port this to other platforms, woohoo!
// default to ARDUINO
#ifndef STM32
#define ARDUINO
#endif

// TODO: review where this should go
#define MWVERS "MW-OSD - R1.6"
#define MWOSDVER 12      // for eeprom layout verification    was 9  

// so we can skip the PROGMEM keyword on platforms that dont support it
// empty by default
#define MAYBE_PROGMEM

// arduino specific
#ifdef ARDUINO
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "config.h"
#include "types.h"
#include "serial.h"
#include "eeprom.h"
#include "spi.h"
#include "font.h"
#include "gps.h"
#include "hardware.h"
#include "def.h"
#include "symbols.h"
#include "globals.h"
#include "math.h"

// we'll use the PROGMEM keyword
#define MAYBE_PROGMEM PROGMEM
#endif

// sensor platform specific
#include "sensors.h"
#include "msp.h"

// shared libs
#include "max7456.h"
#include "screen.h"


// TODO: move
#if defined NAZA
  #include "naza.h"
#endif  

#if defined LOADFONT_LARGE
  #include "fontL.h"
#elif defined LOADFONT_DEFAULT 
  #include "fontD.h"
#elif defined LOADFONT_BOLD 
  #include "fontB.h"
#endif

#endif
