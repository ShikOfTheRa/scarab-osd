#ifndef __PLATFORM_H
#define __PLATFORM_H

// we're going to port this to other platforms, woohoo!
// default to ARDUINO
#ifndef STM32

// arduino studio defines this by default
#ifndef ARDUINO
#define ARDUINO
#endif

#endif

// TODO: review where this should go
#define MWVERS "MW-OSD - R1.6"
#define MWOSDVER 12      // for eeprom layout verification    was 9  

// arduino specific
#ifdef ARDUINO
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "config.h"
#include "def.h"
#include "types.h"
#include "settings.h"
#include "eeprom.h"
#include "spi.h"
#include "font.h"
#include "gps.h"
#include "hardware.h"
#include "globals.h"
#include "math.h"
#endif

// sensor platform specific
#include "sensors.h"
#include "msp.h"
#include "stats.h"

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
