// it would be awesome to get rid of most of these...

#ifndef __GLOBALS_H
#define __GLOBALS_H

#include "settings.h"

// General use variables
struct Timer_s {
  uint8_t tenthSec;
  uint8_t halfSec;
  uint8_t Blink2hz;                          // This is turing on and off at 2hz
  uint8_t Blink10hz;                         // This is turing on and off at 10hz
  uint16_t lastCallSign;                          // Callsign_timer
  uint8_t rssiTimer;
//  uint8_t accCalibrationTimer;
  uint8_t magCalibrationTimer;
  uint32_t fwAltitudeTimer;
  uint32_t seconds;
  uint8_t MSP_active;
  uint8_t GPS_active;
  // armed timer
  uint8_t armed = 255;
  uint32_t allSec;
};

struct Flags_s {
  uint8_t ident;
  uint8_t box;
  uint8_t reset;
};

// Mode bits
struct Mode_s {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint8_t baro;
  uint8_t mag;
  uint16_t camstab;
  uint16_t gpshome;
  uint16_t gpshold;
  uint16_t passthru;
  uint32_t air;
  uint32_t acroplus;
  uint32_t osd_switch;
  uint32_t llights;
  uint32_t gpsmission;
  uint32_t gpsland;
};

// RC Info
struct RC_s {
  uint8_t rcRate8,rcExpo8;
  uint8_t rollPitchRate;
  uint8_t rollRate;
  uint8_t PitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  uint8_t tpa_breakpoint16;
  uint8_t rcYawExpo8;

  // uint8_t ?
  int16_t rssi =0;
  // uint8_t ?
  int16_t oldrssi;
  int16_t pwmRSSI = 0;
  uint16_t MwRssi=0;

  //For Current Throttle
  uint16_t LowT = LOWTHROTTLE;
  uint16_t HighT = HIGHTHROTTLE;
};

// MWOSD's current state
//  stuff here should pry be moved, its mostly here b/c idk where else to put it
struct Mwosd_s {
  // kinda a misnomer, because the osd never really "arms", but it does track armed state independently of the FC
  bool armed;
  bool previousarmedstatus;
  uint16_t pMeterSum=0;

  // For Settings Defaults
  uint8_t P8[PIDITEMS];
  uint8_t I8[PIDITEMS];
  uint8_t D8[PIDITEMS];

  // temperature in degrees Centigrade
  int16_t temperature=0;

  // This hold barometric value
  int32_t  old_MwAltitude=0;
};

struct Debug_s {
  //uint16_t screen_buffer[4];
};

extern struct Timer_s timer;
extern struct Flags_s flags;
extern struct Mode_s mode;
extern struct RC_s rc;
extern struct Mwosd_s mwosd;
extern struct Debug_s debug;

extern uint8_t Settings[EEPROM_SETTINGS];
extern uint16_t Settings16[EEPROM16_SETTINGS];

#endif
