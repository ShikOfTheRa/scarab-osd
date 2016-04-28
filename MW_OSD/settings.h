#ifndef __SETTINGS_H
#define __SETTINGS_H

// Settings Locations
enum Setting16_ {
  S16_AMPMAXn,
  S16_AMPZERO,
  S16_AMPDIVIDERRATIO,
  S16_RSSIMIN,
  S16_RSSIMAX,
  S16_SPARE1,
  S16_SPARE2,

  // EEPROM16_SETTINGS must be last!
  EEPROM16_SETTINGS
};

enum Setting_ {
  S_CHECK_,		// used for check
  S_UNUSED_5,
  S_VIDVOLTAGEMIN,
  S_RSSI_ALARM,
  S_DISPLAYRSSI,
  S_MWRSSI,
  S_PWMRSSI,
  S_DISPLAYVOLTAGE,
  S_VOLTAGEMIN,
  S_BATCELLS,
  S_DIVIDERRATIO,
  S_MAINVOLTAGE_VBAT,
  S_AMPERAGE,
  S_MWAMPERAGE,
  S_AMPER_HOUR,
  S_AMPERAGE_VIRTUAL,
  S_UNUSED_3,
  S_VIDVOLTAGE,
  S_VIDDIVIDERRATIO,
  S_UNUSED_4,
  S_AMPER_HOUR_ALARM,
  S_AMPERAGE_ALARM,
  S_DISPLAYGPS,
  S_COORDINATES,
  S_GPSCOORDTOP, //spare
  S_GPSALTITUDE,
  S_ANGLETOHOME,
  S_SHOWHEADING,
  S_HEADING360,
  S_UNITSYSTEM,
  S_VIDEOSIGNALTYPE,
  S_THROTTLEPOSITION,
  S_DISPLAY_HORIZON_BR,
  S_WITHDECORATION,
  S_SHOWBATLEVELEVOLUTION,
  S_RESETSTATISTICS,
  S_MAPMODE,
  S_VREFERENCE,
  S_USE_BOXNAMES,
  S_MODEICON,
  S_DISPLAY_CS,
  S_GPSTIME,
  S_GPSTZAHEAD,
  S_GPSTZ,
  S_DEBUG,
  S_SCROLLING,
  S_GIMBAL,
  S_VARIO,
  S_BAROALT, //50
  S_COMPASS,
  S_HORIZON_ELEVATION,
  S_TIMER,
  S_MODESENSOR,
  S_SIDEBARTOPS,
  S_UNUSED_6,
  S_UNUSED_1, //S_AMPMAXL,
  S_UNUSED_2, //S_AMPMAXH,
  S_RCWSWITCH,
  S_RCWSWITCH_CH,
  S_HUDSW0,
  S_HUDSW1,
  S_HUDSW2,
  S_DISTANCE_ALARM,
  S_ALTITUDE_ALARM,
  S_SPEED_ALARM,
  S_FLYTIME_ALARM,
  S_CS0,
  S_CS1,
  S_CS2,
  S_CS3,
  S_CS4,
  S_CS5,
  S_CS6,
  S_CS7,
  S_CS8,
  S_CS9,
  // EEPROM_SETTINGS must be last!
  EEPROM_SETTINGS
};

const uint8_t EEPROM_DEFAULT[] PROGMEM = {
MWOSDVER,   // used for check              0
0,   // S_UNUSED_5                  1
0, // S_VIDVOLTAGEMIN             2
60,  // S_RSSI_ALARM                3
0,   // S_DISPLAYRSSI               4
0,   // S_MWRSSI                    5
0,   // S_PWMRSSI                   6
1,   // S_DISPLAYVOLTAGE            7
138, // S_VOLTAGEMIN                8
4,   // S_BATCELLS                  9
200, // S_DIVIDERRATIO              10
0,   // S_MAINVOLTAGE_VBAT          11
0,   // S_AMPERAGE                  12
0,   // S_MWAMPERAGE                12a :)
0,   // S_AMPER_HOUR                13
0,   // S_AMPERAGE_VIRTUAL,
150, // S_UNUSED_3,
0,   // S_VIDVOLTAGE                14
200, // S_VIDDIVIDERRATIO           15
0,   // S_VIDVOLTAGE_VBAT           16
50,  // S_AMPER_HOUR_ALARM          17
100, // S_AMPERAGE_ALARM            18
1,   // S_DISPLAYGPS                20
1,   // S_COORDINATES               21
1,   // S_GPSCOORDTOP               22
0,   // S_GPSALTITUDE               23
0,   // S_ANGLETOHOME               24
0,   // S_SHOWHEADING               25
1,   // S_HEADING360                26
0,   // S_UNITSYSTEM                27
0,   // S_VIDEOSIGNALTYPE           28
0,   // S_THROTTLEPOSITION          29
1,   // S_DISPLAY_HORIZON_BR        30
1,   // S_WITHDECORATION            31
0,   // S_SHOWBATLEVELEVOLUTION     32
0,   // S_RESETSTATISTICS           33
1,   // S_MAPMODE                   34
0,   // S_VREFERENCE,               34a
0,   // S_USE_BOXNAMES              35
1,   // S_MODEICON                  36
0,   // S_DISPLAY_CS,               37
0,   // GPStime                     37a
0,   // GPSTZ +/-                   37b
0,   // GPSTZ                       37c
0,   // DEBUG                       37e
1,   // SCROLLING LADDERS           37f
1,   // SHOW GIMBAL ICON            37g
1,   // SHOW VARIO                  37h
1,   // SHOW BAROALT                38h 50
1,   // SHOW COMPASS                39h
0,   // S_HORIZON_ELEVATION         40h
1,   // S_TIMER                     41h
1,   // S_MODESENSOR                42h
0,   // S_SIDEBARTOPS               43h
4,   // S_UNUSED_6,
0,   // S_UNUSED_1, S_AMPMAXL,
0,   // S_UNUSED_2, S_AMPMAXH,
0,   // S_RCWSWITCH,
4,   // S_RCWSWITCH_CH,
0,   // S_HUDSW0, LOW / NORMAL
1,   // S_HUDSW1, HIGH / OSDSW
0,   // S_HUDSW2, MID
100, // S_DISTANCE_ALARM,
100, // S_ALTITUDE_ALARM,
100, // S_SPEED_ALARM,
30,  // S_FLYTIME_ALARM
0x53,   // S_CS0,
0x48,   // S_CS1,
0x49,   // S_CS2,
0x4B,   // S_CS3,
0x49,   // S_CS4,
0x20,   // S_CS5,
0x20,   // S_CS6,
0x20,   // S_CS7,
0x20,   // S_CS8,
0x20,   // S_CS9,

};

const uint16_t EEPROM16_DEFAULT[] PROGMEM = {
  0,// S16_AMPMAX,
  0,// S16_AMPZERO,
  150,// S16_AMPDIVIDERRATIO,
  0,// S16_RSSIMIN,
  1024,// S16_RSSIMAX,
  500,// S16_SPARE1,
  600,// S16_SPARE2,

};

const uint16_t SCREENLAYOUT_DEFAULT[] PROGMEM = {

(LINE02+2)|DISPLAY_ALWAYS,  // GPS_numSatPosition
(LINE02+22)|DISPLAY_ALWAYS,   // GPS_directionToHomePosition
(LINE02+24)|DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
(LINE07+3)|DISPLAY_ALWAYS,   // speedPosition
(LINE05+24)|DISPLAY_ALWAYS,   // GPS_angleToHomePosition
(LINE03+24)|DISPLAY_ALWAYS,   // MwGPSAltPosition
(LINE02+6)|DISPLAY_ALWAYS,   // sensorPosition
(LINE04+24)|DISPLAY_ALWAYS,   // MwHeadingPosition
(LINE02+10)|DISPLAY_ALWAYS,   // MwHeadingGraphPosition
(LINE07+23)|DISPLAY_ALWAYS,   // MwAltitudePosition
(LINE07+22)|DISPLAY_ALWAYS,   // MwClimbRatePosition
(LINE12+22)|DISPLAY_ALWAYS,   // CurrentThrottlePosition
(LINE13+22)|DISPLAY_ALWAYS,   // UNUSED flyTimePosition
(LINE13+22)|DISPLAY_ALWAYS,   // onTimePosition
(LINE11+11)|DISPLAY_ALWAYS,   // motorArmedPosition
(LINE10+2)|DISPLAY_NEVER,   // pitchAnglePosition
(LINE10+15)|DISPLAY_NEVER,   // rollAnglePosition
(LINE01+2)|DISPLAY_ALWAYS,   // MwGPSLatPositionTop      // On top of screen
(LINE01+15)|DISPLAY_ALWAYS,   // MwGPSLonPositionTop      // On top of screen
(LINE12+3)|DISPLAY_ALWAYS,   // rssiPosition
(LINE09+3)|DISPLAY_ALWAYS,   // temperaturePosition
(LINE13+3)|DISPLAY_ALWAYS,  // voltagePosition
(LINE11+3)|DISPLAY_ALWAYS,   // vidvoltagePosition
(LINE13+9)|DISPLAY_ALWAYS,   // amperagePosition
(LINE13+16)|DISPLAY_ALWAYS,   // pMeterSumPosition
(LINE07+14)|DISPLAY_ALWAYS,   // horizonPosition
(LINE07+7)|DISPLAY_ALWAYS,   // SideBarPosition
(LINE07+7)|DISPLAY_ALWAYS,   // SideBarScrollPosition
(3)|DISPLAY_NEVER,   // SideBarHeight Position
(7)|DISPLAY_NEVER,   // SideBarWidth Position
(LINE05+2)|DISPLAY_ALWAYS,   // Gimbal Position
(LINE12+11)|DISPLAY_ALWAYS,  // GPS_time Position
(LINE09+22)|DISPLAY_ALWAYS,   // SportPosition
(LINE04+2)|DISPLAY_ALWAYS,   // modePosition
(LINE02+22)|DISPLAY_NEVER,   // MapModePosition
(LINE07+15)|DISPLAY_NEVER,   // MapCenterPosition
(LINE04+10)|DISPLAY_ALWAYS,   // APstatusPosition
(LINE12+9)|DISPLAY_NEVER,   // wattPosition
(LINE07+6)|DISPLAY_NEVER,   // glidescopePosition
(LINE10+10)|DISPLAY_ALWAYS,   // CallSign Position
(LINE08+10)|DISPLAY_ALWAYS,   // Debug Position

};


const uint16_t SCREENLAYOUT_DEFAULT_OSDSW[] PROGMEM = {

(LINE02+2)|DISPLAY_NEVER,  // GPS_numSatPosition
(LINE13+19)|DISPLAY_ALWAYS,   // GPS_directionToHomePosition
(LINE02+12)|DISPLAY_NEVER,   // GPS_distanceToHomePosition
(LINE02+3)|DISPLAY_NEVER,   // speedPosition
(LINE05+24)|DISPLAY_NEVER,   // GPS_angleToHomePosition
(LINE03+24)|DISPLAY_NEVER,   // MwGPSAltPosition
(LINE02+6)|DISPLAY_NEVER,   // sensorPosition
(LINE04+24)|DISPLAY_NEVER,   // MwHeadingPosition
(LINE02+9)|DISPLAY_NEVER,   // MwHeadingGraphPosition
(LINE02+23)|DISPLAY_NEVER,   // MwAltitudePosition
(LINE07+23)|DISPLAY_NEVER,   // MwClimbRatePosition
(LINE12+22)|DISPLAY_NEVER,   // CurrentThrottlePosition
(LINE13+22)|DISPLAY_ALWAYS,   // UNUSED flyTimePosition
(LINE13+22)|DISPLAY_ALWAYS,   // onTimePosition
(LINE11+11)|DISPLAY_ALWAYS,   // motorArmedPosition
(LINE10+2)|DISPLAY_NEVER,   // pitchAnglePosition
(LINE10+15)|DISPLAY_NEVER,   // rollAnglePosition
(LINE01+2)|DISPLAY_NEVER,   // MwGPSLatPositionTop      // On top of screen
(LINE01+15)|DISPLAY_NEVER,   // MwGPSLonPositionTop      // On top of screen
(LINE12+2)|DISPLAY_NEVER,   // rssiPosition
(LINE09+2)|DISPLAY_NEVER,   // temperaturePosition
(LINE13+3)|DISPLAY_ALWAYS,  // voltagePosition
(LINE11+3)|DISPLAY_ALWAYS,   // vidvoltagePosition
(LINE13+13)|DISPLAY_NEVER,   // amperagePosition
(LINE13+23)|DISPLAY_NEVER,   // pMeterSumPosition
(LINE07+14)|DISPLAY_NEVER,   // AHIPosition
(LINE07+7)|DISPLAY_NEVER,   // horizonPosition
(LINE07+7)|DISPLAY_NEVER,   // SideBarPosition
(3)|DISPLAY_NEVER,   // SideBarHeight Position
(7)|DISPLAY_NEVER,   // SideBarWidth Position
(LINE05+2)|DISPLAY_NEVER,   // Gimbal Position
(LINE12+11)|DISPLAY_NEVER,  // GPS_time Position
(LINE09+22)|DISPLAY_NEVER,   // SportPosition
(LINE04+2)|DISPLAY_NEVER,   // modePosition
(LINE02+22)|DISPLAY_NEVER,   // MapModePosition
(LINE07+17)|DISPLAY_NEVER,   // MapCenterPosition
(LINE04+10)|DISPLAY_NEVER,   // APstatusPosition
(LINE12+13)|DISPLAY_NEVER,   // wattPosition
(LINE07+6)|DISPLAY_NEVER,   // glidescopePosition
(LINE10+10)|DISPLAY_NEVER,   // CallSign Position
(LINE08+10)|DISPLAY_ALWAYS,   // Debug Position

};

#endif
