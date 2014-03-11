
#define POS_MASK        0x01FF
#define PAL_MASK        0x0003
#define PAL_SHFT             9
#define DISPLAY_MASK    0xC000
#define DISPLAY_ALWAYS  0xC000
#define DISPLAY_NEVER   0x0000
#define DISPLAY_COND    0x4000
#define DISPLAY_MIN_OFF     0x8000

#define POS(pos, pal_off, disp)  (((pos)&POS_MASK)|((pal_off)<<PAL_SHFT)|(disp))

#if defined SHIFTDOWN
#define TOPSHIFT        LINE
#else
#define TOPSHIFT        0
#endif

const uint16_t screenPosition[] PROGMEM = {
  POS(LINE02+2+TOPSHIFT,  0, OSDOFF01 ), // GPS_numSatPosition
  POS(LINE01+13+TOPSHIFT,  0, OSDOFF02), // GPS_numSatPositionTop      // On top of screen
  POS(LINE02+22+TOPSHIFT, 0, OSDOFF03), // GPS_directionToHomePosition
  POS(LINE13+20, 2, OSDOFF03), // GPS_directionToHomePositionBottom
  POS(LINE02+24+TOPSHIFT, 0, OSDOFF04), // GPS_distanceToHomePosition
  POS(LINE07+3, 1, OSDOFF05), // speedPosition
  POS(LINE05+24+TOPSHIFT, 0, OSDOFF06), // GPS_angleToHomePosition
  POS(LINE03+24+TOPSHIFT, 0, OSDOFF07), // MwGPSAltPosition
  POS(LINE03+2,  0, OSDOFF08), // sensorPosition
  POS(LINE04+24+TOPSHIFT, 0, OSDOFF09), // MwHeadingPosition
  POS(LINE02+9+TOPSHIFT, 0, OSDOFF10), // MwHeadingGraphPosition
  POS(LINE07+23,  1, OSDOFF11), // MwAltitudePosition
  POS(LINE03+23+TOPSHIFT, 0, OSDOFF12), // MwClimbRatePosition
  POS(LINE12+22, 2, OSDOFF13), // CurrentThrottlePosition
  POS(LINE13+22, 2, OSDOFF14), // flyTimePosition
  POS(LINE13+22, 2, OSDOFF15), // onTimePosition
  POS(LINE11+11, 2, OSDOFF16), // motorArmedPosition
  POS(LINE10+2,  2, OSDOFF17),  // MwGPSLatPosition
  POS(LINE10+15, 2, OSDOFF18),  // MwGPSLonPosition
  POS(LINE01+2+TOPSHIFT,  0, OSDOFF19),  // MwGPSLatPositionTop      // On top of screen 
  POS(LINE01+15+TOPSHIFT, 0, OSDOFF20),  // MwGPSLonPositionTop      // On top of screen  
  POS(LINE12+2,  2, OSDOFF21), // rssiPosition
  POS(LINE09+2,  2, OSDOFF22), // temperaturePosition
  POS(LINE13+3,  2, OSDOFF23), // voltagePosition
  POS(LINE11+3,  2, OSDOFF24), // vidvoltagePosition
  POS(LINE13+10, 2, OSDOFF25), // amperagePosition
  POS(LINE13+16, 2, OSDOFF26), // pMeterSumPosition
  POS(LINE05+7,  1, OSDOFF27),  // horizonPosition
#ifdef CALLSIGNALWAYS
  POS(CALLSIGNALWAYS, 2, OSDOFF28), // CallSign Position
#else
  POS(LINE10+10, 2, OSDOFF28), // CallSign Position
#endif
  POS(LINE08+10, 2, OSDOFF29), // Debug Position
  POS(LINE05+2,  0, OSDOFF08), // Gimbal Position
  POS(LINE12+11, 2, OSDOFF31), // GPS_time Position
};

uint16_t getPosition(uint8_t pos) {
  uint16_t val = (uint16_t)pgm_read_word(&screenPosition[pos]);
  uint16_t ret = val&POS_MASK;

  if(Settings[S_VIDEOSIGNALTYPE]) {
    ret += LINE * ((val >> PAL_SHFT) & PAL_MASK);
  }

  return ret;
}

uint8_t fieldIsVisible(uint8_t pos) {
  uint16_t val = (uint16_t)pgm_read_word(&screenPosition[pos]);
  switch(val & DISPLAY_MASK) {
  case DISPLAY_ALWAYS:
    return 1;
  case DISPLAY_NEVER:
    return 0;
  case DISPLAY_COND:
    return !!(MwSensorActive&mode_osd_switch);
  case DISPLAY_MIN_OFF:
    return !(MwSensorActive&mode_osd_switch);
  }
}
