#include "platform.h"

// Heading
const char headGraph[] PROGMEM = {
  0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d,0x1c,0x1d,0x18,0x1d,0x1c,0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d};

// GPS
#if defined GPSOSD
  const char satnogps_text[] PROGMEM = " NO GPS ";
#endif

// Status
const char blank_text[] PROGMEM     = "";
const char nodata_text[] PROGMEM    = "NO DATA";
const char nogps_text[] PROGMEM     = " NO GPS";
const char satlow_text[] PROGMEM    = "LOW SATS";
const char disarmed_text[] PROGMEM  = "DISARMED";
const char armed_text[] PROGMEM     = " ARMED";
const char APRTHtext[] PROGMEM      = "AUTO RTH";
const char APHOLDtext[] PROGMEM     = "AUTO HOLD";
const char APWAYPOINTtext[] PROGMEM = " MISSION";
const char lowvolts_text[] PROGMEM  = "LOW VOLTS";

// For Status / warning messages
const char * const message_item[] PROGMEM =
{
  blank_text,  //0
  disarmed_text,  //1
  armed_text,     //2
  nodata_text,
  nogps_text,
  satlow_text,
  APRTHtext,
  APHOLDtext,
  APWAYPOINTtext,
  lowvolts_text,
};


// For Intro
#ifdef INTRO_VERSION
const char message0[] PROGMEM = INTRO_VERSION;
#else
const char message0[] PROGMEM = MWVERS;
#endif

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
const char messageF0[] PROGMEM = "DO NOT POWER OFF";
const char messageF1[] PROGMEM = "SCREEN WILL GO BLANK";
const char messageF2[] PROGMEM = "UPDATE COMPLETE";
#endif

//const char message1[] PROGMEM = "VIDEO SIGNAL NTSC";
//const char message2[] PROGMEM = "VIDEO SIGNAL PAL ";
const char message5[]  PROGMEM = "FC VERSION:";
const char message6[]  PROGMEM = "OPEN MENU: THRT MIDDLE";
const char message7[]  PROGMEM = "+YAW RIGHT";
const char message8[]  PROGMEM = "+PITCH FULL";
const char message9[]  PROGMEM = "ID:";         // Call Sign on the beggining of the transmission
const char message10[] PROGMEM = "TZ UTC:"; //haydent - Time Zone & DST Setting
//const char message11[] PROGMEM = "MORE IN: GUI+CONFIG.H";

// For Config menu common
const char configMsgON[]   PROGMEM = "ON";
const char configMsgOFF[]  PROGMEM = "OFF";
const char configMsgEXT[]  PROGMEM = "EXIT";
const char configMsgSAVE[] PROGMEM = "SAVE+EXIT";
const char configMsgPGS[]  PROGMEM = "<PAGE>";
const char configMsgMWII[] PROGMEM = "USE FC";

// For APSTATUS

// For Config pages
//-----------------------------------------------------------Page0
const char configMsg00[] PROGMEM = "STATISTICS";
const char configMsg01[] PROGMEM = "FLY TIME";
const char configMsg02[] PROGMEM = "TOT DISTANCE";
const char configMsg03[] PROGMEM = "MAX DISTANCE";
const char configMsg04[] PROGMEM = "MAX ALTITUDE";
const char configMsg05[] PROGMEM = "MAX SPEED";
const char configMsg06[] PROGMEM = "MAH USED";
const char configMsg07[] PROGMEM = "MAX AMPS";
//-----------------------------------------------------------Page1
const char configMsg10[] PROGMEM = "PID CONFIG";
const char configMsg11[] PROGMEM = "ROLL";
const char configMsg12[] PROGMEM = "PITCH";
const char configMsg13[] PROGMEM = "YAW";
const char configMsg14[] PROGMEM = "ALT";
const char configMsg15[] PROGMEM = "GPS";
const char configMsg16[] PROGMEM = "LEVEL";
const char configMsg17[] PROGMEM = "MAG";
//-----------------------------------------------------------Page2
const char configMsg20[] PROGMEM = "RC TUNING";
const char configMsg21[] PROGMEM = "RC RATE";
const char configMsg22[] PROGMEM = "RC EXPO";
const char configMsg23[] PROGMEM = "ROLL PITCH RATE";
const char configMsg24[] PROGMEM = "YAW RATE";
const char configMsg25[] PROGMEM = "TPA";
const char configMsg26[] PROGMEM = "THROTTLE MID";
const char configMsg27[] PROGMEM = "THROTTLE EXPO";
#if defined CORRECT_MENU_RCT1
  const char configMsg23a[] PROGMEM = "ROLL RATE";
  const char configMsg23b[] PROGMEM = "PITCH RATE";
#endif
#if defined CORRECT_MENU_RCT2
  const char configMsg23a[] PROGMEM = "ROLL RATE";
  const char configMsg23b[] PROGMEM = "PITCH RATE";
  const char configMSg28[]  PROGMEM = "TPA BREAKPOINT";
#endif
//-----------------------------------------------------------Page3
const char configMsg30[] PROGMEM = "VOLTAGE";
const char configMsg31[] PROGMEM = "DISPLAY MAIN VOLTS";
const char configMsg32[] PROGMEM = "ADJUST VOLTS";
const char configMsg33[] PROGMEM = "MAIN VOLTS ALARM";
const char configMsg34[] PROGMEM = "DISPLAY VID VOLTS";
const char configMsg35[] PROGMEM = "ADJUST VOLTS";
const char configMsg36[] PROGMEM = "CELLS";
const char configMsg37[] PROGMEM = "USE FC";

//-----------------------------------------------------------Page4
const char configMsg40[] PROGMEM = "RSSI";
const char configMsg42[] PROGMEM = "DISPLAY RSSI";
const char configMsg43[] PROGMEM = "SET RSSI";
const char configMsg44[] PROGMEM = "SET RSSI MAX";
const char configMsg45[] PROGMEM = "SET RSSI MIN";
const char configMsg46[] PROGMEM = "USE PWM";

//-----------------------------------------------------------Page5
const char configMsg50[] PROGMEM = "CURRENT";
const char configMsg51[] PROGMEM = "DISPLAY AMPS";
const char configMsg52[] PROGMEM = "DISPLAY MAH";
const char configMsg53[] PROGMEM = "USE VIRTUAL SENSOR";
const char configMsg54[] PROGMEM = "ADJUST AMPS";
const char configMsg55[] PROGMEM = "ADJUST ZERO";
//-----------------------------------------------------------Page6
const char configMsg60[] PROGMEM = "DISPLAY";
const char configMsg61[] PROGMEM = "HORIZON";
const char configMsg62[] PROGMEM = "SIDE BARS";
const char configMsg63[] PROGMEM = "SCROLLING BARS";
const char configMsg64[] PROGMEM = "THROTTLE";
const char configMsg65[] PROGMEM = "GPS COORDS";
const char configMsg66[] PROGMEM = "SENSORS";
const char configMsg67[] PROGMEM = "GIMBAL";
const char configMsg68[] PROGMEM = "MAP MODE";
//-----------------------------------------------------------Page7
const char configMsg70[]  PROGMEM = "ADVANCED";
const char configMsg71[]  PROGMEM = "UNITS";
const char configMsg710[] PROGMEM = "MET";
const char configMsg711[] PROGMEM = "IMP";
const char configMsg72[]  PROGMEM = "SIGNAL";
const char configMsg720[] PROGMEM = "NTSC";
const char configMsg721[] PROGMEM = "PAL";
const char configMsg73[]  PROGMEM = "V REF";
const char configMsg730[] PROGMEM = "5V";
const char configMsg731[] PROGMEM = "1.1V";
const char configMsg74[]  PROGMEM = "DEBUG";
const char configMsg75[]  PROGMEM = "MAG CAL";
const char configMsg76[]  PROGMEM = "OSD TX CH";
//-----------------------------------------------------------Page8
const char configMsg80[] PROGMEM = "GPS TIME";
const char configMsg81[] PROGMEM = "DISPLAY";
const char configMsg82[] PROGMEM = "TZ FORWARD";
const char configMsg83[] PROGMEM = "TZ ADJUST";
//-----------------------------------------------------------Page9
const char configMsg90[] PROGMEM = "ALARMS";
const char configMsg91[] PROGMEM = "DISTANCE X100";
const char configMsg92[] PROGMEM = "ALTITUDE X10";
const char configMsg93[] PROGMEM = "SPEED";
const char configMsg94[] PROGMEM = "TIMER";
const char configMsg95[] PROGMEM = "MAH X100";
const char configMsg96[] PROGMEM = "AMPS";
//-----------------------------------------------------------Page10
const char configMsg100[] PROGMEM = "ADVANCE TUNING";
const char configMsg101[] PROGMEM = "PROFILE";
const char configMsg102[] PROGMEM = "PID CONTROLLER";
const char configMsg103[] PROGMEM = "LOOPTIME";

// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const char speedUnitAdd[2] PROGMEM = {
  0xa5,0xa6} ; // [0][0] and [0][1] = Km/h   [1][0] and [1][1] = Mph
const char temperatureUnitAdd[2] PROGMEM = {
  0x0e,0x0d};
const char MwAltitudeAdd[2] PROGMEM = {
  0xa7,0xa8};
const char MwClimbRateAdd[2] PROGMEM = {
  0x9f,0x99};
const char GPS_distanceToHomeAdd[2] PROGMEM = {
  0xbb,0xb9};
const char MwGPSAltPositionAdd[2] PROGMEM = {
  0xa7,0xa8};

// Menu selections
const char * const menu_choice_unit[] PROGMEM =
{
  configMsg710,
  configMsg711,
};
// Menu selections
const char * const menu_choice_video[] PROGMEM =
{
  configMsg720,
  configMsg721,
};// Menu selections
const char * const menu_choice_ref[] PROGMEM =
{
  configMsg731,
  configMsg730,
};

// Menu
const char * const menu_stats_item[] PROGMEM =
{
  configMsg01,
  configMsg02,
  configMsg03,
  configMsg04,
  configMsg05,
  configMsg06,
  configMsg07,
};

const char * const menu_pid[] PROGMEM =
{
  configMsg11,
  configMsg12,
  configMsg13,
  configMsg14,
  configMsg15,
  configMsg16,
  configMsg17,
};

const char * const menu_rc[] PROGMEM =
{
  #if defined CORRECT_MENU_RCT2
    configMsg21,
    configMsg22,
    configMsg23a,
    configMsg23b,
    configMsg24,
    configMsg25,
    configMsg26,
    configMsg27,
    configMSg28,
  #elif defined CORRECT_MENU_RCT1
    configMsg21,
    configMsg22,
    configMsg23a,
    configMsg23b,
    configMsg24,
    configMsg25,
    configMsg26,
    configMsg27,
  #else //BF original / Cleanflight original / multiwii
    configMsg21,
    configMsg22,
    configMsg23,
    configMsg24,
    configMsg25,
    configMsg26,
    configMsg27,
  #endif

};

const char * const menu_bat[] PROGMEM =
{
  configMsg31,
  configMsg32,
  configMsg33,
  configMsg34,
  configMsg32,
  configMsg36,
  configMsgMWII,
};

const char * const menu_rssi[] PROGMEM =
{
  configMsg42,
  configMsg43,
  configMsgMWII,
  configMsg46,
  configMsg44,
  configMsg45,
};

const char * const menu_amps[] PROGMEM =
{
  configMsg51,
  configMsg52,
  configMsg53,
  configMsg54,
  configMsg55,
};

const char * const menu_display[] PROGMEM =
{
  configMsg61,
  configMsg62,
  configMsg63,
  configMsg64,
  configMsg65,
  configMsg66,
  configMsg67,
  configMsg68,
};

const char * const menu_advanced[] PROGMEM =
{
  configMsg71,
  configMsg72,
  configMsg73,
  configMsg74,
  configMsg75,
  configMsg76,};

const char * const menu_gps_time[] PROGMEM =
{
  configMsg81,
  configMsg82,
  configMsg83,
};

const char * const menu_alarm_item[] PROGMEM =
{
  configMsg91,
  configMsg92,
  configMsg93,
  configMsg94,
  configMsg95,
  configMsg96,
};

const char * const menu_profile[] PROGMEM =
{
  configMsg101,
  configMsg102,
  configMsg103,
};

const char * const menutitle_item[] PROGMEM =
{
#ifdef MENU_STAT
  configMsg00,
#endif
#ifdef MENU_PID
  configMsg10,
#endif
#ifdef MENU_RC
  configMsg20,
#endif
#ifdef MENU_VOLTAGE
  configMsg30,
#endif
#ifdef MENU_RSSI
  configMsg40,
#endif
#ifdef MENU_CURRENT
  configMsg50,
#endif
#ifdef MENU_DISPLAY
  configMsg60,
#endif
#ifdef MENU_ADVANCED
  configMsg70,
#endif
#ifdef MENU_GPS_TIME
  configMsg80,
#endif
#ifdef MENU_ALARMS
  configMsg90,
#endif
#ifdef MENU_PROFILE
  configMsg100,
#endif
};

const char * const menu_on_off[] PROGMEM =
{
  configMsgOFF,
  configMsgON,
};

char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *ItoaUnPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}
char *FormatGPSCoord(int32_t val, char *str, uint8_t p, char pos, char neg) {
  if(val < 0) {
    pos = neg;
    val = -val;
  }

  uint8_t bytes = p+6;
  val = val / 100;

  str[bytes] = 0;
  str[--bytes] = pos;
  for(;;) {
    if(bytes == p) {
      str[--bytes] = DECIMAL;
      continue;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (bytes < 3 && val == 0))
      break;   }

   while(bytes != 0)
     str[--bytes] = ' ';

   return str;
}

// Take time in Seconds and format it as 'MM:SS'
// Alternately Take time in Minutes and format it as 'HH:MM'
// If hhmmss is 1, display as HH:MM:SS
char *formatTime(uint32_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}

uint8_t ScreenClass::findNull(void)
{
  uint8_t xx;
  for(xx=0;_screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}


uint16_t ScreenClass::getPosition(uint8_t pos)
{
  uint16_t val = _screenPosition[pos];
  uint16_t ret = val&POS_MASK;
  return ret;
}

uint8_t ScreenClass::fieldIsVisible(uint8_t pos)
{
//  uint16_t val = (uint16_t)pgm_read_word(&_screenPosition[pos]);
  uint16_t val = _screenPosition[pos];
  if ((val & DISPLAY_MASK)==DISPLAY_ALWAYS)
    return 1;
  else
    return 0;
/*
  switch(val & DISPLAY_MASK) {
  case DISPLAY_ALWAYS:
    return 1;
  case DISPLAY_NEVER:
    return 0;
  case DISPLAY_COND:
    return !!((mode.osd_switch));
  case DISPLAY_MIN_OFF:
    return !((mode.osd_switch));
  }
*/
}


void ScreenClass::DisplayTemperature(void)        // DEPRECATED RUSHDUINO SUPPORT
{
  int xxx;
  if (Settings[S_UNITSYSTEM])
    xxx = mwosd.temperature*1.8+32;       //Fahrenheit conversion for imperial system.
  else
    xxx = mwosd.temperature;

//  if(!fieldIsVisible(temperaturePosition))
//    return;

  itoa(xxx,_screenBuffer,10);
  uint8_t xx = findNull();   // find the NULL
  _screenBuffer[xx++]=temperatureUnitAdd[Settings[S_UNITSYSTEM]];
  _screenBuffer[xx]=0;  // Restore the NULL
  Max.WriteString(_screenBuffer,getPosition(temperaturePosition));
}


void ScreenClass::DisplayMode(void)
{
  if(timer.MSP_active==0){ // no MSP >> mode display not valid
    return;
  }
  uint8_t xx = 0;

  if(((mode.camstab))&&Settings[S_GIMBAL]){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_GIMBAL;
    _screenBuffer[1]=SYM_GIMBAL1;
    if(fieldIsVisible(gimbalPosition))
      Max.WriteString(_screenBuffer,getPosition(gimbalPosition));
  }
  if(Settings[S_MODESENSOR]){
    xx = 0;
    if((mode.stable)||Sensors.IsActive(mode.horizon)){
      _screenBuffer[xx] = SYM_ACC;
      xx++;
    }
    if ((mode.baro)){
      _screenBuffer[xx] = SYM_BAR;
      xx++;
    }
    if ((mode.mag)){
      _screenBuffer[xx] = SYM_MAG;
      xx++;
    }
    _screenBuffer[xx] = 0;
    if(fieldIsVisible(sensorPosition)){
      Max.WriteString(_screenBuffer,getPosition(sensorPosition));
    }
  }


  if((mode.passthru)){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_PASS;
    _screenBuffer[1]=SYM_PASS1;
  }
  else if((mode.gpshome)){
#ifdef APINDICATOR
    _screenBuffer[0] = SYM_GHOME;
    _screenBuffer[1] = SYM_GHOME1;
    _screenBuffer[2]=0;
#else
    uint32_t dist;
    if(Settings[S_UNITSYSTEM]){
      dist = Gps.distanceToHome * 3.2808;           // mt to feet
    }
    else{
      dist = Gps.distanceToHome;                    // Mt
    }
    itoa(dist, _screenBuffer+3, 10);
    xx = findNull()+1;

    if(Settings[S_UNITSYSTEM]==METRIC){
      _screenBuffer[xx] =SYM_M;
    }
    else{
     _screenBuffer[xx] =SYM_FT;
    }
    _screenBuffer[xx] =0;
    _screenBuffer[0] = SYM_GHOME;
    _screenBuffer[1] = SYM_GHOME1;
    _screenBuffer[2] = SYM_COLON;
    _screenBuffer[8]=0;
#endif
  }
  else if((mode.gpshold)){
    _screenBuffer[2]=0;
    _screenBuffer[0] = SYM_GHOLD;
    _screenBuffer[1] = SYM_GHOLD1;
  }
#if defined MULTIWII_V24
  else if((mode.gpsmission)){
    itoa(Gps.waypoint_step,_screenBuffer+2,10);
    _screenBuffer[4]=0;
    _screenBuffer[0] = SYM_GMISSION;
    _screenBuffer[1] = SYM_GMISSION1;
  }
  else if((mode.gpsland)){
    _screenBuffer[2]=0;
    _screenBuffer[0] = SYM_GLAND;
    _screenBuffer[1] = SYM_GLAND1;
  }
#endif //MULTIWII_V24
  else if((mode.stable)){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_STABLE;
    _screenBuffer[1]=SYM_STABLE1;
  }
  else if((mode.horizon)){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_HORIZON;
    _screenBuffer[1]=SYM_HORIZON1;
  }
  else{
    _screenBuffer[2]=0;
    #ifdef FIXEDWING
      _screenBuffer[0]=SYM_ACROGY;
    #else
      _screenBuffer[0]=SYM_ACRO;
    #endif
    _screenBuffer[1]=SYM_ACRO1;
#if defined ACROPLUS
    if((mode.acroplus)){
      _screenBuffer[1]=SYM_PLUS;
    }
#endif //ACROPLUS
  }
  if(Settings[S_MODEICON]){
    if(fieldIsVisible(ModePosition)){
      Max.WriteString(_screenBuffer,getPosition(ModePosition));
    #ifdef AIRMODE
      if(Sensors.IsActive(mode.air)){
        _screenBuffer[0]=SYM_AIR;
        _screenBuffer[1]=SYM_AIR1;
        _screenBuffer[2]=0;
      Max.WriteString(_screenBuffer,getPosition(ModePosition)+AIRMODE);
      }
    #endif //AIRMODE
    }
  }

#ifdef APINDICATOR
  if(timer.Blink2hz)
    return;
  if(!fieldIsVisible(APstatusPosition))
    return;
  uint8_t apactive=0;
  if ((mode.gpshome))
    apactive=6;
  else if ((mode.gpshold))
    apactive=7;
  else if ((mode.gpsmission))
    apactive=8;
  else
    return;
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[apactive])));
  Max.WriteString(_screenBuffer, getPosition(APstatusPosition));
#endif

}


void ScreenClass::DisplayCallsign()
{
      for(uint8_t X=0; X<10; X++) {
          _screenBuffer[X] = char(Settings[S_CS0 + X]);
     }
       _screenBuffer[10] = 0;
       Max.WriteString(_screenBuffer, getPosition(callSignPosition));
}


void ScreenClass::DisplayHorizon(int rollAngle, int pitchAngle)
{
  // TODO: $$$ #define guard
  static uint8_t sidebarsdir; // speed
  static uint8_t sidebaradir; // alt
  static unsigned long sidebarsMillis = 0;
  static unsigned long sidebaraMillis = 0;

#ifdef DISPLAY_PR
  _screenBuffer[0]=0x50;
  int16_t xx=abs(pitchAngle/10);
  uint8_t offset=1;
  if(pitchAngle<0) {
    _screenBuffer[1]='-';
    offset++;
  }
  itoa(xx, _screenBuffer+offset,10);
  if(fieldIsVisible(pitchAnglePosition))
    Max.WriteString(_screenBuffer,getPosition(pitchAnglePosition));
  _screenBuffer[0]=0x52;
  offset=1;
  xx=abs(rollAngle/10);
  if(rollAngle<0) {
    _screenBuffer[1]='-';
    offset++;
  }
  itoa(xx, _screenBuffer+offset,10);
  if(fieldIsVisible(rollAnglePosition))
    Max.WriteString(_screenBuffer,getPosition(rollAnglePosition));
#endif

#ifdef HORIZON
  if (Settings[S_SCROLLING]||Settings[S_SIDEBARTOPS]){
      if(!mwosd.armed) Gps.speed=0;
  // Scrolling decoration
      if (Gps.speed > (Gps.old_speed+15)){
        sidebarsMillis = millis();
        sidebarsdir = 2;
        Gps.old_speed = Gps.speed;
        SYM_AH_DECORATION_LEFT--;
        if (SYM_AH_DECORATION_LEFT<0x10)
          SYM_AH_DECORATION_LEFT=0x15;
      }
      else if ((Gps.speed+15) < Gps.old_speed){
        sidebarsMillis = millis();
        sidebarsdir = 1;
       Gps.old_speed = Gps.speed;
        SYM_AH_DECORATION_LEFT++;
        if (SYM_AH_DECORATION_LEFT>0x15)
          SYM_AH_DECORATION_LEFT=0x10;
      }

      if (Gps.MwAltitude > mwosd.old_MwAltitude+20){
        sidebaraMillis = millis();
        sidebaradir = 2;
        mwosd.old_MwAltitude = Gps.MwAltitude;
        SYM_AH_DECORATION_RIGHT--;
        if (SYM_AH_DECORATION_RIGHT<0x10)
          SYM_AH_DECORATION_RIGHT=0x15;
      }
      else if (Gps.MwAltitude+20 < mwosd.old_MwAltitude){
        sidebaraMillis = millis();
        sidebaradir = 1;
        mwosd.old_MwAltitude = Gps.MwAltitude;
        SYM_AH_DECORATION_RIGHT++;
        if (SYM_AH_DECORATION_RIGHT>0x15)
          SYM_AH_DECORATION_RIGHT=0x10;
      }
  }

  if (!Settings[S_SCROLLING]){
     SYM_AH_DECORATION_LEFT=0x13;
     SYM_AH_DECORATION_RIGHT=0x13;
  }

  uint16_t position = getPosition(horizonPosition);

  if(pitchAngle>AHIPITCHMAX) pitchAngle=AHIPITCHMAX;
  if(pitchAngle<-AHIPITCHMAX) pitchAngle=-AHIPITCHMAX;
  if(rollAngle>AHIROLLMAX) rollAngle=AHIROLLMAX;
  if(rollAngle<-AHIROLLMAX) rollAngle=-AHIROLLMAX;
  #ifndef AHICORRECT
    #define AHICORRECT 10
  #endif
  pitchAngle=pitchAngle+AHICORRECT;
  #if defined REVERSEAHI
    pitchAngle=-pitchAngle;
    rollAngle=-rollAngle;
  #endif //REVERSEAHI

  if(Settings[S_DISPLAY_HORIZON_BR]&fieldIsVisible(horizonPosition)){

  #ifdef FULLAHI
    for(uint8_t X=0; X<=12; X++) {
      if (X==6) X=7;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position -9 + LINE*(Y/9) + 3 - 4*LINE + X;
        Max.WriteChar(SYM_AH_BAR9_0+(Y%9), pos);
        if (Settings[S_HORIZON_ELEVATION]){
          if(X >= 4 && X <= 8) {
            Max.WriteChar(SYM_AH_BAR9_0+(Y%9), pos-3*LINE);
            Max.WriteChar(SYM_AH_BAR9_0+(Y%9), pos+3*LINE);
          }
        }
      }
    }
  #else //FULLAHI
    for(uint8_t X=0; X<=8; X++) {
      if (X==4) X=5;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position -7 + LINE*(Y/9) + 3 - 4*LINE + X;
        Max.WriteChar(SYM_AH_BAR9_0+(Y%9), pos);
        if (Settings[S_HORIZON_ELEVATION]){
          if(X >= 2 && X <= 6) {
            Max.WriteChar(SYM_AH_BAR9_0+(Y%9), pos-3*LINE);
            Max.WriteChar(SYM_AH_BAR9_0+(Y%9), pos+3*LINE);
          }
        }
      }
    }
  #endif //FULLAHI

    if(!fieldIsVisible(MapModePosition)){
      if(Settings[S_DISPLAY_HORIZON_BR]){
        Max.WriteChar(SYM_AH_CENTER_LINE, position-1);
        Max.WriteChar(SYM_AH_CENTER_LINE_RIGHT, position+1);
        Max.WriteChar(SYM_AH_CENTER, position);
      }
    }
  }

  if (Settings[S_WITHDECORATION]&fieldIsVisible(SideBarPosition)){
    // Draw AH sides
    int8_t hudwidth=  getPosition(SideBarWidthPosition)&0x0F;
    int8_t hudheight= getPosition(SideBarHeightPosition)&0x0F;
    for(int8_t X=-hudheight; X<=hudheight; X++) {
      Max.WriteChar(SYM_AH_DECORATION_LEFT, position-hudwidth+(X*LINE));
      Max.WriteChar(SYM_AH_DECORATION_RIGHT, position+hudwidth+(X*LINE));
    }
#if defined AHILEVEL
    Max.WriteChar(SYM_AH_LEFT, position-hudwidth+1);
    Max.WriteChar(SYM_AH_RIGHT, position+hudwidth-1);
#endif //AHILEVEL

  #if defined(USEGLIDESCOPE) && defined(FIXEDWING)
    if(Settings[S_DISPLAYGPS]){
      displayfwglidescope();
    }
  #endif //USEGLIDESCOPE

  #ifdef SBDIRECTION

    if (Settings[S_SIDEBARTOPS]&fieldIsVisible(SideBarScrollPosition)) {
      if (millis()<(sidebarsMillis + 1000)) {
        if (sidebarsdir == 2){
          Max.WriteChar(SYM_AH_DECORATION_UP, position-(hudheight*LINE)-hudwidth);
        }
        else{
          Max.WriteChar(SYM_AH_DECORATION_DOWN, position+(hudheight*LINE)-hudwidth);
        }
      }
      if (millis()<(sidebaraMillis + 1000)) {
        if (sidebaradir == 2){
          Max.WriteChar(SYM_AH_DECORATION_UP, position-(hudheight*LINE)+hudwidth);
        }
        else{
          Max.WriteChar(SYM_AH_DECORATION_DOWN, position+(hudheight*LINE)+hudwidth);
        }
      }
    }
  #endif //SBDIRECTION
  }
#endif //HORIZON
}


void ScreenClass::DisplayVoltage(void)
{
  if (Settings[S_MAINVOLTAGE_VBAT]){
    Sensors.voltage=Msp.MwVBat;
  }

#ifdef AUTOCELL
  uint8_t tcells = ((Sensors.voltage-3) / Msp.MvVBatMaxCellVoltage) + 1;
  if (tcells>_cells){
    if (tcells<11){
      _cells=tcells;
    }
  }
  Sensors.voltageWarning = _cells * Msp.MvVBatWarningCellVoltage;
 #ifdef AUTOCELL_VOLTAGE
  Sensors.voltageWarning = _cells * Settings[S_VOLTAGEMIN];
 #endif // AUTOCELL_VOLTAGE
#else //NOT AUTOCELL
  Sensors.voltageWarning = Settings[S_VOLTAGEMIN];
  _cells = Settings[S_BATCELLS];
#endif //AUTOCELL


#ifdef BATTERYICONVOLTS
  if (Settings[S_SHOWBATLEVELEVOLUTION])
  {
    uint8_t battev = 0;
    int batevlow  = _cells * Msp.MvVBatMinCellVoltage;
    int batevhigh = _cells * Msp.MvVBatMaxCellVoltage;
    battev = constrain(Sensors.voltage, batevlow, batevhigh-2);
    battev = map(battev, batevlow, batevhigh-1, 0, 7);
    _screenBuffer[0]=(SYM_BATT_EMPTY)-battev;
  }
  else
#endif // BATTERYICONVOLTS

  {
    _screenBuffer[0]=SYM_MAIN_BATT;
  }

  if ((Sensors.voltage<Sensors.voltageWarning)&&(timer.Blink2hz))
    return;

#ifdef DISP_LOW_VOLTS_WARNING
  if (Sensors.voltage<=Sensors.voltageWarning&&!timer.armed)
    Max.WriteString_P(lowvolts_text, getPosition(motorArmedPosition));
#endif

#ifdef FORCE_DISP_LOW_VOLTS
  if(fieldIsVisible(voltagePosition)||(Sensors.voltage<=Sensors.voltageWarning))
#else
  if(fieldIsVisible(voltagePosition))
#endif
  {
    ItoaPadded(Sensors.voltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    Max.WriteString(_screenBuffer,getPosition(voltagePosition)-1);
  }
}


void ScreenClass::DisplayVidVoltage(void)
{
  if((Sensors.vidvoltage<Sensors.vidvoltageWarning)&&(timer.Blink2hz))
    return;
#ifdef FORCE_DISP_LOW_VID_VOLTS
  if(fieldIsVisible(vidvoltagePosition)||(Sensors.vidvoltage<=Sensors.vidvoltageWarning))
#else
  if(fieldIsVisible(vidvoltagePosition))
#endif
  {
    _screenBuffer[0]=SYM_VID_BAT;
    ItoaPadded(Sensors.vidvoltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    Max.WriteString(_screenBuffer,getPosition(vidvoltagePosition)-1);
  }
}


void ScreenClass::DisplayCurrentThrottle(void)
{
  if(!fieldIsVisible(CurrentThrottlePosition))
    return;

  #ifndef NOTHROTTLESPACE
    #define THROTTLESPACE 1
  #else
    #define THROTTLESPACE 0
  #endif
  _screenBuffer[1]=' ';
  #ifdef AUTOTHROTTLE
    if (Msp.MwRcData[THROTTLESTICK] > rc.HighT) rc.HighT = Msp.MwRcData[THROTTLESTICK];
    if (Msp.MwRcData[THROTTLESTICK] < rc.LowT) rc.LowT = Msp.MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
    if (rc.HighT>2050) rc.HighT=2050;
    if (rc.LowT<950) rc.LowT=950;
  #else
    rc.HighT=HIGHTHROTTLE;
    rc.LowT=LOWTHROTTLE;
  #endif


  uint32_t Vthrottle = constrain(Msp.MwRcData[THROTTLESTICK],1000,2000);

#ifndef FIXEDWING
  if(!mwosd.armed) {
    _screenBuffer[0+THROTTLESPACE]=' ';
    _screenBuffer[1+THROTTLESPACE]=' ';
    _screenBuffer[2+THROTTLESPACE]='-';
    _screenBuffer[3+THROTTLESPACE]='-';
    _screenBuffer[4+THROTTLESPACE]=' ';

  }
  else
#endif // FIXEDWING
  {
    int CurThrottle = map(Msp.MwRcData[THROTTLESTICK],rc.LowT,rc.HighT,0,100);
    ItoaPadded(CurThrottle,_screenBuffer+1+THROTTLESPACE,3,0);
    _screenBuffer[4+THROTTLESPACE]='%';
  }
    _screenBuffer[0]=SYM_THR;
    _screenBuffer[5+THROTTLESPACE]=0;
    Max.WriteString(_screenBuffer,getPosition(CurrentThrottlePosition));
}


void ScreenClass::DisplayTime(void)
{
  if(!Settings[S_TIMER])
    return;
  if (_screenPosition[onTimePosition]<512)
    return;

  uint32_t displaytime;
  if (mwosd.armed) {
    if(((Stats.flyTime/60)>=Settings[S_FLYTIME_ALARM])&&(timer.Blink2hz))
      return;

    if(Stats.flyTime < 3600) {
      _screenBuffer[0] = SYM_FLY_M;
      displaytime = Stats.flyTime;
    }
    else {
      _screenBuffer[0] = SYM_FLY_H;
      displaytime = Stats.flyTime/60;
    }
  }
  else {
    if(Stats.onTime < 3600) {
      _screenBuffer[0] = SYM_ON_M;
      displaytime = Stats.onTime;
    }
    else {
      _screenBuffer[0] = SYM_ON_H;
      displaytime = Stats.onTime/60;
    }
  }
  formatTime(displaytime, _screenBuffer+1, 0);
  Max.WriteString(_screenBuffer,getPosition(onTimePosition));
}


void ScreenClass::DisplayAmperage(void)
{
  if(Stats.amperage > Stats._ampMAX)
    Stats._ampMAX = Stats.amperage;
  if(!fieldIsVisible(amperagePosition))
    return;
  ItoaPadded(Stats.amperage, _screenBuffer, 5, 4);     // 999.9 ampere max!
  _screenBuffer[5] = SYM_AMP;
  _screenBuffer[6] = 0;
  Max.WriteString(_screenBuffer,getPosition(amperagePosition));
}


void ScreenClass::DisplayWatt(void)
{
  if(!fieldIsVisible(wattPosition))
    return;
  uint16_t WhrPosition = getPosition(wattPosition);
  uint16_t watts = Stats.amperage*Sensors.voltage/100;
  ItoaPadded(watts, _screenBuffer+1 , 5, 5);
  _screenBuffer[0] = SYM_BLANK;
  _screenBuffer[5] = SYM_WATT;
  _screenBuffer[6] = 0;
  Max.WriteString(_screenBuffer,WhrPosition);
}


void ScreenClass::DisplaypMeterSum(void)
{
  if(!fieldIsVisible(pMeterSumPosition))
    return;

  #ifdef BATTERYICONAMPS
  uint16_t battev =0;
  if (Settings[S_SHOWBATLEVELEVOLUTION]){
    battev=Stats.amperagesum/(360*Settings[S_AMPER_HOUR_ALARM]);
    battev=constrain(battev,0,100);
    battev = map(100-battev, 0, 101, 0, 7);
    _screenBuffer[0]=SYM_BATT_EMPTY-battev;
    _screenBuffer[1]=SYM_MAH;
    int xx=Stats.amperagesum/360;
    itoa(xx,_screenBuffer+2,10);
  }
  else
  #endif //BATTERYICONAMPS
  {
  _screenBuffer[0]=SYM_MAH;
  int xx=Stats.amperagesum/360;
  itoa(xx,_screenBuffer+1,10);
  }
  Max.WriteString(_screenBuffer,getPosition(pMeterSumPosition));
}


void ScreenClass::DisplayI2CError(void)
{
#ifdef I2CERROR
  if (I2CError<=I2CERROR)
    return;
  _screenBuffer[0] = 0x49;
  _screenBuffer[1] = 0X3A;
  itoa(I2CError,_screenBuffer+2,10);
  Max.WriteString(_screenBuffer,getPosition(temperaturePosition));
#endif
}


void ScreenClass::DisplayRSSI(void)
{
  if(!fieldIsVisible(rssiPosition))
    return;
  _screenBuffer[0] = SYM_RSSI;
  itoa(rc.rssi,_screenBuffer+1,10);
  uint8_t xx = findNull();
  _screenBuffer[xx++] = '%';
  _screenBuffer[xx] = 0;
  Max.WriteString(_screenBuffer,getPosition(rssiPosition)-1);
}


void ScreenClass::DisplayHeading(void)
{
 if(!fieldIsVisible(MwHeadingPosition))
    return;
 if (Settings[S_SHOWHEADING]) {
   int16_t heading = Gps.MwHeading;
   if (Settings[S_HEADING360]) {
     if(heading < 0)
       heading += 360;
       ItoaPadded(heading,_screenBuffer,3,0);
       _screenBuffer[3]=SYM_DEGREES;
       _screenBuffer[4]=0;
    }
    else {
      ItoaPadded(heading,_screenBuffer,4,0);
      _screenBuffer[4]=SYM_DEGREES;
      _screenBuffer[5]=0;
    }
    Max.WriteString(_screenBuffer,getPosition(MwHeadingPosition));
  }
}


void ScreenClass::DisplayHeadingGraph(void)
{
  if (!fieldIsVisible(MwHeadingGraphPosition))
    return;
  if (!Settings[S_COMPASS])
    return;
  int xx;
  xx = Gps.MwHeading * 4;
  xx = xx + 720 + 45;
  xx = xx / 90;
  uint16_t pos = getPosition(MwHeadingGraphPosition);
  Max.WriteBytes_P(headGraph+xx+1, pos, 9);
}


void ScreenClass::DisplayIntro(void)
{
  const int startCol = 4;
  int line = LINE02;

  Max.WriteString_P(message0, line+startCol);
  line += LINE*2;
#ifndef GPSOSD
  Max.WriteString_P(message5, line+startCol);
  Max.WriteString(ItoaPadded(Msp.MwVersion, _screenBuffer, 4, 2), line+startCol+11);
  line += LINE;
#endif
#ifdef INTRO_CALLSIGN
  Max.WriteString_P(message9, line+startCol);
  DisplayCallsign(line+startCol+4);
  line += LINE;
#endif
#ifdef GPSTIME
#ifdef INTRO_TIMEZONE
//timezone
  Max.WriteString_P(message10, line+startCol);
  if(abs(Settings[S_GPSTZ]) >= 100)ItoaPadded(Settings[S_GPSTZ], _screenBuffer, 5, 4);
  else ItoaPadded(Settings[S_GPSTZ], _screenBuffer, 4, 3);
  if(Settings[S_GPSTZAHEAD] || Settings[S_GPSTZ] == 0)_screenBuffer[0] = '+';
  else _screenBuffer[0] = '-';
  Max.WriteString(_screenBuffer, line+startCol+8);
  line += LINE;
//more settings
  Max.WriteString_P(message11, line+startCol);
#endif
#endif
#ifdef INTRO_MENU
  Max.WriteString_P(message6, line+startCol);
  line += LINE;
  Max.WriteString_P(message7, line+startCol+10);
  line += LINE;
  Max.WriteString_P(message8, line+startCol+10);
  line += LINE;
 #endif
#ifdef HAS_ALARMS
  if (alarmState != ALARM_OK) {
      line += LINE;
      Max.WriteString((const char*)alarmMsg, line+startCol);
  }
#endif
}


void ScreenClass::DisplayGPSPosition(void)
{
  uint16_t position;
  if(!Gps.fix)
    return;
  DisplayGPSAltitude();
  if(!fieldIsVisible(MwGPSLatPositionTop))
    return;
  if (!(mode.gpshome))
    return;
  if(Settings[S_COORDINATES]|((mode.gpshome))){
      position = getPosition(MwGPSLatPositionTop);
      _screenBuffer[0] = SYM_LAT;
      FormatGPSCoord(Gps.latitude,_screenBuffer+1,4,'N','S');
      Max.WriteString(_screenBuffer, position);
      position = getPosition(MwGPSLonPositionTop);
      _screenBuffer[0] = SYM_LON;
      FormatGPSCoord(Gps.longitude,_screenBuffer+1,4,'E','W');
      Max.WriteString(_screenBuffer, position);
  }
}


void ScreenClass::DisplayGPSAltitude(void){
  if(Settings[S_GPSALTITUDE]){
    if(!fieldIsVisible(MwGPSAltPosition))
    return;
    _screenBuffer[0] = MwGPSAltPositionAdd[Settings[S_UNITSYSTEM]];
    uint16_t xx;
    if(Settings[S_UNITSYSTEM])
      xx = Gps.altitude * 3.2808; // Mt to Feet
    else
      xx = Gps.altitude;          // Mt
    if(((xx/10)>=Settings[S_ALTITUDE_ALARM])&&(timer.Blink2hz))
      return;
    itoa(xx,_screenBuffer+1,10);
    Max.WriteString(_screenBuffer,getPosition(MwGPSAltPosition));
  }
}


void ScreenClass::DisplayNumberOfSat(void)
{
  if((Gps.numSat<MINSATFIX)&&(timer.Blink2hz)){
    return;
  }
  if(!fieldIsVisible(GPS_numSatPosition))
    return;
  _screenBuffer[0] = SYM_SAT_L;
  _screenBuffer[1] = SYM_SAT_R;
  itoa(Gps.numSat,_screenBuffer+2,10);
  Max.WriteString(_screenBuffer,getPosition(GPS_numSatPosition));
}


void ScreenClass::DisplayGPSSpeed(void)
{
  if(!Gps.fix) return;
  if(!mwosd.armed) Gps.speed=0;
  uint16_t xx;
  if(!Settings[S_UNITSYSTEM])
    xx = Gps.speed * 0.036;           // From MWii cm/sec to Km/h
  else
    xx = Gps.speed * 0.02236932;      // (0.036*0.62137)  From MWii cm/sec to mph
  if(xx > Stats._speedMAX)
    Stats._speedMAX = xx;
  if(!fieldIsVisible(speedPosition))
    return;
  if((xx>Settings[S_SPEED_ALARM])&&(timer.Blink2hz))
    return;
  _screenBuffer[0]=speedUnitAdd[Settings[S_UNITSYSTEM]];
  itoa(xx,_screenBuffer+1,10);
  Max.WriteString(_screenBuffer,getPosition(speedPosition));
#ifdef SHOW_MAX_SPEED
  itoa(Stats._speedMAX,_screenBuffer+1,10);
  Max.WriteString(_screenBuffer,getPosition(speedPosition)+LINE);
#endif //SHOW_MAX_SPEED
}


void ScreenClass::DisplayGPSTime(void)       //local time of coord calc - haydent
{
  if(!Gps.fix) return;
  if(!Settings[S_GPSTIME]) return;
  if(!fieldIsVisible(GPS_timePosition)) return;

//convert to local
  int TZ_SIGN = (Settings[S_GPSTZAHEAD] ? 1 :-1);
  uint32_t local = Gps.time + (((Settings[S_GPSTZ] * 60 * TZ_SIGN / 10)) * 60000);//make correction for time zone
  local = local % 604800000;//prob not necessary but keeps day of week accurate <= 7
//convert to local

//format and display
//  uint16_t milli = local % 1000;//get milli for later
  uint32_t seconds = (local / 1000) % 86400;//remove millisonds and whole days

  formatTime(seconds, _screenBuffer, 1);
  if(_screenBuffer[0] == ' ')_screenBuffer[0] = '0';//put leading zero if empty space
/*
  _screenBuffer[8] = '.';//add milli indicator
  _screenBuffer[9] = '0' + (milli / 100);//only show first digit of milli due to limit of gps rate
  _screenBuffer[10] = 0;//equivalent of new line or end of buffer
*/
  _screenBuffer[8] = 0;//equivalent of new line or end of buffer
  Max.WriteString(_screenBuffer,getPosition(GPS_timePosition));
}


void ScreenClass::DisplayAltitude(void)
{
  int16_t altitude;
  if(Settings[S_UNITSYSTEM])
    altitude = Gps.MwAltitude*0.032808;    // cm to feet
  else
    altitude = Gps.MwAltitude/100;         // cm to mt

  if(mwosd.armed && timer.allSec>5 && altitude > Stats._altitudeMAX)
    Stats._altitudeMAX = altitude;
  if(!fieldIsVisible(MwAltitudePosition))
    return;
  if(!Settings[S_BAROALT])
    return;
  if(((altitude/10)>=Settings[S_ALTITUDE_ALARM])&&(timer.Blink2hz))
    return;
  _screenBuffer[0]=MwAltitudeAdd[Settings[S_UNITSYSTEM]];
  itoa(altitude,_screenBuffer+1,10);
  Max.WriteString(_screenBuffer,getPosition(MwAltitudePosition));
#ifdef SHOW_MAX_ALTITUDE
  itoa(Stats._altitudeMAX,_screenBuffer+1,10);
  Max.WriteString(_screenBuffer,getPosition(MwAltitudePosition)+LINE);
#endif //SHOW_MAX_ALTITUDE

}


void ScreenClass::DisplayClimbRate(void)
{
  if(!fieldIsVisible(MwClimbRatePosition))
    return;
  if(!Settings[S_VARIO])
    return;
    uint16_t position = getPosition(MwClimbRatePosition);
    for(int8_t X=-1; X<=1; X++) {
      Max.WriteChar(SYM_VARIO, position+(X*LINE));
    }
   int8_t xx=Msp.MwVario;
   if (Msp.MwVario>120) xx=120;
   if (Msp.MwVario<-120) xx=-120;
   xx=map(xx,120,-120,0,17);
   int8_t varline=(xx/6)-1;
   int8_t varsymbol=xx%6;
   Max.WriteChar(0x8F-varsymbol, position+(varline*LINE));
}


void ScreenClass::DisplayDistanceToHome(void)
{
  if(!Gps.fix)
    return;
  uint16_t dist;
  if(Settings[S_UNITSYSTEM])
    dist = Gps.distanceToHome * 3.2808;           // mt to feet
  else
    dist = Gps.distanceToHome;                    // Mt
  if(dist > Stats._distanceMAX)
    Stats._distanceMAX = dist;
  if(!fieldIsVisible(GPS_distanceToHomePosition))
    return;

  if(((dist/100)>=Settings[S_DISTANCE_ALARM])&&(timer.Blink2hz))
    return;

  _screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
  itoa(dist, _screenBuffer+1, 10);

#if defined LONG_RANGE_DISPLAY // Change to decimal KM / Miles
  if (dist>9999){

    if(Settings[S_UNITSYSTEM]){
      dist = int(Gps.distanceToHome * 0.0062137);           // mt to miles*10
    }
    else{
      dist = dist=dist/100;
    }
    itoa(dist, _screenBuffer+1, 10);
    uint8_t xx = findNull();
//    if (xx==2){ // if want to limit distance to 999 or less instead of 9999. This adds a leading 0 for improved display
//      _screenBuffer[2]=_screenBuffer[1];
//      _screenBuffer[1]=0x30;
//      xx++;
//    }
    _screenBuffer[xx]=_screenBuffer[xx-1];
    _screenBuffer[xx-1] = DECIMAL;
    xx++;
    _screenBuffer[xx] = 0;
  }
#endif // LONG_RANGE_DISPLAY

  _screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
  Max.WriteString(_screenBuffer,getPosition(GPS_distanceToHomePosition));
}


void ScreenClass::DisplayAngleToHome(void)
{
  if(!Gps.fix)
      return;
 if(!fieldIsVisible(GPS_angleToHomePosition))
    return;
  if(Settings[S_ANGLETOHOME]){
    if((Gps.numSat < MINSATFIX) && timer.Blink2hz)
      return;
    ItoaPadded(Gps.directionToHome,_screenBuffer,3,0);
    _screenBuffer[3] = SYM_DEGREES;
    _screenBuffer[4] = 0;
    Max.WriteString(_screenBuffer,getPosition(GPS_angleToHomePosition));
  }
}


void ScreenClass::DisplayDirectionToHome(void)
{
  if(!Gps.fix)
    return;
 if(!fieldIsVisible(GPS_directionToHomePosition))
    return;

  if(Gps.distanceToHome <= 2 && timer.Blink2hz)
    return;
  uint16_t position=getPosition(GPS_directionToHomePosition);
  int16_t d = Gps.MwHeading + 180 + 360 - Gps.directionToHome;
  d *= 4;
  d += 45;
  d = (d/90)%16;
  _screenBuffer[0] = SYM_ARROW_SOUTH + d;
  _screenBuffer[1] = 0;
  Max.WriteString(_screenBuffer,position);
}


void ScreenClass::DisplayCursor(void)
{
  int cursorpos;
  if(ROW==10){
    if(COL==3) cursorpos=SAVEP+16-1;    // page
    if(COL==1) cursorpos=SAVEP-1;       // exit
    if(COL==2) cursorpos=SAVEP+6-1;     // save/exit
  }
  if(ROW<10)
    {
#ifdef MENU_PID
    if(configPage==MENU_PID){
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      if(COL==1) cursorpos=(ROW+2)*30+10;
      if(COL==2) cursorpos=(ROW+2)*30+10+6;
      if(COL==3) cursorpos=(ROW+2)*30+10+6+6;
     }
#endif
#ifdef MENU_RC
  #if defined CORRECT_MENU_RCT2
     if(configPage==MENU_RC){
      COL=3;
      cursorpos=(ROW+2)*30+10+6+6;
    }
  #elif defined CORRECT_MENU_RCT1
    if(configPage==MENU_RC)
    {
      if (ROW==9){
        if (oldROW==8)
          ROW=10;
        else
          ROW=8;
      }
      oldROW=ROW;
      COL=3;
      cursorpos=(ROW+2)*30+10+6+6;
      }
  #else
    if(configPage==MENU_RC){
      COL=3;
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      cursorpos=(ROW+2)*30+10+6+6;
      }
  #endif

#endif
#ifdef MENU_VOLTAGE
    if(configPage==MENU_VOLTAGE){
      COL=3;
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_RSSI
    if(configPage==MENU_RSSI){
      COL=3;
      if (ROW==7) ROW=10;
      if (ROW==9) ROW=6;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_CURRENT
    if(configPage==MENU_CURRENT)
      {
      COL=3;
      if (ROW==9) ROW=5;
      if (ROW==6) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_DISPLAY
    if(configPage==MENU_DISPLAY)
      {
        if (ROW==9){
          if (oldROW==8)
            ROW=10;
          else
            ROW=8;
        }
        oldROW=ROW;
        COL=3;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_ADVANCED
    if(configPage==MENU_ADVANCED)
      {
      COL=3;
      if (ROW==9) ROW=6;
      if (ROW==7) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_Gps.TIME
    if(configPage==MENU_Gps.TIME)
      {
      COL=3;
      if (ROW==9) ROW=3;
      if (ROW==4) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_ALARMS
    if(configPage==MENU_ALARMS)
      {
      COL=3;
      if (ROW==9) ROW=6;
      if (ROW==7) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_PROFILE
    if(configPage==MENU_PROFILE)
      {
      #ifdef CORRECTLOOPTIME
        if (ROW==9) ROW=3;
        if (ROW==4) ROW=10;
      #else
        if (ROW==9) ROW=2;
        if (ROW==3) ROW=10;
      #endif

      COL=3;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
  }
  if(timer.Blink10hz) {
    Max.WriteChar(SYM_CURSOR, cursorpos);
  }
}


void ScreenClass::DisplayConfigScreen(void)
{
  int16_t MenuBuffer[10];
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menutitle_item[configPage])));
  Max.WriteString(_screenBuffer, 35);
  #ifdef MENU_PROFILE
//   Max.WriteString(itoa(FCProfile,_screenBuffer,10),50); // Display Profile number
  #endif
  Max.WriteString_P(configMsgEXT, SAVEP);    //EXIT
  if(!mwosd.previousarmedstatus) {
    Max.WriteString_P(configMsgSAVE, SAVEP+6);  //SaveExit
    Max.WriteString_P(configMsgPGS, SAVEP+16); //<Page>
  }

  if(configPage==MENU_STAT)
  {
    int xx;
//    Max.WriteString_P(configMsg00, 35);

#ifdef SHORTSUMMARY
    strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_stats_item[0])));
    Max.WriteString(_screenBuffer, ROLLT);
    formatTime(Stats._flyingTime, _screenBuffer, 1);
    Max.WriteString(_screenBuffer,ROLLD-4);

#else // SHORTSUMMARY

 //     MenuBuffer[0]=rc.rcRate8;
      xx=Stats.amperagesum/360;
      itoa(xx,_screenBuffer,10);
      MenuBuffer[1]=Stats._trip;
      MenuBuffer[2]=Stats._distanceMAX;
      MenuBuffer[3]=Stats._altitudeMAX;
      MenuBuffer[4]=Stats._speedMAX;
      MenuBuffer[5]=xx;
      MenuBuffer[6]=Stats._ampMAX/10;

    for(uint8_t X=0; X<=6; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_stats_item[X])));
      Max.WriteString(_screenBuffer, ROLLT + (X*30));
      Max.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),110+(30*X));
    }

    formatTime(Stats._flyingTime, _screenBuffer, 1);
    Max.WriteString(_screenBuffer,ROLLD-4);
#endif
#ifdef HAS_ALARMS
    if (alarmState != ALARM_OK) {
        Max.WriteString((const char*)alarmMsg, LINE12 + 3);
    }
#endif

    }
#ifdef MENU_PID
  if(configPage==MENU_PID)
  {
    for(uint8_t X=0; X<=6; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_pid[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }

    for(uint8_t Y=0; Y<=8; Y++) {
      if (Y==5) Y=7;
      uint8_t X=Y;
      if (Y>6){
        X=X-2;
      }
      Max.WriteString(itoa(mwosd.P8[Y],_screenBuffer,10),ROLLP+(X*30));
      Max.WriteString(itoa(mwosd.I8[Y],_screenBuffer,10),ROLLI+(X*30));
      Max.WriteString(itoa(mwosd.D8[Y],_screenBuffer,10),ROLLD+(X*30));
    }

    Max.WriteString("P",71);
    Max.WriteString("I",77);
    Max.WriteString("D",83);
  }
#endif
#ifdef MENU_RC
  if(configPage==MENU_RC)
  {
    #if defined CORRECT_MENU_RCT2
      MenuBuffer[0]=rc.rcRate8;
      MenuBuffer[1]=rc.rcExpo8;
      MenuBuffer[2]=rc.rollRate;
      MenuBuffer[3]=rc.PitchRate;
      MenuBuffer[4]=rc.yawRate;
      MenuBuffer[5]=rc.dynThrPID;
      MenuBuffer[6]=rc.thrMid8;
      MenuBuffer[7]=rc.thrExpo8;
      MenuBuffer[8]=rc.tpa_breakpoint16;
       for(uint8_t X=0; X<=8; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rc[X])));
        Max.WriteString(_screenBuffer, ROLLT+ (X*30));
        Max.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    #elif defined CORRECT_MENU_RCT1
      MenuBuffer[0]=rc.rcRate8;
      MenuBuffer[1]=rc.rcExpo8;
      MenuBuffer[2]=rc.rollRate;
      MenuBuffer[3]=rc.PitchRate;
      MenuBuffer[4]=rc.yawRate;
      MenuBuffer[5]=rc.dynThrPID;
      MenuBuffer[6]=rc.thrMid8;
      MenuBuffer[7]=rc.thrExpo8;
       for(uint8_t X=0; X<=7; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rc[X])));
        Max.WriteString(_screenBuffer, ROLLT+ (X*30));
        Max.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    #else
      MenuBuffer[0]=rc.rcRate8;
      MenuBuffer[1]=rc.rcExpo8;
      MenuBuffer[2]=rc.rollPitchRate;
      MenuBuffer[3]=rc.yawRate;
      MenuBuffer[4]=rc.dynThrPID;
      MenuBuffer[5]=rc.thrMid8;
      MenuBuffer[6]=rc.thrExpo8;
     for(uint8_t X=0; X<=6; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rc[X])));
        Max.WriteString(_screenBuffer, ROLLT+ (X*30));
        Max.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    #endif
  }
#endif
#ifdef MENU_VOLTAGE
  if(configPage==MENU_VOLTAGE)
  {
    Sensors.Process();
    _screenBuffer[0]=SYM_MAIN_BATT;
    ItoaPadded(Sensors.voltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    Max.WriteString(_screenBuffer,ROLLD-LINE-LINE-1);

    _screenBuffer[0]=SYM_VID_BAT;
    ItoaPadded(Sensors.vidvoltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    Max.WriteString(_screenBuffer,ROLLI-LINE-LINE-3);

    for(uint8_t X=0; X<=6; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_bat[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    Max.WriteString(itoa(Settings[S_VIDVOLTAGE],_screenBuffer,10),ALTD);
    MenuconfigOnoff(ROLLD,S_DISPLAYVOLTAGE);
    Max.WriteString(itoa(Settings[S_DIVIDERRATIO],_screenBuffer,10),PITCHD);
    Max.WriteString(itoa(Settings[S_VOLTAGEMIN],_screenBuffer,10),YAWD);
    MenuconfigOnoff(ALTD,S_VIDVOLTAGE);
    Max.WriteString(itoa(Settings[S_VIDDIVIDERRATIO],_screenBuffer,10),VELD);
    Max.WriteString(itoa(Settings[S_BATCELLS],_screenBuffer,10),LEVD);
    MenuconfigOnoff(MAGD,S_MAINVOLTAGE_VBAT);
  }
#endif
#ifdef MENU_RSSI
  if(configPage==MENU_RSSI)
  {
    itoa(rc.rssi,_screenBuffer,10);
    uint8_t xx = findNull();
    _screenBuffer[xx++] = '%';
    _screenBuffer[xx] = 0;
    Max.WriteString(_screenBuffer,ROLLD-LINE-LINE);
    for(uint8_t X=0; X<=5; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rssi[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    MenuconfigOnoff(ROLLD,S_DISPLAYRSSI);
    if(timer.rssiTimer>0) {
      Max.WriteString(itoa(timer.rssiTimer,_screenBuffer,10),PITCHD);
    }
    else {
    Max.WriteString("-", PITCHD);
    }
    MenuconfigOnoff(YAWD,S_MWRSSI);
    MenuconfigOnoff(ALTD,S_PWMRSSI);
    Max.WriteString(itoa(Settings16[S16_RSSIMAX],_screenBuffer,10),VELD);
    Max.WriteString(itoa(Settings16[S16_RSSIMIN],_screenBuffer,10),LEVD);
  }
#endif
#ifdef MENU_CURRENT
  if(configPage==MENU_CURRENT)
  {
    ItoaPadded(Stats.amperage, _screenBuffer, 4, 3);     // 99.9 ampere max!
    _screenBuffer[4] = SYM_AMP;
    _screenBuffer[5] = 0;
    Max.WriteString(_screenBuffer,ROLLD-LINE-LINE-1);

    for(uint8_t X=0; X<=4; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_amps[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    MenuconfigOnoff(ROLLD,S_AMPERAGE);
    MenuconfigOnoff(PITCHD,S_AMPER_HOUR);
    MenuconfigOnoff(YAWD,S_AMPERAGE_VIRTUAL);
    Max.WriteString(itoa(Settings16[S16_AMPDIVIDERRATIO],_screenBuffer,10),ALTD);
    Max.WriteString(itoa(Settings16[S16_AMPZERO],_screenBuffer,10),VELD);
  }
#endif
#ifdef MENU_DISPLAY
  if(configPage==MENU_DISPLAY)
  {
    for(uint8_t X=0; X<=7; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_display[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    MenuconfigOnoff(ROLLD,S_DISPLAY_HORIZON_BR);
    MenuconfigOnoff(PITCHD,S_WITHDECORATION);
    MenuconfigOnoff(YAWD,S_SCROLLING);
    MenuconfigOnoff(ALTD,S_THROTTLEPOSITION);
    MenuconfigOnoff(VELD,S_COORDINATES);
    MenuconfigOnoff(LEVD,S_MODESENSOR);
    MenuconfigOnoff(MAGD,S_GIMBAL);
    Max.WriteString(itoa(Settings[S_MAPMODE],_screenBuffer,10),MAGD+LINE);
  }
#endif
#ifdef MENU_ADVANCED
  if(configPage==MENU_ADVANCED)
  {
    for(uint8_t X=0; X<=5; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_advanced[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }

//  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[apactive])));
//  Max.WriteString(_screenBuffer, getPosition(APstatusPosition));


    if(!Settings[S_UNITSYSTEM]){
      Max.WriteString_P(configMsg710, ROLLD);
    }
    else {
      Max.WriteString_P(configMsg711, ROLLD);
      }
    if(!Settings[S_VIDEOSIGNALTYPE]){
      Max.WriteString_P(configMsg720, PITCHD);
    }
    else {
      Max.WriteString_P(configMsg721, PITCHD);
      }
    if(Settings[S_VREFERENCE]){
      Max.WriteString_P(configMsg730, YAWD);
    }
    else {
      Max.WriteString_P(configMsg731, YAWD);
      }

/*
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[Settings[S_UNITSYSTEM]])));
  Max.WriteString(_screenBuffer, ROLLD);
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[Settings[S_VIDEOSIGNALTYPE]])));
  Max.WriteString(_screenBuffer, PITCHD);
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[Settings[S_VREFERENCE]])));
  Max.WriteString(_screenBuffer, YAWD);
*/
    MenuconfigOnoff(ALTD,S_DEBUG);
    if(timer.magCalibrationTimer>0)
      Max.WriteString(itoa(timer.magCalibrationTimer,_screenBuffer,10),VELD);
    else
      Max.WriteString("-",VELD);
    Max.WriteString(itoa(Settings[S_RCWSWITCH_CH],_screenBuffer,10),LEVD);
   }
#endif
#ifdef MENU_Gps.TIME
  if(configPage==MENU_Gps.TIME)
  {
    for(uint8_t X=0; X<=2; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_gps_time[X])));
      Max.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
  MenuconfigOnoff(ROLLD,S_GPSTIME);
  MenuconfigOnoff(PITCHD,S_GPSTZAHEAD);
  Max.WriteString(itoa(Settings[S_GPSTZ],_screenBuffer,10),YAWD);
  }
#endif
#ifdef MENU_ALARMS
    if(configPage==MENU_ALARMS){
      MenuBuffer[0]=Settings[S_DISTANCE_ALARM];
      MenuBuffer[1]=Settings[S_ALTITUDE_ALARM];
      MenuBuffer[2]=Settings[S_SPEED_ALARM];
      MenuBuffer[3]=Settings[S_FLYTIME_ALARM];
      MenuBuffer[4]=Settings[S_AMPER_HOUR_ALARM];
      MenuBuffer[5]=Settings[S_AMPERAGE_ALARM];
      for(uint8_t X=0; X<=5; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_alarm_item[X])));
        Max.WriteString(_screenBuffer, ROLLT+ (X*30));
        Max.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    }
#endif
#ifdef MENU_PROFILE
    if(configPage==MENU_PROFILE){
      #ifdef CORRECTLOOPTIME
        #define MENU10MAX 2
      #else
        #define MENU10MAX 1
      #endif
      MenuBuffer[0]=Msp.FCProfile;
      MenuBuffer[1]=Msp.PIDController;
      MenuBuffer[2]=Msp.LoopTime;
      for(uint8_t X=0; X<=MENU10MAX; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_profile[X])));
        Max.WriteString(_screenBuffer, ROLLT+ (X*30));
        Max.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    }
#endif
    if(configPage > MAXPAGE)configPage=MINPAGE;

  DisplayCursor();
}



void ScreenClass::DisplayDebug(void)
{
#if defined (DEBUG)||defined (DEBUGMW)
  if(!Settings[S_DEBUG])
    return;

  for(uint8_t X=0; X<4; X++) {
    ItoaPadded(debugBuffer[X], _screenBuffer+2,7,0);
    _screenBuffer[0] = 0x30+X;
    _screenBuffer[1] = 0X3A;
    Max.WriteString(_screenBuffer,getPosition(debugPosition)+(X*LINE));
  }

#endif
}

void ScreenClass::DisplayCells(void){

  #ifndef MIN_CELL
    #define MIN_CELL 320
  #endif
  uint16_t sum = 0;
  uint16_t low = 0;
  uint8_t _cells = 0;

  for(uint8_t i=0; i<6; i++) {
    uint16_t volt = Msp.cell_data[i];
    if(!volt)continue;//empty cell
    ++_cells;
    sum += volt;
    if(volt < low || !low)low = volt;
    if((volt>MIN_CELL)||(timer.Blink2hz)){
      int tempvolt=constrain(volt,300,415);
      tempvolt = map(tempvolt,300,415,0,14);
      _screenBuffer[i]=SYM_CELL0+tempvolt;
      }
      else _screenBuffer[i]=' ';
    }

    if(_cells){
      _screenBuffer[_cells] = 0;
      Max.WriteString(_screenBuffer,getPosition(SportPosition)+(6-_cells));//bar chart

      ItoaPadded(low, _screenBuffer+1,4,2);
      _screenBuffer[0] = SYM_MIN;
      _screenBuffer[5] = SYM_VOLT;
      _screenBuffer[6] = 0;

    if((low>MIN_CELL)||(timer.Blink2hz))
      Max.WriteString(_screenBuffer,getPosition(SportPosition)+LINE);//lowest

      uint16_t avg = 0;
      if(_cells)avg = sum / _cells;
      ItoaPadded( avg, _screenBuffer+1,4,2);
      _screenBuffer[0] = SYM_AVG;
      _screenBuffer[5] = SYM_VOLT;
      _screenBuffer[6] = 0;

    if((avg>MIN_CELL)||(timer.Blink2hz))
      Max.WriteString(_screenBuffer,getPosition(SportPosition)+(2*LINE));//average
  }
}


void ScreenClass::MapMode(void) {

#ifdef MAPMODE

  int mapstart=0;
  int mapend=0;

  switch(Settings[S_MAPMODE]) {
    case 1:
      mapstart=0;mapend=1;
      break;
    case 2:
      mapstart=1;mapend=2;
      break;
    case 3:
      mapstart=0;mapend=2;
      break;
    case 4:
      mapstart=1;mapend=2;
      break;
    default:
      return;
  }

  if(!Gps.fix)
    return;
  if(!fieldIsVisible(MapModePosition))
    return;

  int8_t xdir=0;
  int8_t ydir=0;
  int16_t targetx;
  int16_t targety;
  int16_t range=200;
  int16_t angle;
  int16_t targetpos;
  int16_t centerpos;
  uint32_t maxdistance;
  uint8_t mapsymbolcenter;
  uint8_t mapsymboltarget;
  uint8_t mapsymbolrange;
  int16_t tmp;


  for(uint8_t maptype=mapstart; maptype<mapend; maptype++) {

    if (maptype==1) {
      angle=(180+360+Gps.directionToHome-Gps.armedangle)%360;
    }
    else {
      angle=(360+Gps.directionToHome-Gps.MwHeading)%360;
    }

    tmp = angle/90;
    switch (tmp) {
      case 0:
        xdir=+1;
        ydir=-1;
        break;
      case 1:
        xdir=+1;
        ydir=+1;
        angle=180-angle;
        break;
      case 2:
        xdir=-1;
        ydir=+1;
        angle=angle-180;
        break;
      case 3:
        xdir=-1;
        ydir=-1;
        angle=360-angle;
        break;
      }

    float rad  = angle * PI / 180;    // convert to radians
    uint16_t x = (uint16_t)(Gps.distanceToHome * sin(rad));
    uint16_t y = (uint16_t)(Gps.distanceToHome * cos(rad));

    if (y > x) maxdistance=y;
    else maxdistance=x;
    if (maxdistance < 100) {
      range = 100;
      mapsymbolrange=SYM_RANGE_100;
    }
    else if (maxdistance < 500) {
      range = 500;
      mapsymbolrange=SYM_RANGE_500;
    }
    else if (maxdistance < 2500) {
      range = 2500;
      mapsymbolrange=SYM_RANGE_2500;
    }
    else {
      range = maxdistance;
      mapsymbolrange=SYM_RANGE_MAX;
    }

    targetx = xdir*map(x, 0, range, 0, 16);
    targety = ydir*map(y, 0, range, 0, 15);

    if (maxdistance<20) {
      targetx = 0;
      targety = 0;
    }

    centerpos=getPosition(MapCenterPosition);
    targetpos= centerpos + (targetx/2) + (LINE*(targety/3));

    if (maptype==1) {
      mapsymbolcenter = SYM_HOME;
      mapsymboltarget = SYM_AIRCRAFT;
    }
    else {
      mapsymbolcenter = SYM_AIRCRAFT;
      mapsymboltarget = SYM_HOME;
    }

      int8_t symx = (int8_t)abs(targetx)%2;
      int8_t symy = (int8_t)abs(targety)%3;
      if (ydir==1)
        symy=2-symy;
      if (xdir==-1)
        symx=1-symx;
      if (abs(targety)<3)
        symy = 1 - ydir;
      if (abs(targetx)<2){
        if (targetx<0)
          symx=0;
        else
          symx=1;
      }

    if (maptype==0)
      mapsymboltarget = 0xD6;
    else
      mapsymboltarget = 0xD0;

    mapsymboltarget = uint8_t( mapsymboltarget + symy + (symx*3));


    if (Settings[S_MAPMODE]==4) {
      tmp=(360+382+Gps.MwHeading-Gps.armedangle)%360/45;
      mapsymboltarget = SYM_DIRECTION + tmp;
    }

    _screenBuffer[0] = mapsymbolrange;
    _screenBuffer[1] = 0;
    Max.WriteString(_screenBuffer,getPosition(MapModePosition));

    _screenBuffer[0] = mapsymboltarget;
    _screenBuffer[1] = 0;
    Max.WriteString(_screenBuffer,targetpos);

    _screenBuffer[0] = mapsymbolcenter;
    _screenBuffer[1] = 0;
    Max.WriteString(_screenBuffer,centerpos);
  }

#endif
}

#ifdef USEGLIDESCOPE
void ScreenClass::Displayfwglidescope(void){
  if(!fieldIsVisible(glidescopePosition))
    return;

  int8_t GS_deviation_scale   = 0;
  if (Gps.distanceToHome>0){ //watch div 0!!
    int16_t gs_angle          =(573*atan((float)Gps.MwAltitude/10/Gps.distanceToHome));
    int16_t GS_target_delta   = gs_angle-USEGLIDESCOPE;
    GS_target_delta           = constrain(GS_target_delta,-400,400);
    GS_deviation_scale        = map(GS_target_delta,400,-400,0,8);
  }

  int8_t varline              = (GS_deviation_scale/3)-1;
  int8_t varsymbol            = GS_deviation_scale%3;

  uint16_t position = getPosition(glidescopePosition);
    for(int8_t X=-1; X<=1; X++) {
      Max.WriteChar(SYM_GLIDESCOPE, position+(X*LINE));
    }
    Max.WriteChar(SYM_GLIDESCOPE+3-varsymbol, position+(varline*LINE));
}
#endif //USEGLIDESCOPE

void ScreenClass::MenuconfigOnoff(uint16_t pos, uint8_t setting){
    strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_on_off[(Settings[setting])])));
    Max.WriteString(_screenBuffer, pos);
}

void ScreenClass::DisplayArmed(void)
{
  if(!fieldIsVisible(motorArmedPosition))
    return;
  uint8_t message_no = 0;

#ifdef HAS_ALARMS
  if (alarmState != ALARM_OK) {
      // There's an alarm, let it have this space.
      return;
  }
#endif

  if(!mwosd.armed){
    message_no=1;
    timer.armed=30;
  }

  if(Sensors.IsPresent(GPSSENSOR)){
    #ifdef SATACTIVECHECK
    if (Gps.numSat<MINSATFIX){ // below minimum preferred value
      message_no=5;
    }
    #endif //SATACTIVECHECK

    #ifdef GPSACTIVECHECK
    if(timer.GPS_active==0){
      message_no=4;
    }
    #endif //GPSACTIVECHECK
  }

  #ifdef MSPACTIVECHECK
  if(timer.MSP_active==0){
    message_no=3;
  }
  #endif //MSPACTIVECHECK
  if(timer.armed&&mwosd.armed){
    if (timer.Blink10hz){
      timer.armed--;
      message_no=2;
    }
    else{
      message_no=0;
    }
  }

   if(message_no>2){
      if(!mwosd.armed&&timer.Blink2hz){
        message_no=1;
      }
      else if(!mwosd.armed){
      }
      else if(timer.Blink2hz){
        return;
      }
    }

  #ifdef HIDEARMEDSTATUS
  if(message_no<3){
    return;
  }
  #endif //HIDEARMEDSTATUS

  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[message_no])));
  if (message_no>0){
    Max.WriteString(_screenBuffer, getPosition(motorArmedPosition));
  }
}

void ScreenClass::DisplayForcedCrosshair()
{
  uint16_t position = getPosition(horizonPosition);
  Max.WriteChar(SYM_AH_CENTER_LINE, position-1);
  Max.WriteChar(SYM_AH_CENTER_LINE_RIGHT, position+1);
  Max.WriteChar(SYM_AH_CENTER, position);
}

void ScreenClass::ReadLayout()
{
  // $$$ screenlayout is global
  uint16_t EEPROMscreenoffset=EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(screenlayout*POSITIONS_SETTINGS*2);
  for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
    uint16_t pos = (en*2)+EEPROMscreenoffset;
    _screenPosition[en] = Eeprom.Read(pos);
    uint16_t xx = (uint16_t)Eeprom.Read(pos+1)<<8;
    _screenPosition[en] = _screenPosition[en] + xx;

    if(Settings[S_VIDEOSIGNALTYPE]){
      uint16_t x = _screenPosition[en]&0x1FF;
      if (x>LINE06) _screenPosition[en] = _screenPosition[en] + LINE;
      if (x>LINE09) _screenPosition[en] = _screenPosition[en] + LINE;
    }
#ifdef SHIFTDOWN
    if ((_screenPosition[en]&0x1FF)<LINE04) _screenPosition[en] = _screenPosition[en] + LINE;
#endif
  }
}



#ifdef HAS_ALARMS
void ScreenClass::DisplayAlarms()
{
    if (alarmState == ALARM_OK) {
        return;
    }
    if (alarmState == ALARM_CRIT || alarmState == ALARM_ERROR || timer.Blink2hz) {
        Max.WriteString((const char*)alarmMsg, getPosition(motorArmedPosition));
    }
}
#endif

void ScreenClass::UpdateLayout()
{
  #if defined (OSD_SWITCH_RC)
    uint8_t rcswitch_ch = Eeprom.Settings[S_RCWSWITCH_CH];
    screenlayout=0;
    if (Eeprom.Settings[S_RCWSWITCH]){
      #ifdef OSD_SWITCH_RSSI
        Msp.MwRcData[rcswitch_ch]=pwmRSSI;
      #endif
      if (Msp.MwRcData[rcswitch_ch] > 1600){
        screenlayout=1;
      }
      else if (Msp.MwRcData[rcswitch_ch] > 1400){
        screenlayout=2;
      }
    }
    else{
      if (Sensors.IsActive(mode.osd_switch)){
        screenlayout=1;
      }
    }
  #else
    if (Sensors.IsActive(mode.osd_switch))
      screenlayout=1;
    else
      screenlayout=0;
  #endif

#if defined (DEVELOPMENT)
      screenlayout=0;
#endif

  if (screenlayout!=oldscreenlayout){
    ReadLayout();
  }
  oldscreenlayout=screenlayout;
}

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
void ScreenClass::LoadFontLoop() {
  switch(Font.fontStatus) {
    case 0:
      Max.WriteString_P(messageF0, 32);
      Max.DrawScreen();
      delay(3000);
      Max.DisplayFont();
      Max.WriteString_P(messageF1, 32);
      Max.DrawScreen();
      Font.fontStatus++;
      delay(3000);
      break;
    case 1:
      Max.UpdateFont();
      Max.Setup();
      Max.WriteString_P(messageF2, 32);
      Max.DisplayFont();
      Max.DrawScreen();
      Font.fontStatus++;
      break;
  }
  digitalWrite(LEDPIN,LOW);
}
#endif
