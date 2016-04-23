/*
 *  symbols.h
 *
 *
 *
 */

#define SYM_BLANK 0X20

// Satellite Graphics
#define SYM_SAT_L 0X1E
#define SYM_SAT_R 0X1F
//#define SYM_SAT 0X0F  // Not used

// Degrees Icon for HEADING/DIRECTION HOME
#define SYM_DEGREES 0XBD

// Direction arrows
#define SYM_ARROW_SOUTH 0X60
#define SYM_ARROW_2 0X61
#define SYM_ARROW_3 0X62
#define SYM_ARROW_4 0X63
#define SYM_ARROW_EAST 0X64
#define SYM_ARROW_6 0X65
#define SYM_ARROW_7 0X66
#define SYM_ARROW_8 0X67
#define SYM_ARROW_NORTH 0X68
#define SYM_ARROW_10 0X69
#define SYM_ARROW_11 0X6A
#define SYM_ARROW_12 0X6B
#define SYM_ARROW_WEST 0X6C
#define SYM_ARROW_14 0X6D
#define SYM_ARROW_15 0X6E
#define SYM_ARROW_16 0X6F

// Heading Graphics
#define SYM_HEADING_N 0X18
#define SYM_HEADING_S 0X19
#define SYM_HEADING_E 0X1A
#define SYM_HEADING_W 0X1B
#define SYM_HEADING_DIVIDED_LINE 0X1C
#define SYM_HEADING_LINE 0X1D

// FRSKY HUB
#define SYM_CELL0      0xF0
#define SYM_CELL1      0xF1
#define SYM_CELL2      0xF2
#define SYM_CELL3      0xF3
#define SYM_CELL4      0xF4
#define SYM_CELL5      0xF5
#define SYM_CELL6      0xF6
#define SYM_CELL7      0xF7
#define SYM_CELL8      0xF8
#define SYM_CELL9      0xF9
#define SYM_CELLA      0xFA
#define SYM_CELLB      0xFB
#define SYM_CELLC      0xFC
#define SYM_CELLD      0xFD
#define SYM_CELLE      0xFE
#define SYM_CELLF      0xC3

// Map mode
#define SYM_HOME       0x04
#define SYM_AIRCRAFT   0X05
#define SYM_RANGE_100  0x21
#define SYM_RANGE_500  0x22
#define SYM_RANGE_2500 0x23
#define SYM_RANGE_MAX  0x24
#define SYM_DIRECTION  0x72

// GPS Coordinates and Altitude
#define SYM_LAT 0xCA
#define SYM_LON 0XCB
#define SYM_ALT 0XCC

// GPS Mode and Autopilot
#define SYM_3DFIX 0XDF
#define SYM_HOLD 0XEF
#define SYM_G_HOME 0XFF
#define SYM_GHOME 0X9D
#define SYM_GHOME1 0X9E
#define SYM_GHOLD 0XCD
#define SYM_GHOLD1 0XCE
#define SYM_GMISSION 0XB5
#define SYM_GMISSION1 0XB6
#define SYM_GLAND 0XB7
#define SYM_GLAND1 0XB8

// Gimbal active Mode
#define SYM_GIMBAL 0X16
#define SYM_GIMBAL1 0X17


// Sensor´s Presence
#define SYM_ACC 0XA0
#define SYM_MAG 0XA1
#define SYM_BAR 0XA2
#define SYM_GPS 0XA3
#define SYM_MAN 0XC0
#define SYM_MAN1 0XC1
#define SYM_MAN2 0XC2
#define SYM_CHECK 0XBE
#define SYM_BARO10 0XB7
#define SYM_BARO11 0XB8
#define SYM_MAG10 0XB5
#define SYM_MAG11 0XB6

// AH Center screen Graphics
//#define SYM_AH_CENTER 0X01
#ifdef ALT_CENTER
  #define SYM_AH_CENTER_LINE 0XB0
  #define SYM_AH_CENTER 0XB1
  #define SYM_AH_CENTER_LINE_RIGHT 0XB2
#else
  #define SYM_AH_CENTER_LINE 0X26
  #define SYM_AH_CENTER 0X7E
  #define SYM_AH_CENTER_LINE_RIGHT 0XBC
#endif
#define SYM_AH_RIGHT 0X02
#define SYM_AH_LEFT 0X03
#define SYM_AH_DECORATION_UP 0XC9
#define SYM_AH_DECORATION_DOWN 0XCF


// AH Bars
#define SYM_AH_BAR9_0 0x80


// Temperature
#define SYM_TEMP_F 0X0D
#define SYM_TEMP_C 0X0E

// Batt evolution
#define SYM_BATT_FULL 0X90
#define SYM_BATT_5 0X91
#define SYM_BATT_4 0X92
#define SYM_BATT_3 0X93
#define SYM_BATT_2 0X94
#define SYM_BATT_1 0X95
#define SYM_BATT_EMPTY 0X96

// Vario
#define SYM_VARIO 0x7F

// Glidescope
#define SYM_GLIDESCOPE 0xE0

// Batt Icon´s
#define SYM_MAIN_BATT 0X97
#define SYM_VID_BAT 0XBF

// Unit Icon´s (Metric)
#define SYM_MS 0X9F
#define SYM_KMH 0XA5
#define SYM_ALTM 0XA7
#define SYM_DISTHOME_M 0XBB
#define SYM_M 0X0C

// Unit Icon´s (Imperial)
#define SYM_FTS 0X99
#define SYM_MPH 0XA6
#define SYM_ALTFT 0XA8
#define SYM_DISTHOME_FT 0XB9
#define SYM_FT 0X0F

// Voltage and amperage
#define SYM_VOLT 0XA9
#define SYM_AMP 0X9A
#define SYM_MAH 0XA4
#define SYM_WATT 0X57

// Flying Mode
#define SYM_ACRO 0XAE
#define SYM_ACROGY 0X98
#define SYM_ACRO1 0XAF
#define SYM_STABLE 0XAC
#define SYM_STABLE1 0XAD
#define SYM_HORIZON 0XC4
#define SYM_HORIZON1 0XC5
#define SYM_PASS 0XAA
#define SYM_PASS1 0XAB
#define SYM_AIR 0XEA
#define SYM_AIR1 0XEB
#define SYM_PLUS 0X89

// Time
#define SYM_ON_M 0X9B
#define SYM_FLY_M 0X9C
#define SYM_ON_H 0X70
#define SYM_FLY_H 0X71

// Throttle Position (%)
#define SYM_THR 0XC8
#define SYM_THR1 0XC9

// RSSI
#define SYM_RSSI 0XBA

// Menu cursor
#define SYM_CURSOR SYM_AH_LEFT

//Misc
#define SYM_COLON 0X2D

//sport
#define SYM_MIN 0xB3
#define SYM_AVG 0xB4

// Heading
const char headGraph[] MAYBE_PROGMEM = {
  0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d,0x1c,0x1d,0x18,0x1d,0x1c,0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d};

#if defined GPSOSD
  const char satnogps_text[] MAYBE_PROGMEM = " NO GPS ";
#endif
const char blank_text[] MAYBE_PROGMEM    = "";
const char nodata_text[] MAYBE_PROGMEM    = "NO DATA";
const char nogps_text[] MAYBE_PROGMEM     = " NO GPS";
const char satlow_text[] MAYBE_PROGMEM    = "LOW SATS";
const char disarmed_text[] MAYBE_PROGMEM  = "DISARMED";
const char armed_text[] MAYBE_PROGMEM     = " ARMED";
const char APRTHtext[] MAYBE_PROGMEM      = "AUTO RTH";
const char APHOLDtext[] MAYBE_PROGMEM     = "AUTO HOLD";
const char APWAYPOINTtext[] MAYBE_PROGMEM = " MISSION";
const char lowvolts_text[] MAYBE_PROGMEM  = "LOW VOLTS";

// For Status / warning messages
const MAYBE_PROGMEM char * const message_item[] =
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
const char message0[] MAYBE_PROGMEM = INTRO_VERSION;
#else
const char message0[] MAYBE_PROGMEM = MWVERS;
#endif

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
const char messageF0[] MAYBE_PROGMEM = "DO NOT POWER OFF";
const char messageF1[] MAYBE_PROGMEM = "SCREEN WILL GO BLANK";
const char messageF2[] MAYBE_PROGMEM = "UPDATE COMPLETE";
#endif

//const char message1[] MAYBE_PROGMEM = "VIDEO SIGNAL NTSC";
//const char message2[] MAYBE_PROGMEM = "VIDEO SIGNAL PAL ";
const char message5[]  MAYBE_PROGMEM = "FC VERSION:";
const char message6[]  MAYBE_PROGMEM = "OPEN MENU: THRT MIDDLE";
const char message7[]  MAYBE_PROGMEM = "+YAW RIGHT";
const char message8[]  MAYBE_PROGMEM = "+PITCH FULL";
const char message9[]  MAYBE_PROGMEM = "ID:";         // Call Sign on the beggining of the transmission
const char message10[] MAYBE_PROGMEM = "TZ UTC:"; //haydent - Time Zone & DST Setting
//const char message11[] MAYBE_PROGMEM = "MORE IN: GUI+CONFIG.H";

// For Config menu common
const char configMsgON[]   MAYBE_PROGMEM = "ON";
const char configMsgOFF[]  MAYBE_PROGMEM = "OFF";
const char configMsgEXT[]  MAYBE_PROGMEM = "EXIT";
const char configMsgSAVE[] MAYBE_PROGMEM = "SAVE+EXIT";
const char configMsgPGS[]  MAYBE_PROGMEM = "<PAGE>";
const char configMsgMWII[] MAYBE_PROGMEM = "USE FC";

// For APSTATUS

// For Config pages
//-----------------------------------------------------------Page0
const char configMsg00[] MAYBE_PROGMEM = "STATISTICS";
const char configMsg01[] MAYBE_PROGMEM = "FLY TIME";
const char configMsg02[] MAYBE_PROGMEM = "TOT DISTANCE";
const char configMsg03[] MAYBE_PROGMEM = "MAX DISTANCE";
const char configMsg04[] MAYBE_PROGMEM = "MAX ALTITUDE";
const char configMsg05[] MAYBE_PROGMEM = "MAX SPEED";
const char configMsg06[] MAYBE_PROGMEM = "MAH USED";
const char configMsg07[] MAYBE_PROGMEM = "MAX AMPS";
//-----------------------------------------------------------Page1
const char configMsg10[] MAYBE_PROGMEM = "PID CONFIG";
const char configMsg11[] MAYBE_PROGMEM = "ROLL";
const char configMsg12[] MAYBE_PROGMEM = "PITCH";
const char configMsg13[] MAYBE_PROGMEM = "YAW";
const char configMsg14[] MAYBE_PROGMEM = "ALT";
const char configMsg15[] MAYBE_PROGMEM = "GPS";
const char configMsg16[] MAYBE_PROGMEM = "LEVEL";
const char configMsg17[] MAYBE_PROGMEM = "MAG";
//-----------------------------------------------------------Page2
const char configMsg20[] MAYBE_PROGMEM = "RC TUNING";
const char configMsg21[] MAYBE_PROGMEM = "RC RATE";
const char configMsg22[] MAYBE_PROGMEM = "RC EXPO";
const char configMsg23[] MAYBE_PROGMEM = "ROLL PITCH RATE";
const char configMsg24[] MAYBE_PROGMEM = "YAW RATE";
const char configMsg25[] MAYBE_PROGMEM = "TPA";
const char configMsg26[] MAYBE_PROGMEM = "THROTTLE MID";
const char configMsg27[] MAYBE_PROGMEM = "THROTTLE EXPO";
#if defined CORRECT_MENU_RCT1
  const char configMsg23a[] MAYBE_PROGMEM = "ROLL RATE";
  const char configMsg23b[] MAYBE_PROGMEM = "PITCH RATE";
#endif
#if defined CORRECT_MENU_RCT2
  const char configMsg23a[] MAYBE_PROGMEM = "ROLL RATE";
  const char configMsg23b[] MAYBE_PROGMEM = "PITCH RATE";
  const char configMSg28[]  MAYBE_PROGMEM = "TPA BREAKPOINT";
#endif
//-----------------------------------------------------------Page3
const char configMsg30[] MAYBE_PROGMEM = "VOLTAGE";
const char configMsg31[] MAYBE_PROGMEM = "DISPLAY MAIN VOLTS";
const char configMsg32[] MAYBE_PROGMEM = "ADJUST VOLTS";
const char configMsg33[] MAYBE_PROGMEM = "MAIN VOLTS ALARM";
const char configMsg34[] MAYBE_PROGMEM = "DISPLAY VID VOLTS";
const char configMsg35[] MAYBE_PROGMEM = "ADJUST VOLTS";
const char configMsg36[] MAYBE_PROGMEM = "CELLS";
const char configMsg37[] MAYBE_PROGMEM = "USE FC";

//-----------------------------------------------------------Page4
const char configMsg40[] MAYBE_PROGMEM = "RSSI";
const char configMsg42[] MAYBE_PROGMEM = "DISPLAY RSSI";
const char configMsg43[] MAYBE_PROGMEM = "SET RSSI";
const char configMsg44[] MAYBE_PROGMEM = "SET RSSI MAX";
const char configMsg45[] MAYBE_PROGMEM = "SET RSSI MIN";
const char configMsg46[] MAYBE_PROGMEM = "USE PWM";

//-----------------------------------------------------------Page5
const char configMsg50[] MAYBE_PROGMEM = "CURRENT";
const char configMsg51[] MAYBE_PROGMEM = "DISPLAY AMPS";
const char configMsg52[] MAYBE_PROGMEM = "DISPLAY MAH";
const char configMsg53[] MAYBE_PROGMEM = "USE VIRTUAL SENSOR";
const char configMsg54[] MAYBE_PROGMEM = "ADJUST AMPS";
const char configMsg55[] MAYBE_PROGMEM = "ADJUST ZERO";
//-----------------------------------------------------------Page6
const char configMsg60[] MAYBE_PROGMEM = "DISPLAY";
const char configMsg61[] MAYBE_PROGMEM = "HORIZON";
const char configMsg62[] MAYBE_PROGMEM = "SIDE BARS";
const char configMsg63[] MAYBE_PROGMEM = "SCROLLING BARS";
const char configMsg64[] MAYBE_PROGMEM = "THROTTLE";
const char configMsg65[] MAYBE_PROGMEM = "GPS COORDS";
const char configMsg66[] MAYBE_PROGMEM = "SENSORS";
const char configMsg67[] MAYBE_PROGMEM = "GIMBAL";
const char configMsg68[] MAYBE_PROGMEM = "MAP MODE";
//-----------------------------------------------------------Page7
const char configMsg70[]  MAYBE_PROGMEM = "ADVANCED";
const char configMsg71[]  MAYBE_PROGMEM = "UNITS";
const char configMsg710[] MAYBE_PROGMEM = "MET";
const char configMsg711[] MAYBE_PROGMEM = "IMP";
const char configMsg72[]  MAYBE_PROGMEM = "SIGNAL";
const char configMsg720[] MAYBE_PROGMEM = "NTSC";
const char configMsg721[] MAYBE_PROGMEM = "PAL";
const char configMsg73[]  MAYBE_PROGMEM = "V REF";
const char configMsg730[] MAYBE_PROGMEM = "5V";
const char configMsg731[] MAYBE_PROGMEM = "1.1V";
const char configMsg74[]  MAYBE_PROGMEM = "DEBUG";
const char configMsg75[]  MAYBE_PROGMEM = "MAG CAL";
const char configMsg76[]  MAYBE_PROGMEM = "OSD TX CH";
//-----------------------------------------------------------Page8
const char configMsg80[] MAYBE_PROGMEM = "GPS TIME";
const char configMsg81[] MAYBE_PROGMEM = "DISPLAY";
const char configMsg82[] MAYBE_PROGMEM = "TZ FORWARD";
const char configMsg83[] MAYBE_PROGMEM = "TZ ADJUST";
//-----------------------------------------------------------Page9
const char configMsg90[] MAYBE_PROGMEM = "ALARMS";
const char configMsg91[] MAYBE_PROGMEM = "DISTANCE X100";
const char configMsg92[] MAYBE_PROGMEM = "ALTITUDE X10";
const char configMsg93[] MAYBE_PROGMEM = "SPEED";
const char configMsg94[] MAYBE_PROGMEM = "TIMER";
const char configMsg95[] MAYBE_PROGMEM = "MAH X100";
const char configMsg96[] MAYBE_PROGMEM = "AMPS";
//-----------------------------------------------------------Page10
const char configMsg100[] MAYBE_PROGMEM = "ADVANCE TUNING";
const char configMsg101[] MAYBE_PROGMEM = "PROFILE";
const char configMsg102[] MAYBE_PROGMEM = "PID CONTROLLER";
const char configMsg103[] MAYBE_PROGMEM = "LOOPTIME";

// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const unsigned char speedUnitAdd[2] ={
  0xa5,0xa6} ; // [0][0] and [0][1] = Km/h   [1][0] and [1][1] = Mph
const unsigned char temperatureUnitAdd[2] = {
  0x0e,0x0d};

const unsigned char MwAltitudeAdd[2]={
  0xa7,0xa8};
const unsigned char MwClimbRateAdd[2]={
  0x9f,0x99};
const unsigned char GPS_distanceToHomeAdd[2]={
  0xbb,0xb9};
const unsigned char MwGPSAltPositionAdd[2]={
  0xa7,0xa8};


enum Positions {
  GPS_numSatPosition,
  GPS_directionToHomePosition,
  GPS_distanceToHomePosition,
  speedPosition,
  GPS_angleToHomePosition,
  MwGPSAltPosition,
  sensorPosition,
  MwHeadingPosition,
  MwHeadingGraphPosition,
  MwAltitudePosition,
  MwClimbRatePosition,
  CurrentThrottlePosition,
  flyTimePosition,
  onTimePosition,
  motorArmedPosition,
  pitchAnglePosition,
  rollAnglePosition,
  MwGPSLatPositionTop,
  MwGPSLonPositionTop,
  rssiPosition,
  temperaturePosition,
  voltagePosition,
  vidvoltagePosition,
  amperagePosition,
  pMeterSumPosition,
  horizonPosition,
  SideBarPosition,
  SideBarScrollPosition,
  SideBarHeightPosition,
  SideBarWidthPosition,
  gimbalPosition,
  GPS_timePosition,
  SportPosition,
  ModePosition,
  MapModePosition,
  MapCenterPosition,
  APstatusPosition,
  wattPosition,
  glidescopePosition,
  callSignPosition,
  debugPosition,

  POSITIONS_SETTINGS
};

// $$$: share between eeprom and screen
static uint16_t screenPosition[POSITIONS_SETTINGS];

// Menu selections
const MAYBE_PROGMEM char * const menu_choice_unit[] =
{
  configMsg710,
  configMsg711,
};
// Menu selections
const MAYBE_PROGMEM char * const menu_choice_video[] =
{
  configMsg720,
  configMsg721,
};// Menu selections
const MAYBE_PROGMEM char * const menu_choice_ref[] =
{
  configMsg731,
  configMsg730,
};

// Menu
//MAYBE_PROGMEM const char *menu_stats_item[] =
const MAYBE_PROGMEM char * const menu_stats_item[] =
{
  configMsg01,
  configMsg02,
  configMsg03,
  configMsg04,
  configMsg05,
  configMsg06,
  configMsg07,
};

const MAYBE_PROGMEM char * const menu_pid[] =
{
  configMsg11,
  configMsg12,
  configMsg13,
  configMsg14,
  configMsg15,
  configMsg16,
  configMsg17,
};

const MAYBE_PROGMEM char * const menu_rc[] =
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

const MAYBE_PROGMEM char * const menu_bat[] =
{
  configMsg31,
  configMsg32,
  configMsg33,
  configMsg34,
  configMsg32,
  configMsg36,
  configMsgMWII,
};

const MAYBE_PROGMEM char * const menu_rssi[] =
{
  configMsg42,
  configMsg43,
  configMsgMWII,
  configMsg46,
  configMsg44,
  configMsg45,
};

const MAYBE_PROGMEM char * const menu_amps[] =
{
  configMsg51,
  configMsg52,
  configMsg53,
  configMsg54,
  configMsg55,
};

const MAYBE_PROGMEM char * const menu_display[] =
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

const MAYBE_PROGMEM char * const menu_advanced[] =
{
  configMsg71,
  configMsg72,
  configMsg73,
  configMsg74,
  configMsg75,
  configMsg76,};

const MAYBE_PROGMEM char * const menu_gps_time[] =
{
  configMsg81,
  configMsg82,
  configMsg83,
};

const MAYBE_PROGMEM char * const menu_alarm_item[] =
{
  configMsg91,
  configMsg92,
  configMsg93,
  configMsg94,
  configMsg95,
  configMsg96,
};

const MAYBE_PROGMEM char * const menu_profile[] =
{
  configMsg101,
  configMsg102,
  configMsg103,
};

const MAYBE_PROGMEM char * const menutitle_item[] =
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

const MAYBE_PROGMEM char * const menu_on_off[] =
{
  configMsgOFF,
  configMsgON,
};

