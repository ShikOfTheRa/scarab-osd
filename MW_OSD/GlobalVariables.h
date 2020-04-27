#define POS_MASK        0x01FF
#define PAL_MASK        0x0003
#define PAL_SHFT             9
#define DISPLAY_MASK    0xC000
#define DISPLAY_ALWAYS  0xC000
#define DISPLAY_NEVER   0x0000
#ifndef DISPLAY_DEV
  #define DISPLAY_DEV     0x0000
#endif
#define POS(pos, pal_off, disp)  (((pos)&POS_MASK)|((pal_off)<<PAL_SHFT)|(disp))


#define MPH 1
#define KMH 0
#define METRIC 0
#define IMPERIAL 1
#define lo_speed_cycle  100
#define sync_speed_cycle  33

#define CALIBRATION_DELAY 10       // Calibration timeouts   
#define EEPROM_WRITE_DELAY 5       // Calibration timeouts

// DEFINE CONFIGURATION MENU PAGES
#define MINPAGE 0

#ifdef KISS
#define PIDITEMS 3
#else
#define PIDITEMS 10
#endif // KISS

// STICK POSITION
#define MAXSTICK         1850
#define MINSTICK         1150
#define MINTHROTTLE      1000

// FOR POSITION OF PID CONFIG VALUE
#define ROLLT 93
#define ROLLP 101
#define ROLLI 107
#define ROLLD 113
#ifdef KISS
#define ROLLRC 101
#define ROLLRATE 107
#define ROLLCURVE 113
#endif // KISS
#define PITCHT 93+(30*1)
#define PITCHP 101+(30*1)
#define PITCHI 107+(30*1)
#define PITCHD 113+(30*1)
#define YAWT 93+(30*2)
#define YAWP 101+(30*2)
#define YAWI 107+(30*2)
#define YAWD 113+(30*2)
#define ALTT 93+(30*3)
#define ALTP 101+(30*3)
#define ALTI 107+(30*3)
#define ALTD 113+(30*3)
#define VELT 93+(30*4)
#define VELP 101+(30*4)
#define VELI 107+(30*4)
#define VELD 113+(30*4)
#define LEVT 93+(30*5)
#define LEVP 101+(30*5)
#define LEVI 107+(30*5)
#define LEVD 113+(30*5)
#define MAGT 93+(30*6)
#define MAGP 101+(30*6)
#define MAGI 107+(30*6)
#define MAGD 113+(30*6)

#define SAVEP 93+(30*9)

#define LINE      30
#define LINE01    0
#define LINE02    30
#define LINE03    60
#define LINE04    90
#define LINE05    120
#define LINE06    150
#define LINE07    180
#define LINE08    210
#define LINE09    240
#define LINE10    270
#define LINE11    300
#define LINE12    330
#define LINE13    360
#define LINE14    390
#define LINE15    420
#define LINE16    450

#ifdef KISS
#define KISS_LINEFIRSTSUBMENU LINE05+8
#endif

/********************       For Sensors presence      *********************/
#define ACCELEROMETER  1//0b00000001
#define BAROMETER      2//0b00000010
#define MAGNETOMETER   4//0b00000100
#define GPSSENSOR      8//0b00001000
//#define SONAR         16//0b00010000

/********************       VTX      *********************/

#ifdef VTX_RTC6705

uint8_t vtxPower;
uint8_t vtxBand;
uint8_t vtxChannel;

#define VTX_STICK_CMD_DELAY                   2

#ifdef VTX_REGION_UNRESTRICTED

  #define VTX_DEFAULT_CHANNEL                 0
  #define VTX_DEFAULT_BAND                    3
  #define VTX_DEFAULT_POWER                   0
  
  #define VTX_BAND_COUNT                      5
  #define VTX_CHANNEL_COUNT                   8

  // XXX Kludge
  // We take advantage of the co-inciddence that Innova's power selection is 25-200mW.
  // So boring to do this {Hardwares} x {Regulatory woes} correctly.

# ifdef FFPV_INNOVA
#   define VTX_POWER_COUNT                    2
# else
#   define VTX_POWER_COUNT                    3
# endif
  
  const PROGMEM uint16_t vtx_frequencies[VTX_BAND_COUNT][VTX_CHANNEL_COUNT] = {
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, //A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, //B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, //E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, //F
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }  //R
  };
  
#elif defined(VTX_REGION_AUSTRALIA)

  #define VTX_DEFAULT_CHANNEL                 0
  #define VTX_DEFAULT_BAND                    2
  #define VTX_DEFAULT_POWER                   0
  
  #define VTX_BAND_COUNT                      4
  #define VTX_CHANNEL_COUNT                   8

  // XXX Kludge Here, we again take advantage of the co-incidence that
  // Helix and Innova has the same and only power selection for this region.
  #define VTX_POWER_COUNT                     1
  
  const PROGMEM uint16_t vtx_frequencies[VTX_BAND_COUNT][VTX_CHANNEL_COUNT] = {
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, //A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, //B
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5860 }, //F
    { 5732, 5732, 5732, 5769, 5806, 5843, 5843, 5843 }  //R
  };
#endif //VTX_REGION_XXXXX

#endif // VTX_RTC6705

/********************  RX channel rule definitions  *********************/
#if defined TX_GUI_CONTROL   //PITCH,YAW,THROTTLE,ROLL order controlled by GUI    
  uint8_t tx_roll;
  uint8_t tx_pitch;
  uint8_t tx_yaw;
  uint8_t tx_throttle;
  #define ROLLSTICK        tx_roll
  #define PITCHSTICK       tx_pitch
  #define YAWSTICK         tx_yaw
  #define THROTTLESTICK    tx_throttle
#elif defined TX_PYTR      //PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4 //For Graupner/Spektrum    
  #define ROLLSTICK        4
  #define PITCHSTICK       1
  #define YAWSTICK         2
  #define THROTTLESTICK    3
#elif defined TX_RPTY      //ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For Robe/Hitec/Futaba
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         4
  #define THROTTLESTICK    3
#elif defined TX_RPYT      //ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4 //For Multiplex
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         3
  #define THROTTLESTICK    4
#elif defined TX_PRTY      //PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For some Hitec/Sanwa/Others
  #define ROLLSTICK        2
  #define PITCHSTICK       1
  #define YAWSTICK         4
  #define THROTTLESTICK    3
#elif defined TX_TRPY      //THROTTLE,ROLL,PITCH,YAW,AUX1,AUX2,AUX3,AUX4 //For some JR
  #define ROLLSTICK        2
  #define PITCHSTICK       3
  #define YAWSTICK         4
  #define THROTTLESTICK    1
#elif defined KISS
  #define ROLLSTICK        2
  #define PITCHSTICK       3
  #define YAWSTICK         4
  #define THROTTLESTICK    1
#elif defined PX4            //ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For PX4
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         4
  #define THROTTLESTICK    3
#elif defined APM            //ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For APM
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         4
  #define THROTTLESTICK    3
#else
  // RX CHANEL IN MwRcData table
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         3
  #define THROTTLESTICK    4
#endif
/*****************************************/







#if defined (ALARM_MSP)
#define DATA_MSP ALARM_MSP
#else
#define DATA_MSP 5   
#endif

#define NAZA_PWM_LOW  1000
#define NAZA_PMW_MED  1400
#define NAZA_PMW_HIGH 1600

uint16_t MAX_screen_size;
char screen[480];          // Main screen ram for MAX7456
#ifdef INVERTED_CHAR_SUPPORT
uint8_t screenAttr[480/8]; // Attribute (INV) bits for each char in screen[]
#endif
char screenBuffer[20]; 
uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;
uint8_t sensorpinarray[]={VOLTAGEPIN,VIDVOLTAGEPIN,AMPERAGEPIN,AUXPIN,RSSIPIN};  
unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
unsigned long previous_millis_sync =0;
unsigned long previous_millis_rssi =0;

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
uint8_t fontStatus=0;
boolean ledstatus=HIGH;
#endif

#ifdef KKAUDIOVARIO
unsigned int calibrationData[7];
//unsigned long time = 0;
float toneFreq, toneFreqLowpass, pressure, lowpassFast, lowpassSlow ;
int ddsAcc;
#endif

//General use variables
struct  __timer {
  uint8_t  tenthSec;
  uint8_t  halfSec;
  uint8_t  Blink2hz;                          // This is turing on and off at 2hz
  uint8_t  Blink10hz;                         // This is turing on and off at 10hz
  uint16_t lastCallSign;                      // Callsign_timer
  uint8_t  rssiTimer;
//  uint8_t accCalibrationTimer;
  uint8_t  magCalibrationTimer;
  uint32_t fwAltitudeTimer;
  uint32_t seconds;
  uint8_t  MSP_active;
  uint8_t  GPS_active;
  uint8_t  GUI_active;
  uint8_t  GPS_initdelay;
  uint16_t  loopcount;
  uint16_t  packetcount;
  uint16_t  serialrxrate;
#ifdef DEBUGDPOSMAV 
  uint16_t  d0rate;
  uint16_t  d1rate;  
  uint16_t  d2rate;  
  uint16_t  d3rate;
#endif    
  uint32_t alarms;                            // Alarm length timer
  uint32_t vario;                             
  uint32_t audiolooptimer;
  uint32_t GPSOSDstate;
  uint8_t  disarmed;                             
  uint8_t  MAVstatustext;
  uint8_t  armedstatus;                             

}
timer;

struct __flags {
  uint8_t reset;
  uint8_t signaltype;
  uint8_t signalauto;
  uint8_t vario;  
}
flags;

struct __datetime {
  uint32_t unixtime;
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hours;
  uint8_t  minutes;  
  uint8_t  seconds;  
}
datetime;


struct __display {
  uint32_t distance;
}
display;

struct __cfg {
    uint8_t  fw_althold_dir;
    uint16_t fw_gps_maxcorr;                    // Degrees banking Allowed by GPS.
    int16_t  fw_gps_rudder;                     // Maximum input of Rudder Allowed by GPS.
    int16_t  fw_gps_maxclimb;                   // Degrees climbing . To much can stall the plane.
    int16_t  fw_gps_maxdive;                    // Degrees Diving . To much can overspeed the plane.
    uint16_t fw_climb_throttle;                 // Max allowed throttle in GPS modes.
    uint16_t fw_cruise_throttle;                // Throttle to set for cruisespeed.
    uint16_t fw_idle_throttle;                  // Lowest throttleValue during Descend
    uint16_t fw_scaler_throttle;                // Adjust to Match Power/Weight ratio of your model
    uint8_t  fw_roll_comp;                      // Adds Elevator Based on Roll Angle
    uint8_t  fw_rth_alt;                        // Min Altitude to keep during RTH. (Max 200m)
}
cfg;

struct __cfgpa {
  uint16_t rollPitchItermIgnoreRate;
  uint16_t yawItermIgnoreRate;
  uint16_t yaw_p_limit;
  uint8_t  t0;
  uint8_t  vbatPidCompensation;
  uint8_t  ptermSRateWeight;
  uint8_t  dtermSetpointWeight;
  uint8_t  t1;
  uint8_t  t2;
  uint8_t  itermThrottleGain;
  uint16_t rateAccelLimit;
  uint16_t yawRateAccelLimit;
}
cfgpa;

struct __FC {
uint8_t    verMajor;
uint8_t    verMinor;
uint8_t    verPatch;  
}
FC;

#ifdef MENU_SERVO  
#define MAX_SERVOS 8
struct __servo {
    uint16_t settings[5][MAX_SERVOS];
}
servo;
#endif //MENU_SERVO  


/********************       Development/ test parameters      *********************/
uint16_t debug[4];
#ifdef DEBUGDPOSMSPID    
  uint8_t boxidarray[50];
#endif

int8_t menudir;
unsigned int allSec=0;
unsigned int menuSec=0;
uint8_t armedtimer=30;
uint16_t debugerror;
uint16_t debugval=0;
uint16_t cell_data[6]={0,0,0,0,0,0};
uint16_t cycleTime;
uint16_t I2CError;
uint8_t oldROW=0;
uint8_t cells=0;
uint8_t rcswitch_ch=8;
volatile uint16_t pwmval1=0;
volatile uint16_t pwmval2=0;
uint8_t debugtext=0;
uint8_t MSP_home_set=0;
uint8_t variopitch=0;
uint8_t MAVstatuslength;

#if defined CORRECT_MSP_BF1
  uint8_t bfconfig[25];
#endif

// Canvas mode
#ifdef CANVAS_SUPPORT
bool canvasMode = false;
uint32_t lastCanvas = 0;
#define CANVAS_TIMO 2000  // Canvas mode timeout in msec.
#endif

// Config status and cursor location
uint8_t screenlayout=0;
uint8_t oldscreenlayout=0;
uint8_t ROW=10;
uint8_t COL=3;
int8_t configPage=1;
int8_t previousconfigPage=1;
uint8_t configMode=0;
uint8_t fontMode = 0;
uint8_t fontData[54];
uint8_t nextCharToRequest;
uint8_t lastCharToRequest;
uint8_t retransmitQueue;

uint16_t eeaddress = 0;
uint8_t eedata = 0;
uint8_t settingsMode=0;
//uint32_t MSP_OSD_timer=0;
uint16_t framerate = 0;
uint16_t packetrate = 0;
uint16_t serialrxrate = 0;

// Mode bits
struct __mode {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint32_t baro;
  uint32_t mag;
  uint32_t camstab;
  uint32_t gpshome;
  uint32_t gpshold;
  uint32_t passthru;
  uint32_t failsafe;
  uint32_t air;
  uint32_t acroplus;
  uint32_t osd_switch;
  uint32_t llights;
  uint32_t gpsmission;
  uint32_t cruise;
#ifdef EXTENDEDMODESUPPORT
  uint32_t gpsland;
  uint32_t autotune;
  uint32_t autotrim;
  uint32_t launch;
  uint32_t flaperon;
#endif
}mode;

// Settings Locations
enum Setting16_ {
  S16_AMPZERO,
  S16_AMPDIVIDERRATIO,
  S16_RSSIMIN,
  S16_RSSIMAX,
  S16_AUX_ZERO_CAL,
  S16_AUX_CAL,
  
  // EEPROM16_SETTINGS must be last!
  EEPROM16_SETTINGS
};

// Settings Locations
enum Setting_ {
  S_CHECK_,
S_AUTOCELL,
  S_VIDVOLTAGEMIN,
  S_RSSI_ALARM,
S_AUTOCELL_ALARM,
  S_MWRSSI,
S_RSSI_CH,
S_TX_TYPE,
  S_VOLTAGEMIN,
  S_BATCELLS,
  S_DIVIDERRATIO,
  S_MAINVOLTAGE_VBAT,
S_SIDEBARHEIGHT,
  S_MWAMPERAGE,
S_TX_CH_REVERSE,
S_MAV_SYS_ID,
  S_ALARMS_TEXT,
S_CALLSIGN_ALWAYS,
  S_VIDDIVIDERRATIO,
  S_THROTTLE_PWM,
  S_AMPER_HOUR_ALARM,
  S_AMPERAGE_ALARM,
S_VARIO_SCALE,
  S_GPS_MASK,
S_USEGPSHEADING,
  S_UNITSYSTEM,
  S_VIDEOSIGNALTYPE,
  S_SHOWBATLEVELEVOLUTION,
  S_RESETSTATISTICS,
  S_MAPMODE,
  S_VREFERENCE,
S_SIDEBARWIDTH,
  S_GPSTIME,
  S_GPSTZAHEAD,
  S_GPSTZ,
  S_VTX_POWER,
  S_VTX_BAND,
  S_VTX_CHANNEL,
  S_RCWSWITCH,
  S_RCWSWITCH_CH,
  S_DISTANCE_ALARM,
  S_ALTITUDE_ALARM,
  S_SPEED_ALARM,
  S_FLYTIME_ALARM,
  S_AUDVARIO_DEADBAND,
  S_AUDVARIO_TH_CUT,
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
S_PWM_PPM,
S_ELEVATIONS,
S_ALTRESOLUTION, 
S_FLIGHTMODETEXT,
S_BRIGHTNESS,
S_MAV_ALARMLEVEL,
S_GLIDESCOPE,
S_LOSTMODEL,
S_MAV_AUTO,

  // EEPROM_SETTINGS must be last!
  EEPROM_SETTINGS
};


uint8_t  Settings[EEPROM_SETTINGS];
uint16_t Settings16[EEPROM16_SETTINGS];

// Default EEPROM values

#ifdef PROTOCOL_MAVLINK
  #define DEF_S_MAINVOLTAGE_VBAT 1 // 1
  #define DEF_S_TX_TYPE 1        // 1
  #define DEF_S_MWRSSI 2         // 1
  #define DEF_S_MWAMPERAGE 1     // 1
  #define DEF_S_RCWSWITCH 1      // S_RCWSWITCH,
  #define DEF_S_RCWSWITCH_CH 8   // S_RCWSWITCH_CH,
  #define DEF_S_ALTRESOLUTION 0
  #ifdef PX4
    #define DEF_S16_RSSIMAX 2600   // S16_RSSIMAX PX4 non standard 
  #else
    #define DEF_S16_RSSIMAX 1023   // S16_RSSIMAX PX4 default 
  #endif
#else
  #define DEF_S_MAINVOLTAGE_VBAT 0
  #define DEF_S_TX_TYPE 0
  #define DEF_S_MWRSSI 0
  #define DEF_S_MWAMPERAGE 0
  #define DEF_S_RCWSWITCH 0
  #define DEF_S_RCWSWITCH_CH 8
  #define DEF_S_ALTRESOLUTION 10
  #define DEF_S16_RSSIMAX 1023   // S16_RSSIMAX PX4 default 
#endif

#if defined (UBLOX) || defined iNAV  || defined (MAV_RTC)
  #define DEF_S_GPSTIME 1
#else
  #define DEF_S_GPSTIME 0
#endif

#ifdef USE_AIRSPEED_SENSOR
  #define DEF_S16_AUX_ZERO_CAL 512
  #define DEF_S16_AUX_CAL 500
#else
  #define DEF_S16_AUX_ZERO_CAL 0
  #define DEF_S16_AUX_CAL 931
#endif


// For Settings Defaults
PROGMEM const uint8_t EEPROM_DEFAULT[EEPROM_SETTINGS] = {
EEPROMVER, //   S_CHECK_,    
1, // S_AUTOCELL
0, //   S_VIDVOLTAGEMIN,
0, //   S_RSSI_ALARM,
34, // S_AUTOCELL_ALARM
DEF_S_MWRSSI, //   S_MWRSSI,
8, //   S_RSSI_CH,
DEF_S_TX_TYPE, // S_TX_TYPE
0, //   S_VOLTAGEMIN,
4, //   S_BATCELLS,
200, //   S_DIVIDERRATIO,
DEF_S_MAINVOLTAGE_VBAT, //   S_MAINVOLTAGE_VBAT,
3, //   S_SIDEBARHEIGHT,
DEF_S_MWAMPERAGE, //   S_MWAMPERAGE,
0, // S_TX_CH_REVERSE
1, //   S_MAV_SYS_ID,   //MAVLINK SYS id
1, //   S_ALARMS_TEXT,
1, // S_CALLSIGN_ALWAYS
200, //   S_VIDDIVIDERRATIO,
0, //   S_THROTTLE_PWM,
50, //   S_AMPER_HOUR_ALARM,
100, //   S_AMPERAGE_ALARM,
2, // S_VARIO_SCALE
0, // S_GPS_MASK
2, //   S_USEGPSHEADING, // fixedwing only
0, //   S_UNITSYSTEM,
2, //   S_VIDEOSIGNALTYPE,
1, //   S_SHOWBATLEVELEVOLUTION,
0, //   S_RESETSTATISTICS,
1, //   S_MAPMODE,
0, //   S_VREFERENCE,
7, //   S_SIDEBARWIDTH,
DEF_S_GPSTIME, //   S_GPSTIME,
0, //   S_GPSTZAHEAD,
128, //   S_GPSTZ,
#ifdef VTX_RTC6705
VTX_DEFAULT_POWER,    // S_VTX_POWER
VTX_DEFAULT_BAND,     // S_VTX_BAND
VTX_DEFAULT_CHANNEL,  // S_VTX_CHANNEL
#else
0,   // S_VTX_POWER
0,   // S_VTX_BAND
0,   // S_VTX_CHANNEL
#endif
DEF_S_RCWSWITCH,   // S_RCWSWITCH,
8,   // S_RCWSWITCH_CH,
0,   // S_DISTANCE_ALARM,
0,   // S_ALTITUDE_ALARM,
0,   // S_SPEED_ALARM,
30,  // S_FLYTIME_ALARM
30,  // S_AUDVARIO_DEADBAND
20,  // S_AUDVARIO_TH_CUT

0x4D,   // S_CS0,
0x57,   // S_CS1,
0x4F,   // S_CS2,
0x53,   // S_CS3,
0x44,   // S_CS4,
0x00,   // S_CS5,
0x20,   // S_CS6,
0x20,   // S_CS7,
0x20,   // S_CS8,
0x20,   // S_CS9,
0,      // S_PWM_PPM,
0,      // S_ELEVATIONS,
DEF_S_ALTRESOLUTION,     // S_ALTRESOLUTION 
0,      // S_FLIGHTMODETEXT
1,      // S_BRIGHTNESS
6,      // S_MAV_ALARMLEVEL
0,      // S_GLIDESCOPE - not used
0,      // S_LOSTMODEL - not used
1,      // S_MAV_AUTO
};

PROGMEM const uint16_t EEPROM16_DEFAULT[EEPROM16_SETTINGS] = {
  0,// S16_AMPZERO,
  150,// S16_AMPDIVIDERRATIO,
  0,// S16_RSSIMIN,
  DEF_S16_RSSIMAX,// S16_RSSIMAX = 1024 default, 2600 PX4,
  DEF_S16_AUX_ZERO_CAL,// S16_AUX_ZERO_CAL,
  DEF_S16_AUX_CAL,// S16_AUX_CAL,
  
};

enum Positions {
  GPS_numSatPosition,
  GPS_directionToHomePosition,
  GPS_distanceToHomePosition,
  GPS_speedPosition,
  GPS_angleToHomePosition,
  MwGPSAltPosition,
  sensorPosition,
  MwHeadingPosition,
  MwHeadingGraphPosition,
  MwAltitudePosition,
  MwVarioPosition,
  CurrentThrottlePosition,
  timer2Position,
  timer1Position,
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
  SideBarHeightSPARE,           // special function
  SideBarWidthSPARE,            // special function
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
  climbratevaluePosition,
  efficiencyPosition,
  avgefficiencyPosition,
  AIR_speedPosition,
  MAX_speedPosition,
  TotalDistanceposition,
  WIND_speedPosition,
  MaxDistanceposition,
  DOPposition,

  POSITIONS_SETTINGS
};

#ifdef PROTOCOL_MAVLINK
  #define DEF_sensorPosition DISPLAY_NEVER
  #define DEF_horizonPosition DISPLAY_ALWAYS
  #define DEF_modePosition DISPLAY_ALWAYS
#elif defined GPSOSD
  #define DEF_sensorPosition DISPLAY_NEVER
  #define DEF_horizonPosition DISPLAY_NEVER
  #define DEF_modePosition DISPLAY_NEVER
#else
  #define DEF_sensorPosition DISPLAY_ALWAYS
  #define DEF_horizonPosition DISPLAY_ALWAYS
  #define DEF_modePosition DISPLAY_ALWAYS
#endif

uint16_t screenPosition[POSITIONS_SETTINGS];

PROGMEM const uint16_t SCREENLAYOUT_DEFAULT[POSITIONS_SETTINGS] = {
(LINE02+2)|DISPLAY_ALWAYS|DISPLAY_DEV,    // GPS_numSatPosition
(LINE02+20)|DISPLAY_ALWAYS|DISPLAY_DEV,   // GPS_directionToHomePosition
(LINE02+22)|DISPLAY_ALWAYS|DISPLAY_DEV,   // GPS_distanceToHomePosition
(LINE07+2)|DISPLAY_ALWAYS|DISPLAY_DEV,    // GPS_speedPosition
(LINE05+23)|DISPLAY_NEVER|DISPLAY_DEV,    // GPS_angleToHomePosition
(LINE06+23)|DISPLAY_NEVER|DISPLAY_DEV,    // MwGPSAltPosition
(LINE02+6)|DEF_sensorPosition|DISPLAY_DEV,    // sensorPosition
(LINE04+23)|DISPLAY_NEVER|DISPLAY_DEV,    // MwHeadingPosition
(LINE02+10)|DISPLAY_ALWAYS|DISPLAY_DEV,   // MwHeadingGraphPosition
(LINE07+23)|DISPLAY_ALWAYS|DISPLAY_DEV,   // MwAltitudePosition
(LINE07+22)|DISPLAY_ALWAYS|DISPLAY_DEV,   // MwVarioPosition
(LINE12+22)|DISPLAY_ALWAYS|DISPLAY_DEV,   // CurrentThrottlePosition
(LINE14+22)|DISPLAY_NEVER|DISPLAY_DEV,    // remainingTimePosition
(LINE13+22)|DISPLAY_ALWAYS|DISPLAY_DEV,   // onTimePosition
(LINE11+11)|DISPLAY_ALWAYS|DISPLAY_DEV,   // motorArmedPosition
(LINE10+22)|DISPLAY_NEVER|DISPLAY_DEV,    // pitchAnglePosition
(LINE11+22)|DISPLAY_NEVER|DISPLAY_DEV,    // rollAnglePosition
(LINE01+2)|DISPLAY_ALWAYS|DISPLAY_DEV,    // MwGPSLatPositionTop         On top of screen
(LINE01+15)|DISPLAY_ALWAYS|DISPLAY_DEV,   // MwGPSLonPositionTop         On top of screen
(LINE12+2)|DISPLAY_ALWAYS|DISPLAY_DEV,    // rssiPosition
(LINE09+8)|DISPLAY_NEVER|DISPLAY_DEV,     // temperaturePosition
(LINE13+2)|DISPLAY_ALWAYS|DISPLAY_DEV,    // voltagePosition
(LINE11+2)|DISPLAY_NEVER|DISPLAY_DEV,     // vidvoltagePosition
(LINE13+9)|DISPLAY_ALWAYS|DISPLAY_DEV,    // amperagePosition
(LINE13+16)|DISPLAY_ALWAYS|DISPLAY_DEV,   // pMeterSumPosition
(LINE07+14)|DEF_horizonPosition|DISPLAY_DEV,   // horizonPosition
(LINE07+7)|DISPLAY_ALWAYS|DISPLAY_DEV,    // SideBarPosition
(LINE07+7)|DISPLAY_NEVER|DISPLAY_DEV,    // SideBarScrollPosition        Move to 8 bit
(LINE01+3)|DISPLAY_NEVER,                 //                             Unused
(LINE01+7)|DISPLAY_NEVER,                 //                             Unused
(LINE04+2)|DISPLAY_NEVER|DISPLAY_DEV,     // Gimbal Position
(LINE09+11)|DISPLAY_NEVER|DISPLAY_DEV,   // GPS_time Position
(LINE09+22)|DISPLAY_NEVER|DISPLAY_DEV,    // SportPosition
(LINE03+2)|DEF_modePosition|DISPLAY_DEV,  // modePosition
(LINE02+22)|DISPLAY_NEVER,                // MapModePosition
(LINE07+15)|DISPLAY_NEVER,                // MapCenterPosition
(LINE04+10)|DISPLAY_ALWAYS|DISPLAY_DEV,   // APstatusPosition
(LINE10+2)|DISPLAY_NEVER|DISPLAY_DEV,     // wattPosition
(LINE07+6)|DISPLAY_ALWAYS|DISPLAY_DEV,    // glidescopePosition          Only enabled in fixedwing options
(LINE12+12)|DISPLAY_NEVER|DISPLAY_DEV,    // callSignPosition
(LINE08+10)|DISPLAY_NEVER|DISPLAY_DEV,    // Debug Position
(LINE08+23)|DISPLAY_NEVER|DISPLAY_DEV,    // climbratevaluePosition,
(LINE09+2)|DISPLAY_NEVER|DISPLAY_DEV,     // efficiencyPosition,
(LINE08+2)|DISPLAY_NEVER|DISPLAY_DEV,     // avgefficiencyPosition,
(LINE06+2)|DISPLAY_NEVER|DISPLAY_DEV,     // AIR_speedposition,
(LINE05+8)|DISPLAY_NEVER|DISPLAY_DEV,     // MAX_speedposition,
(LINE08+8)|DISPLAY_NEVER|DISPLAY_DEV,     // TotalDistanceposition
(LINE03+22)|DISPLAY_NEVER|DISPLAY_DEV,    // WIND_speedposition,
(LINE06+8)|DISPLAY_NEVER|DISPLAY_DEV,     // MaxDistanceposition
(LINE05+2)|DISPLAY_NEVER|DISPLAY_DEV,     // DOPposition
};

#ifdef KISS
static uint16_t pidP[PIDITEMS], pidI[PIDITEMS], pidD[PIDITEMS];
static uint16_t rateRC[PIDITEMS], rateRate[PIDITEMS], rateCurve[PIDITEMS];
static uint8_t rpLPF = 0;
static uint8_t yawCFilter = 0;
static bool nfRollEnable = false;
static uint16_t nfRollCenter = 0;
static uint16_t nfRollCutoff = 0;
static bool nfPitchEnable = false;
static uint16_t nfPitchCenter = 0;
static uint16_t nfPitchCutoff = 0;
static uint8_t yawLPF = 0;
static uint8_t dtermLPF = 0;
static uint8_t vtxType = 0;
static uint8_t vtxNChannel = 0;
static uint16_t vtxLowPower = 0;
static uint16_t vtxMaxPower = 0;
static uint8_t vtxBand = 0;
static uint8_t vtxChannel = 1;
#else
static uint8_t pidP[PIDITEMS], pidI[PIDITEMS], pidD[PIDITEMS];
static uint8_t rcRate8,rcExpo8;
static uint8_t rollPitchRate;
static uint8_t rollRate;
static uint8_t PitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t thrMid8;
static uint8_t thrExpo8;
static uint16_t tpa_breakpoint16;
static uint8_t rcYawExpo8;
static uint8_t FCProfile;
static uint8_t CurrentFCProfile;
static uint8_t PreviousFCProfile;
static uint8_t PIDController;
static uint16_t LoopTime;
#endif // KISS

int32_t  MwAltitude=0;                         // This hold barometric value
int32_t  old_MwAltitude=0;                     // This hold barometric value


int16_t MwAngle[2]={0,0};           // Those will hold Accelerometer Angle
volatile uint16_t MwRcData[1+16];



// for analogue / PWM sensor filtering 
#define SENSORFILTERSIZE 8
#define SENSORTOTAL 5
#define FHBANDWIDTH 100

int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2]; 

uint16_t  MwSensorPresent=0;
uint32_t  MwSensorActive=0;
uint16_t MwVBat=0;
uint8_t MwVBat2=0;
int16_t MwVario=0;
uint8_t armed=0;
uint8_t previousarmedstatus=0;  // for statistics after disarming
uint16_t armedangle=0;           // for capturing direction at arming
uint32_t GPS_distanceToHome=0;
uint8_t GPS_fix=0;
uint8_t GPS_frame_timer=0;
int32_t GPS_latitude;
int32_t GPS_longitude;
int32_t GPS_altitude;
int16_t MAV_altitude;                          
int32_t GPS_altitude_ASL;
int32_t GPS_altitude_vario;
int32_t GPS_home_altitude;
int32_t previousfwaltitude=0;
int16_t AIR_speed;
int16_t GPS_speed;
int16_t GPS_ground_course; // Unit degree*10 (MSP_RAW_GPS)
int16_t old_GPS_speed;
int16_t GPS_directionToHome=0;
uint8_t GPS_numSat=0;
uint16_t GPS_dop=0;
uint8_t  GPS_waypoint_step=0;
uint16_t GPS_waypoint_dist=0;
//uint16_t I2CError=0;
//uint16_t cycleTime=0;
uint16_t pMeterSum=0;
uint16_t MwRssi=0;
uint16_t FCRssi=0;
uint32_t GPS_time = 0;
uint16_t WIND_direction = 0;
uint16_t WIND_speed = 0;

#ifndef KISS
#define GPS_CONVERSION_UNIT_TO_KM_H 0.036           // From MWii cm/sec to Km/h
#define GPS_CONVERSION_UNIT_TO_M_H 0.02236932       // (0.036*0.62137)  From MWii cm/sec to mph
// For Trip in slow Timed Service Routine (100ms loop)
#define GPS_CONVERSION_UNIT_TO_FT_100MSEC 0.0032808 // 1/100*3,28084(cm/s -> mt/s -> ft/s)/1000*100    => cm/sec ---> ft/100msec
#define GPS_CONVERSION_UNIT_TO_MT_100MSEC 0.0010    // 1/100(cm/s -> mt/s)/1000*100                    => cm/sec ---> mt/100msec (trip var is float)
#else
#define GPS_CONVERSION_UNIT_TO_KM_H 1               // From KISS Km/h to Km/h
#define GPS_CONVERSION_UNIT_TO_M_H 0.62137119       // Km/h to mph
// For Trip in slow Timed Service Routine (100ms loop)
#define GPS_CONVERSION_UNIT_TO_FT_100MSEC 0.0911344 // 3280,84/3600(1Km/h -> 1ft/s)/1000*100           => Km/h ---> ft/100msec
#define GPS_CONVERSION_UNIT_TO_MT_100MSEC 0.0277778 // 1000/3600(1Km/h -> 1mt/s)/1000*100              => Km/h ---> mt/100msec
#endif // Not KISS

#ifdef HAS_ALARMS
#define ALARM_OK 0
#define ALARM_WARN 1
#define ALARM_ERROR 2
#define ALARM_CRIT 3

uint8_t alarmState = ALARM_OK;
uint8_t alarmMsg[MAX_ALARM_LEN];
#endif

// For decoration
uint8_t SYM_AH_DECORATION_LEFT = 0x10;
uint8_t SYM_AH_DECORATION_RIGHT = 0x10;
uint8_t sym_sidebartopspeed = SYM_BLANK;
uint8_t sym_sidebarbottomspeed = SYM_BLANK;
uint8_t sym_sidebartopalt = SYM_BLANK;
uint8_t sym_sidebarbottomalt = SYM_BLANK;
uint8_t sidebarsdir; // speed
uint8_t sidebaradir; // alt
unsigned long sidebarsMillis = 0;
unsigned long sidebaraMillis = 0;

//For Current Throttle
uint16_t LowT = LOWTHROTTLE;
uint16_t HighT = HIGHTHROTTLE;

// For Time
uint16_t onTime=0;
uint16_t flyTime=1;

// For Heading
const char headGraph[] PROGMEM = {
  0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d,0x1c,0x1d,0x18,0x1d,0x1c,0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d};
static int16_t MwHeading=0;

// For Amperage
float amperage = 0;                // its the real value x10
float amperagesum = 0;
int16_t MWAmperage=0;

// Rssi
int16_t rssi =0;   // uint8_t ?
int16_t oldrssi;   // uint8_t ?
volatile int16_t pwmRSSI = 0;
//int rssiADC=0;
//int rssi_Int=0;


// For Voltage
uint16_t voltage=0;                      // its the value x10
uint16_t vidvoltage=0;                   // its the value x10
uint16_t voltageWarning=0;

// For temprature
int16_t temperature=0;                  // temperature in degrees Centigrade


// For Statistics
uint16_t speedMAX=0;
int16_t altitudeMAX=0;
uint32_t distanceMAX=0;
uint16_t ampMAX=0;
uint32_t trip=0;
float tripSum = 0;
uint16_t flyingTime=0; 
uint16_t voltageMIN=254;
int16_t rssiMIN=100;


// For GPSOSD
#if defined GPSOSD
  #define  LAT  0
  #define  LON  1
  #define  GPS_BAUD BAUDRATE
  uint32_t GPS_home_timer=0;
  int32_t  GPS_coord[2];
  int32_t  GPS_home[4];
//  uint16_t GPS_ground_course = 0;                       
  int16_t  GPS_altitude_home;                            
  int16_t  GPS_altitude_home_2;                            
  uint8_t  GPS_Present = 0;                             
//  uint8_t  GPS_SerialInitialised=5;
  uint8_t  GPS_armedangleset = 0;
  uint8_t  GPS_active=5; 
  uint8_t  GPS_fix_HOME=0;
  const char satnogps_text[] PROGMEM = " NO GPS ";
  uint8_t  GPSOSD_state=0;
#endif

// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_FC_VERSION             3   //out message         FC firmware version
//#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120    //out message         Servo settings
#define MSP_NAV_STATUS           121   //out message	     Returns navigation status

#define MSP_DISPLAYPORT          182

#define MSP_CELLS                130   //out message         FrSky SPort Telemtry

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212    //in message          Servo settings

#define MSP_BIND                 240   //in message          no param

#define MSP_ALARMS               242   //in message          poll for alert text

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
#ifdef KISS
#define MSP_KISS_TELEMTRY        255
#define MSP_KISS_SETTINGS        256
#define MSP_KISS_GPS             257
#endif // KISS

// Betaflight specific

// Cleanflight/Betaflight specific
#define MSP_VOLTAGE_METER_CONFIG 56    //out message         powermeter trig
#define MSP_PID_CONTROLLER       59    //in message          no param
#define MSP_SET_PID_CONTROLLER   60    //out message         sets a given pid controller

// Cleanflight specific
#define MSP_LOOP_TIME            73    //out message         Returns FC cycle time i.e looptime 
#define MSP_SET_LOOP_TIME        74    //in message          Sets FC cycle time i.e looptime parameter

// Baseflight specific
#define MSP_CONFIG               66    //out message         baseflight-specific settings that aren't covered elsewhere
#define MSP_SET_CONFIG           67    //in message          baseflight-specific settings save
// Baseflight FIXEDWING specific
#define MSP_FW_CONFIG            123   //out message         Returns parameters specific to Flying Wing mode
#define MSP_SET_FW_CONFIG        216   //in message          Sets parameters specific to Flying Wing mode

// iNAV specific
#define MSP_RTC                  246    //out message         Gets the RTC clock (returns: secs(i32) millis(u16) - (0,0) if time is not known)
#define MSP_SENSOR_STATUS        151    //out message         Gets the sensor HW status. Bit 0 = overall health

// iNAV MSPV2 specific
#define MSP2_INAV_AIR_SPEED      0x2009    //in message          Returns airspeed

// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1333
// ---------------------------------------------------------------------------------------

// Private MSP for use with the GUI
#define MSP_OSD                  220   //in message          starts epprom send to OSD GUI
// Subcommands
#define OSD_NULL                 0
#define OSD_READ_CMD             1
#define OSD_WRITE_CMD            2
#define OSD_GET_FONT             3
#define OSD_SERIAL_SPEED         4
#define OSD_RESET                5
#define OSD_DEFAULT              6
#define OSD_SENSORS              7
#define OSD_WRITE_CMD_EE         8
#define OSD_READ_CMD_EE          9
#define OSD_INFO                 10
#define OSD_SENSORS2             11

// End private MSP for use with the GUI


const char blank_text[]     PROGMEM = "";
const char nodata_text[]    PROGMEM = "NO DATA";
const char nogps_text[]     PROGMEM = " NO GPS";
const char satlow_text[]    PROGMEM = "LOW SATS";
const char disarmed_text[]  PROGMEM = "DISARMED";
const char armed_text[]     PROGMEM = " ARMED";
const char FAILtext[]       PROGMEM = "FAILSAFE";
const char APRTHtext[]      PROGMEM = "AUTO RTL";
const char APHOLDtext[]     PROGMEM = "AUTO HOLD";
const char APWAYPOINTtext[] PROGMEM = " MISSION";
const char lowvolts_text[]  PROGMEM = "LOW VOLTS";
#if defined DEBUGTEXT
const char debug_text[]     PROGMEM = DEBUGTEXT;
#else
const char debug_text[]     PROGMEM = " ";
#endif
const char satwait_text[]   PROGMEM = "  WAIT";
const char launch_text[]    PROGMEM = " LAUNCH";
const char ready_text[]     PROGMEM = " READY";
const char CRUISE_text[]    PROGMEM = " C";
const char AUTOTRIM_text[]  PROGMEM = "AUTOTRIM";
const char AUTOTUNE_text[]  PROGMEM = "AUTOTUNE";

// For Alarm / Message text
const PROGMEM char * const message_text[] =
{   
  blank_text,      //0
  FAILtext,        //1
  APRTHtext,       //2
  APHOLDtext,      //3
  APWAYPOINTtext,  //4
#ifdef EXTENDEDMODESUPPORT
  launch_text,     //5
  AUTOTRIM_text,   //6
  AUTOTUNE_text,   //7
#endif // EXTENDEDMODESUPPORT 
};

const PROGMEM char * const alarm_text[] =
{   
  blank_text,     //0
  #if defined(GPSOSD)
    ready_text,  //1
    launch_text,     //2
    satwait_text,    //3
  #else
   disarmed_text,  //1
   armed_text,     //2
   nodata_text,    //3
  #endif
  nogps_text,     //4
  satlow_text,    //5
  lowvolts_text,  //6
  debug_text,     //7
};

struct __alarms {
  uint8_t active;
  uint8_t  queue;
  uint8_t  index;
  uint8_t  alarm;
}alarms;

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
const char messageF0[] PROGMEM = "DO NOT POWER OFF";
const char messageF1[] PROGMEM = "SCREEN WILL GO BLANK";
const char messageF2[] PROGMEM = "UPDATE COMPLETE";
#endif

#ifdef SKYTRACK
const char skytracktext0[] PROGMEM = "GS LOW VOLTS";
#endif

// For Intro
#ifdef INTRO_VERSION
const char introtext0[] PROGMEM = INTRO_VERSION;
#else
const char introtext0[] PROGMEM = MWVERS;
#endif
const char introtext1[]  PROGMEM = "MENU:THRT MIDDLE";
const char introtext2[]  PROGMEM = "    +YAW RIGHT";
const char introtext3[]  PROGMEM = "    +PITCH FULL";
const char introtext4[]  PROGMEM = "ID:";
const char introtext5[]  PROGMEM = "SI:";
const char introtext6[]  PROGMEM = "FC:";
const char introtextblank[]  PROGMEM = "";

// Intro
const PROGMEM char * const intro_item[] =
{   
  introtext0,
  introtextblank,
#ifdef INTRO_MENU
  introtext1,
  introtext2,
  introtext3,
#else
  introtextblank,
  introtextblank,
  introtextblank,
#endif
  introtextblank,
#ifdef INTRO_CALLSIGN
  introtext4,
#else
  introtextblank,
#endif
#ifdef INTRO_SIGNALTYPE
  introtext5,
#else
  introtextblank,
#endif
#ifdef INTRO_FC
  introtext6,
#else
  introtextblank,
#endif
};

#ifdef SCREENTEST
const char screentest0[]  PROGMEM = "LCD DISPLAY TEST";
const char screentest1[]  PROGMEM = "PAL  : 0-12 ROWS";
const char screentest2[]  PROGMEM = "NTSC : 0-15 ROWS";
const char screentest3[]  PROGMEM = "BOTH : 0-29 COLS";

const PROGMEM char * const screen_test[] =
{   
  screentest0,
  screentest1,
  screentest2,
  screentest3,
};
#endif

#ifdef AUTOCAM 
const char signaltext0[]  PROGMEM = "NTSC";
const char signaltext1[]  PROGMEM = "PAL";
const char signaltext2[]  PROGMEM = "NOT DETECTED";
const PROGMEM char * const signal_type[] =
{   
  signaltext0,
  signaltext1,
  signaltext2,
};
#elif AUTOCAMFULL // For testing
const char signaltext0[]  PROGMEM = "NTSC";
const char signaltext1[]  PROGMEM = "PAL";
const char signaltext2[]  PROGMEM = "NOT DETECTED-NTSC";
const char signaltext3[]  PROGMEM = "NOT DETECTED-PAL";
const PROGMEM char * const signal_type[] =
{   
  signaltext0,
  signaltext1,
  signaltext2,
  signaltext3,
};
#else
const char signaltext0[]  PROGMEM = "NTSC";
const char signaltext1[]  PROGMEM = "PAL";
const char signaltext2[]  PROGMEM = "";
const PROGMEM char * const signal_type[] =
{   
  signaltext0,
  signaltext1,
  signaltext2,
  signaltext2,
};
#endif

// For Config menu common
const char configMsgON[]   PROGMEM = "ON";
const char configMsgOFF[]  PROGMEM = "OFF";
const char configMsgEXT[]  PROGMEM = "EXIT";
const char configMsgSAVE[] PROGMEM = "SAVE+EXIT";
const char configMsgPGS[]  PROGMEM = "<PAGE>";
const char configMsgMWII[] PROGMEM = "USE FC";
#ifdef KISS
const char configMsgBack[]  PROGMEM = "BACK";
const char configMsgSAVEAndBack[] PROGMEM = "SAVE+BACK";
#endif

// For APSTATUS

// For Config pages
//-----------------------------------------------------------Page0
const char configMsg00[] PROGMEM = "STATS";
const char configMsg01[] PROGMEM = "FLY TIME";
const char configMsg02[] PROGMEM = "TOT DIST";
const char configMsg03[] PROGMEM = "MAX DIST";
const char configMsg04[] PROGMEM = "MAX ALT";
const char configMsg05[] PROGMEM = "MAX SPEED";
const char configMsg06[] PROGMEM = "MAH USED";
const char configMsg07[] PROGMEM = "MAX AMPS";
const char configMsg08[] PROGMEM = "VOLTAGE MIN";
const char configMsg09[] PROGMEM = "RSSI MIN";
//-----------------------------------------------------------Page1
const char configMsg10[] PROGMEM = "PID CONFIG";

#ifndef USE_MSP_PIDNAMES
const char configMsg11[] PROGMEM = "ROLL";
const char configMsg12[] PROGMEM = "PITCH";
const char configMsg13[] PROGMEM = "YAW";
const char configMsg14[] PROGMEM = "ALT";
const char configMsg15[] PROGMEM = "GPS";
const char configMsg16[] PROGMEM = "LEVEL";
const char configMsg17[] PROGMEM = "MAG";
#ifdef MENU_PID_VEL
const char configMsg18[] PROGMEM = "Z_VEL";
#endif
#endif /* !USE_MSP_PIDNAMES */

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
#endif
//-----------------------------------------------------------Page3
const char configMsg30[] PROGMEM = "VOLTAGE";
const char configMsg31[] PROGMEM = "MAIN VOLTS ALARM";
const char configMsg32[] PROGMEM = "ADJUST MAIN";
const char configMsg33[] PROGMEM = "ADJUST VID";
const char configMsg34[] PROGMEM = "CELLS";

//-----------------------------------------------------------Page4
const char configMsg40[] PROGMEM = "RSSI";
const char configMsg41[] PROGMEM = "CAL RSSI";
const char configMsg42[] PROGMEM = "SET RSSI MAX";
const char configMsg43[] PROGMEM = "SET RSSI MIN";

//-----------------------------------------------------------Page5
const char configMsg50[] PROGMEM = "CURRENT";
const char configMsg51[] PROGMEM = "ADJUST AMPS";
const char configMsg52[] PROGMEM = "ADJUST ZERO";
//-----------------------------------------------------------Page6
const char configMsg60[] PROGMEM = "DISPLAY";
const char configMsg61[] PROGMEM = "MAP MODE";
//-----------------------------------------------------------Page7
const char configMsg70[]  PROGMEM = "ADVANCED";
const char configMsg71[]  PROGMEM = "MAG CAL";
const char configMsg72[]  PROGMEM = "THROTTLE PWM";
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
const char configMsg97[] PROGMEM = "TEXT ALARMS";

//-----------------------------------------------------------Page10
const char configMsg100[] PROGMEM = "ADVANCE TUNING";
const char configMsg101[] PROGMEM = "PROFILE";
const char configMsg102[] PROGMEM = "PID CONTROLLER";
const char configMsg103[] PROGMEM = "LOOPTIME";
//-----------------------------------------------------------BF FIXEDWING Page
const char configMsg110[] PROGMEM = "FIXEDWING";
const char configMsg111[] PROGMEM = "MAX CORRECT";
const char configMsg112[] PROGMEM = "RUDDER";
const char configMsg113[] PROGMEM = "MAX CLIMB";
const char configMsg114[] PROGMEM = "MAX DIVE";
const char configMsg115[] PROGMEM = "THR CLIMB";
const char configMsg116[] PROGMEM = "THR CRUISE";
const char configMsg117[] PROGMEM = "THR IDLE";
const char configMsg118[] PROGMEM = "RTL ALT";
//-----------------------------------------------------------SERVO Page
const char configMsg120[] PROGMEM = "SERVOS";
const char configMsg121[] PROGMEM = "S0";
const char configMsg122[] PROGMEM = "S1";
const char configMsg123[] PROGMEM = "S2";
const char configMsg124[] PROGMEM = "S3";
const char configMsg125[] PROGMEM = "S4";
const char configMsg126[] PROGMEM = "S5";
const char configMsg127[] PROGMEM = "S6";
const char configMsg128[] PROGMEM = "S7";
//-----------------------------------------------------------RC TUNING Page 2
const char configMsg130[] PROGMEM = "RC TUNING 2";
const char configMsg131[] PROGMEM = "TPA BREAKPOINT";
const char configMsg132[] PROGMEM = "YAW RC EXPO";
//-----------------------------------------------------------INFO Page
const char configMsg140[] PROGMEM = "ACCESS FC SETTINGS";
const char configMsg141[] PROGMEM = "TX  :THRT MIDDLE";
const char configMsg142[] PROGMEM = "    +YAW LEFT";
const char configMsg143[] PROGMEM = "    +PITCH FULL";
const char configMsg144[] PROGMEM = " ";
const char configMsg145[] PROGMEM = "IF SUPPORTED";
//-----------------------------------------------------------DEBUG Page
const char configMsg150[] PROGMEM = " ";
//-----------------------------------------------------------VTX Page
#ifdef USE_MENU_VTX
const char configMsg160[] PROGMEM = "VTX";
const char configMsg161[] PROGMEM = "POWER";
const char configMsg162[] PROGMEM = "BAND";
const char configMsg163[] PROGMEM = "CHANNEL";
const char configMsg164[] PROGMEM = "";
const char configMsg1610[] PROGMEM = "25";
const char configMsg1611[] PROGMEM = "200";
const char configMsg1612[] PROGMEM = "500";
const char configMsg1620[] PROGMEM = "BOSCAM A";
const char configMsg1621[] PROGMEM = "BOSCAM B";
const char configMsg1622[] PROGMEM = "BOSCAM E";
const char configMsg1623[] PROGMEM = "FATSHARK";
const char configMsg1624[] PROGMEM = "RACE";

  #ifdef VTX_REGION_UNRESTRICTED
  // Menu selections
  const PROGMEM char * const vtxPowerNames[] =
  {   
    configMsg1610,
    configMsg1611,
    configMsg1612
  };
  // Menu selections
  const PROGMEM char * const vtxBandNames[] =
  {   
    configMsg1620,
    configMsg1621,
    configMsg1622,
    configMsg1623,
    configMsg1624
  };
  const PROGMEM char vtxBandLetters[] = "ABEFR";

  #elif defined(VTX_REGION_AUSTRALIA)

  // Menu selections
  const PROGMEM char * const vtxPowerNames[] =
  {   
    configMsg1610
  };
  // Menu selections
  const PROGMEM char * const vtxBandNames[] =
  {   
    configMsg1620,
    configMsg1621,
    configMsg1623,
    configMsg1624
  };
  const PROGMEM char vtxBandLetters[] = "ABFR";
  #endif //VTX_REGION_XXXX

const PROGMEM char * const menu_vtx[] = 
{   
  configMsg161,
  configMsg162,
  configMsg163,
  configMsg164,
};
#endif //MENU_VTX
#ifdef KISS
//-----------------------------------------------------------KiSS Page
const char configMsg170[] PROGMEM = "KISS SETTINGS";
const char configMsg171[] PROGMEM = "PID";
const char configMsg172[] PROGMEM = "RATE";
const char configMsg173[] PROGMEM = "NOTCH FILTER";
const char configMsg174[] PROGMEM = "LPF / YAW FILTER";
const char configMsg175[] PROGMEM = "VTX";

const char configMsg1710[] PROGMEM = "RC";
const char configMsg1711[] PROGMEM = "RATE";
const char configMsg1712[] PROGMEM = "CURVE";

const char configMsg1730[] PROGMEM = "ENABLE";
const char configMsg1731[] PROGMEM = "CENTER";
const char configMsg1732[] PROGMEM = "CUTOFF";
const char configMsg1733[] PROGMEM = "ROLL";
const char configMsg1734[] PROGMEM = "PITCH";

const char configMsg1740[] PROGMEM = "YAW FILTER";
const char configMsg1741[] PROGMEM = "ROLL/PITCH LPF";
const char configMsg1742[] PROGMEM = "YAW LPF";
const char configMsg1743[] PROGMEM = "DTERM LPF";
const char configMsg1744[] PROGMEM = "HIGH";
const char configMsg1745[] PROGMEM = "MED. HIGH";
const char configMsg1746[] PROGMEM = "MEDIUM";
const char configMsg1747[] PROGMEM = "MED. LOW";
const char configMsg1748[] PROGMEM = "LOW";
const char configMsg1749[] PROGMEM = "VERY LOW";


const char configMsg1750[] PROGMEM = "TYPE";
const char configMsg1751[] PROGMEM = "LOW POWER";
const char configMsg1752[] PROGMEM = "MAX POWER";
const char configMsg1753[] PROGMEM = "BAND";
const char configMsg1754[] PROGMEM = "CHANNEL";
const char configMsg1755[] PROGMEM = "--";
const char configMsg1756[] PROGMEM = "DUMMY VTX";
const char configMsg1757[] PROGMEM = "IRC TRAMP HV";
const char configMsg1758[] PROGMEM = "TBS SMART AUDIO";
const char configMsg1759[] PROGMEM = "TBS EVO CROSSFIRE";

const char vtxBandA[] PROGMEM = "A";
const char vtxBandB[] PROGMEM = "B";
const char vtxBandE[] PROGMEM = "E";
const char vtxBandFS[] PROGMEM = "FS";
const char vtxBandRB[] PROGMEM = "RB";
const PROGMEM  char * const vtxBandLetters[] = {
  vtxBandA,
  vtxBandB,
  vtxBandE,
  vtxBandFS,
  vtxBandRB
};


#endif // KISS
//-----------------------------------------------------------MENU END

// BETAFLIGHT RTC setting
const uint8_t monthDays[]=
    {31,28,31,30,31,30,31,31,30,31,30,31}; 
    
// GPS lat/lon display 
const unsigned char compass[] = {'N','S','E','W'};

// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const unsigned char flightUnitAdd[4] ={
  SYM_ON_M,SYM_ON_H, SYM_FLY_M,SYM_FLY_H} ; 

const unsigned char speedUnitAdd[2] ={
  SYM_KMH,SYM_MPH} ; // [0][0] and [0][1] = Km/h   [1][0] and [1][1] = Mph

const unsigned char varioUnitAdd[2] ={
  SYM_MS,SYM_FTS} ; // [0][0] and [0][1] = m/s   [1][0] and [1][1] = ft/s

const unsigned char temperatureUnitAdd[2] ={
  SYM_TEMP_C,SYM_TEMP_F};

const unsigned char UnitsIcon[10]={
  SYM_M,SYM_FT,SYM_M,SYM_FT,SYM_KM,SYM_M,SYM_KM,SYM_M,SYM_TEMP_C,SYM_TEMP_F};
//  SYM_ALTM,SYM_ALTFT,SYM_DISTHOME_M,SYM_DISTHOME_FT,SYM_ALTKM,SYM_ALTMI,SYM_DISTHOME_KM,SYM_DISTHOME_MI};

#define REQ_MSP_IDENT     (1 <<  0)
#define REQ_MSP_STATUS    (1 <<  1)
#define REQ_MSP_RAW_IMU   (1 <<  2)
#define REQ_MSP_RC        (1 <<  3)
#define REQ_MSP_ALTITUDE  (1 <<  4)
#define REQ_MSP_COMP_GPS  (1 <<  5)
#define REQ_MSP_ATTITUDE  (1 <<  6)
#define REQ_MSP_RAW_GPS   (1 <<  7)
#define REQ_MSP_ANALOG    (1 <<  8)
#define REQ_MSP_RC_TUNING (1 <<  9)
#define REQ_MSP_PID       (1 << 10)
#define REQ_MSP_BOX       (1 << 11)
#define REQ_MSP_FONT      (1 << 12)
#define REQ_MSP_DEBUG     (1 << 13)
#define REQ_MSP_CELLS     (1 << 14)
#define REQ_MSP_NAV_STATUS           (1L<<15)
#define REQ_MSP_CONFIG               (1L<<15)
#define REQ_MSP_MISC                 (1L<<16)
#define REQ_MSP_ALARMS               (1L<<17)
#define REQ_MSP_PID_CONTROLLER       (1L<<18)
#define REQ_MSP_LOOP_TIME            (1L<<19) 
#define REQ_MSP_FW_CONFIG            (1L<<20) 
#define REQ_MSP_PIDNAMES             (1L<<21)
#define REQ_MSP_SERVO_CONF           (1L<<22)
#define REQ_MSP_VOLTAGE_METER_CONFIG (1L<<23)
#define REQ_MSP_FC_VERSION           (1L<<24)

#define REQ_MSP_RTC                  (1L<<26)
#define REQ_MSP2_INAV_AIR_SPEED      (1L<<27)
#ifdef KISS
#define REQ_MSP_KISS_TELEMTRY        (1L<<28)
#define REQ_MSP_KISS_SETTINGS        (1L<<29)
#define REQ_MSP_KISS_GPS             (1L<<30)
#endif
// Menu selections


// Menu
//PROGMEM const char *menu_stats_item[] =
const PROGMEM char * const menu_stats_item[] =
{   
  configMsg01,
  configMsg02,
  configMsg03,
  configMsg04,
  configMsg05,
  configMsg06,
  configMsg07,
  configMsg08,
  configMsg09,
};

#ifdef USE_MSP_PIDNAMES

#define PIDNAME_BUFSIZE  8
#ifdef MENU_PID_VEL
#define PIDNAME_NUMITEMS 8
#else
#define PIDNAME_NUMITEMS 7
#endif

char menu_pid[PIDNAME_NUMITEMS][PIDNAME_BUFSIZE];

#else /* USE_MSP_PIDNAMES */
const PROGMEM char * const menu_pid[] = 
{   
  configMsg11,
  configMsg12,
  configMsg13,
  configMsg14,
  configMsg15,
  configMsg16,
  configMsg17,
#ifdef MENU_PID_VEL
  configMsg18,
#endif
};
#endif

const PROGMEM char * const menu_rc[] = 
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

const PROGMEM char * const menu_bat[] = 
{   
  configMsg31,
  configMsg32,
  configMsg33,
  configMsg34,
};

const PROGMEM char * const menu_rssi[] = 
{   
  configMsg41,
  configMsg42,
  configMsg43,
};

const PROGMEM char * const menu_amps[] = 
{   
  configMsg51,
  configMsg52,
};

const PROGMEM char * const menu_display[] = 
{   
  configMsg61,
};

const PROGMEM char * const menu_advanced[] = 
{   
  configMsg71,
  configMsg72,
};

const PROGMEM char * const menu_gps_time[] = 
{   
  configMsg81,
  configMsg82,
  configMsg83,
};

const PROGMEM char * const menu_alarm_item[] = 
{   
  configMsg91,
  configMsg92,
  configMsg93,
  configMsg94,
  configMsg95,
  configMsg96,
  configMsg97,
};

const PROGMEM char * const menu_profile[] = 
{   
  configMsg101,
  configMsg102,
  configMsg103,
};

const PROGMEM char * const menu_fixedwing_bf[] = 
{   
  configMsg111,
  configMsg112,
  configMsg113,
  configMsg114,
  configMsg115,
  configMsg116,
  configMsg117,
  configMsg118,
};

const PROGMEM char * const menu_servo[] = 
{   
  configMsg121,
  configMsg122,
  configMsg123,
  configMsg124,
  configMsg125,
  configMsg126,
  configMsg127,
  configMsg128,
};

const PROGMEM char * const menu_rc_2[] = 
{   
  configMsg131,
  configMsg132,
};

const PROGMEM char * const menu_info[] = 
{   
  configMsg141,
  configMsg142,
  configMsg143,
  configMsg144,
  configMsg145,
};

const PROGMEM char * const menutitle_item[] = 
{   
#ifdef MENU_STAT
  configMsg00,
#endif
#ifdef MENU_KISS
  configMsg170,
#endif
#ifdef MENU_PID
  configMsg10,
#endif
#ifdef MENU_RC
  configMsg20,
#endif
#ifdef MENU_2RC
  configMsg130,
#endif
#ifdef MENU_INFO
  configMsg140,
#endif
#ifdef MENU_SERVO
  configMsg120,
#endif
#ifdef MENU_FIXEDWING
  configMsg110,
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
#ifdef MENU_DEBUG
  configMsg150,
#endif
#ifdef USE_MENU_VTX
  configMsg160,
#endif
};

const PROGMEM char * const menu_on_off[] = 
{   
  configMsgOFF,
  configMsgON,
};

#ifdef KISS
const PROGMEM char * const menu_kiss[] = {
  configMsg171,
  configMsg172,
  configMsg173,
  configMsg174,
  configMsg175
};

const PROGMEM char * const menu_kiss_rates[] = {
  configMsg1710,
  configMsg1711,
  configMsg1712
};

const PROGMEM char * const menu_kiss_notch_filters[] = {
  configMsg1730,
  configMsg1731,
  configMsg1732,
  configMsg1733,
  configMsg1734
};

const PROGMEM char * const menu_kiss_lpf[] = {
  configMsg1740,
  configMsg1741,
  configMsg1742,
  configMsg1743,
  configMsgOFF,
  configMsg1744,
  configMsg1745,
  configMsg1746,
  configMsg1747,
  configMsg1748,
  configMsg1749
};

const PROGMEM char * const menu_kiss_vtx[] = {
  configMsg1750,
  configMsg1751,
  configMsg1752,
  configMsg1753,
  configMsg1754
};

const PROGMEM char * const menu_kiss_vtx_type[] = {
  configMsg1755,
  configMsg1756,
  configMsg1757,
  configMsg1758,
  configMsg1759
};

#endif

#ifdef PROTOCOL_MSP
#ifdef FIXEDWING
const char msp_mode_ACRO[] PROGMEM   = "GYRO"; //Acrobatic: rate control
#else
const char msp_mode_ACRO[] PROGMEM   = "ACRO"; //Acrobatic: rate control
#endif

const char msp_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char msp_mode_HOZN[] PROGMEM   = "HOZN"; //Horizon
const char msp_mode_HOLD[] PROGMEM   = "HOLD"; //3D Hold
const char msp_mode_FAIL[] PROGMEM   = "*FS*"; //Failsafe: auto control
const char msp_mode_WAYP[] PROGMEM   = "WAYP"; //Mission/Waypoint: auto control
const char msp_mode_PASS[] PROGMEM   = "MANU"; //Passthrough
const char msp_mode_RTH[]  PROGMEM   = "RTL "; //Return to Launch: auto control
const char msp_mode_CRUZ[] PROGMEM   = "CRUZ"; //Cruise mode
const char msp_mode_AIR[]  PROGMEM   = "AIR "; //Air mode
const char msp_mode_MSP[]  PROGMEM   = ""; //Unknown MSP mode

#ifdef FIXEDWING
const char msp_mode_SYM_ACRO[] PROGMEM   = {(const char) (SYM_ACROGY),(const char) (SYM_ACRO1),(const char) (0)}; //Acrobatic: rate control
#else
const char msp_mode_SYM_ACRO[] PROGMEM   = {(const char) (SYM_ACRO),(const char) (SYM_ACRO1),(const char) (0)}; //Acrobatic: rate control
#endif
const char msp_mode_SYM_STAB[] PROGMEM   = {(const char) (SYM_STABLE),(const char) (SYM_STABLE1),(const char) (0)}; //Stabilize: hold level position
const char msp_mode_SYM_HOZN[] PROGMEM   = {(const char) (SYM_HORIZON),(const char) (SYM_HORIZON1),(const char) (0)}; //Horizon
const char msp_mode_SYM_HOLD[] PROGMEM   = {(const char) (SYM_GHOLD),(const char) (SYM_GHOLD1),(const char) (0)}; //3D Hold
//const char msp_mode_SYM_FAIL[] PROGMEM   = "*FS*"; //Failsafe: auto control
const char msp_mode_SYM_WAYP[] PROGMEM   = {(const char) (SYM_GMISSION),(const char) (SYM_GMISSION1),(const char) (0)}; //Mission/Waypoint: auto control
const char msp_mode_SYM_PASS[] PROGMEM   = {(const char) (SYM_PASS),(const char) (SYM_PASS1),(const char) (0)}; //Passthrough
const char msp_mode_SYM_RTH[]  PROGMEM   = {(const char) (SYM_GHOME),(const char) (SYM_GHOME1),(const char) (0)}; //Return to Launch: auto control
const char msp_mode_SYM_CRUZ[] PROGMEM   = "CZ"; //Cruise mode
const char msp_mode_SYM_AIR[]  PROGMEM   = {(const char) (SYM_AIR),(const char) (SYM_AIR1),(const char) (0)}; //Air mode
const char msp_mode_SYM_MSP[]  PROGMEM   = {(const char) (0)}; //Unknown MSP mode

const PROGMEM char * const msp_mode_index[] =  
{   
 msp_mode_MSP, 
 msp_mode_PASS, 
 msp_mode_FAIL, 
 msp_mode_RTH,
 msp_mode_CRUZ,
 msp_mode_WAYP, 
 msp_mode_HOLD,
 msp_mode_STAB,
 msp_mode_HOZN,
 msp_mode_AIR,
 msp_mode_ACRO,

 msp_mode_SYM_MSP, 
 msp_mode_SYM_PASS, 
 msp_mode_FAIL, 
 msp_mode_SYM_RTH,
 msp_mode_SYM_CRUZ,
 msp_mode_SYM_WAYP, 
 msp_mode_SYM_HOLD,
 msp_mode_SYM_STAB,
 msp_mode_SYM_HOZN,
 msp_mode_SYM_AIR,
 msp_mode_SYM_ACRO,
 
};
#endif // PROTOCOL_MSP

#ifdef PROTOCOL_MAVLINK

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_MAGIC 50
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_VFR_HUD_MAGIC 20
#define MAVLINK_MSG_ID_VFR_HUD_LEN 20
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_MAGIC 39
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC 244
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22    
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_RC_CHANNELS_MAGIC 118
#define MAVLINK_MSG_ID_RC_CHANNELS_LEN 42    
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_MAGIC 124
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31  
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MAGIC 148
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6
#define MAVLINK_MSG_ID_WIND 168
#define MAVLINK_MSG_ID_WIND_MAGIC 1
#define MAVLINK_MSG_ID_WIND_LEN 12
#define MAVLINK_MSG_ID_MISSION_CURRENT 42
#define MAVLINK_MSG_ID_MISSION_CURRENT_MAGIC 28
#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 2
#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62
#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_MAGIC 183
#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN 26 
#define MAVLINK_MSG_ID_RADIO_STATUS 109
#define MAVLINK_MSG_ID_RADIO_STATUS_MAGIC 185
#define MAVLINK_MSG_ID_RADIO_STATUS_LEN 9 
#define MAVLINK_MSG_ID_RADIO 166
#define MAVLINK_MSG_ID_RADIO_MAGIC 21
#define MAVLINK_MSG_ID_RADIO_LEN 9 
#define MAVLINK_MSG_ID_SCALED_PRESSURE 29
#define MAVLINK_MSG_ID_SCALED_PRESSURE_MAGIC 115
#define MAVLINK_MSG_ID_SCALED_PRESSURE_LEN 14 
#define MAVLINK_MSG_ID_SCALED_PRESSURE2 137
#define MAVLINK_MSG_ID_SCALED_PRESSURE2_MAGIC 14
#define MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN 195 
#define MAVLINK_MSG_ID_BATTERY2 181
#define MAVLINK_MSG_ID_BATTERY2_MAGIC 174
#define MAVLINK_MSG_ID_BATTERY2_LEN 4 
#define MAVLINK_MSG_ID_STATUSTEXT 253
#define MAVLINK_MSG_ID_STATUSTEXT_MAGIC 83
#define MAVLINK_MSG_ID_STATUSTEXT_LEN 51 
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR 132
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR_MAGIC 85
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR_LEN 14 
#define MAVLINK_MSG_ID_RANGEFINDER 173
#define MAVLINK_MSG_ID_RANGEFINDER_MAGIC 83
#define MAVLINK_MSG_ID_RANGEFINDER_LEN 8 
#define MAVLINK_MSG_ID_SYSTEM_TIME 2
#define MAVLINK_MSG_ID_SYSTEM_TIME_MAGIC 137
#define MAVLINK_MSG_ID_SYSTEM_TIME_LEN 12
#define MAVLINK_MESSAGE_INFO_ADSB_VEHICLE 246 
#define MAVLINK_MESSAGE_INFO_ADSB_VEHICLE_MAGIC 184
#define MAVLINK_MESSAGE_INFO_ADSB_VEHICLE_LEN 38
#define MAV_CMD_SET_MESSAGE_INTERVAL 511
#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_LONG_MAGIC 152
#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33

// Mavlink stream requests (APM only?)
#define MAV_STREAMS 7
#define MAV_DATA_STREAM_RAW_SENSORS 1
#define MAV_DATA_STREAM_EXTENDED_STATUS 2
#define MAV_DATA_STREAM_RC_CHANNELS 3
#define MAV_DATA_STREAM_POSITION 6
#define MAV_DATA_STREAM_EXTRA1 10
#define MAV_DATA_STREAM_EXTRA2 11
#define MAV_DATA_STREAM_EXTRA3 12

#define  LAT  0
#define  LON  1

//Common Mavlink:
const char mav_mode_APM[]  PROGMEM   = "APM "; //Unknown APM mode
const char mav_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char mav_mode_ACRO[] PROGMEM   = "ACRO"; //Acrobatic: rate control
const char mav_mode_LOIT[] PROGMEM   = "LOIT"; //Loiter: hold a single location
const char mav_mode_RETL[] PROGMEM   = "RETL"; //Return to Launch: auto control
const char mav_mode_CIRC[] PROGMEM   = "CIRC"; //Circle: auto control
const char mav_mode_ATUN[] PROGMEM   = "ATUN"; //Auto Tune: autotune the vehicle's roll and pitch gains
const char mav_mode_GUID[] PROGMEM   = "GUID"; //Guided: auto control
#ifdef PX4 // within MAVLINK
const char mav_mode_PX4[]  PROGMEM   = "PX4 "; // Unknown PX4 mode
const char mav_mode_MANU[] PROGMEM   = "MANU"; // Manual
const char mav_mode_ALTC[] PROGMEM   = "ALTC"; // Altitude control
const char mav_mode_POSC[] PROGMEM   = "POSC"; // Position control
const char mav_mode_AUTO[] PROGMEM   = "WAYP"; //Auto: auto control
//const char mav_mode_AUTO[] PROGMEM   = "AUTO"; // Auto: auto control
const char mav_mode_TKOF[] PROGMEM   = "TKOF"; // Takeoff
//const char mav_mode_LOIT[] PROGMEM   = "LOIT"; // Loiter: hold a single location
const char mav_mode_MISS[] PROGMEM   = "MISS"; // Mission
//const char mav_mode_RETL[] PROGMEM   = "RETL"; // Return to Launch: auto control
const char mav_mode_LAND[] PROGMEM   = "LAND"; // Landing
//const char mav_mode_ACRO[] PROGMEM   = "ACRO"; // Acrobatic: rate control
const char mav_mode_OFFB[] PROGMEM   = "OFFB"; // Offboard control
//const char mav_mode_STAB[] PROGMEM   = "STAB"; // Stabilize: hold level position
const PROGMEM char * const mav_mode_index[] = 
{   
 mav_mode_MANU, //0
 mav_mode_ALTC,
 mav_mode_POSC,
 mav_mode_TKOF,
 mav_mode_LOIT,
 mav_mode_MISS,
 mav_mode_RETL,
 mav_mode_LAND,
 mav_mode_ACRO,
 mav_mode_OFFB,
 mav_mode_STAB, //10
 mav_mode_PX4,  //11
};
#define MAV_MODE_MAX 11
#elif defined FIXEDWING // APM within MAVLINK. Not PX4
const char mav_mode_MANU[] PROGMEM   = "MANU"; //Manual
const char mav_mode_TRNG[] PROGMEM   = "TRNG"; //Training
const char mav_mode_FBWA[] PROGMEM   = "FBWA"; //Fly-by-wire A
const char mav_mode_FBWB[] PROGMEM   = "FBWB"; //Fly-by-wire B
const char mav_mode_CRUI[] PROGMEM   = "CRUI"; //Cruise
const char mav_mode_INIT[] PROGMEM   = "INIT"; //Init
const char mav_mode_HOLD[] PROGMEM   = "HOLD";
const char mav_mode_LAND[] PROGMEM   = "LAND"; // Landing
const char mav_mode_AUTO[] PROGMEM   = "WAYP"; //Auto: auto control
#ifdef QUADPLANE 
const char mav_mode_QSTB[] PROGMEM   = "QSTB"; // Quadplane - like copter STABILIZE
const char mav_mode_QHOV[] PROGMEM   = "QHOV"; // Quadplane - like copter ALT HOLD
const char mav_mode_QLTR[] PROGMEM   = "QLTR"; // Quadplane - like copter LOITER
const char mav_mode_QLND[] PROGMEM   = "QLND"; // Quadplane - like copter LAND
const char mav_mode_QRTL[] PROGMEM   = "QRTL"; // Quadplane - like copter RTL
const char mav_mode_QACR[] PROGMEM   = "QACR"; // Quadplane - ACRO
#endif //QUADPLANE

const PROGMEM char * const mav_mode_index[] = 
{   
 mav_mode_MANU, //0
 mav_mode_CIRC,
 mav_mode_STAB,
 mav_mode_TRNG,
 mav_mode_ACRO,
 mav_mode_FBWA,
 mav_mode_FBWB,
 mav_mode_CRUI,
 mav_mode_ATUN, 
 mav_mode_APM , 
 mav_mode_AUTO,
 mav_mode_RETL,
 mav_mode_LOIT,
 mav_mode_APM , 
 mav_mode_APM , 
 mav_mode_GUID,
 mav_mode_INIT, //16
#ifdef QUADPLANE
 mav_mode_QSTB, //17
 mav_mode_QHOV, //18  
 mav_mode_QLTR, //19
 mav_mode_QLND, //20
 mav_mode_QRTL, //21
 mav_mode_APM,  //22
 mav_mode_QACR, //23
#endif
 mav_mode_APM , 
};
#ifdef QUADPLANE
#define MAV_MODE_MAX 24
#else // FIXEWDWING
#define MAV_MODE_MAX 17
#endif
#elif defined SUBMARINE // // APM within MAVLINK. Not PX4
const char mav_mode_AHLD[] PROGMEM   = "AHLD"; //Manual angle with automatic depth/throttle
const char mav_mode_AUTO[] PROGMEM   = "AUTO"; //Fully automatic waypoint control using mission commands
const char mav_mode_SURF[] PROGMEM   = "SURF"; //Automatically return to surface, pilot maintains horizontal control
const char mav_mode_PHLD[] PROGMEM   = "PHLD"; //Automatic position hold with manual override, with automatic throttle
const char mav_mode_MANU[] PROGMEM   = "MANU"; //Pass-through input with no stabilization
const char mav_mode_MOTD[] PROGMEM   = "MOTD"; //Automatically detect motors orientation
const PROGMEM char * const mav_mode_index[] = 
{   
 mav_mode_STAB, //0
 mav_mode_ACRO,
 mav_mode_AHLD,
 mav_mode_AUTO,
 mav_mode_GUID,
 mav_mode_APM, //5
 mav_mode_APM,
 mav_mode_CIRC,
 mav_mode_APM,              
 mav_mode_SURF,
 mav_mode_APM, //10
 mav_mode_APM,
 mav_mode_APM, 
 mav_mode_APM,
 mav_mode_APM,
 mav_mode_APM, //15
 mav_mode_PHLD,         
 mav_mode_APM , 
 mav_mode_APM ,
 mav_mode_MANU ,
 mav_mode_MOTD , //20
 mav_mode_APM , //21
};
#define MAV_MODE_MAX 21
#else // APM copter
const char mav_mode_ALTH[] PROGMEM   = "ALTH"; //Altitude Hold: auto control
const char mav_mode_POSH[] PROGMEM   = "POSH"; //Position: auto control
const char mav_mode_LAND[] PROGMEM   = "LAND"; //Land:: auto control
const char mav_mode_OFLO[] PROGMEM   = "OFLO"; //OF_Loiter: hold a single location using optical flow sensor
const char mav_mode_DRIF[] PROGMEM   = "DRIF"; //Drift mode: 
const char mav_mode_SPRT[] PROGMEM   = "SPRT"; //Sport: earth frame rate control
const char mav_mode_FLIP[] PROGMEM   = "FLIP"; //Flip: flip the vehicle on the roll axis
const char mav_mode_HOLD[] PROGMEM   = "HOLD";
const char mav_mode_BRK[] PROGMEM    = "BRK";
const char mav_mode_THRW[] PROGMEM   = "THRW";
const char mav_mode_ADSB[] PROGMEM   = "ADSB";
const char mav_mode_NGPS[] PROGMEM   = "NGPS";
const char mav_mode_SRTL[] PROGMEM   = "SRTL";
const char mav_mode_FLOW[] PROGMEM   = "FLOW";
const char mav_mode_FOLL[] PROGMEM   = "FOLL";
const char mav_mode_AUTO[] PROGMEM   = "WAYP"; //Auto: auto control
const PROGMEM char * const mav_mode_index[] = 
{   
 mav_mode_STAB, //0
 mav_mode_ACRO,
 mav_mode_ALTH,
 mav_mode_AUTO,
 mav_mode_GUID,
 mav_mode_LOIT,
 mav_mode_RETL,
 mav_mode_CIRC,
 mav_mode_POSH,              // changed
 mav_mode_LAND,
 mav_mode_OFLO,
 mav_mode_DRIF,
 mav_mode_APM, 
 mav_mode_SPRT,
 mav_mode_FLIP,
 mav_mode_ATUN,
 mav_mode_HOLD, //16        //changed
 mav_mode_BRK , //17
 mav_mode_THRW ,
 mav_mode_ADSB ,
 mav_mode_NGPS ,
 mav_mode_SRTL ,
 mav_mode_FLOW ,
 mav_mode_FOLL ,
 mav_mode_APM , //24
};
#define MAV_MODE_MAX 24
#endif //FIXEDWING or COPTER

// Vars
struct __mw_mav {
  uint8_t  message_cmd;
  uint8_t  message_length;
  uint8_t  mode;
  uint8_t  sequence;
  uint16_t serial_checksum;
  uint16_t tx_checksum;
  uint16_t throttle;
}mw_mav;

int32_t  GPS_home[2];
uint8_t  GPS_fix_HOME;
int16_t  GPS_altitude_home;  
int32_t  MwAltitude_home;   
#endif //MAVLINK


#ifdef PROTOCOL_LTM

#define LIGHTTELEMETRY_START1 0x24 //$
#define LIGHTTELEMETRY_START2 0x54 //T
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_OFRAME 0x4F  //O OSD additionals data ( home pos, home alt, direction to home )
#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11
#define LIGHTTELEMETRY_OFRAMELENGTH 18
#define  LAT  0
#define  LON  1

int32_t  GPS_home[2];
uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH - 4];


const char ltm_mode_MANU[] PROGMEM   = "MANU"; //Manual
const char ltm_mode_RATE[] PROGMEM   = "RATE"; //Rate
const char ltm_mode_ACRO[] PROGMEM   = "ACRO"; //Acrobatic: rate control
const char ltm_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char ltm_mode_HOZN[] PROGMEM   = "HOZN"; //Horizon
const char ltm_mode_HOLD[] PROGMEM   = "HOLD"; //Hold
const char ltm_mode_HEAD[] PROGMEM   = "HEAD"; //Head
const char ltm_mode_WAYP[] PROGMEM   = "WAYP"; //Waypoint
const char ltm_mode_RTH[]  PROGMEM   = "RTL "; //Return to Launch: auto control
const char ltm_mode_FOLL[] PROGMEM   = "FOLL"; //Follow me
const char ltm_mode_CIRC[] PROGMEM   = "CIRC"; //Circle: auto control
const char ltm_mode_FBWA[] PROGMEM   = "FBWA"; //Fly-by-wire A
const char ltm_mode_FBWB[] PROGMEM   = "FBWB"; //Fly-by-wire B
const char ltm_mode_CRUI[] PROGMEM   = "CRUI"; //Cruise
const char ltm_mode_LAND[] PROGMEM   = "LAND"; //Land
const char ltm_mode_LTM[]  PROGMEM   = "LTM "; //Unknown LTM mode

const PROGMEM char * const ltm_mode_index[] = 
{   
 ltm_mode_MANU, //0
 ltm_mode_RATE,
 ltm_mode_STAB,
 ltm_mode_HOZN,
 ltm_mode_ACRO,
 ltm_mode_STAB,
 ltm_mode_STAB,
 ltm_mode_STAB,
 ltm_mode_HOLD, 
 ltm_mode_HOLD, 
 ltm_mode_WAYP,
 ltm_mode_HEAD,
 ltm_mode_CIRC,
 ltm_mode_RTH , 
 ltm_mode_FOLL, 
 ltm_mode_LAND,
 ltm_mode_FBWA,
 ltm_mode_FBWB, 
 ltm_mode_CRUI, 
 ltm_mode_LTM , 
};

// Vars
struct __mw_ltm {
  uint8_t mode;
  uint8_t LTMreceiverIndex;
  uint8_t LTMcmd;
  uint8_t LTMrcvChecksum;
  uint8_t LTMreadIndex;
  uint8_t LTMframelength;
  uint16_t GPS_altitude_home;
  uint16_t batUsedCapacity;  
}mw_ltm;

#endif // PROTOCOL_LTM

#ifdef PROTOCOL_KISS

const char KISS_mode_ACRO[] PROGMEM   = ""; //Acrobatic: rate control
const char KISS_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char KISS_mode_3D[]   PROGMEM   = "3D"; //Stabilize: hold level position

const PROGMEM char * const KISS_mode_index[] = 
{   
 KISS_mode_ACRO,
 KISS_mode_STAB, 
 KISS_mode_3D, 
};

#define ESC_FILTER 10
#define KISS_GET_TELEMETRY 0x20
#define KISS_GET_GPS 0x54
#define KISS_GET_SETTINGS 0x30
#define KISS_SET_PIDS 0x44
#define KISS_SET_RATES 0x4E
#define KISS_SET_FILTERS 0x48
#define KISS_SET_VTX 0x46

// Indexes of informations in the serial protocol(8 bits)
#define KISS_INDEX_THROTTLE 0 // INT 16
#define KISS_INDEX_RC1 2 // INT 16
#define KISS_INDEX_RC2 4 // INT 16
#define KISS_INDEX_RC3 6 // INT 16
#define KISS_INDEX_RC4 8 // INT 16
#define KISS_INDEX_RC5 10 // INT 16
#define KISS_INDEX_RC6 12 // INT 16
#define KISS_INDEX_RC7 14 // INT 16
#define KISS_INDEX_CURRENT_ARMED 16
#define KISS_INDEX_LIPOVOLT 17 // INT 16
#define KISS_INDEX_ANGLE0 31 // INT 16
#define KISS_INDEX_ANGLE1 33 // INT 16
#define KISS_INDEX_MODE 65
#define KISS_INDEX_ESC1_AMP 87 // INT 16
#define KISS_INDEX_ESC2_AMP 97 // INT 16
#define KISS_INDEX_ESC3_AMP 107 // INT 16
#define KISS_INDEX_ESC4_AMP 117 // INT 16
#define KISS_INDEX_ESC5_AMP 127 // INT 16
#define KISS_INDEX_ESC6_AMP 137 // INT 16
#define KISS_INDEX_MAH 148 // INT 16

#define KISS_INDEX_GPS_LATITUDE 0   // UINT 32
#define KISS_INDEX_GPS_LONGITUDE 4  // UINT 32
#define KISS_INDEX_GPS_SPEED 8      // UINT 16
#define KISS_INDEX_GPS_COURSE 10    // UINT 16
#define KISS_INDEX_GPS_ALTITUDE 12  // INT 16
#define KISS_INDEX_GPS_NUMSATFIX 14 // UINT 8

// Indexes of settings information in serial protocol (8 bits)
// PIDs
#define KISS_SETTINGS_IDX_PID_ROLL_P 0 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_PITCH_P 2 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_YAW_P 4 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_ROLL_I 6 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_PITCH_I 8 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_YAW_I 10 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_ROLL_D 12 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_PITCH_D 14 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_PID_YAW_D 16 // INT 16 (value * 1000)
// Rates
#define KISS_SETTINGS_IDX_RATE_ROLL_RC 28 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_PITCH_RC 30 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_YAW_RC 32 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_ROLL_RATE 34 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_PITCH_RATE 36 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_YAW_RATE 38 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_ROLL_CURVE 40 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_PITCH_CURVE 42 // INT 16 (value * 1000)
#define KISS_SETTINGS_IDX_RATE_YAW_CURVE 44 // INT 16 (value * 1000)
// ----
#define KISS_SETTINGS_IDX_RP_LPF 79 // UINT 8
#define KISS_SETTINGS_IDX_VERSION 92 // UINT 8
#define KISS_SETTINGS_IDX_VTX_N_CHANNEL 120 // UINT 8
// Notch Filters
#define KISS_SETTINGS_IDX_NF_ROLL_ENABLE 138 // INT 8
#define KISS_SETTINGS_IDX_NF_ROLL_CENTER 139 // INT 16
#define KISS_SETTINGS_IDX_NF_ROLL_CUTOFF 141 // INT 16
#define KISS_SETTINGS_IDX_NF_PITCH_ENABLE 143 // INT 8
#define KISS_SETTINGS_IDX_NF_PITCH_CENTER 144 // INT 16
#define KISS_SETTINGS_IDX_NF_PITCH_CUTOFF 146 // INT 16
// ----
#define KISS_SETTINGS_IDX_YAW_C_FILTER 148 // INT 8
#define KISS_SETTINGS_IDX_VTX_TYPE 149 // INT 8
#define KISS_SETTINGS_IDX_VTX_LOW_POWER 150 // INT 16
#define KISS_SETTINGS_IDX_VTX_MAX_POWER 152 // INT 16
#define KISS_SETTINGS_IDX_YAW_LPF 165 // INT 8
#define KISS_SETTINGS_IDX_DTERM_LPF 166 // INT 8

// Indexes of SET_PIDS
#define KISS_SET_PID_IDX_PID_ROLL_P 0 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_ROLL_I 2 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_ROLL_D 4 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_PITCH_P 6 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_PITCH_I 8 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_PITCH_D 10 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_YAW_P 12 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_YAW_I 14 // INT 16 (value * 1000)
#define KISS_SET_PID_IDX_PID_YAW_D 16 // INT 16 (value * 1000)

// Indexes of SET_RATES
#define KISS_SET_RATE_IDX_ROLL_RC 0 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_ROLL_RATE 2 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_ROLL_CURVE 4 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_PITCH_RC 6 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_PITCH_RATE 8 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_PITCH_CURVE 10 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_YAW_RC 12 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_YAW_RATE 14 // INT 16 (value * 1000)
#define KISS_SET_RATE_IDX_YAW_CURVE 16 // INT 16 (value * 1000)

// Indexes of SET_FILTERS
#define KISS_SET_FILTER_IDX_RP_LPF 0 // INT 8
#define KISS_SET_FILTER_IDX_YAW_C_FILTER 1 // INT 8
#define KISS_SET_FILTER_IDX_NF_ROLL_ENABLE 2 // INT 8
#define KISS_SET_FILTER_IDX_NF_ROLL_CENTER 3 // INT 16
#define KISS_SET_FILTER_IDX_NF_ROLL_CUTOFF 5 // INT 16
#define KISS_SET_FILTER_IDX_NF_PITCH_ENABLE 7 // INT 8
#define KISS_SET_FILTER_IDX_NF_PITCH_CENTER 8 // INT 16
#define KISS_SET_FILTER_IDX_NF_PITCH_CUTOFF 10 // INT 16
#define KISS_SET_FILTER_IDX_YAW_LPF 12 // INT 8
#define KISS_SET_FILTER_IDX_DTERM_LPF 13 // INT 8

// Indexes of SET_VTX
#define KISS_SET_VTX_IDX_TYPE 0 // INT 8
#define KISS_SET_VTX_IDX_N_CHANNEL 1 // INT 8
#define KISS_SET_VTX_IDX_LOW_POWER 2 // INT 16
#define KISS_SET_VTX_IDX_MAX_POWER 4 // INT 16

#define KISSFRAMEINIT 5
#define KISSFRAMELENGTH KISS_SETTINGS_IDX_DTERM_LPF + 2 // Size of serial buffer defined with max index used

uint8_t KISSserialBuffer[KISSFRAMELENGTH];
uint8_t KISScurrentRequest = 0x00;
uint8_t KISSgetcmd=0;

#define KISS_MIN_NF_CENTER 0
#define KISS_MAX_NF_CENTER 490
#define KISS_MIN_NF_CUTOFF 0
#define KISS_MAX_NF_CUTOFF 490
#define KISS_MIN_YAW_FILTER 0
#define KISS_MAX_YAW_FILTER 97
#define KISS_MIN_LPF 0
#define KISS_MAX_LPF 6

#define KISS_MIN_VTX_TYPE 0
#define KISS_MAX_VTX_TYPE 4
#define KISS_MIN_VTX_POWER 0
#define KISS_MAX_VTX_POWER 2000
#define KISS_INC_VTX_POWER 25
#define VTX_BAND_COUNT 5
#define VTX_CHANNEL_COUNT 8

int8_t subConfigPage=-1;
#define SUBMENU_KISS_SIZE 5
#define SUBMENU_KISS_PID 0
#define SUBMENU_KISS_RATE 1
#define SUBMENU_KISS_NOTCH_FILTERS 2
#define SUBMENU_KISS_LPF 3
#define SUBMENU_KISS_VTX 4
// Vars
struct __Kvar {
  uint8_t mode;
  uint8_t index;
  uint8_t payloadlength;
  uint8_t readIndex;
  uint8_t framelength;
  uint16_t cksumtmp;
  uint8_t crc8;
  uint8_t version;
}
Kvar;

uint8_t  GPS_fix_HOME=0;
int32_t  GPS_home[2];
#define  LAT  0
#define  LON  1

#endif // PROTOCOL_KISS


#ifdef NAZA
// Vars
struct __Naza {
  uint8_t mode;
}
Naza; 

const char naza_mode_GPSA[] PROGMEM   = "GPSA"; //GPS ATTI
const char naza_mode_ATTI[] PROGMEM   = "ATTI"; //ATTI
const char naza_mode_MANU[] PROGMEM   = "MANU";
const char naza_mode_FAIL[] PROGMEM   = "FAIL";
const PROGMEM char * const NAZA_mode_index[] = 
{   
 naza_mode_FAIL, //0
 naza_mode_MANU,
 naza_mode_ATTI,
 naza_mode_GPSA,
};
#endif // NAZA

// Serial Buffer must be at least 65 for font transfers
#if defined APM
  #define SERIALBUFFERSIZE 75
#elif defined NAZA
  #define SERIALBUFFERSIZE 125
#elif defined SUBMERSIBLE
  #define SERIALBUFFERSIZE 65
#elif defined iNAV // 40 max in test
  #define SERIALBUFFERSIZE 65
#elif defined KISS
  #define SERIALBUFFERSIZE 65
#else
  #define SERIALBUFFERSIZE 100
#endif

static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
