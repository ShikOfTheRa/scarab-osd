
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

#define PIDITEMS 10

// STICK POSITION
#define MAXSTICK         1850
#define MINSTICK         1150
//#define MINTROTTLE       1000

// FOR POSITION OF PID CONFIG VALUE
#define ROLLT 93
#define ROLLP 101
#define ROLLI 107
#define ROLLD 113
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

#if defined (DEVELOPMENT)
#define DEBUGDEF 1
#else
#define DEBUGDEF 0   
#endif


#define NAZA_PWM_LOW  1000
#define NAZA_PMW_MED  1400
#define NAZA_PMW_HIGH 1600

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
  uint16_t  loopcount;
  uint16_t  packetcount;
  uint16_t  serialrxrate;
  uint32_t alarms;                            // Alarm length timer
}
timer;

struct __flags {
  uint8_t ident;
  uint8_t box;
  uint8_t reset;
  uint8_t signaltype;
}
flags;

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

#ifdef USE_MENU_SERVO  
#define MAX_SERVOS 8
struct __servo {
    uint16_t settings[5][MAX_SERVOS];
}
servo;
#endif //USE_MENU_SERVO  


uint16_t debug[4];   // int32_t ?...
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
int8_t PulseType = 0; //0 PWM 1 PPM
uint16_t pwmval1=0;
uint16_t pwmval2=0;
uint8_t debugtext=0;
uint8_t MSP_home_set=0;
#if defined CORRECT_MSP_BF1
  uint8_t bfconfig[25];
#endif

// Canvas mode
#ifdef CANVAS_SUPPORT
bool canvasMode = false;
bool canvasFirst = true;
uint32_t lastCanvas = 0;
#define CANVAS_TIMO 2000  // Canvas mode timeout in msec.
#endif

// Config status and cursor location
uint8_t screenlayout=0;
uint8_t oldscreenlayout=0;
uint8_t ROW=10;
uint8_t COL=3;
int8_t configPage;
int8_t configIndex;
int8_t previousConfigIndex=1;
uint8_t configMode=0;
uint8_t fontMode = 0;
uint8_t fontData[54];
uint8_t nextCharToRequest;
uint8_t lastCharToRequest;
uint8_t retransmitQueue;

uint16_t eeaddress = 0;
uint8_t eedata = 0;
uint8_t settingsMode=0;
uint32_t MSP_OSD_timer=0;
uint16_t framerate = 0;
uint16_t packetrate = 0;
uint16_t serialrxrate = 0;

// Mode bits
struct __mode {
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
}mode;

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

// Settings Locations
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
  S_ALARMS_TEXT,
  S_VIDVOLTAGE,
  S_VIDDIVIDERRATIO,
  S_THROTTLE_PWM,
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
  S_VTX_POWER,
  S_VTX_BAND,
  S_VTX_CHANNEL,
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


uint8_t  Settings[EEPROM_SETTINGS];
uint16_t Settings16[EEPROM16_SETTINGS];

//const uint8_t screenlayoutoffset=((EEPROM_SETTINGS-EEPROM16_SETTINGS_START)>>2);


// For Settings Defaults
PROGMEM const uint8_t EEPROM_DEFAULT[EEPROM_SETTINGS] = {
EEPROMVER,   // used for check              0
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
0,   // S_AMPERAGE_VIRTUAL,         13a
1  , // S_ALARMS_TEXT,              13b
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
DEBUGDEF,   // DEBUG                       37e
1,   // SCROLLING LADDERS           37f
1,   // SHOW GIMBAL ICON            37g
1,   // SHOW VARIO                  37h
1,   // SHOW BAROALT                38h 50
1,   // SHOW COMPASS                39h
0,   // S_HORIZON_ELEVATION         40h
1,   // S_TIMER                     41h
1,   // S_MODESENSOR                42h
0,   // S_SIDEBARTOPS               43h
#ifdef VTX_RTC6705
VTX_DEFAULT_POWER,    // S_VTX_POWER
VTX_DEFAULT_BAND,     // S_VTX_BAND
VTX_DEFAULT_CHANNEL,  // S_VTX_CHANNEL
#else
0,   // S_VTX_POWER
0,   // S_VTX_BAND
0,   // S_VTX_CHANNEL
#endif
0,   // S_RCWSWITCH,
5,   // S_RCWSWITCH_CH,
0,   // S_HUDSW0, LOW / NORMAL
1,   // S_HUDSW1, HIGH / OSDSW
0,   // S_HUDSW2, MID
0,   // S_DISTANCE_ALARM,
0,   // S_ALTITUDE_ALARM,
0,   // S_SPEED_ALARM,
30,  // S_FLYTIME_ALARM
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
};

PROGMEM const uint16_t EEPROM16_DEFAULT[EEPROM16_SETTINGS] = {
  0,// S16_AMPMAX,
  0,// S16_AMPZERO,
  150,// S16_AMPDIVIDERRATIO,
  0,// S16_RSSIMIN,
  1024,// S16_RSSIMAX,
  500,// S16_SPARE1,
  600,// S16_SPARE2,
};


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

uint16_t screenPosition[POSITIONS_SETTINGS];

PROGMEM const uint16_t SCREENLAYOUT_DEFAULT[POSITIONS_SETTINGS] = {

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


PROGMEM const uint16_t SCREENLAYOUT_DEFAULT_OSDSW[POSITIONS_SETTINGS] = {

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

static uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];

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
static uint8_t PreviousFCProfile;
static uint8_t CurrentFCProfile;
static uint8_t PIDController;
static uint16_t LoopTime;

int32_t  MwAltitude=0;                         // This hold barometric value
int32_t  old_MwAltitude=0;                     // This hold barometric value


int16_t MwAngle[2]={0,0};           // Those will hold Accelerometer Angle
static uint16_t MwRcData[1+16]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500} ;

// for analogue / PWM sensor filtering 
#define SENSORFILTERSIZE 8
#define SENSORTOTAL 5
#define FHBANDWIDTH 100

int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2]; 

uint16_t  MwSensorPresent=0;
uint32_t  MwSensorActive=0;
uint8_t MwVersion=0;
uint8_t MwVBat=0;
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
int32_t GPS_home_altitude;
int16_t previousfwaltitude=0;
int16_t interimfwaltitude=0;
uint16_t GPS_speed;
int16_t  GPS_ground_course;
uint16_t old_GPS_speed;
int16_t GPS_directionToHome=0;
uint8_t GPS_numSat=0;
uint8_t GPS_waypoint_step=0;
//uint16_t I2CError=0;
//uint16_t cycleTime=0;
uint16_t pMeterSum=0;
uint16_t MwRssi=0;
uint32_t GPS_time = 0;        //local time of coord calc - haydent

#ifdef HAS_ALARMS
#define ALARM_OK 0
#define ALARM_WARN 1
#define ALARM_ERROR 2
#define ALARM_CRIT 3

uint8_t alarmState = ALARM_OK;
uint8_t alarmMsg[MAX_ALARM_LEN];
#endif

uint8_t MvVBatMinCellVoltage=CELL_VOLTS_MIN;
uint8_t MvVBatMaxCellVoltage=CELL_VOLTS_MAX;
uint8_t MvVBatWarningCellVoltage=CELL_VOLTS_WARN;

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
uint16_t flyTime=0;

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
int16_t pwmRSSI = 0;
//int rssiADC=0;
//int rssi_Int=0;


// For Voltage
uint16_t voltage=0;                      // its the value x10
uint16_t vidvoltage=0;                   // its the value x10
uint8_t voltageWarning=0;
uint8_t vidvoltageWarning=0;

// For temprature
int16_t temperature=0;                  // temperature in degrees Centigrade


// For Statistics
uint16_t speedMAX=0;
int16_t altitudeMAX=0;
uint32_t distanceMAX=0;
uint16_t ampMAX=0;
uint32_t trip=0;
uint16_t flyingTime=0; 


// For GPSOSD
#if defined GPSOSD
  #define  LAT  0
  #define  LON  1
  #define  GPS_BAUD BAUDRATE
  uint32_t GPS_home_timer=0;
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
//  uint16_t GPS_ground_course = 0;                       
  int16_t  GPS_altitude_home;                            
  uint8_t  GPS_Present = 0;                             
//  uint8_t  GPS_SerialInitialised=5;
  uint8_t  GPS_armedangleset = 0;
  uint8_t  GPS_active=5; 
  uint8_t  GPS_fix_HOME=0;
  uint16_t GPS_pdop=100;
  const char satnogps_text[] PROGMEM = " NO GPS ";
#endif


// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
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

// Betaflight specific

// Cleanflight/Betaflight specific
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
// End private MSP for use with the GUI


const char blank_text[]     PROGMEM = "";
const char nodata_text[]    PROGMEM = "NO DATA";
const char nogps_text[]     PROGMEM = " NO GPS";
const char satlow_text[]    PROGMEM = "LOW SATS";
const char disarmed_text[]  PROGMEM = "DISARMED";
const char armed_text[]     PROGMEM = " ARMED";
const char APRTHtext[]      PROGMEM = "AUTO RTH";
const char APHOLDtext[]     PROGMEM = "AUTO HOLD";
const char APWAYPOINTtext[] PROGMEM = " MISSION";
const char lowvolts_text[]  PROGMEM = "LOW VOLTS";
const char debug_text[]     PROGMEM = DEBUGTEXT;

// For Alarm / Message text
const PROGMEM char * const message_text[] =
{   
  blank_text,     //0
  APRTHtext,      //1
  APHOLDtext,     //2
  APWAYPOINTtext,
};

const PROGMEM char * const alarm_text[] =
{   
  blank_text,     //0
  disarmed_text,  //1
  armed_text,     //2
  nodata_text,    //3
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
};

#ifdef AUTOCAM 
const char signaltext0[]  PROGMEM = "AUTO-NTSC";
const char signaltext1[]  PROGMEM = "AUTO-PAL";
const char signaltext2[]  PROGMEM = "NOT DETECTED-NTSC";
const char signaltext3[]  PROGMEM = "NOT DETECTED-PAL";
#else
const char signaltext0[]  PROGMEM = "NTSC";
const char signaltext1[]  PROGMEM = "PAL";
const char signaltext2[]  PROGMEM = "";
const char signaltext3[]  PROGMEM = "";
#endif
const PROGMEM char * const signal_type[] =
{   
  signaltext0,
  signaltext1,
  signaltext2,
  signaltext3,
};

// For Config menu common
const char configMsgON[]   PROGMEM = "ON";
const char configMsgOFF[]  PROGMEM = "OFF";
const char configMsgEXT[]  PROGMEM = "EXIT";
const char configMsgSAVE[] PROGMEM = "SAVE+EXIT";
const char configMsgPGS[]  PROGMEM = "<PAGE>";
const char configMsgMWII[] PROGMEM = "USE FC";

// For APSTATUS

// For Config pages

// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const unsigned char speedUnitAdd[2] ={
  SYM_KMH,SYM_MPH} ; // [0][0] and [0][1] = Km/h   [1][0] and [1][1] = Mph

const unsigned char temperatureUnitAdd[2] ={
  SYM_TEMP_C,SYM_TEMP_F};

const unsigned char UnitsIcon[8]={
  SYM_ALTM,SYM_ALTFT,SYM_DISTHOME_M,SYM_DISTHOME_FT,SYM_ALTKM,SYM_ALTMI,SYM_DISTHOME_KM,SYM_DISTHOME_MI};

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
#define REQ_MSP_NAV_STATUS     (1L<<15)
#define REQ_MSP_CONFIG         (1L<<15)
#define REQ_MSP_MISC           (1L<<16)
#define REQ_MSP_ALARMS         (1L<<17)
#define REQ_MSP_PID_CONTROLLER (1L<<18)
#define REQ_MSP_LOOP_TIME      (1L<<19) 
#define REQ_MSP_FW_CONFIG      (1L<<20) 
#define REQ_MSP_PIDNAMES       (1L<<21)
#define REQ_MSP_SERVO_CONF     (1L<<22)

// Menu selections

typedef enum {
#ifdef USE_MENU_STAT
  MENU_STAT = 0,  // STATISTICS
#endif
#ifdef USE_MENU_PID
  MENU_PID,       // PID CONFIG
#endif
#ifdef USE_MENU_RC
  MENU_RC,        // RC TUNING
#endif
#ifdef USE_MENU_RC_2
  MENU_RC_2,      // RC TUNING PAGE 2
#endif
#ifdef USE_MENU_INFO
  MENU_INFO,      // BF CMS guide
#endif
#ifdef USE_MENU_VOLTAGE
  MENU_VOLTAGE,   // VOLTAGE
#endif
#ifdef USE_MENU_RSSI
  MENU_RSSI,      // RSSI
#endif
#ifdef USE_MENU_CURRENT
  MENU_CURRENT,   // CURRENT
#endif
#ifdef USE_MENU_DISPLAY
  MENU_DISPLAY,   // DISPLAY
#endif
#ifdef USE_MENU_ADVANCED
  MENU_ADVANCED,  // ADVANCED
#endif
#ifdef USE_MENU_ALARMS
  MENU_ALARMS,    // ALARMS
#endif
#ifdef USE_MENU_PROFILE
  MENU_PROFILE,   // PROFILE+PID CONTROLLER
#endif
#ifdef USE_MENU_FIXEDWING
  MENU_FIXEDWING, //FIXEDWING adjustments
#endif
#ifdef USE_MENU_SERVO
  MENU_SERVO,    //FIXEDWING adjustments
#endif
#ifdef USE_MENU_GPS_TIME
  MENU_GPS_TIME, //GPS time
#endif
#ifdef USE_MENU_VTX
  MENU_VTX,      // VTX
#endif
  MENU_LAST,
  MAXPAGE = MENU_LAST - 1
} menuId_e;

uint8_t menuConfig[] = {
#ifdef USE_MENU_STAT
    MENU_STAT,
#endif
#ifdef USE_MENU_VTX
    MENU_VTX,
#endif
    MENU_CONFIG
};

// Menu strings


//-----------------------------------------------------------Page0
const char configMsg00[] PROGMEM = "STATISTICS";
const char configMsg01[] PROGMEM = "FLY TIME";
const char configMsg02[] PROGMEM = "TOT DISTANCE";
const char configMsg03[] PROGMEM = "MAX DISTANCE";
const char configMsg04[] PROGMEM = "MAX ALTITUDE";
const char configMsg05[] PROGMEM = "MAX SPEED";
const char configMsg06[] PROGMEM = "MAH USED";
const char configMsg07[] PROGMEM = "MAX AMPS";

const PROGMEM char * const menu_stats_item[] =
{   
  configMsg01,
  configMsg02,
  configMsg03,
  configMsg04,
  configMsg05,
  configMsg06,
  configMsg07,
};

//-----------------------------------------------------------Page1
#ifdef USE_MENU_PID
const char configMsg10[] PROGMEM = "PID CONFIG";

#ifndef USE_MSP_PIDNAMES
const char configMsg11[] PROGMEM = "ROLL";
const char configMsg12[] PROGMEM = "PITCH";
const char configMsg13[] PROGMEM = "YAW";
const char configMsg14[] PROGMEM = "ALT";
const char configMsg15[] PROGMEM = "GPS";
const char configMsg16[] PROGMEM = "LEVEL";
const char configMsg17[] PROGMEM = "MAG";
# ifdef MENU_PID_VEL
const char configMsg18[] PROGMEM = "Z_VEL";
# endif
#endif /* !USE_MSP_PIDNAMES */

#ifdef USE_MSP_PIDNAMES

# define PIDNAME_BUFSIZE  8
# ifdef MENU_PID_VEL
#  define PIDNAME_NUMITEMS 8
# else
#  define PIDNAME_NUMITEMS 7
# endif

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
# ifdef MENU_PID_VEL
  configMsg18,
# endif
};
#endif // USE_MSP_PIDNAMES

#endif // USE_MENU_PID

//-----------------------------------------------------------Page2
#ifdef USE_MENU_RC
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
#endif

//-----------------------------------------------------------Page3
#ifdef USE_MENU_VOLTAGE
const char configMsg30[] PROGMEM = "VOLTAGE";
const char configMsg31[] PROGMEM = "DISPLAY MAIN VOLTS";
const char configMsg32[] PROGMEM = "ADJUST VOLTS";
const char configMsg33[] PROGMEM = "MAIN VOLTS ALARM";
const char configMsg34[] PROGMEM = "DISPLAY VID VOLTS";
const char configMsg35[] PROGMEM = "ADJUST VOLTS";
const char configMsg36[] PROGMEM = "CELLS";
const char configMsg37[] PROGMEM = "USE FC";

const PROGMEM char * const menu_bat[] = 
{   
  configMsg31,
  configMsg32,
  configMsg33,
  configMsg34,
  configMsg32,
  configMsg36,
  configMsgMWII,
};
#endif

//-----------------------------------------------------------Page4
#ifdef USE_MENU_RSSI
const char configMsg40[] PROGMEM = "RSSI";
const char configMsg42[] PROGMEM = "DISPLAY RSSI";
const char configMsg43[] PROGMEM = "SET RSSI";
const char configMsg44[] PROGMEM = "SET RSSI MAX";
const char configMsg45[] PROGMEM = "SET RSSI MIN";
const char configMsg46[] PROGMEM = "USE PWM";

const PROGMEM char * const menu_rssi[] = 
{   
  configMsg42,
  configMsg43,
  configMsgMWII,
  configMsg46,
  configMsg44,
  configMsg45,
};
#endif

//-----------------------------------------------------------Page5
#ifdef USE_MENU_CURRENT
const char configMsg50[] PROGMEM = "CURRENT";
const char configMsg51[] PROGMEM = "DISPLAY AMPS";
const char configMsg52[] PROGMEM = "DISPLAY MAH";
const char configMsg53[] PROGMEM = "USE VIRTUAL SENSOR";
const char configMsg54[] PROGMEM = "ADJUST AMPS";
const char configMsg55[] PROGMEM = "ADJUST ZERO";

const PROGMEM char * const menu_amps[] = 
{   
  configMsg51,
  configMsg52,
  configMsg53,
  configMsg54,
  configMsg55,
};
#endif

//-----------------------------------------------------------Page6
#ifdef USE_MENU_DISPLAY
const char configMsg60[] PROGMEM = "DISPLAY";
const char configMsg61[] PROGMEM = "HORIZON";
const char configMsg62[] PROGMEM = "SIDE BARS";
const char configMsg63[] PROGMEM = "SCROLLING BARS";
const char configMsg64[] PROGMEM = "THROTTLE";
const char configMsg65[] PROGMEM = "GPS COORDS";
const char configMsg66[] PROGMEM = "SENSORS";
const char configMsg67[] PROGMEM = "GIMBAL";
const char configMsg68[] PROGMEM = "MAP MODE";

const PROGMEM char * const menu_display[] = 
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
#endif

//-----------------------------------------------------------Page7
#ifdef USE_MENU_ADVANCED
const char configMsg70[]  PROGMEM = "ADVANCED";
const char configMsg71[]  PROGMEM = "UNITS";
const char configMsg710[] PROGMEM = "MET";
const char configMsg711[] PROGMEM = "IMP";
const char configMsg73[]  PROGMEM = "V REF";
const char configMsg730[] PROGMEM = "5V";
const char configMsg731[] PROGMEM = "1.1V";
const char configMsg74[]  PROGMEM = "DEBUG";
const char configMsg75[]  PROGMEM = "MAG CAL";
const char configMsg76[]  PROGMEM = "OSD TX CH";
const char configMsg77[]  PROGMEM = "THROTTLE PWM";

const PROGMEM char * const menu_choice_unit[] =
{   
  configMsg710,
  configMsg711,
};

const PROGMEM char * const menu_choice_ref[] =
{   
  configMsg731,
  configMsg730,
};

const PROGMEM char * const menu_advanced[] = 
{   
  configMsg71,
  configMsg73,
  configMsg74,
  configMsg75,
  configMsg76,
  configMsg77,
};
#endif

//-----------------------------------------------------------Page8
#ifdef USE_MENU_GPS_TIME
const char configMsg80[] PROGMEM = "GPS TIME";
const char configMsg81[] PROGMEM = "DISPLAY";
const char configMsg82[] PROGMEM = "TZ FORWARD";
const char configMsg83[] PROGMEM = "TZ ADJUST";
const PROGMEM char * const menu_gps_time[] = 
{   
  configMsg81,
  configMsg82,
  configMsg83,
};
#endif

//-----------------------------------------------------------Page9
#ifdef USE_MENU_ALARMS
const char configMsg90[] PROGMEM = "ALARMS";
const char configMsg91[] PROGMEM = "DISTANCE X100";
const char configMsg92[] PROGMEM = "ALTITUDE X10";
const char configMsg93[] PROGMEM = "SPEED";
const char configMsg94[] PROGMEM = "TIMER";
const char configMsg95[] PROGMEM = "MAH X100";
const char configMsg96[] PROGMEM = "AMPS";
const char configMsg97[] PROGMEM = "TEXT ALARMS";

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
#endif

//-----------------------------------------------------------Page10
#ifdef USE_MENU_PROFILE
const char configMsg100[] PROGMEM = "ADVANCE TUNING";
const char configMsg101[] PROGMEM = "PROFILE";
const char configMsg102[] PROGMEM = "PID CONTROLLER";
const char configMsg103[] PROGMEM = "LOOPTIME";

const PROGMEM char * const menu_profile[] = 
{   
  configMsg101,
  configMsg102,
  configMsg103,
};
#endif

//-----------------------------------------------------------BF FIXEDWING Page
#ifdef USE_MENU_FIXEDWING
const char configMsg110[] PROGMEM = "FIXEDWING";
const char configMsg111[] PROGMEM = "MAX CORRECT";
const char configMsg112[] PROGMEM = "RUDDER";
const char configMsg113[] PROGMEM = "MAX CLIMB";
const char configMsg114[] PROGMEM = "MAX DIVE";
const char configMsg115[] PROGMEM = "THR CLIMB";
const char configMsg116[] PROGMEM = "THR CRUISE";
const char configMsg117[] PROGMEM = "THR IDLE";
const char configMsg118[] PROGMEM = "RTH ALT";

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
#endif

//-----------------------------------------------------------SERVO Page
#ifdef USE_MENU_SERVO
const char configMsg120[] PROGMEM = "SERVOS";
const char configMsg121[] PROGMEM = "S0";
const char configMsg122[] PROGMEM = "S1";
const char configMsg123[] PROGMEM = "S2";
const char configMsg124[] PROGMEM = "S3";
const char configMsg125[] PROGMEM = "S4";
const char configMsg126[] PROGMEM = "S5";
const char configMsg127[] PROGMEM = "S6";
const char configMsg128[] PROGMEM = "S7";
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
#endif

//-----------------------------------------------------------RC TUNING Page 2
#ifdef USE_MENU_RC_2
const char configMsg130[] PROGMEM = "RC TUNING 2";
const char configMsg131[] PROGMEM = "TPA BREAKPOINT";
const char configMsg132[] PROGMEM = "YAW RC EXPO";
const PROGMEM char * const menu_rc_2[] = 
{   
  configMsg131,
  configMsg132,
};
#endif

//-----------------------------------------------------------INFO Page
#ifdef USE_MENU_INFO
const char configMsg140[] PROGMEM = "FC-SIDE MENU INVOCATION";
const char configMsg141[] PROGMEM = "TX  :THRT MIDDLE";
const char configMsg142[] PROGMEM = "    +YAW LEFT";
const char configMsg143[] PROGMEM = "    +PITCH FULL";
const char configMsg144[] PROGMEM = " ";
const char configMsg145[] PROGMEM = "BF3.1 AND LATER ONLY";
const PROGMEM char * const menu_info[] = 
{   
  configMsg141,
  configMsg142,
  configMsg143,
  configMsg144,
  configMsg145,
};
#endif

//-----------------------------------------------------------VTX Page
#ifdef USE_MENU_VTX
const char configMsg150[] PROGMEM = "VTX";
const char configMsg151[] PROGMEM = "POWER";
const char configMsg152[] PROGMEM = "BAND";
const char configMsg153[] PROGMEM = "CHANNEL";
const char configMsg154[] PROGMEM = "";
const char configMsg1510[] PROGMEM = "25";
const char configMsg1511[] PROGMEM = "200";
const char configMsg1512[] PROGMEM = "500";
const char configMsg1520[] PROGMEM = "BOSCAM A";
const char configMsg1521[] PROGMEM = "BOSCAM B";
const char configMsg1522[] PROGMEM = "BOSCAM E";
const char configMsg1523[] PROGMEM = "FATSHARK";
const char configMsg1524[] PROGMEM = "RACE";

  #ifdef VTX_REGION_UNRESTRICTED
  // Menu selections
  const PROGMEM char * const vtxPowerNames[] =
  {   
    configMsg1510,
    configMsg1511,
    configMsg1512
  };
  // Menu selections
  const PROGMEM char * const vtxBandNames[] =
  {   
    configMsg1520,
    configMsg1521,
    configMsg1522,
    configMsg1523,
    configMsg1524
  };
  const PROGMEM char vtxBandLetters[] = "ABEFR";

  #elif defined(VTX_REGION_AUSTRALIA)

  // Menu selections
  const PROGMEM char * const vtxPowerNames[] =
  {   
    configMsg1510
  };
  // Menu selections
  const PROGMEM char * const vtxBandNames[] =
  {   
    configMsg1520,
    configMsg1521,
    configMsg1523,
    configMsg1524
  };
  const PROGMEM char vtxBandLetters[] = "ABFR";
  #endif //VTX_REGION_XXXX

const PROGMEM char * const menu_vtx[] = 
{   
  configMsg151,
  configMsg152,
  configMsg153,
  configMsg154,
};
#endif //MENU_VTX

const PROGMEM char * const menutitle_item[] = 
{   
#ifdef USE_MENU_STAT
  configMsg00,
#endif
#ifdef USE_MENU_PID
  configMsg10,
#endif
#ifdef USE_MENU_RC
  configMsg20,
#endif
#ifdef USE_MENU_RC_2
  configMsg130,
#endif
#ifdef USE_MENU_INFO
  configMsg140,
#endif
#ifdef USE_MENU_SERVO
  configMsg120,
#endif
#ifdef USE_MENU_FIXEDWING
  configMsg110,
#endif
#ifdef USE_MENU_VOLTAGE
  configMsg30,
#endif
#ifdef USE_MENU_RSSI
  configMsg40,
#endif
#ifdef USE_MENU_CURRENT
  configMsg50,
#endif
#ifdef USE_MENU_DISPLAY
  configMsg60,
#endif
#ifdef USE_MENU_ADVANCED
  configMsg70,
#endif
#ifdef USE_MENU_GPS_TIME
  configMsg80,
#endif
#ifdef USE_MENU_ALARMS
  configMsg90,
#endif
#ifdef USE_MENU_PROFILE
  configMsg100,
#endif
#ifdef USE_MENU_VTX
  configMsg150,
#endif
};

const PROGMEM char * const menu_on_off[] = 
{   
  configMsgOFF,
  configMsgON,
};

#ifdef MODETEXTMODE

//Ordering to be resolved - for priority of confilicting modes
const char msp_mode_ACRO[] PROGMEM   = "ACRO"; //Acrobatic: rate control
const char msp_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char msp_mode_HOZN[] PROGMEM   = "HOZN"; //Horizon
const char msp_mode_HOLD[] PROGMEM   = "HOLD"; //3D Hold
const char msp_mode_2D[]   PROGMEM   = "2D  "; //2D Hold No alt hold
const char msp_mode_HEAD[] PROGMEM   = "HEAD"; //Head
const char msp_mode_FAIL[] PROGMEM   = "FAIL"; //Failsafe: auto control
const char msp_mode_WAYP[] PROGMEM   = "WAYP"; //Mission/Waypoint: auto control
const char msp_mode_PASS[] PROGMEM   = "PASS"; //Passthrough
const char msp_mode_RTH[]  PROGMEM   = "RTH "; //Return to Launch: auto control
const char msp_mode_LNCH[] PROGMEM   = "LNCH"; //Launch mode - or gone to lunch :)
const char msp_mode_AIR[]  PROGMEM   = "AIR "; //Air mode
const char msp_mode_MSP[]  PROGMEM   = "****"; //Unknown MSP mode

const PROGMEM char * const msp_mode_index[] = 
{   
 msp_mode_ACRO, //0
 msp_mode_STAB,
 msp_mode_HOZN,
 msp_mode_BARO,
 msp_mode_MAG,
 msp_mode_RTH,
 msp_mode_HOLD,
 msp_mode_WAYP, 
 msp_mode_CAM, 
 msp_mode_AIR,
 msp_mode_LNCH, 
 msp_mode_2D, 
 msp_mode_FAIL, 
 msp_mode_PASS, 
 msp_mode_MSP, 
};
#endif

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
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_MAGIC 124
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31  
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MAGIC 148
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6

#define MAV_DATA_STREAM_RAW_SENSORS 1
#define MAV_DATA_STREAM_EXTENDED_STATUS 2
#define MAV_DATA_STREAM_RC_CHANNELS 3
#define MAV_DATA_STREAM_POSITION 6
#define MAV_DATA_STREAM_EXTRA1 10
#define MAV_DATA_STREAM_EXTRA2 11
#define  LAT  0
#define  LON  1

const char mav_mode_APM[] PROGMEM    = "APM "; //Unknown APM mode
const char mav_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char mav_mode_ACRO[] PROGMEM   = "ACRO"; //Acrobatic: rate control
const char mav_mode_AUTO[] PROGMEM   = "AUTO"; //Auto: auto control
const char mav_mode_LOIT[] PROGMEM   = "LOIT"; //Loiter: hold a single location
const char mav_mode_RETL[] PROGMEM   = "RETL"; //Return to Launch: auto control
const char mav_mode_CIRC[] PROGMEM   = "CIRC"; //Circle: auto control
const char mav_mode_ATUN[] PROGMEM   = "ATUN"; //Auto Tune: autotune the vehicle's roll and pitch gains
const char mav_mode_GUID[] PROGMEM   = "GUID"; //Guided: auto control
#ifdef FIXEDWING // within MAVLINK
const char mav_mode_MANU[] PROGMEM   = "MANU"; //Manual
const char mav_mode_TRNG[] PROGMEM   = "TRNG"; //Training
const char mav_mode_FBWA[] PROGMEM   = "FBWA"; //Fly-by-wire A
const char mav_mode_FBWB[] PROGMEM   = "FBWB"; //Fly-by-wire B
const char mav_mode_CRUI[] PROGMEM   = "CRUI"; //Cruise
const char mav_mode_INIT[] PROGMEM   = "INIT"; //Init
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
 mav_mode_APM , //17
};
#define MAV_MODE_MAX 17
#else
const char mav_mode_ALTH[] PROGMEM   = "ALTH"; //Altitude Hold: auto control
const char mav_mode_POSI[] PROGMEM   = "POSI"; //Position: auto control
const char mav_mode_LAND[] PROGMEM   = "LAND"; //Land:: auto control
const char mav_mode_OFLO[] PROGMEM   = "OFLO"; //OF_Loiter: hold a single location using optical flow sensor
const char mav_mode_DRIF[] PROGMEM   = "DRIF"; //Drift mode: 
const char mav_mode_SPRT[] PROGMEM   = "SPRT"; //Sport: earth frame rate control
const char mav_mode_FLIP[] PROGMEM   = "FLIP"; //Flip: flip the vehicle on the roll axis
const char mav_mode_HYBR[] PROGMEM   = "HYBR"; //Hybrid: position hold with manual override
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
 mav_mode_POSI,
 mav_mode_LAND,
 mav_mode_OFLO,
 mav_mode_DRIF,
 mav_mode_APM, 
 mav_mode_SPRT,
 mav_mode_FLIP,
 mav_mode_ATUN,
 mav_mode_HYBR, //16
 mav_mode_APM , //17
};
#define MAV_MODE_MAX 17
#endif //FIXEDWING

// Vars
struct __mw_mav {
  uint8_t  message_cmd;
  uint8_t  message_length;
  uint8_t  mode;
  uint8_t  sequence;
  uint16_t serial_checksum;
  uint16_t tx_checksum;
  float    GPS_scaleLonDown;
}mw_mav;

int32_t  GPS_home[2];
uint8_t  GPS_fix_HOME;
int16_t  GPS_altitude_home;                            
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
const char ltm_mode_RTH[]  PROGMEM   = "RTH "; //Return to Launch: auto control
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
  float    GPS_scaleLonDown;  
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
#define KISSFRAMEINIT 5
#define KISSFRAMELENGTH 165 // max frame size
uint8_t KISSserialBuffer[KISSFRAMELENGTH];

// Vars
struct __Kvar {
  uint8_t mode;
  uint8_t index;
  uint8_t payloadlength;
  uint8_t readIndex;
  uint8_t framelength;
  uint16_t cksumtmp;
}
Kvar;

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


