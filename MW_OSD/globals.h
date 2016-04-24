// it would be awesome to get rid of most of these...

#ifndef __GLOBALS_H
#define __GLOBALS_H

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


// TODO: $$$ arduino only?
static uint8_t sensorpinarray[]={VOLTAGEPIN,VIDVOLTAGEPIN,AMPERAGEPIN,TEMPPIN,RSSIPIN};
// TODO: $$$ MSP only?
static uint32_t modeMSPRequests;
static uint32_t queuedMSPRequests;

//General use variables
static struct {
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
}
timer;

static struct {
  uint8_t ident;
  uint8_t box;
  uint8_t reset;
}
flags;

static uint16_t debug[4];   // int32_t ?...
static int8_t menudir;
static unsigned int allSec=0;
static unsigned int menuSec=0;
static uint8_t armedtimer=255;
static uint16_t debugerror;
static uint16_t debugval=0;
static uint16_t cell_data[6]={0,0,0,0,0,0};
static uint16_t cycleTime;
static uint16_t I2CError;
static uint8_t oldROW=0;
static uint8_t cells=0;
#if defined CORRECT_MSP_BF1
static uint8_t bfconfig[25];
#endif


// Config status and cursor location
static uint8_t screenlayout=0;
static uint8_t oldscreenlayout=0;
static uint8_t ROW=10;
static uint8_t COL=3;
static int8_t configPage=1;
static int8_t previousconfigPage=1;
static uint8_t configMode=0;
static uint8_t fontData[54];
static uint8_t nextCharToRequest;
static uint8_t lastCharToRequest;
static uint8_t retransmitQueue;
static uint16_t eeaddress = 0;
static uint8_t eedata = 0;
static uint8_t settingsMode=0;
static uint32_t MSP_OSD_timer=0;

// Mode bits
static struct {
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
} mode;

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


static uint8_t  Settings[EEPROM_SETTINGS];
static uint16_t Settings16[EEPROM16_SETTINGS];

//const uint8_t screenlayoutoffset=((EEPROM_SETTINGS-EEPROM16_SETTINGS_START)>>2);


// For Settings Defaults
static uint8_t EEPROM_DEFAULT[EEPROM_SETTINGS] = {
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

static uint16_t EEPROM16_DEFAULT[EEPROM16_SETTINGS] = {
  0,// S16_AMPMAX,
  0,// S16_AMPZERO,
  150,// S16_AMPDIVIDERRATIO,
  0,// S16_RSSIMIN,
  1024,// S16_RSSIMAX,
  500,// S16_SPARE1,
  600,// S16_SPARE2,

};
static uint16_t SCREENLAYOUT_DEFAULT[EEPROM_SETTINGS] = {

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


static uint16_t SCREENLAYOUT_DEFAULT_OSDSW[EEPROM_SETTINGS] = {

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

static int32_t  MwAltitude=0;                         // This hold barometric value
static int32_t  old_MwAltitude=0;                     // This hold barometric value


static int MwAngle[2]={0,0};           // Those will hold Accelerator Angle
static uint16_t MwRcData[8]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500} ;

// for analogue / PWM sensor filtering
#define SENSORFILTERSIZE 8
#define SENSORTOTAL 5
static int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2];

static uint16_t  MwSensorPresent=0;
static uint32_t  MwSensorActive=0;
static uint8_t MwVersion=0;
static uint8_t MwVBat=0;
static int16_t MwVario=0;
static uint8_t armed=0;
static uint8_t previousarmedstatus=0;  // for statistics after disarming
static uint16_t armedangle=0;           // for capturing direction at arming
static uint32_t GPS_distanceToHome=0;
static uint8_t GPS_fix=0;
static uint8_t GPS_frame_timer=0;
static int32_t GPS_latitude;
static int32_t GPS_longitude;
static int16_t GPS_altitude;
static int16_t GPS_home_altitude;
static int16_t previousfwaltitude=0;
static int16_t interimfwaltitude=0;
static uint16_t GPS_speed;
static int16_t  GPS_ground_course;
static uint16_t old_GPS_speed;
static int16_t GPS_directionToHome=0;
static uint8_t GPS_numSat=0;
static uint8_t GPS_waypoint_step=0;
//static uint16_t I2CError=0;
//static uint16_t cycleTime=0;
static uint16_t pMeterSum=0;
static uint16_t MwRssi=0;
static uint32_t GPS_time = 0;        //local time of coord calc - haydent

#ifdef HAS_ALARMS
#define ALARM_OK 0
#define ALARM_WARN 1
#define ALARM_ERROR 2
#define ALARM_CRIT 3

static uint8_t alarmState = ALARM_OK;
static uint8_t alarmMsg[MAX_ALARM_LEN];
#endif

static uint8_t MvVBatMinCellVoltage=CELL_VOLTS_MIN;
static uint8_t MvVBatMaxCellVoltage=CELL_VOLTS_MAX;
static uint8_t MvVBatWarningCellVoltage=CELL_VOLTS_WARN;

//For Current Throttle
static uint16_t LowT = LOWTHROTTLE;
static uint16_t HighT = HIGHTHROTTLE;

// For Time
static uint16_t onTime=0;
static uint16_t flyTime=0;

// For Heading
static int16_t MwHeading=0;

// For Amperage
static float amperage = 0;                // its the real value x10
static float amperagesum = 0;
static uint16_t MWAmperage=0;

// Rssi
static int16_t rssi =0;   // uint8_t ?
static int16_t oldrssi;   // uint8_t ?
static int16_t pwmRSSI = 0;
//static int rssiADC=0;
//static int rssi_Int=0;


// For Voltage
static uint16_t voltage=0;                      // its the value x10
static uint16_t vidvoltage=0;                   // its the value x10
static uint8_t voltageWarning=0;
static uint8_t vidvoltageWarning=0;

// For temprature
static int16_t temperature=0;                  // temperature in degrees Centigrade


// For Statistics
static uint16_t speedMAX=0;
static int16_t altitudeMAX=0;
static uint32_t distanceMAX=0;
static uint16_t ampMAX=0;
static uint32_t trip=0;
static uint16_t flyingTime=0;


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
#endif

#endif
