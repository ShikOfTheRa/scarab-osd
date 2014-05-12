

#define MPH 1
#define KMH 0

#define METRIC 0
#define IMPERIAL 1

#define hi_speed_cycle  50
#define lo_speed_cycle  100

#define CALIBRATION_DELAY 10       // Calibration timeouts   
#define EEPROM_WRITE_DELAY 5       // Calibration timeouts

// DEFINE CONFIGURATION MENU PAGES
#define MINPAGE 0
#define MAXPAGE 8

#define PIDITEMS 10

// RX CHANEL IN MwRcData table
#define ROLLSTICK        0
#define PITCHSTICK       1
#define YAWSTICK         2
#define THROTTLESTICK    3

// STICK POSITION
#define MAXSTICK         1900
#define MINSTICK         1100
#define MINTROTTLE       1000

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

//General use variables
struct {
uint8_t tenthSec;
uint8_t halfSec;
uint8_t Blink2hz;                          // This is turing on and off at 2hz
uint8_t Blink10hz;                         // This is turing on and off at 10hz
int lastCallSign;                          // Callsign_timer
uint8_t rssiTimer;
uint8_t accCalibrationTimer;
uint8_t magCalibrationTimer;
}
timer;

uint16_t debug[4];   // int32_t ?...
int8_t menudir;
unsigned int allSec=0;
uint8_t armedtimer=255;
uint16_t debugerror;

uint16_t cell_data[6];

// Config status and cursor location
uint8_t ROW=10;
uint8_t COL=3;
int8_t configPage=1;
uint8_t configMode=0;
uint8_t fontMode = 0;
uint8_t fontData[54];
uint8_t nextCharToRequest;
uint8_t lastCharToRequest;
uint8_t retransmitQueue;

// Mode bits
struct {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint8_t baro;
  uint8_t mag;
  uint16_t gpshome;
  uint16_t gpshold;
  uint16_t camstab;
  uint32_t osd_switch;
  uint32_t llights;
;}mode;


// Settings Locations
enum Setting_ {
  S_CHECK_,		// used for check
  S_RSSIMIN,
  S_RSSIMAX,
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
  S_AMPDIVIDERRATIO,
  S_VIDVOLTAGE,
  S_VIDDIVIDERRATIO,
  S_VIDVOLTAGE_VBAT,
  S_DISPLAYTEMPERATURE,
  S_TEMPERATUREMAX,
  S_DISPLAYGPS,
  S_COORDINATES,
  S_GPSCOORDTOP,
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
  S_AMPMIN,
  S_AMPMAXL,
  S_AMPMAXH,
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

uint16_t S16_AMPMAX = 999; // 16 bit eeprom setting of AMPMAX  

uint8_t Settings[EEPROM_SETTINGS];


// For Settings Defaults
uint8_t EEPROM_DEFAULT[EEPROM_SETTINGS] = {
1,   // used for check              0
0,   // S_RSSIMIN                   1
150, // S_RSSIMAX                   2
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
1,   // S_AMPERAGE_VIRTUAL,
150, // S_AMPDIVIDERRATIO,
0,   // S_VIDVOLTAGE                14
200, // S_VIDDIVIDERRATIO           15
0,   // S_VIDVOLTAGE_VBAT           16 
0,   // S_DISPLAYTEMPERATURE        17
255, // S_TEMPERATUREMAX            18
1,   // S_DISPLAYGPS                20
0,   // S_COORDINATES               21
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
0,   // S_ENABLEADC                 34
0,   // S_VREFERENCE,
0,   // S_USE_BOXNAMES              35
1,   // S_MODEICON                  36
0,   // S_DISPLAY_CS,               37
0,   // GPStime                     37a
0,   // GPSTZ +/-                   37b
0,   // GPSTZ                       37c
// 0,   // GPSDS                       37d
0,   // DEBUG                       37e
1,   // SCROLOLING LADDERS          37f
1,   // SHOW GIMBAL ICON            37g
1,   // SHOW VARIO                  37h
1,   // SHOW BAROALT                38h 50
1,   // SHOW COMPASS                39h
0,   // S_HORIZON_ELEVATION         40h
1,   // S_TIMER                     41h
1,   // S_MODESENSOR                42h
1,   // S_SIDEBARTOPS               43h
4,   // S_AMPMIN,
150,  // S_AMPMAXL,
0,   // S_AMPMAXH,
0,   // S_CS0,
0,   // S_CS1,
0,   // S_CS2,
0,   // S_CS3,
0,   // S_CS4,
0,   // S_CS5,
0,   // S_CS6,
0,   // S_CS7,
0,   // S_CS8,
0,   // S_CS9,
};

static uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];

static uint8_t rcRate8,rcExpo8;
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t thrMid8;
static uint8_t thrExpo8;


static uint16_t  MwAccSmooth[3]={0,0,0};       // Those will hold Accelerator data
int32_t  MwAltitude=0;                         // This hold barometric value
int32_t  old_MwAltitude=0;                     // This hold barometric value


int MwAngle[2]={0,0};           // Those will hold Accelerator Angle
static uint16_t MwRcData[8]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500} ;

// for analogue / PWM sensor filtering 
#define SENSORFILTERSIZE 8
#define SENSORTOTAL 5
int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2]; 

uint16_t  MwSensorPresent=0;
uint32_t  MwSensorActive=0;
uint8_t MwVersion=0;
uint8_t MwVBat=0;
int16_t MwVario=0;
uint8_t armed=0;
uint8_t previousarmedstatus=0;  // for statistics after disarming
uint16_t armedangle=0;           // for capturing direction at arming
uint16_t GPS_distanceToHome=0;
uint8_t GPS_fix=0;
int32_t GPS_latitude;
int32_t GPS_longitude;
int16_t GPS_altitude;
uint16_t GPS_speed;
uint16_t old_GPS_speed;
int16_t GPS_directionToHome=0;
uint8_t GPS_numSat=0;
uint16_t I2CError=0;
uint16_t cycleTime=0;
uint16_t pMeterSum=0;
uint16_t MwRssi=0;
uint32_t GPS_time = 0;        //local time of coord calc - haydent

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
uint16_t LowT = 1100;
uint16_t HighT = 1900;

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
uint16_t MWAmperage=0;

// Rssi
uint16_t rssi =0;   // uint8_t ?
uint16_t oldrssi;   // uint8_t ?
//int rssiADC=0;
//int rssi_Int=0;


// For Voltage
uint16_t voltage=0;                      // its the value x10
uint16_t vidvoltage=0;                   // its the value x10

// For temprature
int16_t temperature=0;                  // temperature in degrees Centigrade


// For Statistics
uint16_t speedMAX=0;
int16_t altitudeMAX=0;
int16_t distanceMAX=0;
float trip=0;
uint16_t flyingTime=0; 


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
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_CELLS                121   //out message         FrSky SPort Telemtry

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

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
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
// End private MSP for use with the GUI

const char disarmed_text[] PROGMEM = "DISARMED";
const char armed_text[] PROGMEM = " ARMED";


// For Intro
const char message0[] PROGMEM = "MULTIWII MWOSD - R1.1";
//const char message1[] PROGMEM = "VIDEO SIGNAL NTSC";
//const char message2[] PROGMEM = "VIDEO SIGNAL PAL ";
const char message5[] PROGMEM = "MW VERSION:";
//const char message6[] PROGMEM = "MENU:THRT MIDDLE";
//const char message7[] PROGMEM = "YAW RIGHT";
//const char message8[] PROGMEM = "PITCH FULL";
const char message9[] PROGMEM = "ID:";         // Call Sign on the beggining of the transmission   
//const char message10[] PROGMEM = "TZ UTC:"; //haydent - Time Zone & DST Setting
//const char message11[] PROGMEM = "DST:"; //haydent - Time Zone & DST Setting

// For Config menu common
const char configMsgON[] PROGMEM = "ON";
const char configMsgOFF[] PROGMEM = "OFF";
const char configMsgEXT[] PROGMEM = "EXIT";
const char configMsgSAVE[] PROGMEM = "SAVE+EXIT";
const char configMsgPGS[] PROGMEM = "<PAGE>";
const char configMsgMWII[] PROGMEM = "USE MWII";

// For Config pages
//-----------------------------------------------------------Page0
const char configMsg00[] PROGMEM = "STATISTICS";
const char configMsg01[] PROGMEM = "TOT DISTANCE";
const char configMsg02[] PROGMEM = "MAX DISTANCE";
const char configMsg03[] PROGMEM = "MAX ALTITUDE";
const char configMsg04[] PROGMEM = "MAX SPEED";
const char configMsg05[] PROGMEM = "FLY TIME";
const char configMsg06[] PROGMEM = "MAH USED";
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
const char configMsg22[] PROGMEM = "EXPONENTIAL";
const char configMsg23[] PROGMEM = "ROLL PITCH RATE";
const char configMsg24[] PROGMEM = "YAW RATE";
const char configMsg25[] PROGMEM = "THROTTLE PID ATT";
const char configMsg26[] PROGMEM = "SAVE TO MULTIWII";
//-----------------------------------------------------------Page3
const char configMsg30[] PROGMEM = "VOLTAGE";
const char configMsg31[] PROGMEM = "DISPLAY MAIN VOLTS";
const char configMsg32[] PROGMEM = "ADJUST VOLTS";
const char configMsg33[] PROGMEM = "MAIN VOLTS ALARM";
const char configMsg34[] PROGMEM = "DISPLAY VID VOLTS";
const char configMsg35[] PROGMEM = "CELLS";
//-----------------------------------------------------------Page4
const char configMsg40[] PROGMEM = "RSSI";
//const char configMsg41[] PROGMEM = "ACTUAL RSSI";
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
const char configMsg68[] PROGMEM = "GPS TIME";
const char configMsg69[] PROGMEM = "MAP MODE";
//-----------------------------------------------------------Page7
const char configMsg70[] PROGMEM = "ADVANCED";
const char configMsg71[] PROGMEM = "UNITS";
const char configMsg710[] PROGMEM = "MET";
const char configMsg711[] PROGMEM = "IMP";
const char configMsg72[] PROGMEM = "SIGNAL";
const char configMsg720[] PROGMEM = "NTSC";
const char configMsg721[] PROGMEM = "PAL";
const char configMsg73[] PROGMEM = "VOLT REF";
const char configMsg730[] PROGMEM = "5V";
const char configMsg731[] PROGMEM = "1.1V";
const char configMsg74[] PROGMEM = "DEBUG";
const char configMsg75[] PROGMEM = "MAG CAL";
//const char configMsg76[] PROGMEM = "TOP SHIFT";
//-----------------------------------------------------------Page8
const char configMsg80[] PROGMEM = "GPS TIME";
const char configMsg81[] PROGMEM = "DISPLAY";
const char configMsg82[] PROGMEM = "TZ FORWARD";
const char configMsg83[] PROGMEM = "TZ ADJUST";


// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const unsigned char speedUnitAdd[2] ={
  0xa5,0xa6} ; // [0][0] and [0][1] = Km/h   [1][0] and [1][1] = Mph
const unsigned char temperatureUnitAdd[2] = {
  0x0e,0x0d};

const char MultiWiiLogoL1Add[17] PROGMEM = {
  0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xd5,0};
const char MultiWiiLogoL2Add[17] PROGMEM = {
  0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0};
const char MultiWiiLogoL3Add[17] PROGMEM = {
  0xf0,0xf1,0xf2,0xf3,0xd5,0xd5,0xd5,0xd5,0xd5,0xd5,0xd5,0xd5,0xd5,0xd5,0xd5,0};

const unsigned char MwAltitudeAdd[2]={
  0xa7,0xa8};
const unsigned char MwClimbRateAdd[2]={
  0x9f,0x99};
const unsigned char GPS_distanceToHomeAdd[2]={
  0xbb,0xb9};
const unsigned char MwGPSAltPositionAdd[2]={
  0xa7,0xa8};
const char MWOSDVersionPosition = 65;


// All screen locations defines in ScreenLayout.ino
enum Positions {
  GPS_numSatPosition,
  GPS_numSatPositionTop,
  GPS_directionToHomePosition,
  GPS_directionToHomePositionBottom,
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
  MwGPSLatPosition,
  MwGPSLonPosition,
  MwGPSLatPositionTop,
  MwGPSLonPositionTop,
  rssiPosition,
  temperaturePosition,
  voltagePosition,
  vidvoltagePosition,
  amperagePosition,
  pMeterSumPosition,
  horizonPosition,
  callSignPosition,
  debugPosition,
  gimbalPosition,
  GPS_timePosition
};

#define REQ_MSP_IDENT     (1 <<  0)
#define REQ_MSP_STATUS    (1 <<  1)
#define REQ_MSP_RAW_IMU   (1 <<  2)
#define REQ_MSP_RC        (1 <<  3)
#define REQ_MSP_RAW_GPS   (1 <<  4)
#define REQ_MSP_COMP_GPS  (1 <<  5)
#define REQ_MSP_ATTITUDE  (1 <<  6)
#define REQ_MSP_ALTITUDE  (1 <<  7)
#define REQ_MSP_ANALOG    (1 <<  8)
#define REQ_MSP_RC_TUNING (1 <<  9)
#define REQ_MSP_PID       (1 << 10)
#define REQ_MSP_BOX       (1 << 11)
#define REQ_MSP_FONT      (1 << 12)
#define REQ_MSP_DEBUG     (1 << 13)
#define REQ_MSP_CELLS     (1 << 14)
