
/*--------------------------------------------------       configurable parameters      ----------------------------------------------------*/
 
#define MAPMODE 1       // 1 for MAP - home is center, map shows location of aircraft relative to home 
                        // 0 for RADAR - home is aircraft, map shows location of home relative to aircraft 
//#define MAPMODENORTH // Enable to use North as MAP reference in MODE 1 instead of take off direction (Default = disable) 

/********************       USE HARDWARE SYNC      *********************/

//#define USE_VSYNC // Remove "sparklies" on boards that support VSYNC 
 
/********************       HARDWARE PINS      *********************/
#define PWMRSSIPIN    A3              
#define RSSIPIN       A3              
#define TEMPPIN       6           
#define VOLTAGEPIN    0
#define VIDVOLTAGEPIN 2
#define AMPERAGEPIN   1
#define LEDPIN     7

/********************       MW 2.0 / 2.1 support      *********************/
//#define BOXNAMES                 // required to support multiwii 2.1 / 2.0 

/**********************       Serial speed      ************************/

//#define SERIAL_SPEED 115200

/**********    Here you can define time out for Mag calibration and EEProm write (mostly useful for mag calibration)    ***********/

#define CALIBRATION_DELAY 10
#define EEPROM_WRITE_DELAY 5

/***************************************         Amperage        ********************************************/
#define AMPERAGE_VIRTUAL_IDLE 4     // Set AMPERAGE_VIRTUAL_IDLE value for motors off current draw * 10. 4 = 0.4A (typical FPV equipment current draw) 

/**********************************         Display Settings         ************************/

#define DECIMAL '.'                 // Decimal point character, change to what suits you best (.) (,)
//#define SHIFTDOWN                   // Select if your monitor cannot display top line fully. It shifts some lines down

/**********************************   MSP Options and compatibility **********************/

#define BOX_OSD_SWITCH              // Comment to use LLIGHT switch instead. ( OSD switch will be default and only option after MW 2.2 release).

/********************       For Sensors presence      *********************/

#define ACCELEROMETER  1//0b00000001
#define BAROMETER      2//0b00000010
#define MAGNETOMETER   4//0b00000100
#define GPSSENSOR      8//0b00001000
//#define SONAR         16//0b00010000

/********************       For OSD SWITCH      *********************/
// In following defines, use DISPLAY_MIN_OFF to hide when using OSD_SWITCH
// Otherwise select DISPLAY_ALWAYS (it can be enabled / disbled by GUI

#define  OSDOFF01 DISPLAY_MIN_OFF // GPS_numSatPosition
#define  OSDOFF02 DISPLAY_MIN_OFF // GPS_numSatPositionTop      // On top of screen
#define  OSDOFF03 DISPLAY_MIN_OFF // GPS_directionToHomePosition
#define  OSDOFF04 DISPLAY_MIN_OFF // GPS_distanceToHomePosition
#define  OSDOFF05 DISPLAY_MIN_OFF // speedPosition
#define  OSDOFF06 DISPLAY_MIN_OFF // GPS_angleToHomePosition
#define  OSDOFF07 DISPLAY_MIN_OFF // MwGPSAltPosition
#define  OSDOFF08 DISPLAY_MIN_OFF // sensorPosition
#define  OSDOFF09 DISPLAY_MIN_OFF // MwHeadingPosition
#define  OSDOFF10 DISPLAY_MIN_OFF // MwHeadingGraphPosition
#define  OSDOFF11 DISPLAY_MIN_OFF // MwAltitudePosition
#define  OSDOFF12 DISPLAY_MIN_OFF // MwClimbRatePosition
#define  OSDOFF13 DISPLAY_MIN_OFF // CurrentThrottlePosition
#define  OSDOFF14 DISPLAY_ALWAYS // flyTimePosition
#define  OSDOFF15 DISPLAY_ALWAYS // onTimePosition
#define  OSDOFF16 DISPLAY_ALWAYS // motorArmedPosition
#define  OSDOFF17 DISPLAY_MIN_OFF  // MwGPSLatPosition
#define  OSDOFF18 DISPLAY_MIN_OFF  // MwGPSLonPosition
#define  OSDOFF19 DISPLAY_MIN_OFF  // MwGPSLatPositionTop      // On top of screen
#define  OSDOFF20 DISPLAY_MIN_OFF  // MwGPSLonPositionTop      // On top of screen
#define  OSDOFF21 DISPLAY_MIN_OFF // rssiPosition
#define  OSDOFF22 DISPLAY_MIN_OFF // temperaturePosition
#define  OSDOFF23 DISPLAY_ALWAYS // voltagePosition
#define  OSDOFF24 DISPLAY_ALWAYS // vidvoltagePosition
#define  OSDOFF25 DISPLAY_MIN_OFF // amperagePosition
#define  OSDOFF26 DISPLAY_MIN_OFF // pMeterSumPosition
#define  OSDOFF27 DISPLAY_MIN_OFF  // horizonPosition
#define  OSDOFF28 DISPLAY_MIN_OFF // CallSign Position
#define  OSDOFF29 DISPLAY_ALWAYS // Debug Position
//#define  OSDOFF30 DISPLAY_MIN_OFF // Gimbal Position
#define  OSDOFF31 DISPLAY_MIN_OFF // GPS_time Position

/*----------------------------------------------       End of configurable parameters      ----------------------------------------------------*/


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

// DEFINE CONFIGURATION MENU PAGES
#define MINPAGE 1
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

/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/

//#define DEBUG   // For developers - remove if memory required
#define HORIZON // For developers - remove horizon if required

//#ifdef MAPMODE 
//  #undef HORIZON
//  #undef DEBUG
//#endif
