/*--------------------------       configurable parameters      ----------------------------------------------------*/

/********************       HARDWARE settings      *********************/
//#define RUSHDUINO                 // Select board type - RUSHDUINO. Defaults to MINIM
//#define USE_VSYNC                 // Remove "sparklies" on boards that support VSYNC 
//#define TEMPSENSOR                // Enable is you have a hardware temperature sensor
#define AMPERAGEMAX     500         // Size of current sensor * 10 e.g. 50A sensor = 500
#define AMPERAGEOFFSET  0           // Optional extra for high offset sensors not supported in GUI (typically bidirectional sensors use a value of 256-512) 
#define STAGE2FILTER                // Optional 2nd stage filter for all ADC and PWM RSSI. Enable for smoother readings 


/********************       CONTROLLER settings      *********************/
//#define BASEFLIGHT                // Undefine this if you are using BASEFLIGHT / others to correct for incorrect heading
//#define FIXEDWING                 // Undefine this if you are using MW fixed wing from PatrickE - to use GPS heading and altitude instead of BARO/MAG


/********************       GPS type settings      *********************/
//#define I2CGPS_SPEED              // Undefine this if you are using older I2CGPS - and need to correct for speed error (10x too slow)               
//#define I2CGPS_DISTANCE           // Undefine this if you are using older I2CGPS - and need to correct for distance error (650m max) UNTESTED               


/********************       MAP MODE Settings       *********************/
#define MAPMODE 1                   // 1 for MAP - home is center, map shows location of aircraft relative to home 
                                    // 0 for RADAR - home is aircraft, map shows location of home relative to aircraft 
//#define MAPMODENORTH              // Enable to use North as MAP reference in MODE 1 instead of take off direction (Default = disable) 


/********************       Display Settings         ************************/
#define DECIMAL '.'                 // Decimal point character, change to what suits you best (.) (,)
//#define SHIFTDOWN                 // Select if your monitor cannot display top line fully. It shifts some lines down
//#define CALLSIGNALWAYS 340        // Enable to permanently display callsign. Number =screen position (row*30 + column)
//#define ALT_CENTER                // Enbale alternative center crosshair
//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status
//#define FASTPIXEL                 // Optional - may improve resolution - especially hi res cams


/********************       Serial speed      ************************/
//#define SERIAL_SPEED 115200


/********************  MSP Options and compatibility **********************/
//#define BOXNAMES                  // required to support multiwii 2.1 / 2.0 


/*--------------------------       advanced parameters      ----------------------------------------------------*/

/********************       HARDWARE PINS settings      *********************/
#define VOLTAGEPIN    A0
#define VIDVOLTAGEPIN A2
#define AMPERAGEPIN   A1
#define TEMPPIN       A6           
#define RSSIPIN       A3              
#define PWMRSSIPIN    A3              
#define LEDPIN        7
#ifdef RUSHDUINO                     //RUSHDUINO
    # define MAX7456SELECT 10        // ss 
    # define MAX7456RESET  9         // RESET
#else                                // MINIM  
    # define MAX7456SELECT 6         // ss
    # define MAX7456RESET  10        // RESET
#endif


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


/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
#define DEBUG         // Enable/disable option to display Mutltiwii debug values or free memory 
#define SBDIRECTION     // Enable/disable sidebar indicators (changes in speed or altitude) or free memory
#define HORIZON         // Enable/disable HORIZON indicator or free memory

