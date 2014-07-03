/*--------------------------       configurable parameters      ----------------------------------------------------*/



/********************       FEATURES      *********************/
// Currently there is not enough memory to run all options
// Disable features if you require memory for other features
// Further configuration may be require elsewhere in config.h + option enabled on GUI
#define SBDIRECTION     // Enable/disable sidebar indicators (changes in speed or altitude)
#define HORIZON         // Enable/disable HORIZON indicator
#define MAPMODE         // Enable/disable MAP MODE - map indication of relative positions of home and aircraft
#define GPSTIME         // Enable/disable GPS Time functions
#define SPORT           // Enable/disable FRSKY S.PORT cell code


/********************       HARDWARE OSD settings      *********************/
//#define USE_VSYNC                 // Remove "sparklies" on boards that support VSYNC 


/********************       HARDWARE CURRENT sensor settings      *********************/
#define AMPERAGEMAX     500         // Size of current sensor / maximum current draw (* 10) e.g. 50A sensor = 500, 100A sensor = 1000
#define AMPERAGEOFFSET  0           // Optional extra for high offset sensors not supported in GUI (typically bidirectional sensors use a value of 256-512) 


/********************       FILTER settings      *********************/
//#define STAGE2FILTER              // Enable for smoother readings of voltage / current / RSSI. 
//#define SMOOTHFILTER              // Enable for smoothest readings of voltage / current / RSSI. Uses more memory. NOT FINISHED


/********************       CONTROLLER settings      *********************/
//#define BASEFLIGHT                // Undefine this if you are using BASEFLIGHT / others to correct for incorrect heading
//#define FIXEDWING                 // Undefine this if you are using MW fixed wing from PatrikE - to use GPS heading and altitude instead of BARO/MAG
//#define FASTPWMRSSI               // Undefine this if you are using non standard PWM for RSSI ( doubles resolution ) 


/********************       CALLSIGN settings      *********************/
//#define CALLSIGNSTARTUP           // Enable to display callsign at startup
//#define CALLSIGNALWAYS  341       // Enable to permanently display callsign. Number = screen position (row*30 + column)
//#define FREETEXTLLIGHTS 342       // Enable to display freetext (or callsign) when LLIGHTS Switch active on TX. Number = screen position (row*30 + column)
//#define FREETEXTGIMBAL  342       // Enable to display freetext (or callsign) when GIMBAL Switch active on TX. Number = screen position (row*30 + column)


/********************       GPS type settings      *********************/
//#define I2CGPS_SPEED              // Undefine this if you are using older I2CGPS - and need to correct for speed error (10x too slow)               
//#define I2CGPS_DISTANCE           // Undefine this if you are using older I2CGPS - and need to correct for distance error (650m max) UNTESTED               


/********************       MAP MODE Settings       *********************/
#define MAPTYPE 1                   // 1 for MAP - home is center, map shows location of aircraft relative to home 
                                    // 0 for RADAR - home is aircraft, map shows location of home relative to aircraft 
//#define MAPMODENORTH              // Enable to use North as MAP reference in MODE 1 instead of take off direction (Default = disable) 


/********************       FrSky S.Port settings      *********************/
//enables data transfer from frsky reciever s.port to osd via multiwii
//requires serial inverter cable & multiwii with s.port code
//Auto detected cell graph from s.port, 16 steps @ 0.05v 
//To show battery voltage from s.port, enable "Use MWii" under "Main Voltage" in GUI
//To show amperage from s.port, enable "Use MWii" under Amperage in GUI
//more details: http://code.google.com/p/scarab-osd/wiki/Frsky_SPort
#define MIN_CELL 320 //Cell Low Flash - No decimal, 3 Digits ie 320 = 3.20v


/********************       Display Settings         ************************/
#define DECIMAL '.'                 // Decimal point character, change to what suits you best (.) (,)
//#define SHIFTDOWN                 // Select if your monitor cannot display top line fully. It shifts some lines down
//#define ALT_CENTER                // Enable alternative center crosshair
//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status
//#define FASTPIXEL                 // Optional - may improve resolution - especially hi res cams
//#define WHITEBRIGHTNESS 0x00      // Optional change from default 0x00=120%,0x10=90%0x11=80%  default is 0x01=100%,
//#define INTRO_DELAY 5             // Seconds intro screen should show for. Default is 10 
#define TIMEZONESTARTUP             // Enable to display timezone at startup - if GPS TIME is enabled


/********************   ENABLE/DISABLE CONFIG PAGES via STICK MENU     *********************/
//large memory savings if not needed, comment to disable
#define PAGE1 //PID CONFIG
#define PAGE2 //RC TUNING
#define PAGE3 //VOLTAGE
#define PAGE4 //RSSI
#define PAGE5 //CURRENT
#define PAGE6 //DISPLAY
#define PAGE7 //ADVANCED
#define PAGE8 //GPS TIME


/********************  LEGACY compatibility **********************/
//#define BOXNAMES                  // required to support multiwii 2.1 / 2.0 and HARIKIRI


/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/

/********************       ADVANCED HARDWARE settings      *********************/
//#define RUSHDUINO                 // Select board type - RUSHDUINO. Defaults to MINIM
//#define TEMPSENSOR                // Enable if you have a hardware temperature sensor
#define STARTUPDELAY                // Enable startup delay of 1 sec to allow MAX chip voltage to rise fully and initialise before configuring 
#define DIVIDER1v1      0.0001      // Voltage divider for 1.1v reference. Use 0.0001 default unless advised otherwise.
#define DIVIDER5v       0.0005      // Voltage divider for 5v reference. Use 0.0005 default unless advised otherwise.


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
#define  OSDOFF24 DISPLAY_MIN_OFF // vidvoltagePosition
#define  OSDOFF25 DISPLAY_MIN_OFF // amperagePosition
#define  OSDOFF26 DISPLAY_MIN_OFF // pMeterSumPosition
#define  OSDOFF27 DISPLAY_MIN_OFF  // horizonPosition
#define  OSDOFF28 DISPLAY_MIN_OFF // CallSign Position
#define  OSDOFF29 DISPLAY_ALWAYS // Debug Position
//#define  OSDOFF30 DISPLAY_MIN_OFF // Gimbal Position
#define  OSDOFF31 DISPLAY_MIN_OFF // GPS_time Position


/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
//#define DEBUG         // Enable/disable option to display Mutltiwii debug values 

