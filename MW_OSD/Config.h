/*--------------------------       configurable parameters      ----------------------------------------------------*/

/********************       FEATURES      *********************/
// Disable features if you require memory for other features
// Further configuration may be require elsewhere in config.h + option enabled on GUI
#define SBDIRECTION     // Enable/disable sidebar indicators (changes in speed or altitude)
#define HORIZON         // Enable/disable HORIZON indicator
#define MAPMODE         // Enable/disable MAP MODE - map indication of relative positions of home and aircraft
#define GPSTIME         // Enable/disable GPS Time functions
//#define SPORT           // Enable/disable FRSKY S.PORT cell code


/********************       HARDWARE OSD settings      *********************/
//#define USE_VSYNC                 // Remove "sparklies" on boards that support VSYNC 
//#define WITESPYV2                 // If using Witespy V2 OSD, select this to use alternative resistors / pinouts 
//#define RUSHDUINO                 // If using Rushduino, select this


/********************       HARDWARE CURRENT sensor settings      *********************/
#define AMPERAGEMAX     500         // Size of current sensor / maximum current draw (* 10) e.g. 50A sensor = 500, 100A sensor = 1000
#define AMPERAGEOFFSET  0           // Optional extra for high offset sensors not supported in GUI (typically bidirectional sensors use a value of 256-512) 


/********************       FILTER settings      *********************/
// Choose only one of the following
#define STAGE2FILTER              // Enable for smoother readings of voltage / current / RSSI. 
//#define SMOOTHFILTER              // Enable for smoothest readings of voltage / current / RSSI. Uses more memory. Prototype


/********************       RSSI settings      *********************/
//#define FASTPWMRSSI               // Undefine this if you are using non standard PWM for RSSI ( high frequency ) 


/********************       CONTROLLER settings      *********************/
//#define BASEFLIGHT                // Undefine this if you are using BASEFLIGHT / NAZE32 to correct for heading dispaly issues
//#define HARIKIRI                  // Undefine this if you are using HARIKIRI (for BOXNAMES compatibility)
//#define FIXEDWING                 // Undefine this if you are using MW fixed wing from PatrikE - to use GPS heading and altitude instead of BARO/MAG
//#define MULTIWII_V21              // Undefine this if you are using MW versions 2.0/2.1  (for BOXNAMES compatibility)
#define MULTIWII_V23                // Undefine this if you are using MW versions 2.2/2.3  
//#define MULTIWII_V24              // Undefine this if you are using MW versions 2.4  


/********************       CALLSIGN settings      *********************/
//#define CALLSIGNALWAYS  341       // Enable to permanently display callsign. Number = screen position (row*30 + column)
//#define FREETEXTLLIGHTS 342       // Enable to display freetext (or callsign) when LLIGHTS Switch active on TX. Number = screen position (row*30 + column)
//#define FREETEXTGIMBAL  342       // Enable to display freetext (or callsign) when GIMBAL Switch active on TX. Number = screen position (row*30 + column)


/********************       STARTUP settings      *********************/
//#define INTRO_VERSION               "SCARAB OSD - R1.2" // Call the OSD something else if you prefer. KVOSD is not permitted - LOL. 
//#define INTRO_CALLSIGN            // Enable to display callsign at startup
//#define INTRO_TIMEZONE            // Enable to display timezone at startup - if GPS TIME is enabled
//#define INTRO_DELAY 5             // Seconds intro screen should show for. Default is 10 
#define INTRO_MENU                  // Enable to display TX stick MENU 


/********************       GPS type settings      *********************/
//#define I2CGPS_SPEED              // Undefine this if you are using older I2CGPS - and need to correct for speed error (10x too slow)               
//#define I2CGPS_DISTANCE           // Undefine this if you are using older I2CGPS - and need to correct for distance error (650m max) UNTESTED               


/********************       MAP MODE Settings       *********************/
#define MAPTYPE 1                   // 0 for RADAR - home is aircraft, map shows location of home relative to aircraft 
                                    // 1 for MAP - home is center, map shows location of aircraft relative to home 
//#define MAPRESLOW                 // enable to use low res original directional arrow in map mode 1. Disable to use high res position 
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
//#define SHIFTDOWN                 // Select if your monitor cannot display top line fully. It shifts top 3 lines down. Not suitable for all layouts
//#define ALT_CENTER                // Enable alternative center crosshair
//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status
//#define FASTPIXEL                 // Optional - may improve resolution - especially hi res cams
//#define WHITEBRIGHTNESS 0x00      // Optional change from default 0x00=120%,0x10=90%0x11=80%  default is 0x01=100%,
//#define FULLAHI                   // Enable to display a slightly longer AHI line
//#define I2CERROR 3                // Autodisplay Mutltiwii I2C errors if exceeds specified count 
#define APINDICATOR                 // Enable to display AUTOPILOT instead of RTH distance 

/********************       Serial speed settings      *********************/
#define BAUDRATE 115200             // Serial comms speed               




/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/

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


/********************  CONTROLLER compatibility **********************/
#ifdef BASEFLIGHT                     
    #define AMPERAGECORRECT         // required to use Higher MW amperage but with less resolution
    #define HEADINGCORRECT          // required to use alternative MW heading -180 to +180
#endif
#ifdef HARIKIRI                     
    #define BOXNAMES                // required to support HARIKIRI
#endif
#ifdef MULTIWII_V21                     
    #define BOXNAMES                // required to support multiwii 2.1 / 2.0
#endif
#ifdef MULTIWII_V24                     
    #define AMPERAGECORRECT         // required to use Higher MW amperage but with less resolution
#endif
#ifdef FIXEDWING                     
#endif


/********************  ADVANCED HARDWARE settings      *********************/
//#define TEMPSENSOR                // Enable if you have a hardware temperature sensor
//#define STARTUPDELAY 2000         // Enable alternative startup delay (in ms) to allow MAX chip voltage to rise fully and initialise before configuring 


/********************  HARDWARE PINS settings      *********************/
#define AMPERAGEPIN   A1
#define TEMPPIN       A6           
#define RSSIPIN       A3              
#define PWMRSSIPIN    A3              
#define LEDPIN        7


/********************  OSD HARDWARE BOARD SPECIFIC settings      *********************/

#ifdef RUSHDUINO                     
    # define MAX7456SELECT 10        // ss 
    # define MAX7456RESET  9         // RESET
#else                                  
    # define MAX7456SELECT 6         // ss
    # define MAX7456RESET  10        // RESET
#endif
#ifdef WITESPYV2                     
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
    #define DIVIDER1v1      0.0002      // Voltage divider for 1.1v reference. 
    #define DIVIDER5v       0.0008      // Voltage divider for 5v reference. 
#else                                  
    #define VOLTAGEPIN    A0
    #define VIDVOLTAGEPIN A2
    #define DIVIDER1v1      0.0001      // Voltage divider for 1.1v reference. Use 0.0001 default unless advised otherwise.
    #define DIVIDER5v       0.0005      // Voltage divider for 5v reference. Use 0.0005 default unless advised otherwise.
#endif


/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
#define DEBUG         // Enable/disable option to display OSD debug values 
#define DEBUGMW       // Disable to prevent load Mutltiwii debug values from MSP 

