/*--------------------------       configurable parameters      ----------------------------------------------------*/


/********************       OSD HARDWARE settings      *********************/
//Choose ONLY ONE option:
#define MINIMOSD                    // Uncomment this if using standard MINIMOSD hardware (default for 95% of boards) 
//#define WITESPYV1.1               // Uncomment this if using Witespy V1.1 OSD, select this to correct for both swapped bat1/bat 2 and to also use alternative resistors / pinouts.  
//#define RUSHDUINO                 // Uncomment this if using Rushduino

// NOTE-some of the popular RTFQ/Witespy boards have swapped bat1/bat2 pins and alternative voltage measuring resistors
// If having difficulties, first select default MINIMOSD as above, then use the following to correct: 
// #define SWAPVOLTAGEPINS          // For RTFQ boards with batt voltage appearing on vid voltage
// #define ALTERNATEDIVIDERS        // For RFTQ boards with voltage unable to be adjusted high enough

 
/********************       CONTROLLER SOFTWARE      *********************/
//Choose ONLY ONE option:-
#define MULTIWII_V24                // Uncomment this if you are using MW versions 2.4  
//#define MULTIWII_V23              // Uncomment this if you are using MW versions 2.2/2.3  
//#define MULTIWII_V21              // Uncomment this if you are using MW versions 2.0/2.1  (for BOXNAMES compatibility)
//#define BASEFLIGHT                // Uncomment this if you are using BASEFLIGHT (Stable 2015.05.13 at time of release)
//#define BASEFLIGHT_PR             // UNTESTED - use this if you are using BASEFLIGHT with separate Pitch / Roll. 
//#define CLEANFLIGHT               // Uncomment this if you are using latest CLEANFLIGHT version from repository (1.9.0 at time or release)
//#define CLEANFLIGHT180            // Uncomment this if you are using CLEANFLIGHT versions up to and including 1.8.0
//#define CLEANFLIGHT181            // Uncomment this if you are using CLEANFLIGHT versions 1.8.1 
//#define HARIKIRI                  // Uncomment this if you are using HARIKIRI (for BOXNAMES compatibility)
//#define GPSOSD_GPSOSD_UBLOX              // Uncomment this if you are using a UBLOX GPS module for a GPS based OSD
//#define GPSOSD_NMEA               // Uncomment this if you are using a NMEA compatible GPS module for a GPS based OSD
//#define GPSOSD_MTK                // Uncomment this if you are using a MTK module for a GPS based OSD
//#define NOCONTROLLER              // Uncomment this if you ahave nothing connected to the serial port - no controller or GPS module


/********************       AIRCRAFT/INSTALLATION TYPE settings      *********************/
//Choose ONLY ONE option:
#define ROTORCRAFT                  // Default for multirotors etc. 
//#define FIXEDWING                 // Uncomment this if you are using fixed wing MultiWii or Baseflight 


/********************       HARDWARE CURRENT sensor settings      *********************/
#define AMPERAGEMAX     500         // Size of current sensor / maximum current draw (* 10) e.g. 50A sensor = 500, 100A sensor = 1000
#define AMPERAGEOFFSET  0           // Optional extra for high offset sensors not supported in GUI (typically bidirectional sensors use a value of 256-512) 


/********************       OSD SCREEN SWITCH settings      *********************/
// This functionality enables :
// a, 2 different screen layouts to be selected using the Flight controller "OSD_SWITCH" feature or
// b, 2 or 3 different screen layouts to be selected using a specificed RC channel assigned to a TX switch
//Choose ONLY ONE option:
//#define OSD_SWITCH                // Uses original 2 way screen switch using OSD Switch via Flight Controller. MUST Ensure enabled on flight controller - e.g. #define OSD_SWITCH on multiwii
#define OSD_SWITCH_RC               // Enables GUI to use 2 way OSD_SWITCh or 3 way screen switch using RC data. Specify channel on GUI (range 0-7 AUX1=4 AUX4=7)


/********************       FILTER settings      *********************/
//Choose ONLY ONE option:
#define STAGE2FILTER               // Enable for smoother readings of voltage / current / RSSI. 
//#define SMOOTHFILTER             // Enable for smoothest readings of voltage / current / RSSI. Uses more memory. Prototype


/********************       RSSI settings      *********************/
//#define FASTPWMRSSI              // Undefine this if you are using non standard PWM for RSSI ( high frequency ) 


/********************       GPS settings      *********************/
#define MINSATFIX 5               // Number of sats required for a fix. 5 minimum. More = better


/********************       WARNING/STATUS settings      *********************/
//#define SATACTIVECHECK            // Alerts if sats below MINSATFIX - in addition to flashing sat indicator
//#define GPSACTIVECHECK 5          // Alerts if no GPS data for more than x secs. Sets GPS sats to zero
#define MSPACTIVECHECK 3          // Alerts if no Flight controller data for more than x secs. 
#define DISP_LOW_VOLTS_WARNING    // Alerts if low voltage
#define FORCE_DISP_LOW_VOLTS      // Enable display low voltage warning override for screen layouts where its disabled


/********************       AIRCRAFT type=FIXEDWING settings      *********************/
// **ONLY** valid when using fixed wing
//#define USEMAGHEADING             // Undefine this to use MAG for FW heading instead of GPS (requires controller with MAG sensor) 
//#define USEBAROALTITUDE           // Undefine this if you have a BARO to use BARO for FW altitude instead of GPS (requires controller with BARO sensor) ** Recommended **
//#define USEGLIDESCOPE 40          // Enables ILS glidescope where 40 = 4.0° glidescope. 1.0 deg gradiented scope scale
//#define DISABLEGPSALTITUDERESET   // Disables automatic reset of GPS Altitude to zero at arm for FC that already provide this functionality. 


/******************** Serial speed settings *********************/
// Choose ONLY ONE option:
#define BAUDRATE 115200
//#define BAUDRATE 57600
//#define BAUDRATE 38400
//#define BAUDRATE 19200


/********************       CALLSIGN settings      *********************/
#define   CALLSIGNINTERVAL 60      // How frequently to display Callsign (in seconds)
#define   CALLSIGNDURATION 4       // How long to display Callsign (in seconds)
//#define CALLSIGNALWAYS           // Alternative option - enable to permanently display callsign.
//#define FREETEXTLLIGHTS          // Alternative option - enable to display freetext (or callsign) when LLIGHTS Switch active on TX.
//#define FREETEXTGIMBAL           // Alternative option - enable to display freetext (or callsign) when GIMBAL Switch active on TX.


/********************       STARTUP settings      *********************/
//#define INTRO_VERSION               "SHIKI-OSD - R1.4" // Call the OSD something else if you prefer. KVOSD is not permitted - LOL. 
//#define INTRO_CALLSIGN            // Enable to display callsign at startup
//#define INTRO_TIMEZONE            // Enable to display timezone at startup - if GPS TIME is enabled
//#define INTRO_DELAY 5             // Seconds intro screen should show for. Default is 10 
//#define INTRO_MENU                  // Enable to display TX stick MENU 
//#define STARTUPDELAY 2000         // Enable alternative startup delay (in ms) to allow MAX chip voltage to rise fully and initialise before configuring 


/********************       GPS type settings      *********************/
//#define I2CGPS_SPEED              // Uncomment this if you are using older I2CGPS - and need to correct for speed error (10x too slow)               
//#define I2CGPS_DISTANCE           // Uncomment this if you are using older I2CGPS - and need to correct for distance error (650m max) UNTESTED               


/********************       MAP MODE Settings       *********************/
//#define MAPMODENORTH              // Enable to use North as MAP reference in MODE 1 instead of take off direction (Default = disable) 


/********************       FEATURES      *********************/
// Disable features if you require memory for other features
// Further configuration may be require elsewhere in config.h + option enabled on GUI
#define SBDIRECTION     // Enable/disable sidebar indicators (changes in speed or altitude)
#define HORIZON         // Enable/disable HORIZON indicator
#define MAPMODE         // Enable/disable MAP MODE - map indication of relative positions of home and aircraft
//#define GPSTIME       // Enable/disable GPS Time functions
//#define SPORT         // Enable/disable FRSKY S.PORT cell code


/********************       Display Settings         ************************/
//#define AUTOCAM                     // Disable if no screen display. Enables autodetect Camera type PAL/NTSC. Overrides GUI/OSD settings.
//#define AUTOCAMWAIT               // **UNTESTED** - Use with AUTOCAM - waits until camera is ready - i.e. if power up cameras after FC. 
#define DECIMAL '.'                 // Decimal point character, change to what suits you best (.) (,)
//#define USE_VSYNC                   // Disable if no screen display. Removes sparklies as updates screen during blanking time period. 
//#define SHIFTDOWN                 // Select if your monitor cannot display top line fully. It shifts top 3 lines down. Not suitable for all layouts
//#define ALT_CENTER                // Enable alternative center crosshair
//#define FORCECROSSHAIR            // Forces a crosshair even if no AHI / horizon used
//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status
//#define FASTPIXEL                 // Optional - may improve resolution - especially hi res cams
//#define WHITEBRIGHTNESS 0x00      // Optional change from default 0x00=120%,0x01=100%,0x10=90%,0x11=80%  default is 0x01=100%
//#define BLACKBRIGHTNESS 0x00      // Optional change from default 0x00=0%,0x01=10%,0x10=20%0x11=30%  default is 0x00=0%
//#define FULLAHI                   // Enable to display a slightly longer AHI line
//#define I2CERROR 3                // Autodisplay Mutltiwii I2C errors if exceeds specified count 
//#define SHORTSTATS                // Display only timer on flight summary 
//#define FASTMSP                   // Enable for soft serial / slow baud rates if don't need GPS/BARO/HORIZON data. Speeds up remainder
//#define NOTHROTTLESPACE           // Enable to remove space between throttle symbol and the data
//#define REVERSEAHI                // Reverse pitch / roll direction of AHI - for DJI / Eastern bloc OSD users
//#define AHICORRECT 10             // Enable to adjust AHI on display to match horizon. -10 = -1 degree
#define APINDICATOR                 // Enable to display AUTOPILOT instead of RTH distance 


/********************       FrSky S.Port settings      *********************/
//enables data transfer from frsky reciever s.port to osd via multiwii
//requires serial inverter cable & multiwii with s.port code
//Auto detected cell graph from s.port, 16 steps @ 0.05v 
//To show battery voltage from s.port, enable "Use MWii" under "Main Voltage" in GUI
//To show amperage from s.port, enable "Use MWii" under Amperage in GUI
//more details: http://code.google.com/p/scarab-osd/wiki/Frsky_SPort
#define MIN_CELL 320 //Cell Low Flash - No decimal, 3 Digits ie 320 = 3.20v


/********************  TEMPERATURE  settings      *********************/
//#define TEMPSENSOR                // Enable if you have a hardware temperature sensor - DEPRECATED
#define TEMPERATUREMAX 50           // Temperature warning value



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
#define PAGE9 //ALARMS


/********************  HARDWARE PINS definitions  *********************/
#define AMPERAGEPIN   A1
#define TEMPPIN       A6           
#define RSSIPIN       A3              
#define PWMRSSIPIN    A3              
#define LEDPIN        7


/********************  CONTROLLER rule definitions  **********************/
#ifdef BASEFLIGHT                    
    #define AMPERAGECORRECT         // required to use Higher MW amperage but with less resolution
#endif

#if defined(HARIKIRI) || defined(MULTIWII_V21)                     
  #define BOXNAMES                  // required to support legacy protocol
#endif

#ifdef NOCONTROLLER
  #undef  INTRO_MENU
  #undef  MSPACTIVECHECK
  #undef  SATACTIVECHECK
  #undef  GPSACTIVECHECK
  #undef  OSD_SWITCH_RC
#endif

#ifdef MULTIWII_V24                     
#endif

#ifdef CLEANFLIGHT
  #define CLEANFLIGHT190
#endif

#ifdef FIXEDWING                     
  #define USEGPSHEADING
  #define USEGPSALTITUDE
  #if defined USEMAGHEADING 
    #undef USEGPSHEADING
  #endif  
  #if defined USEBAROALTITUDE
    #undef USEGPSALTITUDE
  #endif
  #define FORCESENSORS
#endif

// All aircraft / FC types...
#define RESETGPSALTITUDEATARM
#define HEADINGCORRECT              // required to correct for MWheading being 0>360 vs MWII -180>+180. Leave permanently enabled

#if defined DISABLEGPSALTITUDERESET
  #undef RESETGPSALTITUDEATARM
#endif


/********************  OSD HARDWARE rule definitions  *********************/
#ifdef RUSHDUINO                     
    # define MAX7456SELECT 10        // ss 
    # define MAX7456RESET  9         // RESET
#else                                  
    # define MAX7456SELECT 6         // ss
    # define MAX7456RESET  10        // RESET
#endif

#ifdef WITESPYV1.1                     
    #define SWAPVOLTAGEPINS
    #define ALTERNATEDIVIDERS
#endif

#ifdef SWAPVOLTAGEPINS                     
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
#else                                  
    #define VOLTAGEPIN    A0
    #define VIDVOLTAGEPIN A2
#endif

#ifdef ALTERNATEDIVIDERS                     
    #define DIVIDER1v1      0.0002      // Voltage divider for 1.1v reference. 
    #define DIVIDER5v       0.0008      // Voltage divider for 5v reference. 
#else                                  
    #define DIVIDER1v1      0.0001      // Voltage divider for 1.1v reference. Use 0.0001 default unless advised otherwise.
    #define DIVIDER5v       0.0005      // Voltage divider for 5v reference. Use 0.0005 default unless advised otherwise.
#endif

/********************  GPS OSD rule definitions  *********************/

#if defined GPSOSD_UBLOX
  #define UBLOX
#endif
#if defined GPSOSD_NMEA
  #define NMEA
#endif
#if defined GPSOSD_MTK
  #define MTK
#endif


#if defined MTK_BINARY16
  #define GPSOSD
  #define NMEA
  #define INIT_MTK_GPS
  #define MTK_BINARY16
#endif

#if defined MTK_BINARY19
  #define GPSOSD
  #define NMEA
  #define INIT_MTK_GPS
  #define MTK_BINARY19
#endif

#if defined MTK
  #define GPSOSD
  #define NMEA
  #define INIT_MTK_GPS
#endif

#if defined UBLOX
  #define GPSOSD
#endif

#if defined NMEA
  #define GPSOSD
#endif

#if defined NAZA
  #define GPSOSD
#endif

#if defined GPSOSD
  #undef  INTRO_MENU
  #undef  MSPACTIVECHECK
  #undef  OSD_SWITCH_RC
  #define FORCESENSORS
  #define HIDEARMEDSTATUS
#endif


/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
//#define DEBUG         // Enable/disable option to display OSD debug values 
//#define DEBUGMW       // Disable to prevent load Mutltiwii debug values from MSP 

