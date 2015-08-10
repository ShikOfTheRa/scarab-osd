/*--------------------------       initialisation options      ----------------------------------------------------*/
// Ignore this section unless you know you need to use it !!
// This section contains initialisation options that only require to be run once.
// Once the initialisation has completed, all sections should be commented and the sketch re-uploaded.

//#define EEPROM_CLEAR             // Uncomment to force a wipe and reload of default settings at each OSD start. Same as EEPROM_CLEAR sketch.  
//#define LOADFONT_DEFAULT         // Uncomment to force an upload of default font instead of using GUI
//#define LOADFONT_LARGE           // Uncomment to force an upload of large font instead of using GUI



/*--------------------------       configurable parameters      ----------------------------------------------------*/

/********************       OSD HARDWARE settings      *********************/
//Choose ONLY ONE option:
#define MINIMOSD                    // Uncomment this if using standard MINIMOSD hardware (default for 95% of boards) 
//#define WITESPYV1                 // Uncomment this if using Witespy V1.1 OSD, select this to correct for both swapped bat1/bat 2 and to also use alternative resistors / pinouts.  
//#define WITESPYMICRO              // Uncomment this if using Witespy Micro Minim OSD, select this to correct for swapped bat1/bat 2.  
//#define RUSHDUINO                 // Uncomment this if using Rushduino

// NOTE-some of the popular RTFQ/Witespy boards have swapped bat1/bat2 pins and alternative voltage measuring resistors
// If having difficulties, first select default MINIMOSD as above, then use the following to correct: 
// #define SWAPVOLTAGEPINS          // For boards with batt voltage appearing on vid voltage
// #define ALTERNATEDIVIDERS        // For boards with voltage unable to be adjusted high enough

 
/********************       CONTROLLER SOFTWARE      *********************/
// Choose ONLY ONE option:-
// Note - choose carefully to ensure correct settings are written to flight controller.
// The first three are for convenience - they set the OSD for the latest FC version. 
// IMPORTANT - remember to update MWOSD when updating FC software!!

// Choose ONLY ONE option from the following long list :-

// latest release...
#define MULTIWII                  // Uncomment this if you are using latest MULTIWII version from repository (2.4 at time of this MWOSD release)
//#define BASEFLIGHT                // Uncomment this if you are using latest BASEFLIGHT version from repository (Stable 2015.06.27 at time of this MWOSD release)
//#define CLEANFLIGHT               // Uncomment this if you are using latest CLEANFLIGHT version from repository (1.9.0 at time or this MWOSD release)
//#define HARIKIRI                  // Uncomment this if you are using HARIKIRI (for BOXNAMES compatibility)
//#define NAZA                      // Uncomment this if you are using NAZA flight controller
//#define GPSOSD_UBLOX              // Uncomment this if you are using a UBLOX GPS module for a GPS based OSD
//#define GPSOSD_NMEA               // Uncomment this if you are using a NMEA compatible GPS module for a GPS based OSD
//#define GPSOSD_MTK                // Uncomment this if you are using a MTK module for a GPS based OSD
//#define NOCONTROLLER              // Uncomment this if you ahave nothing connected to the serial port - no controller or GPS module
// old releases supported...
//#define MULTIWII_V23              // Uncomment this if you are using MW versions 2.2/2.3  
//#define MULTIWII_V21              // Uncomment this if you are using MW versions 2.0/2.1  (for BOXNAMES compatibility)
//#define BASEFLIGHT20150327        // Uncomment this if you are using BASEFLIGHT up to and including version Stable 2015.03.27
//#define CLEANFLIGHT172            // Uncomment this if you are using CLEANFLIGHT versions up to and including 1.7.2
//#define CLEANFLIGHT180            // Uncomment this if you are using CLEANFLIGHT versions 1.8.0 & 1.8.1 



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
#define OSD_SWITCH_RC               // Enables 3 way screen switch using a TX channel via FC. Specify channel on GUI (range 0-7 AUX1=4 AUX4=7)
//#define OSD_SWITCH_RSSI           // Enables 3 way screen switch using a TX channel via a RX channel connected to the OSD RSSI pin. Typically used for GPSOSD.


/********************       GPS OSD settings      *********************/
//#define PPMOSDCONTROL             // Enables full OSD menu, screen switching, RSSI, Throttle fature, virtual current sensor, etc using a PPM signal into OSD RSSI pin 
//#define SERIAL_SUM_PPM_RHF        // Enable for Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM_GS         // Enable for Graupner/Spektrum    
//#define SERIAL_SUM_PPM_M          // Enable for Multiplex
//#define SERIAL_SUM_PPM_HS         // Enable for Hitec/Sanwa


/********************       FILTER settings      *********************/
//Choose ONLY ONE option:
#define STAGE2FILTER                // Enable for smoother readings of voltage / current / RSSI. 
//#define SMOOTHFILTER              // Enable for smoothest readings of voltage / current / RSSI. Uses more memory. Prototype


/********************       RSSI settings      *********************/
//Choose ONLY ONE option:
#define INTPWMRSSI                  // Undefine this to use new interrup PWM RSSI method (standard PWM 750-2250ms pulse width)
//#define PULSEINPWMRSSI            // Undefine this to use legacy non interrupt PWM RSSI method (pulse width 0 - 2250ms pulse width)
//#define FASTPWMRSSI               // Undefine this to use high PWM refresh frequency RSSI 


/********************       GPS settings      *********************/
#define MINSATFIX 5                 // Number of sats required for a fix. 5 minimum. More = better


/********************       WARNING/STATUS settings      *********************/
#define SATACTIVECHECK              // Alerts if sats below MINSATFIX - in addition to flashing sat indicator
#define GPSACTIVECHECK 5            // Alerts if no GPS data for more than x secs. Sets GPS sats to zero
#define MSPACTIVECHECK 3            // Alerts if no Flight controller data for more than x secs. 
#define DISP_LOW_VOLTS_WARNING      // Alerts if low voltage
#define FORCE_DISP_LOW_VOLTS        // Enable display low voltage warning override for screen layouts where its disabled


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
//#define BAUDRATE 9600


/******************** Serial MSP speed settings *********************/
// Choose ONLY ONE option: increases speeds of serial update - but with impact to flight controller 
#define MSP_SPEED_LOW
//#define MSP_SPEED_MED
//#define MSP_SPEED_HIGH


/********************       CALLSIGN settings      *********************/
#define   CALLSIGNINTERVAL 60      // How frequently to display Callsign (in seconds)
#define   CALLSIGNDURATION 4       // How long to display Callsign (in seconds)
//#define CALLSIGNALWAYS           // Alternative option - enable to permanently display callsign.
//#define FREETEXTLLIGHTS          // Alternative option - enable to display freetext (or callsign) when LLIGHTS Switch active on TX.
//#define FREETEXTGIMBAL           // Alternative option - enable to display freetext (or callsign) when GIMBAL Switch active on TX.


/********************       STARTUP settings      *********************/
//#define INTRO_VERSION               "MWOSD - DEV 1.5.1" // Call the OSD something else if you prefer. KVOSD is not permitted - LOL. 
//#define INTRO_CALLSIGN            // Enable to display callsign at startup
//#define INTRO_TIMEZONE            // Enable to display timezone at startup - if GPS TIME is enabled
//#define INTRO_DELAY 5             // Seconds intro screen should show for. Default is 10 
#define INTRO_MENU                  // Enable to display TX stick MENU 
//#define STARTUPDELAY 2000         // Enable alternative startup delay (in ms) to allow MAX chip voltage to rise fully and initialise before configuring 


/********************       I2CGPS type settings      *********************/
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
#define MAXSTALLDETECT              // Enable to attempt to detect MAX chip stall from bad power. Attempts to restart.
//#define AUTOCAM                   // Disable if no screen display. Enables autodetect Camera type PAL/NTSC. Overrides GUI/OSD settings.
//#define AUTOCAMWAIT               // **UNTESTED** - Use with AUTOCAM - waits until camera is ready - i.e. if power up cameras after FC. 
#define DECIMAL '.'                 // Decimal point character, change to what suits you best (.) (,)
#define USE_VSYNC                   // Disable if no screen display. Removes sparklies as updates screen during blanking time period. 
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
//#define DISPLAY_PR                // Display pitch / roll angles. Requires relevant layout ppositions to be enabled
//#define AHICORRECT 10             // Enable to adjust AHI on display to match horizon. -10 = -1 degree
#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees 
#define APINDICATOR                 // Enable to display AUTOPILOT instead of RTH distance 


/********************       NAZA Settings         ************************/
//#define NAZAMODECONTROL           // Enables NAZA mode control display using a PWM signal into OSD RSSI pin. Can be used with OSD_SWITCH_RSSI   
#define NAZA_MODE_GPS 1600
#define NAZA_MODE_ATI 
#define NAZA_MODE_MAN 1400

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


