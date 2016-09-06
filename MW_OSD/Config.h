/*--------------------------       MANDATORY configurable parameters      ----------------------------------------------------*/
/*--------------------------       MANDATORY configurable parameters      ----------------------------------------------------*/

/********************       OSD HARDWARE settings      *********************/
//Choose ONLY ONE option:
#define MINIMOSD                    // Uncomment this if using standard MINIMOSD hardware
//#define MICROMINIMOSD             // Uncomment this if using the MICRO MINIMOSD hardware
//#define RTFQV1                    // Uncomment this if using standard RTFQ/Witespy V1.1 OSD, select this to correct for both swapped bat1/bat 2 and to also use alternative resistors / pinouts.  
//#define RTFQMICRO                 // Uncomment this if using micro RTFQ/Witespy Micro Minim OSD, select this to correct for swapped bat1/bat 2.  
//#define RUSHDUINO                 // Uncomment this if using Rushduino
//#define KYLIN250PDB               // Uncomment this if using a Kylin 250 FPV PDB (Using A6 as VOLTAGEPIN)

// NOTE-some of the popular RTFQ/Witespy boards have swapped bat1/bat2 pins and alternative voltage measuring resistors
// If having difficulties, first select default MINIMOSD as above, then use the following to correct: 
// #define SWAPVOLTAGEPINS          // For boards with batt voltage appearing on vid voltage
// #define ALTERNATEDIVIDERS        // For boards with voltage unable to be adjusted high enough

// For hardwares WITHOUT access to MAX RESET pin (e.g. Airbot MicroOSD v2.3)
//#define MAX_SOFTRESET


/********************       CONTROLLER SOFTWARE      *********************/
// Choose ONLY ONE option:-
// Note - choose carefully to ensure correct settings are written to flight controller.
// The first three are for convenience - they set the OSD for the latest FC version. 
// IMPORTANT - remember to update MWOSD when updating FC software!!

// Choose ONLY ONE option from the following long list :-

// latest release...
//#define MULTIWII                  // Uncomment this if you are using latest MULTIWII version from repository (2.4 at time of this MWOSD release)
//#define BASEFLIGHT                // Uncomment this if you are using latest BASEFLIGHT version from repository (Stable 2015.08.27 at time of this MWOSD release)
//#define LIBREPILOT                 // Uncomment this if you are using the latest LibrePilot MSP Module
//#define TAULABS                   // Uncomment this if you are using the latest Tau Labs MSP Module
//#define DRONIN                    // Uncomment this if you are using the latest DRONIN MSP Module
#define CLEANFLIGHT               // Uncomment this if you are using latest CLEANFLIGHT version from repository (1.11.0 at time of this MWOSD release)
//#define BETAFLIGHT                // Uncomment this if you are using BETAFLIGHT (same as CLEANFLIGHT at time of this MWOSD release)
//#define FIXEDWING_BF              // Uncomment this if you are using fixed wing Baseflight 
//#define FIXEDWING_BF_SERVO        // Uncomment this if you are using fixed wing Baseflight with additional SERVO adjustment menu.
//#define HARAKIRI                  // Uncomment this if you are using HARAKIRI (for BOXNAMES compatibility)
//#define NAZA                      // Uncomment this if you are using NAZA flight controller
//#define iNAV                      // Uncomment this if you are using latest iNAV version from repository (1.01 at time of this MWOSD release)
//#define KISS                      // Uncomment this if you are using KISS FC
//#define APM                       // Uncomment this if you are using APM MAVLINK compatible FC (Requires testing)
//#define SKYTRACK                  // Under development
//#define GPSOSD_UBLOX              // Uncomment this if you are using a UBLOX GPS module for a GPS based OSD
//#define GPSOSD_NMEA               // Uncomment this if you are using a NMEA compatible GPS module for a GPS based OSD
//#define GPSOSD_MTK                // Uncomment this if you are using a MTK module for a GPS based OSD
//#define NOCONTROLLER              // Uncomment this if you have nothing connected to the serial port - no controller or GPS module
// old releases supported...
//#define MULTIWII_V23              // Uncomment this if you are using MW versions 2.2/2.3  
//#define MULTIWII_V21              // Uncomment this if you are using MW versions 2.0/2.1  (for BOXNAMES compatibility)
//#define BASEFLIGHT20150327        // Uncomment this if you are using BASEFLIGHT up to and including version Stable 2015.03.27
//#define CLEANFLIGHT172            // Uncomment this if you are using CLEANFLIGHT versions up to and including 1.7.2
//#define CLEANFLIGHT180            // Uncomment this if you are using CLEANFLIGHT versions 1.8.0 & 1.8.1 

/********************       AIRCRAFT/INSTALLATION TYPE settings      *********************/
//Choose ONLY ONE option:
#define ROTORCRAFT                  // Default for multirotors etc. 
//#define FIXEDWING                 // Uncomment this if you are using fixed wing with MultiWii or Baseflight 

/*--------------------------       INITIALISATION options       ----------------------------------------------------*/
/*--------------------------       INITIALISATION options       ----------------------------------------------------*/
// Ignore this section unless you know you need to use it !!
// This section contains initialisation options that only require to be run once.
// Once the initialisation has completed, all sections should be commented and the sketch re-uploaded.
// Font upload will take 90 seconds after upload is completed. If connected to a camera, you will see teh font table displayed.

//#define EEPROM_CLEAR             // Uncomment to force a wipe and reload of default settings at each OSD start. Same as EEPROM_CLEAR sketch.  
//#define LOADFONT_DEFAULT         // Uncomment to force an upload of default font instead of using GUI
//#define LOADFONT_LARGE           // Uncomment to force an upload of large font instead of using GUI
//#define LOADFONT_BOLD            // Uncomment to force an upload of bold font instead of using GUI



/*--------------------------       OPTIONAL configurable parameters      ----------------------------------------------------*/
/*--------------------------       OPTIONAL configurable parameters      ----------------------------------------------------*/

/********************       FEATURES      *********************/
// Disable features if you require memory for other features
// Further configuration may be require elsewhere in config.h + option enabled on GUI
#define SBDIRECTION     // Enable/disable sidebar indicators (changes in speed or altitude)
#define HORIZON         // Enable/disable HORIZON indicator
#define MAPMODE         // Enable/disable MAP MODE - map indication of relative positions of home and aircraft
//#define GPSTIME       // Enable/disable GPS Time functions
//#define SPORT         // Enable/disable FRSKY S.PORT cell code


/********************       TELEMETRY settings      *********************/
//Select ONLY if you are sure your OSD is connected to a telemetry feed:
//#define TELEMETRY_LTM             // Uncomment this if you are using a LTM telemetry feed (under development)
//#define TELEMETRY_MAVLINK         // Uncomment this if you are using a MAVLINK telemetry feed (under development)
//#define TELEMETRY_SKYTRACK        // Under development
//#define SETHOMEARMED              // Uncomment this to disable reset home position when arming. Use if FC armed data is sent via LTM
//#define FORCE_MSP                 // Uncomment to enable use of MSP as well as telemetry. Uses more memory 


/********************       GPS OSD settings      *********************/
#define GPSHOMEFIX 50               // Number of consecutive valid fixes before home will be set. 50@ 10hz = 5 seconds of valid fixes.
#define HOMESATFIX 6                // Minimum number of sats required when setting initial home location. 
#define GPSOSDARMDISTANCE 20        // distance from home in meters when GPSOSD arms. Starts flight timer etc.
#define GPSOSDHOMEDISTANCE 40       // distance from home in meters when GPSOSD is home. When speed is low it disarms and displays summary screen.
//#define OSD_SWITCH_RSSI           // Enables 3 way screen switch using a TX channel via a RX channel connected to the OSD RSSI pin. Typically used for GPSOSD.
//#define PWMTHROTTLE               // Enables throttle feature, virtual current sensor using RC throttle connected into OSD RSSI pin. Calibrate throttle using GUI RSSI cal functions 
//#define PPMOSDCONTROL             // Enables full OSD menu, screen switching, RSSI, Throttle feature, virtual current sensor, etc using a PPM signal into OSD RSSI pin. Requires TX type to be set below. 
//#define SERIAL_SUM_PPM_RHF        // Enable for Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM_GS         // Enable for Graupner/Spektrum    
//#define SERIAL_SUM_PPM_M          // Enable for Multiplex
//#define SERIAL_SUM_PPM_HS         // Enable for Hitec/Sanwa


/********************       FILTER settings      *********************/
//Choose ONLY ONE option to enable filtered smoother readings of voltage / current / RSSI :
#define FILTER_AVG                // Enable standard averaging filter  
//#define FILTER_HYSTERYSIS 2       // Alternative filter with hysteris to provide smoother changes. Higher number = more filtering. Max 4  

/********************       RSSI settings (PWM/PPM)      *********************/
//Choose ONLY ONE option: 
//Note all require PWM RSSI to be enabled on GUI
#define INTPWMRSSI                  // Undefine this to use new interrupt PWM RSSI method (standard PWM 0-2000ms pulse width)
//#define PULSEINPWMRSSI            // DEPRECATED Undefine this to use legacy non interrupt PWM RSSI method (pulse width 0 - 2000ms pulse width)
//#define RCRSSI 3                  // Undefine this to use RC channel (0-7) for RSSI (this can be from the FC - or a PPM channel with GPSOSD)


/********************       GPS settings      *********************/
#define MINSATFIX 5                 // Number of sats required for a fix. 5 minimum. More = better.


/********************       ALARM/STATUS settings      *********************/
#define ALARM_VOLTAGE               // Text alerts if voltage below voltage alarm - in addition to flashing voltage indicator
#define ALARM_SATS                  // Text alerts if sats below MINSATFIX - in addition to flashing sat indicator
#define ALARM_GPS 5                 // Text alerts if no GPS data for more than x secs. Sets GPS sats to zero
#define ALARM_MSP 3                 // Text alerts if no Flight controller data for more than x secs. 
#define FORCE_DISP_LOW_VOLTS        // Enable display low voltage warning override for screen layouts where its disabled
//#define FORCE_DISP_LOW_VID_VOLTS  // Enable display low VIDEO voltage warning override for screen layouts where its disabled


/********************       AIRCRAFT type=FIXEDWING settings      *********************/
// **ONLY** valid when using fixed wing
//#define USEMAGHEADING             // Undefine this to use MAG for FW heading instead of GPS (requires controller with MAG sensor) 
//#define USEBAROALTITUDE           // Undefine this if you have a BARO to use BARO for FW altitude instead of GPS (requires controller with BARO sensor) ** Recommended **
//#define USEGLIDESCOPE 40          // Enables ILS glidescope where 40 = 4.0° glidescope. 1.0 deg gradiented scope scale requires enabling in layouts
//#define DISABLEGPSALTITUDERESET   // Disables automatic reset of GPS Altitude to zero at arm for FC that already provide this functionality. 
//#define LONG_RANGE_DISPLAY        // Enable this to for long range display consolidation - displays distance in KM or feet when exceed 9999m or ft. Auto enabled for FIXEDWING


/******************** Serial speed settings *********************/
// Overides defaults if required (57.6k for MAVLINK based or 115k for all others). 
//#define BAUDRATE 115200
//#define BAUDRATE 57600
//#define BAUDRATE 38400
//#define BAUDRATE 19200
//#define BAUDRATE 9600


/******************** Mavlink settings *********************/
#define MAVLINKREQ                // Enable this for mavlink systems where the Mavlink data requires requesting. 


/******************** Serial MSP speed settings *********************/
// Choose ONLY ONE option: increases speeds of serial update - but with impact to flight controller 
//#define MSP_SPEED_LOW     // Enable for soft serial / slow baud rates.
#define MSP_SPEED_MED       // Default
//#define MSP_SPEED_HIGH    // Enable for faster AHI and speed updates. Requires higher baud rates and increases overhead on the FC to process


/********************       CALLSIGN settings      *********************/
#define   CALLSIGNINTERVAL 60      // How frequently to display Callsign (in seconds)
#define   CALLSIGNDURATION 4       // How long to display Callsign (in seconds)
//#define CALLSIGNALWAYS           // Alternative option - enable to permanently display callsign.
//#define FREETEXTLLIGHTS          // Alternative option - enable to display freetext (or callsign) when LLIGHTS Switch active on TX.
//#define FREETEXTGIMBAL           // Alternative option - enable to display freetext (or callsign) when GIMBAL Switch active on TX.

/********************       STARTUP settings      *********************/
#define INTRO_VERSION               "MWOSD - DEV 1.6.2.0" // Call the OSD something else if you prefer. 
#define INTRO_MENU                  // Enable to display TX stick MENU 
#define INTRO_CALLSIGN            // Enable to display callsign at startup
#define INTRO_SIGNALTYPE          // Enable to display video type at startup
//#define INTRO_DELAY 5             // Seconds intro screen should show for. Default is 8 
//#define STARTUPDELAY 2000         // Enable alternative startup delay (in ms) to allow MAX chip voltage to rise fully and initialise before configuring 


/********************       I2CGPS type settings      *********************/
//#define I2CGPS_SPEED              // Uncomment this if you are using older I2CGPS - and need to correct for speed error (10x too slow)               
//#define I2CGPS_DISTANCE           // Uncomment this if you are using older I2CGPS - and need to correct for distance error (650m max) UNTESTED               


/********************       MAP MODE Settings       *********************/
//#define MAPMODENORTH              // Enable to use North as MAP reference in MODE 1 instead of take off direction (Default = disable) 


/********************       Display Settings         ************************/
#define MAXSTALLDETECT              // Enable to attempt to detect MAX chip stall from bad power. Attempts to restart.
#define AUTOCAM                     // Disable if no screen display. Enables autodetect Camera type PAL/NTSC. Overrides GUI/OSD settings.
#define USE_VSYNC                   // Disable if no screen display. Removes sparklies as updates screen during blanking time period. 
#define DECIMAL '.'                 // Decimal point character, change to what suits you best (.) (,)
//#define SHIFTDOWN                 // Select if your monitor cannot display top line fully. It shifts top 3 lines down. Not suitable for all layouts
//#define ALT_CENTER                // Enable alternative center crosshair
//#define FORCECROSSHAIR            // Forces a crosshair even if no AHI / horizon used
//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status
//#define HIDESUMMARY               // Enable to suspend display of summary screen when disarming
//#define SHORTSUMMARY              // Display only timer on flight summary 
//#define FASTPIXEL                 // Optional - may improve resolution - especially hi res cams
//#define WHITEBRIGHTNESS 0x01      // Optional change from default 0x00=120%,0x01=100%,0x10=90%,0x11=80%  default is 0x01=100%
//#define BLACKBRIGHTNESS 0x00      // Optional change from default 0x00=0%,0x01=10%,0x10=20%0x11=30%  default is 0x00=0%
//#define I2CERROR 3                // Autodisplay Mutltiwii I2C errors if exceeds specified count 
//#define NOTHROTTLESPACE           // Enable to remove space between throttle symbol and the data
//#define DISPLAY_PR                // Display pitch / roll angles. Requires relevant layout ppositions to be enabled
//#define FULLAHI                   // Enable to display a slightly longer AHI line
//#define REVERSEAHI                // Reverse pitch / roll direction of AHI - for DJI / Eastern bloc OSD users
//#define AHICORRECT 10             // Enable to adjust AHI on display to match horizon. -10 = -1 degree
#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees 
#define AHILEVEL                    // Enable to display AHI level indicators on sidebars 
#define APINDICATOR                 // Enable to display AUTOPILOT instead of RTH distance 
#define GUISENSORS                  // Enable if wish to view raw sensor data on GUI
//#define DISPLAYWATTS              // Enable this to display Watts (if Watts selected in layouts)
//#define DISPLAYEFFICIENCY         // Enable this to display Watts/KMh or Mph for efficiency (if Watts selected in layouts)
//#define LONG_RANGE_DISPLAY        // Enable this to for long range display consolidation - displays distance in KM or feet when exceed 9999m or ft
#define AIRMODE 30                  // Enable this to display BETAFLIGHT airmode icon. Value determines distance in characters between mode icon and airmode icon. 2 = next to it. 30 = below it
//#define CROPGPSPOSITION           // Crop GPS coordinate display to decimals only ".DDDDDDD"

/********************   RC TX Settings     *********************/
//#define MODE1                     // Enable this if wish to use cursor controls on same stick - for MODE 1 TX users
//#define RCCHANNELS 8              // Amend if require up to 16 RC channels

/********************       Airspeed Settings         ************************/
// Completely UNTESTED for future integration of support for airspeed sensor
// Uses temp pin
// Overrides GPS speed
//#define USE_AIRSPEED_SENSOR
#define AIRSPEED_ZERO 512           // AIRSPEED ZERO calibration (0-1024) typically 512 for HK pilot sensor
#define AIRSPEED_CAL  78.125        // Adjusting factor


/********************       NAZA Settings         ************************/
//#define NAZAMODECONTROL           // Enables NAZA mode control display using a PWM signal into OSD RSSI pin. Can be used with OSD_SWITCH_RSSI   
#define NAZA_MODE_GPS 1600
#define NAZA_MODE_ATI 
#define NAZA_MODE_MAN 1400


/********************       Voltage Warning Settings         ************************/
//#define AUTOCELL                  // Uncomment this to use varying cell count batteries. Overrides GUI/OSD voltage warning settings. Uses CELL_VOLTS_* below unless AUTOCELL_ALARM defined
//#define AUTOCELL_ALARM            // Use with Autocell - uses the Main battery Alarm value on GUI OSD instead of CELL_VOLTS_WARN. Main battery Alarm is a per cell value instead of full battery. i.e. 3.4 = 10.2v on a 3s 
//The following variables are available for adjustment unless using FC_VOLTAGE_CONFIG 
#define CELL_VOLTS_WARN 35          // Specify the cell voltage level at which low voltage warning takes place eg. 35 = 3.5 volts per cell
#define CELL_VOLTS_MIN 34           // Specify the cell voltage at which it is considered empty
#define CELL_VOLTS_MAX 42           // Specify the max normal LIPO cell voltage
//#define FC_VOLTAGE_CONFIG         // Additionally uncomment this if you want to use the vbat voltage config with BASEFLIGHT and CLEANFLIGHT on the flight controller (include: min cell voltage, max cell voltage and warning cell voltage)


/********************       Battery Status Settings         ************************/
// This works in conjunction with the GUI switch "Display Battery Status
// Enable to use a battery icon that indicates capacity remaining dependant upon battery voltage or mAh used. Or both if required.
#define BATTERYICONVOLTS          //Enable to use with voltage as indicator of capacity remaining
//#define BATTERYICONAMPS           //Enable to use with mAh used percentage of AMPHR alarm limit. Warning will now be at 80% of that GUI value


/********************       FrSky S.Port settings      *********************/
//enables data transfer from frsky reciever s.port to osd via multiwii
//requires serial inverter cable & multiwii with s.port code
//Auto detected cell graph from s.port, 16 steps @ 0.05v 
//To show battery voltage from s.port, enable "Use MWii" under "Main Voltage" in GUI
//To show amperage from s.port, enable "Use MWii" under Amperage in GUI
//more details: http://code.google.com/p/scarab-osd/wiki/Frsky_SPort
#define MIN_CELL 320 //Cell Low Flash - No decimal, 3 Digits ie 320 = 3.20v


/********************  TEMPERATURE  settings      *********************/
//#define TEMPSENSOR                // Enable if you have a hardware temperature sensor - e.g. LM35 **UNTESTED**
#define TEMPERATUREMAX 50           // Temperature warning value
#define TEMPZERO 0                  // Temperature Zero calibration (range = 0-1024 :512 = 2.5v with vref of 5v and 0.55v for vref of 1.1v) 
#define TEMPMAX  500                // Temperature when at sensor output at VCC. Might be  atheoreticla value 


/********************  RECORD CAPTURE  settings      *********************/
// This is used for those who are attempting records to always show the maximum achieved.
// Maximum values (as shown on statistics summary screen will be displayed on line IMMEDAITELY BELOW where current live data is displayed
// It may require layouts to be amended to show data without overwriting other information
//#define SHOW_MAX_SPEED                 // Enable to display MAX speed achieved on line below current speed
//#define SHOW_MAX_ALTITUDE              // Enable to display MAX altitude achieved on line below current altitude


/********************  THROTTLE calibration  settings      *********************/
// This is used for those who want to specify non default throttle calibration values. 
// To use comment out AUTOTHROTTLE and adjusts the maximum and minimum throttle values 
#define AUTOTHROTTLE 
#define HIGHTHROTTLE 1900                // Maximum recognised value for throttle 
#define LOWTHROTTLE  1100                // Minimum recognised value for throttle






