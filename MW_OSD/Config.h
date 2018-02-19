/*--------------------------       MANDATORY configurable parameters      ----------------------------------------------------*/
/*--------------------------       MANDATORY configurable parameters      ----------------------------------------------------*/


/********************       OSD HARDWARE settings      *********************/
//Choose ONLY ONE option:
//#define MINIMOSD                  // Uncomment this if using standard MINIMOSD hardware (default)
//#define MICROMINIMOSD             // Uncomment this if using the MICRO MINIMOSD hardware
//#define AEROMAX                   // Uncomment this if using MWOSD AEROMAX hardware
//#define RTFQV1                    // Uncomment this if using standard RTFQ/Witespy V1.1 OSD, select this to correct for both swapped bat1/bat 2 and to also use alternative resistors / pinouts.  
//#define RTFQMICRO                 // Uncomment this if using micro RTFQ/Witespy Micro Minim OSD, select this to correct for swapped bat1/bat 2.  
//#define RUSHDUINO                 // Uncomment this if using Rushduino
//#define KYLIN250PDB               // Uncomment this if using a Kylin 250 FPV PDB (Using A6 as VOLTAGEPIN)
//#define AIRBOTMICRO               // Uncomment this if using an airbot MicroOSD
//#define ANDROMEDA                 // Uncomment this if using an Andromeda http://www.multiwiicopter.com/
//#define HOLYBROPDB                // Uncomment this if using a HOLYBRO PDB (Using A6 as VOLTAGEPIN) http://www.holybro.com/
//#define IMPULSERC_HELIX           // Uncomment this if using an ImpulseRC integrated OSD/VTX (STILL UNDER DEVELOPMENT PENDING TESTING)
//#define FFPV_INNOVA               // Uncomment this if using a FURIOUS FPV Innova integrated OSD/VTX (STILL UNDER DEVELOPMENT PENDING TESTING)

// NOTE-some boards have swapped bat1/bat2 pins and alternative voltage measuring resistors
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
//#define BETAFLIGHT                // Uncomment this if you are using latest BETAFLIGHT version 3.1 onwards
//#define CLEANFLIGHT               // Uncomment this if you are using latest CLEANFLIGHT version from repository (2.2.0 at time of this MWOSD release)
//#define iNAV                      // Uncomment this if you are using latest iNAV version from repository (1.01 at time of this MWOSD release)
//#define iNAV_KK                   // Uncomment this if you are using AEROMAX OSD and BARO sensor addition with iNAV with KK audio vario
//#define APM                       // Uncomment this if you are using Ardupilot on APM / PIXHAWK / other supported hardware. Supports most MAVLINK 1.0 compatible FC
//#define PX4                       // Uncomment this if you are using PX4 stack on PIXHAWK and other supported hardware
//#define BASEFLIGHT                // Uncomment this if you are using latest BASEFLIGHT version from repository (Stable 2015.08.27 at time of this MWOSD release)
//#define MULTIWII                  // Uncomment this if you are using latest 2.4 MULTIWII
//#define MAHOWII                   // Uncomment this if you are using MAHOWII (https://github.com/mahowik/mahowii)
//#define KISS                      // Uncomment this if you are using KISS FC
//#define DRONIN                    // Uncomment this if you are using the latest DRONIN MSP Module
//#define NAZA                      // Uncomment this if you are using NAZA flight controller
//#define LIBREPILOT                // Uncomment this if you are using the latest LibrePilot MSP Module
//#define TAULABS                   // Uncomment this if you are using the latest Tau Labs MSP Module
//#define FIXEDWING_BF              // Uncomment this if you are using fixed wing Baseflight 
//#define FIXEDWING_BF_SERVO        // Uncomment this if you are using fixed wing Baseflight with additional SERVO adjustment menu.
//#define HARAKIRI                  // Uncomment this if you are using HARAKIRI (for BOXNAMES compatibility)
//#define RACEFLIGHT                // Uncomment this if you are using RACEFLIGHT - untested. Test and feedback required
//#define SKYTRACK                  // Under development
//#define GPSOSD_UBLOX              // Uncomment this if you are using a UBLOX GPS module for a GPS based OSD
//#define GPSOSD_UBLOX_KK           // Uncomment this if you are using AEROMAX OSD and BARO sensor addition with UBLOX GPS module and KK audio vario
//#define GPSOSD_NMEA               // Uncomment this if you are using a NMEA compatible GPS module for a GPS based OSD
//#define GPSOSD_MTK                // Uncomment this if you are using a MTK module for a GPS based OSD
//#define NOCONTROLLER              // Uncomment this if you have nothing connected to the serial port - no controller or GPS module
// old releases supported...
//#define MULTIWII_V23              // Uncomment this if you are using MW versions 2.2/2.3  
//#define MULTIWII_V21              // Uncomment this if you are using MW versions 2.0/2.1  (for BOXNAMES compatibility)
//#define BASEFLIGHT20150327        // Uncomment this if you are using BASEFLIGHT up to and including version Stable 2015.03.27
//#define CLEANFLIGHT190            // Uncomment this if you are using CLEANFLIGHT versions 1.9 through 1.14.2
//#define CLEANFLIGHT172            // Uncomment this if you are using CLEANFLIGHT versions up to and including 1.7.2
//#define CLEANFLIGHT180            // Uncomment this if you are using CLEANFLIGHT versions 1.8.0 & 1.8.1 
//#define BETAFLIGHT3               // Uncomment this if you are using BETAFLIGHT versions prior to 3.1


/********************       AIRCRAFT/INSTALLATION TYPE settings      *********************/
//Choose ONLY ONE option:
//#define ROTORCRAFT                // Default for multirotors etc.
//#define FIXEDWING                 // Uncomment this if you are using fixed wing with MultiWii or Baseflight 


/********************       Debug      *********************/
#define MENU_DEBUG                  // Enable to display debug values in OSD menu 

/*--------------------------       INITIALISATION options       ----------------------------------------------------*/
/*--------------------------       INITIALISATION options       ----------------------------------------------------*/
// Ignore this section unless you know you need to use it !!
// This section contains initialisation options that only require to be run once.
// Once the initialisation has completed, all sections should be commented and the sketch re-uploaded.
// Font upload will take 90 seconds after upload is completed. If connected to a camera, you will see teh font table displayed.

//#define EEPROM_CLEAR              // Uncomment to force a wipe and reload of default settings at each OSD start. Same as EEPROM_CLEAR sketch.  
//#define LOADFONT_DEFAULT          // Uncomment to force an upload of default font instead of using GUI
//#define LOADFONT_LARGE            // Uncomment to force an upload of large font instead of using GUI
//#define LOADFONT_BOLD             // Uncomment to force an upload of bold font instead of using GUI
//#define DISPLAYFONTS              // Uncomment to display installed fonts for testing



/*--------------------------       OPTIONAL configurable parameters      ----------------------------------------------------*/
/*--------------------------       OPTIONAL configurable parameters      ----------------------------------------------------*/


/********************       FEATURES      *********************/
// Disable features if you require memory for other features
// Further configuration may be require elsewhere in config.h + option enabled on GUI
#define HORIZON                     // Enable/disable HORIZON indicator
#define MAPMODE                     // Enable/disable MAP MODE - map indication of relative positions of home and aircraft
//#define SBDIRECTION               // Enable/disable sidebar indicators (icons indicationg changes in speed or altitude)
//#define GPSTIME                   // Enable/disable GPS Time functions
//#define SPORT                     // Enable/disable FRSKY S.PORT cell code

//#define CANVAS_SUPPORT            // Enable CANVAS mode support for post betaflight 3.1.0 CMS
//#define INVERTED_CHAR_SUPPORT     // Enable inverted char support


/********************       TELEMETRY settings      *********************/
//Select ONLY if you are sure your OSD is connected to a telemetry feed such as MAVLINK/LTM:
//#define RESETHOMEARMED            // Uncomment this ONLY if armed information is sent within telemetry feeed AND you do not want to reset home position when re-arming. DO NOT DISARM IN FLIGHT 
//#define FORCE_MSP                 // Uncomment to enable use of MSP as well as telemetry. Uses more memory 
//#define PROTOCOL_LTM              // To use LTM protocol instead of MSP


/********************       GPS OSD settings (GPS / NAZA)      *********************/
#define GPSHOMEFIX         10       // Number of consecutive seconds of valid fixes before home will be set. Max 32 
#define HOMESATFIX          6       // Minimum number of sats required when setting initial home location. 
#define GPSOSDARMDISTANCE  20       // distance from home in meters when GPSOSD arms. Starts flight timer etc.
//#define GPSOSDHOMEDISTANCE 40     // distance from home in meters when GPSOSD is home. When speed is low it disarms and displays summary screen.
#define GPSOSDLANDED        8       // seconds OSD needs to have landed (within home distance and speed less than xxx) before triggering summary. Max 32
#define GPSDOP 500                  // UBLOX only - minimum PDOP required for sat fix. A DOP of 2.00 is considered good 
//#define PPM_CONTROL               // Enables full OSD menu, screen switching, RSSI, Throttle feature, virtual current sensor, etc using a PPM signal into OSD PWM input pin (RSSI pin on basic MINIM hardware). Requires TX type to be set below. 
//#define PWM_OSD_SWITCH            // Enables 3 way screen switch from a RX PWM channel connected to the OSD PWM input pin (RSSI pin on basic MINIM hardware). AEROMAX OSD has a dedicated connection.
//#define PWM_THROTTLE              // Enables throttle feature, virtual current sensor using RC throttle connected into OSD PWM input pin (RSSI pin on basic MINIM hardware). AEROMAX OSD has a dedicated connection with this enabled by default. 


/********************       FILTER settings      *********************/
//Choose ONLY ONE option to enable filtered smoother readings of voltage / current / RSSI :
#define FILTER_AVG                  // Enable standard averaging filter  
//#define FILTER_HYSTERYSIS 2       // Alternative filter with hysteris to provide smoother changes. Higher number = more filtering. Max 4  


/********************       GPS settings      *********************/
#define MINSATFIX 5                 // Number of sats required for a fix. 5 minimum. More = better.


/********************       ALARM/STATUS settings      *********************/
#define ALARM_VOLTAGE               // Text alerts if voltage below voltage alarm - in addition to flashing voltage indicator
#define ALARM_SATS                  // Text alerts if sats below MINSATFIX - in addition to flashing sat indicator
//#define ALARM_GPS 5               // Text alerts if no GPS data for more than x secs. Sets GPS sats to zero
#define ALARM_MSP 5                 // Text alerts if no Flight controller data for more than x secs. 


/********************       AIRCRAFT type=FIXEDWING settings      *********************/
// **ONLY** valid when using fixed wing
//#define DISABLEGPSALTITUDERESET   // Disables automatic reset of GPS Altitude to zero at arm for FC that already provide this functionality. 
//#define AUTOSENSEMAG              // Undefine this to force autodetect MAG (to use MAG or GPS data for heading - otherwise use GUI). MSP data FC only. 
//#define AUTOSENSEBARO             // Undefine this to force autodetect BARO(to use BARO or GPS data for altitude - otherwise use GUI). MSP data FC only.  
#define USEGLIDESCOPE               // Undefine if not used to save memory. ILS glidescope can be enabled / disabled on GUI.
#define GLIDEANGLE  80              // ILS glidescope angle where 40 = 4.0° requires enabling in layouts. Auto enabled for FIXEDWING
#define GLIDEWINDOW 40              // ILS glidescope angle where Window of 40 = 4.0° - 1.0 deg scope gradients, 80 =  2.0 deg scope gradients. Requires enabling in layouts. Auto enabled for FIXEDWING


/******************** Serial speed settings *********************/
// Overides defaults if required (57.6k for MAVLINK based or 115k for all others). 
//#define BAUDRATE 115200
//#define BAUDRATE 57600
//#define BAUDRATE 38400
//#define BAUDRATE 19200
//#define BAUDRATE 9600


/******************** Mavlink settings *********************/
//#define MAVLINKREQ                // Enable this for mavlink systems where the Mavlink data requires requesting. 
#define MAV_COM_ID 1                // Component ID of MAV. Change if required. 
//#define MAV_ALL                   // To act on data from all MAV SYSID in stream. NOT recommended. Specify ID in GUI. Default=1 upon reset.
//#define MAV_ARMED                 // Forces OSD to be always armed (for when MAV does not send armed status in heartbeat).
//#define MAV_RESET_HOME            // Resets home position when not armed. When enabled, note that RX glitch etc. could potentially reset home position.
//#define MAV_ALT_THROTTLE          // Use alternative MAV throttle value. Not raw RC channel

/******************** Serial MSP speed settings *********************/
// Choose ONLY ONE SPEED option: increases speeds of serial update - but with impact to flight controller 
//#define MSP_SPEED_LOW             // Enable for soft serial / slow baud rates.
#define MSP_SPEED_MED               // Default
//#define MSP_SPEED_HIGH            // Enable for faster AHI and speed updates. Requires higher baud rates and increases overhead on the FC to process
#define MSP_USE_BARO                // Disable if not used to increase serial speed update for MSP based FC: Baro IC data such as  altitude
#define MSP_USE_GPS                 // Disable if not used to increase serial speed update for MSP based FC: GPS data such as speed , distance
#define MSP_USE_ANALOG              // Disable if not used to increase serial speed update for MSP based FC: Voltage, Amperage or RSSI from the FC


/********************       CALLSIGN settings      *********************/
#define   CALLSIGNINTERVAL 60       // How frequently to display Callsign (in seconds) 
#define   CALLSIGNDURATION 4        // How long to display Callsign (in seconds)
//#define FREETEXTLLIGHTS           // Alternative option - enable to display freetext (or callsign) when LLIGHTS Switch active on TX.
//#define FREETEXTGIMBAL            // Alternative option - enable to display freetext (or callsign) when GIMBAL Switch active on TX.

/********************       STARTUP settings      *********************/
//#define INTRO_VERSION               "MWOSD" // Call the OSD something else if you prefer. 
#define INTRO_MENU                  // Enable to display TX stick MENU 
#define INTRO_CALLSIGN              // Enable to display callsign at startup
#define INTRO_SIGNALTYPE            // Enable to display video type at startup
//#define INTRO_FC                  // Enable to display FC version at startup
#define INTRO_DELAY 5               // Seconds intro screen should show for. Default is 5 
//#define STARTUPDELAY 500          // Enable alternative startup delay (in ms) to allow MAX chip voltage to rise fully and initialise before configuring. Deafult = 1000 


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
#define MINSUMMARY                  // Hide summary screen values that are 0
//#define FASTPIXEL                 // Optional - may improve resolution - especially hi res cams
//#define WHITEBRIGHTNESS 0x01      // Optional change from default 0x00=120%,0x01=100%,0x10=90%,0x11=80%  default is 0x01=100%
//#define BLACKBRIGHTNESS 0x00      // Optional change from default 0x00=0%,0x01=10%,0x10=20%0x11=30%  default is 0x00=0%
//#define I2CERROR 3                // Autodisplay Mutltiwii I2C errors if exceeds specified count 
//#define NOTHROTTLESPACE           // Enable to remove space between throttle symbol and the data
#define DISPLAY_PR                  // Display pitch / roll angles. Requires relevant layout ppositions to be enabled
//#define REVERSE_AHI_PITCH         // Reverse pitch / roll direction of AHI - for DJI / Eastern bloc OSD users
//#define REVERSE_AHI_ROLL          // Reverse pitch / roll direction of AHI - for DJI / Eastern bloc OSD users
//#define AHICORRECT 10             // Enable to adjust AHI on display to match horizon. -10 = -1 degree
//#define INVERT_PITCH_SIGN         // Invert the sign of the displayed numeric value for the pitch angle (ex: pitch up = positive )
//#define INVERT_ROLL_SIGN          // Invert the sign of the displayed numeric value for the roll angle (ex: roll right = negative )
//#define AHIINVERTSUPPORT          // Support for inverted flight. AHI flow terrain when inverted
//#define FULLAHI                   // Enable to display a slightly longer AHI line
//#define SECONDARYAHI              // Enable to display secondary AHI lines
#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees 
//#define AHIPITCHSCALE 100         // Specify scaling sensitvity for Pitch. Higher number = pitches more on OSD         
//#define AHIROLLSCALE 100          // Specify scaling sensitvity for Roll. Higher number = rolls more on OSD         
#define AHILEVEL                    // Enable to display AHI level indicators on sidebars 
#define APINDICATOR                 // Enable to display AUTOPILOT instead of RTH distance 
#define GUISENSORS                  // Enable if wish to view raw sensor data on GUI
//#define LONG_RANGE_DISPLAY        // Enable this to for long range display consolidation - displays distance in KM or feet when exceed 9999m or ft. Auto enabled for FIXEDWING
#define AIRMODE                     // Enable this to display airmode icon. 
//#define CROPGPSPOSITION           // Crop GPS coordinate display to decimals only ".DDDDDDD"
#define MASKGPSLOCATION             // Disable to save memeory if not used. Enables MASK GPS settings on GUI. Coordinates displayed with major digits XXX set to random location "XXX.DDDDDDD" 
//#define TEXTMODE                  // Enable to display the flyingmode as text instead of symbols
//#define OSD_SWITCH                // Forces original 2 way multiwii screen switch using OSD Switch via Flight Controller. MUST Ensure enabled on flight controller - e.g. #define OSD_SWITCH on multiwii
//#define NOSUMMARYTHROTTLERESET    // Enable to supress summary display clearing from throttle
#define OSDSUMMARY 30               // Seconds summary is displayed for after landing. Max 254


/********************       Power / efficiency display Settings         ************************/
#define DISPLAYWATTS                // Disable to save memeory if not used. Enable this to display Watts
#define DISPLAYEFFICIENCY           // Disable to save memeory if not used. Enable this to display Watts/KMh or Mph for efficiency
#define DISPLAYMAHMIN               // Disable to save memeory if not used. Enable this to display average mAh/minKMh


/********************       Visual Vario / climbrate Settings         ************************/
//#define VARIOSTANDARD             // Enable this for single icon representation of vario. Less memory.
#define VARIOENHANCED               // Enable this for multi line more accurate visual slider representation of vario. Configurable from GUI
//#define VARIOSCALE 200            // Scale used for Vario - 200 =2.00 m/s. Multirotor defualt = 200, Plane = 300

/********************       FC BARO Audio Vario / climbrate Settings         ************************/
// A basic Audio Vario using vario data from FC 
// Uses simple circuit added to OSD
// Wiki: https://github.com/ShikOfTheRa/scarab-osd/wiki/Audio-Vario
//#define AUDIOVARIO AUDIOPIN         // Enable this for audio vario. AUDIOPIN = D2 on AEROMAX hardware. Alternatively use A3 (RSSI) with other hardware  
//#define AUDIOVARIOSWITCH            // Enable this to use screen layouts to enable/disable vario. If visual vario is displayed, the audio vario is on
#define AUDIOVARIOTHRESHOLDCLIMB  10  // Threshold for climbing (cm/s)
#define AUDIOVARIOTHRESHOLDSINK  -20  // Threshold for sinking  (cm/s) 
#define AUDIOVARIOSILENTDEADBAND      // Enable for silent deadband otherwise will hear near thermal tone


/********************       KK Audio Vario / climbrate Settings         ************************/
// A highly accurate / sensitive Audio Vario from KapeteinKuk
// Uses MS5611 pressure sensor connected by I2C and simple circuit added to OSD
// Only supported by VIRTUALPILOT AEROMAX hardware unless able to carry out very fine soldering. Refer to links below 
// Wiki:  https://github.com/ShikOfTheRa/scarab-osd/wiki/Audio-Vario
// Hardware: https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/VIRTUALPILOT%20AEROMAX/VP%20AEROMAX%20with%20I2C%20BARO%20VARIO.jpg 
//#define KKAUDIOVARIO AUDIOPIN     // Enable this for audio vario. AUDIOPIN = D2 on AEROMAX hardware. Alternatively use A3 (RSSI) with other hardware  
//#define AUDIOVARIOSWITCH          // Enable this to use screen layouts to enable/disable vario. If visual vario is displayed, the audio vario is on
#define KKDEADBANDLOW  -25          // Deadband threshold for sinking (Set to -500 to disable sink tones). 0 for no deadband
#define KKDEADBANDHIGH  15          // Deadband threshold for climbing. 0 for no deadband 


/********************   RC TX Settings     *********************/
// R=Roll, P=Pitch, Y=Yaw, T=Throttle
//#define TX_MODE1                  // Enable this if wish to use cursor controls on same stick - for MODE 1 TX users
//#define TX_CHANNELS 8             // Amend if require up to 16 RC channels (APM/PX4/MAVLINK are 16 by default)
//#define TX_PYTR                   // Enable for Robe/Hitec/Futaba (Only use for GPSOSD/NAZA/APM/PX4/MAVLINK)
//#define TX_RPTY                   // Enable for Graupner/Spektrum (Only use for GPSOSD/NAZA/APM/PX4/MAVLINK)    
//#define TX_RPYT                   // Enable for Multiplex (Only use for GPSOSD/NAZA/APM/PX4/MAVLINK)
//#define TX_PRTY                   // Enable for Hitec/Sanwa (Only use for GPSOSD/NAZA/APM/PX4/MAVLINK)
//#define TX_TRPY                   // Enable for JR (Only use for GPSOSD/NAZA/APM/PX4/MAVLINK)
#define TX_CHAN_MID 1400            // Value for determining RC SWITCH LOW / MID transsition
#define TX_CHAN_HIGH 1600           // Value for determining RC SWITCH MID / HIGH transition


/********************       Airspeed Sensor         ************************/
// UNTESTED in flight. Uses MPXV700DP sensor connected to AUXPIN A6  (Aeromax OSD hardware only)
//#define USE_AIRSPEED_SENSOR


/********************       NAZA Settings         ************************/
// Note: PWM_OSD_SWITCH can be used in conjunction with NAZAMODECONTROL to display different layout in different modes
//#define NAZAMODECONTROL           // Enables NAZA mode control display using a PWM signal into OSD PWM pin.   
#define NAZA_MODE_HIGH 3            // NAZA mode TX low: 0=FAIL, 1= MANU, 2=ATTI, 3=GPS ATTI
#define NAZA_MODE_MED  2            // NAZA mode TX low: 0=FAIL, 1= MANU, 2=ATTI, 3=GPS ATTI
#define NAZA_MODE_LOW  1            // NAZA mode TX low: 0=FAIL, 1= MANU, 2=ATTI, 3=GPS ATTI


/********************       Voltage Warning Settings         ************************/
//The following variables are available for adjustment of battery icon only 
#define CELL_VOLTS_MIN 32           // Specify the cell voltage at which it is considered empty. Used for battery guage icon only
#define CELL_VOLTS_MAX 42           // Specify the max normal LIPO cell voltage. Used for auto cell count determination and battery guage icon
//#define FC_VOLTAGE_CONFIG         // Additionally uncomment this if you want to use the vbat voltage config with BASEFLIGHT, CLEANFLIGHT and BETAFLIGHT on the flight controller (include: min cell voltage, max cell voltage and warning cell voltage)


/********************       Battery Status Settings         ************************/
// This works in conjunction with the GUI switch "Display Battery Status
// Enable to use a battery icon that indicates capacity remaining dependant upon battery voltage or mAh used. Or both if required.
#define BATTERYICONVOLTS            //Enable to use with voltage as indicator of capacity remaining
//#define BATTERYICONAMPS           //Enable to use with mAh used percentage of AMPHR alarm limit. Warning will now be at 80% of that GUI value


/********************       Headtracker support         ************************/
//#define VIRTUAL_NOSE              // Enables the use of a virtual nose for headtracker users where aircraft nose is not visible
#define HTCHANNEL   3               // RC chanel uses ch 1 - 8/16
#define HTSCALE     10              // Scaling of Pan Axis - Max of 10
#define HTLINE      11              // Row on which Headtracker info is displayed
#define HTDIRECTION +               // Reverses direction of pan action


/********************       Vendor support         ************************/
// This is for vendor use only for custom GUI support
#define VENDOR   1                  // For vendor use for custom GUI


/********************  TEMPERATURE  settings      *********************/
#define SHOW_TEMPERATURE            // Enable if you have a hardware temperature sensor - e.g. LM35 **UNTESTED**
#define TEMPERATUREMAX 50           // Temperature warning value
#define TEMPZERO 0                  // Temperature Zero calibration (range = 0-1024 :512 = 2.5v with vref of 5v and 0.55v for vref of 1.1v) 
#define TEMPMAX  500                // Temperature when at sensor output at VCC. Might be  atheoreticla value 


/********************  RECORD CAPTURE  settings      *********************/
// This is used for those who are attempting records to always show the maximum achieved.
// Maximum values (as shown on statistics summary screen will be displayed on line IMMEDAITELY BELOW where current live data is displayed
// It may require layouts to be amended to show data without overwriting other information
#define SHOW_MAX_SPEED              // Enable to display MAX speed achieved
//#define SHOW_MAX_ALTITUDE         // Enable to display MAX altitude achieved on line below current altitude
#define SHOW_MAX_DISTANCE           // Enable to display MAX distance achieved
#define SHOW_TOTAL_DISTANCE         // Enable to display TOTAL distance achieved


/********************  THROTTLE calibration  settings      *********************/
// This is used for those who want to specify non default throttle calibration values. 
// To use comment out AUTOTHROTTLE and adjusts the maximum and minimum throttle values 
#define AUTOTHROTTLE 
#define HIGHTHROTTLE 1900           // Maximum recognised value for throttle 
#define LOWTHROTTLE  1100           // Minimum recognised value for throttle


/********************  AMPERAGE calibration  settings      *********************/
// This is used for those who want to specify non default amperage calibration values. 
// These should only be used if unable to calibrate within limits on GUI. 
#define AMPCALLOW  0                     
#define AMPCALHIGH 1023                  


/********************           VTX settings           *********************/
// Regional RF frequency regulations: Choose ONLY ONE option:
#define VTX_REGION_UNRESTRICTED     // Enable for all 40 channels
//#define VTX_REGION_AUSTRALIA      // Enable for AU legal channels and power level only
#define DISPLAY_VTX_INFO            // Enable to show frequency in menu display. 


/********************           Display items lead icon           *********************/
// comment out to not display lead icon for displayed items
#define ICON_ANGLE_RTH              // Direction to home
#define ICON_ANGLE_HDG              // Heading
#define ICON_DOP                    // GPS SAT DOP
#define ICON_SPEED_GPS              // GPS speed
#define ICON_SPEED_AIR              // Air Speed
#define ICON_MAX                    // Max value - speed/distance/alt
#define ICON_CLIMBRATE              // CLimb rate
#define ICON_EFF                    // Efficiency
#define ICON_PITCH                  // Pitch
#define ICON_ROLL                   // Roll
#define ICON_POWER                  // Power
#define ICON_AVG_EFF                // Average Efficiency
#define ICON_TOTAL                  // Total distance travelled
#define ICON_TMP                    // Temperature
#define ICON_DTH                    // Distance to home
#define ICON_ALT                    // Altitude prime
#define ICON_GPS_ALT                // GPS altitude
#define ICON_MAIN_BATT              // Main battery icon
#define ICON_VID_BAT                // Video battery icon
#define ICON_RSSI                   // RSSI
#define ICON_SAT                    // Sattelite enable for large icon, disable for small icon


/********************  Advanced parameters  settings      *********************/
// This is to enable rarely used advanced parameter saving. Off by default to minimise risk 
// Enabling may create ability to write invalid data to FC
//#define ADVANCEDSAVE              // Enables saving of advanced paramters where supported (Looptime/profile)
//#define FIXEDLOOP                 // Enables 1ms loop max for consistency


