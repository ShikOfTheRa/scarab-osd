/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/

/*----------------------------------------------       Developer / debug parameters      ----------------------------------------------------*/

// Display Debug screen display options
//#define DEBUGMW            // Enable to display MSP debug values (assumes debug[x] values are not set elsewhere) 
#define DEBUGDPOSMENU 3      // dispaly debug title at position X
#define DEBUGDPOSRCDATA 33   // display RCDATA values at position X
#define DEBUGDPOSANAL 84     // display sensor values at position X
#define DEBUGDPOSPWM 264     // display PWM values at position X
#define DEBUGDPOSVAL 40      // display debug values at position X
#define DEBUGDPOSLOOP 160    // display loop rate value at position X
#define DEBUGDPOSSAT 190     // display sat value at position X
#define DEBUGDPOSARMED 250   // display armed value at position X
#define DEBUGDPOSPACKET 280  // display serial packet rate rate value at position X
#define DEBUGDPOSMEMORY 310  // display free heap/stack memory at position X. Requires MEMCHECK and not valid in latest Arduino versions
#define DEBUGDPOSRX 220      // display serial data rate at position X
//#define DEBUGDPOSMSPID 33  // display MSP ID received

//#define DEBUG 4                   // Enable/disable option to display OSD debug values. Define which OSD switch position to show debug on screen display 0 (default), 1 or 2. 4 for always on

// Display Debug text message in standard screen text warning message area
// Enable and set debugtext=1 in code when required 
//#define DEBUGTEXT "DEBUG"    // Set text you wish to display when debug text message required. Must be CAPSLOCK text

#define HARDRESET            // Enables watchdog timer reset rather than fixed memory jmp 
//#define BOOTRESET          // Enables reset from default Atmega 328 bootloader address (instead of 0) 


/********************           Under development           *********************/
// development and test
//#define PILOTICON                 // Enable code to display pilot ICON as an alternative to CHARACTER display. Requires GUI > 1.8.0
//#define MAV_ADSB                  // Use Baro altitude from mavlink instead of GPS. Requires ADSB data to be configured in mavlink.


//#define DEVELOPMENT               // For development set only 
#ifdef DEVELOPMENT                  // Development pre-set test paramters only 
//#define DISPLAYAVGEFFICIENCY                // Display average mAh used / per KM instead of mAh/min KMh. 

  //#define DEBUG 4                 // Enable/disable option to display OSD debug values. Define which OSD switch position to show debug on screen display 0 (default), 1 or 2. 4 for always on
  //#define AIRBOTMICRO             // Uncomment this if using an airbot MicroOSD
  //#define EEPROMVER 16              // for eeprom layout verification
  #define AEROMAX                 // Uncomment this if using MWOSD AEROMAX hardware
  //#define MINIMOSD                  // Uncomment this if using standard MINIMOSD hardware (default)
  //#define GPSOSD_NMEA             // Uncomment this if you are using a NMEA compatible GPS module for a GPS based OSD
  //#define GPSOSD_UBLOX            // Uncomment this if you are using a UBLOX GPS module for a GPS based OSD
  //#define BETAFLIGHT                // Uncomment this if you are using latest BETAFLIGHT version 3.1 onwards
  //#define APM
  //#define PX4                     // Uncomment this if you are using PIXHAWK with PX4 stack
  #define iNAV                    // Uncomment this if you are using latest iNAV version from repository (1.01 at time of this MWOSD release)
  //#define FIXEDWING               // Uncomment this if you are using fixed wing with MultiWii or Baseflight
  //#define EEPROM_CLEAR            // Uncomment to force a wipe and reload of default settings at each OSD start. Same as EEPROM_CLEAR sketch.  
  //#define INTRO_DELAY 1           // To speed up startup
  //#define DISPLAY_DEV 0xC000        // Use screen layout dev position - display all items...
  //#define KKAUDIOVARIO A3         // Enable this for audio vario on Arduino pin XX. A3=RSSI. Use AUDIOPIN on AEROMAX 
  //#define MAVSENSOR173
  //#define MSPV2  
  //#define USE_AIRSPEED_SENSOR
  //#undef GPSTIME
  //#define MAV_ALT_THROTTLE          // Use alternative MAV throttle value. Not raw RC channel.
  #define TX_CHANNELS 16 
  
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
//#define NOCONTROLLER             

#endif


//#define TIMETEST
#ifdef TIMETEST
  //#define DEBUG 4                 // Enable/disable option to display OSD debug values. Define which OSD switch position to show debug on screen display 0 (default), 1 or 2. 4 for always on
  #define DISPLAY_DEV 0xC000        // Use screen layout dev position - display all items...
  #define MSP_RTC_SUPPORT           // Enable to set RTC time via MSP
  #define GPSTIME                   // Enable to use GPS time display functions
  #define DATEFORMAT_UTC            // Display UTC date when enabled - do not use time zone settings
  //#define MENU_GPS_TIME             // Enable GPS time adjustments in OSD menu
  #define AEROMAX                 // Uncomment this if using MWOSD AEROMAX hardware
  //#define GPSOSD_NMEA             // Uncomment this if you are using a NMEA compatible GPS module for a GPS based OSD
  //#define GPSOSD_UBLOX            // Uncomment this if you are using a UBLOX GPS module for a GPS based OSD
  //#define APM
  //#define PX4                     // Uncomment this if you are using PIXHAWK with PX4 stack
  #define iNAV                    // Uncomment this if you are using latest iNAV version from repository (1.01 at time of this MWOSD release)
  //#define FIXEDWING               // Uncomment this if you are using fixed wing with MultiWii or Baseflight
#endif
//

/*--------------------------       DEPRECATED parameters for reference only      ----------------------------------------------------*/


/********************       OSD SCREEN SWITCH settings      *********************/
#define OSD_SWITCH_RC               // Enables 3 way screen switch using a TX channel via FC. Specify channel on GUI (range 1-16 AUX1=5 AUX4=8)
#ifdef  OSD_SWITCH
  #undef OSD_SWITCH_RC
#endif



/********************  HARDWARE rule definitions  **********************/

// VTX_RTC6705 Support for RTC6705 based VTX
// VTX_RC      Support for SINGULARITY-style direct RC stick command
// VTX_LED     Support for dedicated single status LED (not a LED strip support)
// MENU_VTX    Support for VTX page in MWOSD MENU

#ifdef IMPULSERC_HELIX
//  #define RUSHDUINO
  #define VTX_RTC6705
  #define VTX_RC
  #define VTX_LED
  #define USE_MENU_VTX
#endif

#ifdef FFPV_INNOVA
  #define MINIMOSD
  #define VTX_RTC6705
  //#define VTX_RC    // Can be turned on, hard to use without VTX_LED
  //#define VTX_LED   // Needs VTXLED_* definitions in RTC6705.ino
  #define USE_MENU_VTX
#endif




/********************  CONTROLLER rule definitions  **********************/

#ifdef MULTIWII       //set up latest at time of release
  #define MULTIWII_V24
#endif

#ifdef MAHOWII       //set up latest at time of release
  #define MULTIWII_V24
#endif

#ifdef BETAFLIGHT    //set up latest at time of release
  #define BETAFLIGHT31
#endif

#ifdef TAULABS    //set up latest at time of release
//  #define TAULABS
#endif

#ifdef DRONIN    //set up latest at time of release
  #define TAULABS
#endif

#ifdef LIBREPILOT
  #define TAULABS
  #define USE_MSP_PIDNAMES
#endif

#ifdef CLEANFLIGHT    //set up latest at time of release
  #define BETAFLIGHT31
#endif

#ifdef iNAV    //set up latest at time of release
//  #define iNAV
  #define MSPV2
#endif

#ifdef iNAV_KK // iNAV with KK VARIO
  #define iNAV
  #define KKAUDIOVARIO AUDIOPIN     // Enable this for audio vario. AUDIOPIN = D2 on AEROMAX hardware. Alternatively use A3 (RSSI) with other hardware  
  #undef MAPMODE
  #undef MENU_DEBUG
//  #undef DISPLAY_PR
  #undef SHOW_TEMPERATURE
  #undef INTRO_MENU                  // Enable to display TX stick MENU 
  #undef INTRO_CALLSIGN              // Enable to display callsign at startup
  #undef INTRO_SIGNALTYPE            // Enable to display video type at startup
  #undef INTRO_FC                    // Enable to display FC version at startup
  #undef MENU_DEBUG                  // Enable to display debug values in OSD menu 

#endif

#ifdef SUBMERSIBLE // MSP FC with depth sensor
//  #define iNAV
  #define FORCESENSORS
  #define USEMS5837                    // Enable this for BAR30  
  #undef MAPMODE
  #undef MENU_DEBUG
  #undef INTRO_MENU                  // Enable to display TX stick MENU 
  //#undef INTRO_CALLSIGN              // Enable to display callsign at startup
  //#undef INTRO_SIGNALTYPE            // Enable to display video type at startup
  #undef INTRO_FC                    // Enable to display FC version at startup
  #undef DISPLAYWATTS                // Disable to save memeory if not used. Enable this to display Watts
  #undef DISPLAYEFFICIENCY           // Disable to save memeory if not used. Enable this to display Watts/KMh or Mph for efficiency
  #undef DISPLAYMAHMIN               // Disable to save memeory if not used. Enable this to display average mAh/minKMh
  #undef PILOTICON                   // Enable code to display pilot ICON as an alternative to CHARACTER display. Requires GUI > 1.8.0
  #undef MASKGPSLOCATION             // Disable to save memeory if not used. Enables MASK GPS settings on GUI. Coordinates displayed with major digits XXX set to random location "XXX.DDDDDDD" 
  #undef FILTER_AVG                  // Enable standard averaging filter  
  //#undef HORIZON                     // Enable/disable HORIZON indicator
  //#undef DISPLAY_PR
  //#undef SHOW_TEMPERATURE
  
#endif

#ifdef BASEFLIGHT     //set up latest at time of release
  #define BASEFLIGHT20150627
#endif
  
#ifdef RACEFLIGHT     //set up latest at time of release
#endif

#ifdef FIXEDWING_BF     //set up latest at time of release
  // based upon these..
  // #define BASEFLIGHT20150627
  // #define FIXEDWING
#endif

#ifdef FIXEDWING_BF_SERVO //set up latest at time of release
  // based upon these..
  // #define BASEFLIGHT20150627
  // #define FIXEDWING
#endif

#ifdef HARAKIRI
  #define BOXNAMES
  #define MULTIWII_V24
#endif

#ifdef PX4   //set up latest at time of release
  #define APM
  #define MAVLINKREQ
#endif

#ifdef APM       //set up latest at time of release
#endif

#ifdef KISS      //set up latest at time of release
#endif

#ifdef SKYTRACK  //set up latest at time of release
#endif

// The unit of current varies across implementations.  There are effectively three set:
// * 100mA, for which case the value is usable as it comes aross the wire. default
// * 10mA, which sends a value 10x higher than we work wth
// * 1ma, which sends a value 100x higher than normal

//CORRECT_MSP_BF1 introduced BF201506 - adds seperate Pitch/Roll/Yaw + TPA     (MSP support)
//CORRECT_MENU_RCT1 introduced CF180/BF201506 - adds seperate Pitch/Roll/Yaw + TPA     (Menu Support)
//CORRECT_MENU_RCT2 introduced CF190 - adds seperate Pitch/Roll/Yaw + TPA + Yaw expo     (Menu support)
//ENABLE_MSP_SAVE_ADVANCED - adds the code to read/write PROFILE+LOOPIME+PID CONTROLLER if supported
//CORRECTLOOPTIME show looptime option in Adavanced tuning menu

#if defined BETAFLIGHT3
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_2RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define MENU_PROFILE
#endif

#if defined BETAFLIGHT31
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED
  #define CANVAS_SUPPORT
  #ifdef GPSTIME
    #define MSP_RTC_SUPPORT
  #endif

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_2RC
  #define MENU_INFO
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define MENU_PROFILE
#endif

#if defined CLEANFLIGHT190
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED
  #define CORRECTLOOPTIME
  #ifdef GPSTIME
    #define MSP_RTC_SUPPORT
  #endif

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_2RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define MENU_PROFILE
#endif

#if defined iNAV // same as CLEANFLIGHT190 + CMS
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED
  #define CORRECTLOOPTIME
  #define CANVAS_SUPPORT
  #define MSP_DOP_SUPPORT
  #define MAV_COMP_ALL 
  #ifdef GPSTIME
    #define MSP_RTC_SUPPORT
  #endif
  #define TX_CHANNELS 16 
  
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_2RC
  #define MENU_VOLTAGE
  #ifndef iNAV_KK
    #define MENU_PROFILE
    #define MENU_RSSI
    #define MENU_CURRENT
    #define MENU_DISPLAY
    #define MENU_ADVANCED
    #define MENU_ALARMS  
  #endif  
#endif

#if defined CLEANFLIGHT180
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF1
  #define CORRECT_MENU_RCT1

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

#if defined CLEANFLIGHT172
  #define AMPERAGE_DIV  10
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

#if defined FIXEDWING_BF
  #define FIXEDWING
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_FIXEDWING
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define MENU_PROFILE
#endif

#if defined FIXEDWING_BF_SERVO
  #define FIXEDWING
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_SERVO
  #define MENU_FIXEDWING
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define MENU_PROFILE
#endif

#if defined BASEFLIGHT20150627
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define MENU_PROFILE
#endif

#if defined (BASEFLIGHT20150327)
  #define AMPERAGE_DIV  10
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

#if defined (RACEFLIGHT)
  #define AMPERAGE_DIV  10
  #define MENU_STAT
  #define MENU_PID
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

#if defined (MULTIWII_V24)
  #define AMPERAGE_DIV  1
  #undef INTRO_FC
  #ifdef GPSTIME
    #define MSP_RTC_SUPPORT
  #endif
  
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_GPS_TIME
  #define MENU_ALARMS
  
#endif

#if defined (MULTIWII_V23)
  #undef INTRO_FC
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS

#endif

#if defined (MULTIWII_V21)
  #define BOXNAMES              // required to support legacy protocol
  #undef INTRO_FC
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

#if defined(TAULABS)
  #define AMPERAGE_DIV  10
  #undef INTRO_FC
  #define HAS_ALARMS
  #define ACROPLUS
  #define MENU_STAT
  #define MENU_PID
  #define MENU_RC
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

#if defined(APM)
  #undef INTRO_FC
  #define RESETHOMEARMED  
  #define TX_GUI_CONTROL 
  #define MAV_RTC
  #define MENU_STAT
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define PROTOCOL_MAVLINK
  #define AMPERAGE_DIV 10
#endif

#if defined(KISS)
  #undef INTRO_FC
  #define MENU_STAT
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
  #define PROTOCOL_KISS
  #define AMPERAGE_DIV 10
#endif

#ifdef SKYTRACK
  #undef INTRO_FC
  #undef  INTRO_MENU
  #undef  ALARM_MSP
  #undef  ALARM_SATS
  #undef  ALARM_GPS
  #undef  OSD_SWITCH_RC
  #define MENU_STAT
  #define PROTOCOL_SKYTRACK
#endif

#ifdef NOCONTROLLER
  #undef INTRO_FC
  #undef  INTRO_MENU
  #undef  ALARM_MSP
  #undef  ALARM_SATS
  #undef  ALARM_GPS
  #undef  OSD_SWITCH_RC
  #undef  ALARM_ARMED
  #define MENU_STAT
#endif

#if defined GPSOSD_UBLOX_KK
  #define UBLOX
  #define KKAUDIOVARIO AUDIOPIN     // Enable this for audio vario. AUDIOPIN = D2 on AEROMAX hardware. Alternatively use A3 (RSSI) with other hardware  
#endif
#if defined GPSOSD_UBLOX
  #define UBLOX
  #ifndef ROTORCRAFT
    #define FIXEDWING
  #endif
#endif
#if defined GPSOSD_NMEA
  #define NMEA
  #ifndef ROTORCRAFT
    #define FIXEDWING
  #endif
#endif
#if defined GPSOSD_MTK
  #define MTK
  #ifndef ROTORCRAFT
    #define FIXEDWING
  #endif
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
  #undef INTRO_FC
  #undef  INTRO_MENU
  #ifndef NAZA
    #undef  ALARM_MSP
  #endif
  #undef  OSD_SWITCH_RC
  #undef  DISPLAY_PR
  #define NOAHI
  #define NOSUMMARYTHROTTLERESET
  #define TX_GUI_CONTROL  
  #ifndef ALARM_GPS
    #define ALARM_GPS 5
  #endif
  #define MENU_STAT
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif

// Flight Controller Software types to be added before here...

#ifndef MENU_STAT // default
  #define INFO_CONTROLLER 0
  #define MENU_STAT
  #define MENU_VOLTAGE
  #define MENU_RSSI
  #define MENU_CURRENT
  #define MENU_DISPLAY
  #define MENU_ADVANCED
  #define MENU_ALARMS
#endif


/*
  #define MENU_STAT          0       //STATISTICS
  #define MENU_PID           1       //PID CONFIG
  #define MENU_RC            2       //RC TUNING
  #define MENU_SERVO         3       //SERVO
  #define MENU_FIXEDWING     4       //FIXEDWING adjustments
  #define MENU_VOLTAGE       5       //VOLTAGE
  #define MENU_RSSI          6       //RSSI
  #define MENU_CURRENT       7       //CURRENT
  #define MENU_DISPLAY       8       //DISPLAY
  #define MENU_ADVANCED      9       //ADVANCED
  #define MENU_ALARMS        10      //ALARMS
  #define MENU_PROFILE       11      //PROFILE+PID CONTROLLER
  #define MENU_DEBUG         12      //DEBUG
*/


#undef MAXPAGE
#define MAXPAGE 0

#ifdef MENU_STAT
  const uint8_t MENU_STAT_tmp = 0;
  #define MENU_STAT MENU_STAT_tmp
  #define MAXPAGE MENU_STAT 
#endif

#ifdef MENU_PID
  const uint8_t MENU_PID_tmp = MAXPAGE+1;
  #define MENU_PID MENU_PID_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_PID 
#endif

#ifdef MENU_RC
  const uint8_t MENU_RC_tmp = MAXPAGE+1;
  #define MENU_RC MENU_RC_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_RC 
#endif

#ifdef MENU_2RC
  const uint8_t MENU_2RC_tmp = MAXPAGE+1;
  #define MENU_2RC MENU_2RC_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_2RC 
#endif

#warning "AMPERAGE_DIV"

#ifdef MENU_SERVO
  const uint8_t MENU_SERVO_tmp = MAXPAGE+1;
  #define MENU_SERVO MENU_SERVO_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_SERVO 
#endif

#ifdef MENU_FIXEDWING
  const uint8_t MENU_FIXEDWING_tmp = MAXPAGE+1;
  #define MENU_FIXEDWING MENU_FIXEDWING_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_FIXEDWING 
#endif

#ifdef MENU_INFO
  const uint8_t MENU_INFO_tmp = MAXPAGE+1;
  #define MENU_INFO MENU_INFO_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_INFO 
#endif

#ifdef MENU_VOLTAGE
  const uint8_t MENU_VOLTAGE_tmp = MAXPAGE+1;
  #define MENU_VOLTAGE MENU_VOLTAGE_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_VOLTAGE 
#endif

#ifdef MENU_RSSI
  const uint8_t MENU_RSSI_tmp = MAXPAGE+1;
  #define MENU_RSSI MENU_RSSI_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_RSSI 
#endif

#ifdef MENU_CURRENT
  const uint8_t MENU_CURRENT_tmp = MAXPAGE+1;
  #define MENU_CURRENT MENU_CURRENT_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_CURRENT 
#endif

#ifdef MENU_DISPLAY
  const uint8_t MENU_DISPLAY_tmp = MAXPAGE+1;
  #define MENU_DISPLAY MENU_DISPLAY_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_DISPLAY 
#endif

#ifdef MENU_ADVANCED
  const uint8_t MENU_ADVANCED_tmp = MAXPAGE+1;
  #define MENU_ADVANCED MENU_ADVANCED_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_ADVANCED 
#endif

#ifdef MENU_GPS_TIME
  const uint8_t MENU_GPS_TIME_tmp = MAXPAGE+1;
  #define MENU_GPS_TIME MENU_GPS_TIME_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_GPS_TIME 
#endif

#ifdef MENU_ALARMS
  const uint8_t MENU_ALARMS_tmp = MAXPAGE+1;
  #define MENU_ALARMS MENU_ALARMS_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_ALARMS 
#endif

#ifdef MENU_PROFILE
  const uint8_t MENU_PROFILE_tmp = MAXPAGE+1;
  #define MENU_PROFILE MENU_PROFILE_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_PROFILE 
#endif

#ifdef MENU_DEBUG
  const uint8_t MENU_DEBUG_tmp = MAXPAGE+1;
  #define MENU_DEBUG MENU_DEBUG_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_DEBUG 
#endif

#ifdef USE_MENU_VTX
  const uint8_t MENU_VTX_tmp = MAXPAGE+1;
  #define MENU_VTX MENU_VTX_tmp
  #undef  MAXPAGE
  #define MAXPAGE MENU_VTX 
#endif

/*
enum {
    EMENU_VTX,
}
*/


#ifdef HAS_ALARMS
  #define MAX_ALARM_LEN 30
#endif

#ifndef AMPERAGE_DIV 
  #define AMPERAGE_DIV 100
#endif

#if defined (FC_VOLTAGE_CONFIG) && (defined (CLEANFLIGHT) || defined(BASEFLIGHT) || defined(BETAFLIGHT))
  #define USE_FC_VOLTS_CONFIG
#endif

/********************  OPTIONS enabled definitions  *********************/
#ifdef USE_MENU_VTX
  #define INFO_OPTIONS_4 1<<4
#else
  #define INFO_OPTIONS_4 0
#endif
#if defined TX_GUI_CONTROL   //PITCH,YAW,THROTTLE,ROLL order controlled by GUI 
  #define INFO_OPTIONS_5 1<<5
#else
  #define INFO_OPTIONS_5 0
#endif
#define INFO_OPTIONS 0|INFO_OPTIONS_4|INFO_OPTIONS_5

/********************  FIXEDWING definitions  *********************/
#ifdef FIXEDWING                     
  //#define LONG_RANGE_DISPLAY
  //#define USEGPSHEADING  // defined in config.h now
  //#define USEGPSALTITUDE // defined in config.h now
  #define FORCESENSORS
  #ifndef USEGLIDESCOPE 
    #define USEGLIDESCOPE
  #endif
  #ifndef VARIOSCALE 
    #define VARIOSCALE 250 
  #endif
#endif

/********************  ROTORCRAFT definitions  *********************/
#ifndef FIXEDWING  
  #ifndef ROTORCRAFT  
    #define ROTORCRAFT 
  #endif  

  #if defined SKYTRACK || defined NOCONTROLLER 
    #undef ROTORCRAFT
  #endif 
#endif                   

#ifdef ROTORCRAFT                       
  #ifndef GPSOSDHOMEDISTANCE
    #define GPSOSDHOMEDISTANCE 15     // distance from home in meters to start check  for when when GPSOSD is home.
  #endif
#endif


// All aircraft / FC types defaults...
#define RESETGPSALTITUDEATARM
#define HEADINGCORRECT              // required to correct for MWheading being 0>360 vs MWII -180>+180. Leave permanently enabled

#define cfgck 7
//#define cfgActive EEPROM.write(LINE+onTime,0);

#if defined DISABLEGPSALTITUDERESET
  #undef RESETGPSALTITUDEATARM
#endif

#ifndef GPSOSDHOMEDISTANCE
  #define GPSOSDHOMEDISTANCE 40     // distance from home in meters to start check  for when when GPSOSD is home.
#endif

/********************  Submersible settings *********************/
#if defined FRESHWATER
  #define FLUID_DENSITY 997
#elif defined SEAWATER
  #define FLUID_DENSITY 1029
#elif !defined FLUID_DENSITY
  #define FLUID_DENSITY 997 // default fresh
#endif

/********************  OSD HARDWARE rule definitions  *********************/

// default pin mappings:
#define VOLTAGEPIN    A0
#define VIDVOLTAGEPIN A2
#define AMPERAGEPIN   A1
#define RSSIPIN       A3              
#define LEDPIN        7
#define RCPIN         5   // Aeromax hardware only      
#define AUXPIN        A1  // A6 for Aeromax hardware only        
#define AUDIOPIN      2   // Aeromax hardware only  
#define INTC3             // Arduino A3 enabled for PWM/PPM interrupts) Arduino A3 == Atmega Port C bit 3 for PWM trigger on RSSI pin
//#define INTD5           // Atmega Port D bit 5 PWM/PPM interrupts) Aeromax hardware used for RC input

// board specific amendments:
#ifdef AEROMAX
    #define ATMEGASETHARDWAREPORTS DDRC &= B11110111;DDRD &= B11011111;
    #define INTD5    
    #undef  AUXPIN
    #define AUXPIN    A6  // A6 for Aeromax hardware only        
#elif defined AIRBOTMICRO
    #undef VOLTAGEPIN
    #undef VIDVOLTAGEPIN
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
    #define MAX_SOFTRESET
#elif defined ANDROMEDA
    #define MAX_SOFTRESET
#elif defined RTFQV1                     
    #undef VOLTAGEPIN
    #undef VIDVOLTAGEPIN
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
    #define ALTERNATEDIVIDERS
#elif defined RTFQMICRO                     
    #undef VOLTAGEPIN
    #undef VIDVOLTAGEPIN
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
#elif defined KYLIN250PDB
    #undef VOLTAGEPIN
    #define VOLTAGEPIN    A6
#elif defined HOLYBROPDB
    #undef VOLTAGEPIN
    #define VOLTAGEPIN    A6
#endif

#ifdef SWAPVOLTAGEPINS                     
    #undef VOLTAGEPIN
    #undef VIDVOLTAGEPIN
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
#endif

#ifndef ATMEGASETHARDWAREPORTS
    # define ATMEGASETHARDWAREPORTS pinMode(RSSIPIN, INPUT);pinMode(RCPIN, INPUT);
#endif 

#ifdef RUSHDUINO                    
    # define DATAOUT          11 // MOSI
    # define DATAIN           12 // MISO
    # define SPICLOCK         13 // sck
    # define VSYNC             2 // INT0
    # define MAX7456RESET      9 // RESET
    # define MAX7456SELECT    10 // CHIP SELECT 
    # define MAX7456SETHARDWAREPORTS  pinMode(MAX7456RESET,OUTPUT);pinMode(MAX7456SELECT,OUTPUT);pinMode(DATAOUT, OUTPUT);pinMode(DATAIN, INPUT);pinMode(SPICLOCK,OUTPUT);pinMode(VSYNC, INPUT);
    # define MAX7456HWRESET   digitalWrite(MAX7456RESET,LOW);delay(60);digitalWrite(MAX7456RESET,HIGH);delay(40);
    # define MAX7456ENABLE    digitalWrite(MAX7456SELECT,LOW); 
    # define MAX7456DISABLE   digitalWrite(MAX7456SELECT,HIGH); 
    # define LEDINIT          pinMode(LEDPIN,OUTPUT);
    # define LEDON            digitalWrite(LEDPIN,HIGH);
    # define LEDOFF           digitalWrite(LEDPIN,LOW); 
#elif defined ARDUINO_OSD // Example for Arduino guys                     
    # define DATAOUT          11 // MOSI
    # define DATAIN           12 // MISO
    # define SPICLOCK         13 // sck
    # define VSYNC             2 // INT0
    # define MAX7456SELECT     6 // ss
    # define MAX7456RESET     10 // RESET
    # define MAX7456SETHARDWAREPORTS  pinMode(MAX7456RESET,OUTPUT);pinMode(MAX7456SELECT,OUTPUT);pinMode(DATAOUT, OUTPUT);pinMode(DATAIN, INPUT);pinMode(SPICLOCK,OUTPUT);pinMode(VSYNC, INPUT);
    # define MAX7456HWRESET   digitalWrite(MAX7456RESET,LOW);delay(60);digitalWrite(MAX7456RESET,HIGH);delay(40);
    # define MAX7456ENABLE    digitalWrite(MAX7456SELECT,LOW); 
    # define MAX7456DISABLE   digitalWrite(MAX7456SELECT,HIGH); 
    # define LEDINIT          pinMode(LEDPIN,OUTPUT);
    # define LEDON            digitalWrite(LEDPIN,HIGH);
    # define LEDOFF           digitalWrite(LEDPIN,LOW);
#else                                  
    # define MAX7456ENABLE    PORTD&=B10111111; 
    # define MAX7456DISABLE   PORTD|=B01000000; 
    # define MAX7456SETHARDWAREPORTS  DDRB|=B00101100;DDRB&=B11101111;DDRD|=B01000000;DDRD&=B11111011;
    # define MAX7456HWRESET   PORTB&=B11111011;delay(100);PORTB|=B00000100;
    # define LEDINIT          DDRD = DDRD|B10000000;
    # define LEDON            PORTD|=B10000000;
    # define LEDOFF           PORTD&=B01111111;
#endif

#if defined  KKAUDIOVARIO
  #define I2C_SUPPORT
  #undef FIXEDLOOP
#endif

#if defined USEMS5837 
  #define I2C_SUPPORT
#endif

#ifdef AEROMAX
    #define DIVIDER1v1      0.000107      // Voltage divider for 1.1v reference.
    #define DIVIDER5v       0.000475      // Voltage divider for 5v reference.
#elif ALTERNATEDIVIDERS
    #define DIVIDER1v1      0.0002        // Voltage divider for 1.1v reference.
    #define DIVIDER5v       0.0005        // Voltage divider for 5v reference.
#else
    #define DIVIDER1v1      0.0001045     // Voltage divider for 1.1v reference. Use 0.0001 default unless advised otherwise.
    #define DIVIDER5v       0.0005        // Voltage divider for 5v reference. Use 0.0005 default unless advised otherwise.
#endif

#ifdef I2C_UB_SUPPORT
    #define I2C_UB_ADDR      0x19
  //#define I2C_UB_IRQPIN    3
    #define I2C_UB_BREQUIV   115200UL   // Pretend baudrate of 115200
    #define MSP2CFG                     // Duplicate MSP request to config port
#endif

/********************  END OSD HARDWARE rule definitions  *********************/


/********************  GPS OSD rule definitions  *********************/

#if defined PPM_CONTROL
  #undef OSD_SWITCH
  #undef PWM_OSD_SWITCH
  #define OSD_SWITCH_RC               // Enables 3 way screen switch using a TX channel via FC. Specify channel on GUI (range 0-7 AUX1=4 AUX4=7)
#endif

#if defined (PWM_OSD_SWITCH)
  #define OSD_SWITCH_RC
#endif


/********************  PROTOCOL rule definitions  *********************/
#define PROTOCOL_MSP // on by default

#ifdef GPSOSD
#undef  PROTOCOL_MSP
#define FORCESENSORS
#endif

#ifdef  NAZA
#undef  PROTOCOL_MSP
#define FORCESENSORS
#endif

#ifdef PROTOCOL_MAVLINK
#undef  PROTOCOL_MSP
#define FORCESENSORS
//#define NOSUMMARYTHROTTLERESET
#ifndef TX_CHANNELS
  #define TX_CHANNELS 16 
#endif
#endif

#ifdef PROTOCOL_LTM
#undef  PROTOCOL_MSP
#define FORCESENSORS
#endif

#ifdef PROTOCOL_KISS
#undef  PROTOCOL_MSP
#define FORECSENSORACC
#endif

#ifdef FORCE_MSP
#define PROTOCOL_MSP
#endif

#ifndef PROTOCOL_MAVLINK
 #ifdef MAV_STATUS
   #undef MAV_STATUS
 #endif  
#endif

/********************  MSP speed enhancements rule definitions  *********************/

#if defined MSP_SPEED_HIGH
  #define hi_speed_cycle  10  // updates everything approx 6.3 times per second, updates attitude 30 times per second
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  30  // same as low, but also updates attitude 10 times per second
#else
  #define hi_speed_cycle  30  // updates everything approx 3 times per second
#endif


/********************  RX channel rule definitions  *********************/


#ifndef TX_CHANNELS
  #define TX_CHANNELS 8 
#endif

/********************  other paramters  *********************/

#ifdef PIODEBUG // This is for travis build only
  #define DEBUG 4 // Display debug secreen at boot
#endif

#ifdef PIOAUDIOVARIO // This is for travis build only
  #define AUDIOVARIO A3 // Enable AUDIOVARIO on RSSI
#endif

#ifdef PIOKKVARIO // This is for travis build only
  #define KKAUDIOVARIO A3 // Enable KKAUDIOVARIO on RSSI
#endif

#ifdef PIOAUDIOVARIOAEROMAX // This is for travis build only
  #define AUDIOVARIO AUDIOPIN // Enable AUDIOVARIO on on defined audio pin (AEROMAX hardware)
#endif

#ifdef PIOKKVARIOAEROMAX // This is for travis build only
  #define KKAUDIOVARIO AUDIOPIN // Enable KKAUDIOVARIO on defined audio pin (AEROMAX hardware)
#endif

#ifdef PWM_THROTTLE
  #define ALWAYSARMED  // starts OSD in armed mode
#endif

#ifdef MAV_ARMED
  #define ALWAYSARMED  // starts OSD in armed mode
#endif

#ifndef BAUDRATE 
  #ifdef PROTOCOL_MAVLINK 
    #define BAUDRATE 57600
  #else
    #define BAUDRATE 115200
  #endif // PROTOCOL_MAVLINK
#endif // BAUDRATE


/********************  BOXID compatibility  *********************/

#ifdef iNAV
  #define IDBOXAIR 29
  #define IDBOXWP 28
  #define IDBOXGPSLAND 21
#elif defined BETAFLIGHT
  #define IDBOXAIR 28
  #define IDBOXWP 199 // random unused to disable
  #define IDBOXGPSLAND 199 // random unused to disable
#else
  #define IDBOXAIR 28
  #define IDBOXWP 20
  #define IDBOXGPSLAND 21
#endif

#ifdef MULTIWII
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #undef  INTRO_MENU
  #define INFO_CONTROLLER 1
#endif
#ifdef BASEFLIGHT
  #define INFO_CONTROLLER 2
#endif
#ifdef TAULABS
  #define INFO_CONTROLLER 4
#endif
#ifdef LIBREPILOT
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 3
#endif
#ifdef DRONIN
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 5
#endif
#ifdef CLEANFLIGHT
  #define INFO_CONTROLLER 6
#endif
#ifdef BETAFLIGHT
  #define INFO_CONTROLLER 7
#endif
#ifdef FIXEDWING_BF
  #define INFO_CONTROLLER 8
#endif
#ifdef FIXEDWING_BF_SERVO
  #define INFO_CONTROLLER 9
#endif
#ifdef HARAKIRI
  #define INFO_CONTROLLER 10
#endif
#ifdef NAZA
  #define INFO_CONTROLLER 11
#endif
#ifdef iNAV
  #define INFO_CONTROLLER 12
#endif
#ifdef KISS
  #define INFO_CONTROLLER 13
#endif
#ifdef APM
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 14
#endif
#ifdef PX4
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 15
#endif
#ifdef SKYTRACK
  #define INFO_CONTROLLER 16
#endif
#ifdef GPSOSD_UBLOX
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 17
#endif
#ifdef GPSOSD_NMEA
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 18
#endif
#ifdef GPSOSD_NMEA
  #ifdef INFO_CONTROLLER
   #undef INFO_CONTROLLER
  #endif;  
  #define INFO_CONTROLLER 19
#endif
#ifdef NOCONTROLLER
  #define INFO_CONTROLLER 20
#endif
#ifdef RACEFLIGHT
  #define INFO_CONTROLLER 21
#endif
#ifdef SUBMERSIBLE
  #define INFO_CONTROLLER 22
#endif

#ifdef MINIMOSD
  #define INFO_HARDWARE 1
#endif
#ifdef MICROMINIMOSD
  #define INFO_HARDWARE 2
#endif
#ifdef AEROMAX
  #define INFO_HARDWARE 3
#endif
#ifdef RTFQV1
  #define INFO_HARDWARE 4
#endif
#ifdef RTFQMICRO
  #define INFO_HARDWARE 5
#endif
#ifdef RUSHDUINO
  #define INFO_HARDWARE 6
#endif
#ifdef KYLIN250PDB
  #define INFO_HARDWARE 7
#endif
#ifdef AIRBOTMICRO
  #define INFO_HARDWARE 8
#endif
#ifdef ANDROMEDA
  #define INFO_HARDWARE 9
#endif
#ifdef ANDROMEDA
  #define HOLYBROPDB 10
#endif


#ifdef ROTORCRAFT
  #define INFO_AIRCRAFT 1
#endif
#ifdef FIXEDWING
  #define INFO_AIRCRAFT 2
#endif


/********************  info for GUI  *********************/
#ifndef INFO_CONTROLLER
  #define INFO_CONTROLLER          0            // default - unknown 
#endif

#ifndef INFO_VERSION
  #define INFO_VERSION             MWOSDVERSION // version specified 
#endif

#ifndef INFO_VENDOR
  #define INFO_VENDOR              VENDOR       // vendor specific reference 
#endif

#ifndef INFO_HARDWARE
  #define INFO_HARDWARE            0            // default - unknown 
#endif

#ifndef INFO_AIRCRAFT
  #define INFO_AIRCRAFT            0            // default - unknown 
#endif

#ifndef INFO_OPTIONS
  #define INFO_OPTIONS            0            // default - unknown 
#endif



