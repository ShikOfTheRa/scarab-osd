/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/

/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
#define DEBUG         // Enable/disable option to display OSD debug values 
//#define DEBUGMW       // Disable to prevent load Mutltiwii debug values from MSP 


/*--------------------------       DEPRECATED parameters for reference only      ----------------------------------------------------*/

/********************       OSD SCREEN SWITCH settings      *********************/
// This functionality enables :
// a, 2 different screen layouts to be selected using the Flight controller "OSD_SWITCH" feature or
// b, 2 or 3 different screen layouts to be selected using a specificed RC channel assigned to a TX switch
//Choose ONLY ONE option:
#define OSD_SWITCH_RC               // Enables 3 way screen switch using a TX channel via FC. Specify channel on GUI (range 0-7 AUX1=4 AUX4=7)
//#define OSD_SWITCH                // DEPRECATED Forces original 2 way screen switch using OSD Switch via Flight Controller. MUST Ensure enabled on flight controller - e.g. #define OSD_SWITCH on multiwii




/********************  CONTROLLER rule definitions  **********************/

#ifdef MULTIWII       //set up latest at time of release
  #define MULTIWII_V24
#endif

#ifdef BETAFLIGHT    //set up latest at time of release
//  #define BETAFLIGHT
#endif

#ifdef TAULABS    //set up latest at time of release
//  #define TAULABS
#endif

#ifdef DRONIN    //set up latest at time of release
  #define TAULABS
#endif

#ifdef CLEANFLIGHT    //set up latest at time of release
  #define CLEANFLIGHT190
#endif

#ifdef iNAV    //set up latest at time of release
  #define CLEANFLIGHT190
#endif

#ifdef BASEFLIGHT     //set up latest at time of release
  #define BASEFLIGHT20150627
#endif

#ifdef HARIKIRI
  #define BOXNAMES
  #define MULTIWII_V24
#endif

#ifdef APM     //set up latest at time of release
  #define MAVLINK
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

#if defined BETAFLIGHT
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED
  #define ACROPLUS

  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
  #define MENU_PROFILE  9       //PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_PROFILE
#endif

#if defined CLEANFLIGHT190
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
  #define MENU_PROFILE  9       //PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_PROFILE
  #define CORRECTLOOPTIME
#endif

#if defined CLEANFLIGHT180
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF1
  #define CORRECT_MENU_RCT1

  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined CLEANFLIGHT172
  #define AMPERAGE_DIV  10
  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined BASEFLIGHT20150627
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
                       #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

 #define MENU_STAT  0           //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
  #define MENU_PROFILE  9       //PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_PROFILE
#endif

#if defined (BASEFLIGHT20150327)
  #define AMPERAGE_DIV  10
  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU_ALARMS
#endif

#if defined (MULTIWII_V24)
  #define AMPERAGE_DIV  1
  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
  #define MENU_GPS_TIME 8       //GPS TIME
  #define MENU_ALARMS   9 //ALARMS
//  #define MENU_PROFILE 10//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined (MULTIWII_V23)
  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined (MULTIWII_V21)
  #define BOXNAMES              // required to support legacy protocol
  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6 //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined(TAULABS)
  #define AMPERAGE_DIV  10
  #define HAS_ALARMS
  #define ACROPLUS
  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   8       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined(APM)
  #define MENU_STAT     0       //STATISTICS
  #define MENU_VOLTAGE  1       //VOLTAGE
  #define MENU_RSSI     2       //RSSI
  #define MENU_CURRENT  3       //CURRENT
  #define MENU_DISPLAY  4       //DISPLAY
  #define MENU_ADVANCED 5       //ADVANCED
  #define MENU_ALARMS   6       //ALARMS
  #define MAXPAGE       MENU_ALARMS
  #define MAVLINK
#endif

#ifdef NOCONTROLLER
  #undef  INTRO_MENU
  #undef  MSPACTIVECHECK
  #undef  SATACTIVECHECK
  #undef  GPSACTIVECHECK
  #undef  OSD_SWITCH_RC
  #define ALWAYSARMED
  #define MENU_STAT  0 //STATISTICS
  #define MAXPAGE MENU_STAT
#endif

#ifdef HAS_ALARMS
  #define MAX_ALARM_LEN 30
  #define ALARM_OK 0
  #define ALARM_WARN 1
  #define ALARM_ERROR 2
  #define ALARM_CRIT 3
#endif

#ifndef AMPERAGE_DIV 
  #define AMPERAGE_DIV 100
#endif

#if defined (FC_VOLTAGE_CONFIG) && (defined (CLEANFLIGHT) || defined(BASEFLIGHT))
  #define USE_FC_VOLTS_CONFIG
#endif

/********************  FIXEDWING definitions  *********************/
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

/********************  HARDWARE PINS definitions  *********************/
#define AMPERAGEPIN   A1
#define TEMPPIN       A3  // also used for airspeed         
#define RSSIPIN       A3              
#define PWMRSSIPIN    A3              
#define LEDPIN        7

// All aircraft / FC types defaults...
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

#ifdef WITESPYV1                     
    #define SWAPVOLTAGEPINS
    #define ALTERNATEDIVIDERS
#endif

#ifdef WITESPYMICRO                     
    #define SWAPVOLTAGEPINS
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

#define GPSOSDARMDISTANCE   20 // distance from home in meters when GPSOSD arms. Starts flight timer etc.
#define GPSOSDHOMEDISTANCE  40 // distance from home in meters when GPSOSD is home. When speed is low it disarms and displays summary screen.

#if defined PPMOSDCONTROL
  #undef OSD_SWITCH
  #undef OSD_SWITCH_RSSI
  #undef INTPWMRSSI
  #define OSD_SWITCH_RC               // Enables 3 way screen switch using a TX channel via FC. Specify channel on GUI (range 0-7 AUX1=4 AUX4=7)
#endif

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
  #define GPSACTIVECHECK 5

  #define MENU_STAT     0       //STATISTICS
//  #define MENU_PID  X //PID CONFIG
//  #define MENU_RC  X //RC TUNING
  #define MENU_VOLTAGE  1       //VOLTAGE
  #define MENU_RSSI     2       //RSSI
  #define MENU_CURRENT  3       //CURRENT
  #define MENU_DISPLAY  4       //DISPLAY
  #define MENU_ADVANCED 5       //ADVANCED
//  #define MENU_GPS_TIME  X //GPS TIME
  #define MENU_ALARMS   6       //ALARMS
//  #define MENU_PROFILE 9//PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined (OSD_SWITCH_RSSI)
  #define OSD_SWITCH_RC
#endif


/********************  MSP speed enhancements rule definitions  *********************/

#if defined MSP_SPEED_HIGH
  #define hi_speed_cycle  10  // updates everything approx 6.3 times per second, updates attitude 30 times per second
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  50  // same as low, but also updates attitude 10 times per second
#else
  #define hi_speed_cycle  50  // updates everything approx 1.3 times per second
#endif


/********************  RX channel rule definitions  *********************/

#if defined SERIAL_SUM_PPM_GS            //PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum    
  #define ROLLSTICK        3
  #define PITCHSTICK       0
  #define YAWSTICK         1
  #define THROTTLESTICK    2
#elif defined SERIAL_SUM_PPM_RHF         //ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
  #define ROLLSTICK        0
  #define PITCHSTICK       1
  #define YAWSTICK         3
  #define THROTTLESTICK    2
#elif defined SERIAL_SUM_PPM_M           //ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
  #define ROLLSTICK        0
  #define PITCHSTICK       1
  #define YAWSTICK         2
  #define THROTTLESTICK    3
#elif defined SERIAL_SUM_PPM_HS          //PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others
  #define ROLLSTICK        1
  #define PITCHSTICK       0
  #define YAWSTICK         3
  #define THROTTLESTICK    2
#else
  // RX CHANEL IN MwRcData table
  #define ROLLSTICK        0
  #define PITCHSTICK       1
  #define YAWSTICK         2
  #define THROTTLESTICK    3
#endif

/********************  RSSI  *********************/
#if defined FASTPWMRSSI
  #define INTPWMRSSI
#endif  

/********************  other paramters  *********************/
#define RSSIhz           10 

#ifdef PWMTHROTTLE
  #define ALWAYSARMED  // starts OSD in armed mode
#endif

/********************  CONTROLLER rule definitions  **********************/
#ifndef INTRO_DELAY
#define INTRO_DELAY 8
#endif

// TODO: $$$ sort these out
/********************  moved from globals.h  *********************/
#define POS_MASK        0x01FF
#define PAL_MASK        0x0003
#define PAL_SHFT             9
#define DISPLAY_MASK    0xC000
#define DISPLAY_ALWAYS  0xC000
#define DISPLAY_NEVER   0x0000
#define DISPLAY_COND    0x4000
#define DISPLAY_MIN_OFF     0x8000

#define POS(pos, pal_off, disp)  (((pos)&POS_MASK)|((pal_off)<<PAL_SHFT)|(disp))
#if defined SHIFTDOWN
#define TOPSHIFT        LINE
#else
#define TOPSHIFT        0
#endif

#define MPH 1
#define KMH 0
#define METRIC 0
#define IMPERIAL 1

#define lo_speed_cycle  100
#define sync_speed_cycle  33

#define CALIBRATION_DELAY 10       // Calibration timeouts
#define EEPROM_WRITE_DELAY 5       // Calibration timeouts

// DEFINE CONFIGURATION MENU PAGES
#define MINPAGE 0

#define PIDITEMS 10

// STICK POSITION
#define MAXSTICK         1850
#define MINSTICK         1150
//#define MINTROTTLE       1000

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

