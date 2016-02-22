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
  #define BETAFLIGHT
#endif

#ifdef CLEANFLIGHT    //set up latest at time of release
  #define CLEANFLIGHT190
#endif

#ifdef BASEFLIGHT     //set up latest at time of release
  #define BASEFLIGHT20150627
#endif

#ifdef HARIKIRI
  #define BOXNAMES
  #define MULTIWII_V24
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

  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU10  
#endif

#if defined CLEANFLIGHT190
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF2
  #define CORRECT_MENU_RCT2
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU10  
  #define CORRECTLOOPTIME
#endif

#if defined CLEANFLIGHT180
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_CF1
  #define CORRECT_MENU_RCT1

  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#if defined CLEANFLIGHT172
  #define AMPERAGE_DIV 10
  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#if defined BASEFLIGHT20150627
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU10   
#endif  

#if defined (BASEFLIGHT20150327)
  #define AMPERAGE_DIV 10
  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#if defined (MULTIWII_V24)
  #define AMPERAGE_DIV 1
  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
  #define MENU8  8 //GPS TIME
  #define MENU9  9 //ALARMS
//  #define MENU10 10//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#if defined (MULTIWII_V23)
  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#if defined (MULTIWII_V21)
  #define BOXNAMES                  // required to support legacy protocol
  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#if defined(TAULABS)
  #define AMPERAGE_DIV 10
  #define HAS_ALARMS
  #define MENU0  0 //STATISTICS
  #define MENU1  1 //PID CONFIG
  #define MENU2  2 //RC TUNING
  #define MENU3  3 //VOLTAGE
  #define MENU4  4 //RSSI
  #define MENU5  5 //CURRENT
  #define MENU6  6 //DISPLAY
  #define MENU7  7 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  8 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
#endif

#ifdef NOCONTROLLER
  #undef  INTRO_MENU
  #undef  MSPACTIVECHECK
  #undef  SATACTIVECHECK
  #undef  GPSACTIVECHECK
  #undef  OSD_SWITCH_RC
  #define ALWAYSARMED
  #define MENU0  0 //STATISTICS
  #define MAXPAGE MENU0   
#endif

#ifdef HAS_ALARMS
  #define MAX_ALARM_LEN 30
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

  #define MENU0  0 //STATISTICS
//  #define MENU1  X //PID CONFIG
//  #define MENU2  X //RC TUNING
  #define MENU3  1 //VOLTAGE
  #define MENU4  2 //RSSI
  #define MENU5  3 //CURRENT
  #define MENU6  4 //DISPLAY
  #define MENU7  5 //ADVANCED
//  #define MENU8  X //GPS TIME
  #define MENU9  6 //ALARMS
//  #define MENU10 9//PROFILE+PID CONTROLLER
  #define MAXPAGE MENU9   
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


