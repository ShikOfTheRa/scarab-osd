/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/

/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
//#define DEBUG         // Enable/disable option to display OSD debug values 
//#define DEBUGMW       // Disable to prevent load Mutltiwii debug values from MSP 
//#define ALWAYSARMED
//#define DEVELOPMENT   // to force layout 0, set debug default eeprombit = 1
//#define FORCEDEBUG    // to force debug display
//#define FORCESENSORS


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

#ifdef LIBREPILOT
  #define TAULABS
  #define USE_MSP_PIDNAMES
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
    #define MENU_PID_VEL
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
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
  #define MENU_ALARMS   8       //ALARMS
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
  #define MENU_ALARMS   8       //ALARMS
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined FIXEDWING_BF
  #define FIXEDWING
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT          0       //STATISTICS
  #define MENU_PID           1       //PID CONFIG
  #define MENU_RC            2       //RC TUNING
  #define MENU_FIXEDWING     3       //FIXEDWING adjustments
  #define MENU_VOLTAGE       4       //VOLTAGE
  #define MENU_RSSI          5       //RSSI
  #define MENU_CURRENT       6       //CURRENT
  #define MENU_DISPLAY       7       //DISPLAY
  #define MENU_ADVANCED      8       //ADVANCED
  #define MENU_ALARMS        9       //ALARMS
  #define MENU_PROFILE       10      //PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_PROFILE
#endif

#if defined FIXEDWING_BF_SERVO
  #define FIXEDWING
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

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
  #define MENU_ALARMS        10       //ALARMS
  #define MENU_PROFILE       11      //PROFILE+PID CONTROLLER
  #define MAXPAGE       MENU_PROFILE
#endif

#if defined BASEFLIGHT20150627
  #define AMPERAGE_DIV 10
  #define CORRECT_MSP_BF1
  #define CORRECT_MENU_RCT1
  #define ENABLE_MSP_SAVE_ADVANCED

  #define MENU_STAT     0       //STATISTICS
  #define MENU_PID      1       //PID CONFIG
  #define MENU_RC       2       //RC TUNING
  #define MENU_VOLTAGE  3       //VOLTAGE
  #define MENU_RSSI     4       //RSSI
  #define MENU_CURRENT  5       //CURRENT
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
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
  #define MENU_ALARMS   8       //ALARMS
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
  #define MENU_ALARMS   9       //ALARMS
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
  #define MENU_ALARMS   8       //ALARMS
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
  #define MENU_DISPLAY  6       //DISPLAY
  #define MENU_ADVANCED 7       //ADVANCED
  #define MENU_ALARMS   8       //ALARMS
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
  #define MENU_ALARMS   8       //ALARMS
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
  #define PROTOCOL_MAVLINK
  #define AMPERAGE_DIV 10
#endif

#if defined(KISS)
  #define MENU_STAT     0       //STATISTICS
  #define MENU_VOLTAGE  1       //VOLTAGE
  #define MENU_RSSI     2       //RSSI
  #define MENU_CURRENT  3       //CURRENT
  #define MENU_DISPLAY  4       //DISPLAY
  #define MENU_ADVANCED 5       //ADVANCED
  #define MENU_ALARMS   6       //ALARMS
  #define MAXPAGE       MENU_ALARMS
  #define PROTOCOL_KISS
  #define AMPERAGE_DIV 10
#endif

#ifdef NOCONTROLLER
  #undef  INTRO_MENU
  #undef  ALARM_MSP
  #undef  ALARM_SATS
  #undef  ALARM_GPS
  #undef  OSD_SWITCH_RC
  #define HIDEARMEDSTATUS
  #define MENU_STAT  0           //STATISTICS
  #define MAXPAGE MENU_STAT
#endif

#ifdef SKYTRACK
  #undef  INTRO_MENU
  #undef  ALARM_MSP
  #undef  ALARM_SATS
  #undef  ALARM_GPS
  #undef  OSD_SWITCH_RC
  #define MENU_STAT  0           //STATISTICS
  #define MAXPAGE MENU_STAT
  #define PROTOCOL_SKYTRACK
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
  #define LONG_RANGE_DISPLAY
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
#ifdef RUSHDUINO // Example for Arduino guys                     
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
#else                                  
    # define MAX7456ENABLE    PORTD&=B10111111; 
    # define MAX7456DISABLE   PORTD|=B01000000; 
    # define MAX7456SETHARDWAREPORTS  DDRB|=B00101100;DDRB&=B11101111;DDRD|=B01000000;DDRD&=B11111011;
    # define MAX7456HWRESET   PORTB&=B11111011;delay(100);PORTB|=B00000100;
#endif

#ifdef RTFQV1                     
    #define SWAPVOLTAGEPINS
    #define ALTERNATEDIVIDERS
#endif

#ifdef RTFQMICRO                     
    #define SWAPVOLTAGEPINS
#endif

#ifdef SWAPVOLTAGEPINS                     
    #define VOLTAGEPIN    A2
    #define VIDVOLTAGEPIN A0
#else                                  
    #define VOLTAGEPIN    A0
    #define VIDVOLTAGEPIN A2
#endif

#ifdef KYLIN250PDB
    #undef VOLTAGEPIN
    #define VOLTAGEPIN    A6
#endif

#ifdef ALTERNATEDIVIDERS
    #define DIVIDER1v1      0.0002      // Voltage divider for 1.1v reference.
    #define DIVIDER5v       0.0008      // Voltage divider for 5v reference.
#else
    #define DIVIDER1v1      0.0001      // Voltage divider for 1.1v reference. Use 0.0001 default unless advised otherwise.
    #define DIVIDER5v       0.0005      // Voltage divider for 5v reference. Use 0.0005 default unless advised otherwise.
#endif


/********************  GPS OSD rule definitions  *********************/

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
  #undef  ALARM_MSP
  #undef  OSD_SWITCH_RC
  #define HIDEARMEDSTATUS
  #define ALARM_GPS 5

  #define MENU_STAT     0       //STATISTICS
  #define MENU_VOLTAGE  1       //VOLTAGE
  #define MENU_RSSI     2       //RSSI
  #define MENU_CURRENT  3       //CURRENT
  #define MENU_DISPLAY  4       //DISPLAY
  #define MENU_ADVANCED 5       //ADVANCED
  #define MENU_ALARMS   6       //ALARMS
  #define MAXPAGE       MENU_ALARMS
#endif

#if defined (OSD_SWITCH_RSSI)
  #define OSD_SWITCH_RC
#endif


/********************  TELEMTERY rule definitions  *********************/
#ifdef TELEMETRY_LTM
#define PROTOCOL_LTM
#endif

#ifdef TELEMETRY_MAVLINK
#define PROTOCOL_MAVLINK
#endif

#ifdef TELEMETRY_SKYTRACK
#define PROTOCOL_SKYTRACK
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


/********************  MSP speed enhancements rule definitions  *********************/

#if defined MSP_SPEED_HIGH
  #define hi_speed_cycle  10  // updates everything approx 6.3 times per second, updates attitude 30 times per second
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  50  // same as low, but also updates attitude 10 times per second
#else
  #define hi_speed_cycle  50  // updates everything approx 1.3 times per second
#endif


/********************  RX channel rule definitions  *********************/

#if defined SERIAL_SUM_PPM_GS            //PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4 //For Graupner/Spektrum    
  #define ROLLSTICK        4
  #define PITCHSTICK       1
  #define YAWSTICK         2
  #define THROTTLESTICK    3
#elif defined SERIAL_SUM_PPM_RHF         //ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For Robe/Hitec/Futaba
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         4
  #define THROTTLESTICK    3
#elif defined SERIAL_SUM_PPM_M           //ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4 //For Multiplex
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         3
  #define THROTTLESTICK    4
#elif defined SERIAL_SUM_PPM_HS          //PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For some Hitec/Sanwa/Others
  #define ROLLSTICK        2
  #define PITCHSTICK       1
  #define YAWSTICK         4
  #define THROTTLESTICK    3
#elif defined KISS
  #define ROLLSTICK        2
  #define PITCHSTICK       3
  #define YAWSTICK         4
  #define THROTTLESTICK    1
#else
  // RX CHANEL IN MwRcData table
  #define ROLLSTICK        1
  #define PITCHSTICK       2
  #define YAWSTICK         3
  #define THROTTLESTICK    4
#endif

#ifndef RCCHANNELS
  #define RCCHANNELS 8 
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

#ifndef BAUDRATE 
  #ifdef PROTOCOL_MAVLINK 
    #define BAUDRATE 57600
  #else
    #define BAUDRATE 115200
  #endif // PROTOCOL_MAVLINK
#endif // BAUDRATE

