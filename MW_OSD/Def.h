/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/
/*--------------------------       advanced parameters      ----------------------------------------------------*/

/*----------------------------------------------       Developer parameters      ----------------------------------------------------*/
#define DEBUG         // Enable/disable option to display OSD debug values 
//#define DEBUGMW       // Disable to prevent load Mutltiwii debug values from MSP 

/********************  CONTROLLER rule definitions  **********************/

#ifdef NOCONTROLLER
  #undef  INTRO_MENU
  #undef  MSPACTIVECHECK
  #undef  SATACTIVECHECK
  #undef  GPSACTIVECHECK
  #undef  OSD_SWITCH_RC
#endif

#ifdef MULTIWII       //set up latest at time of release
  #define MULTIWII_V24                // Using amperage corrections  
//  #define MULTIWII_V23              // Using boxid  
//  #define MULTIWII_V21              // Using boxnames
#endif

#ifdef CLEANFLIGHT    //set up latest at time of release
  #define CLEANFLIGHT190   //MSP added P+R+DTHR
  //#define CLEANFLIGHT180           //MSP added P+R
  //#define CLEANFLIGHT172           //default original MSP request
#endif

#ifdef BASEFLIGHT     //set up latest at time of release
  #define BASEFLIGHT20150627         //MSP added P+R
  //#define BASEFLIGHT20150327       //default original MSP request
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

#if defined(HARIKIRI)
  #define BOXNAMES                  // required to support legacy protocol
#endif

#if defined (BASEFLIGHT20150327)            
  #define AMPERAGE_10ma
#endif

#if defined (BASEFLIGHT20150627)
  #define AMPERAGE_10ma
  #define SETCONFIG 25                  //for BASEFLIGHT20150627 to use MSP_SET_CONFIG
#endif

#if defined (CLEANFLIGHT190)         
  #define AMPERAGE_10ma
#endif

#if defined (CLEANFLIGHT180)
  #define AMPERAGE_10ma
#endif

#if defined (CLEANFLIGHT172)
  #define AMPERAGE_10ma
#endif

#if defined (MULTIWII_V24)                
  #define AMPERAGE_100ma
#endif

#if defined (MULTIWII_V23)                
  #define AMPERAGE_1ma
#endif

#if defined (MULTIWII_V21)                
  #define AMPERAGE_1ma         
  #define BOXNAMES                  // required to support legacy protocol
#endif

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
#endif

#if defined (OSD_SWITCH_RSSI)  
  #define OSD_SWITCH_RC
#endif


/********************  MSP enhancements rule definitions  *********************/

#if defined MSP_SPEED_HIGH
  #define hi_speed_cycle  5
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  15
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  50
#else
  #define hi_speed_cycle  50
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

