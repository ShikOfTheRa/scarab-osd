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
#if defined (BASEFLIGHT) || defined(BASEFLIGHT_PR)                   
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

#if defined MSP_SPEED_HIGH
  #define hi_speed_cycle  5
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  15
#elif defined MSP_SPEED_MED
  #define hi_speed_cycle  50
#else
  #define hi_speed_cycle  50
#endif



