// omghax -- it's all in the .h file...
// GPS protocol GGA and RMC  sentences are needed
// Ublox config con be set using u-blox-config.ublox.txt


#if defined GPSOSD

  #define  LAT  0
  #define  LON  1
  #define  BAUD BAUDRATE

  #if defined(INIT_MTK_GPS)
    #define MTK_SET_BINARY          PSTR("$PGCMD,16,0,0,0,0,0*6A\r\n")
    #define MTK_SET_NMEA            PSTR("$PGCMD,16,1,1,1,1,1*6B\r\n")
    #define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
    #define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
    #define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
    #define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")
    #define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n") // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s
    #define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
    #define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
    #define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")  //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)
  #endif

#endif // GPSOSD

class GPSClass {
  public:
    void Check(void);

    //local time of coord calc - haydent
    uint32_t time = 0;

    uint16_t armedangle=0;

    int16_t MwHeading=0;
    int32_t MwAltitude=0;

    uint32_t distanceToHome=0;
    uint8_t fix=0;
    uint8_t frame_timer=0;
    int32_t latitude;
    int32_t longitude;
    int16_t altitude;
    int16_t home_altitude;
    int16_t previousfwaltitude=0;
    int16_t interimfwaltitude=0;
    uint16_t speed;
    int16_t  ground_course;
    uint16_t old_speed;
    int16_t directionToHome=0;
    uint8_t numSat=0;
    uint8_t waypoint_step=0;

#ifdef GPSOSD
    uint32_t home_timer=0;
    int32_t  coord[2];
    int32_t  home[2];
  //  uint16_t ground_course = 0;
    int16_t  altitude_home;
    uint8_t  Present = 0;
  //  uint8_t  SerialInitialised=5;
    uint16_t armedangle=0;
    uint8_t  armedangleset = 0;
    uint8_t  active=5;
    uint8_t  fix_HOME=0;
#endif
};

extern GPSClass Gps;
