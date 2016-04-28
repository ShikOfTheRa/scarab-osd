// TODO: $$$ compile with GPSOSD, some stuff from gps.h should go here
#if defined GPSOSD
  #define  LAT  0
  #define  LON  1
  #define  GPS_BAUD BAUDRATE
  uint32_t GPS_home_timer=0;
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
//  uint16_t GPS_ground_course = 0;
  int16_t  GPS_altitude_home;
  uint8_t  GPS_Present = 0;
//  uint8_t  GPS_SerialInitialised=5;
  uint8_t  GPS_armedangleset = 0;
  uint8_t  GPS_active=5;
  uint8_t  GPS_fix_HOME=0;
#endif
