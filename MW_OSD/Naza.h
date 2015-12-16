#ifdef NAZA
  #include "NazaDecoderLib.h"
  #include "NazaDecoderLib.cpp"

void NAZA_NewData(uint8_t c){
  uint8_t decodedMessage = NazaDecoder.decode(c);
  int8_t sattemp;
  switch (decodedMessage){
    uint8_t GPS_fix_temp;
    case NAZA_MESSAGE_GPS:
      sattemp=NazaDecoder.getNumSat();
      if (sattemp>4){
        #ifdef GPSACTIVECHECK
          timer.GPS_active=GPSACTIVECHECK;
        #endif //GPSACTIVECHECK
        GPS_numSat=NazaDecoder.getNumSat();
        GPS_coord[LAT]=(int32_t)(10000000*NazaDecoder.getLat());
        GPS_coord[LON]=(int32_t)(10000000*NazaDecoder.getLon());
        GPS_altitude=NazaDecoder.getGpsAlt();
        GPS_fix_temp=NazaDecoder.getFixType();
        GPS_numSat=NazaDecoder.getNumSat();
        GPS_speed=100*NazaDecoder.getSpeed();
        gpsvario();            
        if (GPS_fix_temp>0){
          GPS_fix=1;
        }
        GPS_NewData();
      }            
      break;
    case NAZA_MESSAGE_COMPASS:
      GPS_ground_course=10*NazaDecoder.getHeadingNc();
      int16_t MwHeading360=GPS_ground_course/10;
      if (MwHeading360>180)
        MwHeading360 = MwHeading360-360;
        MwHeading   = MwHeading360;
      break;
  }
}
#endif



