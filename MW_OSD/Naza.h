#ifdef NAZA
  #include "NazaDecoderLib.h"
  #include "NazaDecoderLib.cpp"


static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles
  // moving average filter variables

void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  GPS_scaleLonDown = cos(rads);
}



void GPS_reset_home_position() {
  if (GPS_fix && GPS_numSat >= MINSATFIX) {
      GPS_home[LAT] = GPS_coord[LAT];
      GPS_home[LON] = GPS_coord[LON];
      GPS_altitude_home = GPS_altitude;
      GPS_calc_longitude_scaling(GPS_coord[LAT]);  //need an initial value for distance and bearing calc
      GPS_fix_HOME = 1;
  }
}


////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}


void GPS_NewData() {
  static uint8_t GPS_fix_HOME_validation=GPSHOMEFIX;

  if (GPS_fix && (GPS_numSat >= MINSATFIX)) {
    if (GPS_fix_HOME_validation>0){
#if defined HOMESATFIX
      if (GPS_numSat>=HOMESATFIX)
#endif // HOMESATFIX
        GPS_fix_HOME_validation--;
      GPS_numSat=1;
    }
    else{
      if (GPS_fix_HOME == 0){
        GPS_reset_home_position();
        GPS_fix_HOME=1;
      }
      else{
    //calculate distance. bearings etc
    uint32_t dist;
    int32_t  dir;
    GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
    GPS_distanceToHome = dist/100;
    GPS_directionToHome = dir/100;
    GPS_altitude =  GPS_altitude- GPS_altitude_home;
    MwAltitude = (int32_t)GPS_altitude *100;
    GPS_latitude = GPS_coord[LAT];
    GPS_longitude = GPS_coord[LON];
    int16_t MwHeading360=GPS_ground_course/10;
    if (MwHeading360>180)
    MwHeading360 = MwHeading360-360;
    MwHeading   = MwHeading360;
    
    if (GPS_armedangleset==0)  
      armedangle=MwHeading;
    if (GPS_distanceToHome>GPSOSDARMDISTANCE){
      GPS_armedangleset = 1;
      armed=1;
    }
  
    if (GPS_armedangleset==1){
      if ((GPS_distanceToHome<GPSOSDHOMEDISTANCE)&&(GPS_speed<75)){
        if ((GPS_home_timer+7000)>millis()){
        }
        else if((GPS_home_timer+22000)>millis()){
          configPage=0;
          armed=0;
        }      
        else{
           configMode=0;
           GPS_armedangleset=0;
           previousarmedstatus=0;
        }      
      }
      else {
        GPS_home_timer=millis();
      }    
    }
      }
    }
  }
  else{
    GPS_fix_HOME_validation=GPSHOMEFIX;
/*
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
      GPS_altitude = 0;
      MwAltitude = 0;
*/  
  }

}


void NAZA_NewData(uint8_t c){
  uint8_t decodedMessage = NazaDecoder.decode(c);
  int8_t sattemp;
  switch (decodedMessage){
    uint8_t GPS_fix_temp;
    case NAZA_MESSAGE_GPS:
     timer.packetcount++;
     #ifdef ALARM_GPS
       timer.GPS_active=ALARM_GPS;
     #endif //ALARM_GPS
     sattemp=NazaDecoder.getNumSat();
      if (sattemp>4){
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
     timer.packetcount++;
     #ifdef DATA_MSP
       timer.MSP_active=DATA_MSP;             // getting something on serial port
     #endif //DATA_MSP
      GPS_ground_course=10*NazaDecoder.getHeadingNc();
      int16_t MwHeading360=GPS_ground_course/10;
      if (MwHeading360>180)
        MwHeading360 = MwHeading360-360;
        MwHeading   = MwHeading360;
      break;
  }
}


#endif



