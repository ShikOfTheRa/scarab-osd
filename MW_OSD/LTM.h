
uint8_t  GPS_fix_HOME=0;

uint8_t ltmread_u8()  {
  return LTMserialBuffer[mw_ltm.LTMreadIndex++];
}

uint16_t ltmread_u16() {
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8() << 8;
  return t;
}

uint32_t ltmread_u32() {
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16() << 16;
  return t;
}

void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  mw_ltm.GPS_scaleLonDown = cos(rads);
}

void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                 // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * mw_ltm.GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;

  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output radians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_reset_home_position() {
  if (GPS_fix && GPS_numSat >= MINSATFIX) {
    GPS_home[LAT] = GPS_latitude;
    GPS_home[LON] = GPS_longitude;
    if (!MSP_home_set)
      mw_ltm.GPS_altitude_home = GPS_altitude;
    GPS_calc_longitude_scaling(GPS_latitude);  //need an initial value for distance and bearing calc
  }
}

uint16_t calculateCurrentFromConsumedCapacity(uint16_t mahUsed)
{
  static unsigned long previous_millis = 0;
  static uint16_t previous_mahUsed = 0;
  static uint16_t calculatedCurrent = 0;

  unsigned long current_millis = millis();

  if ((current_millis - previous_millis) > 5000 || (previous_mahUsed > mahUsed)) {
    // Stalled or invalid telemetry. Reset statistics
    calculatedCurrent = 0;
    previous_mahUsed = mahUsed;
    previous_millis = current_millis;
  }
  else if ((current_millis - previous_millis) > 500 && (previous_mahUsed < mahUsed)) {
    calculatedCurrent = (mahUsed - previous_mahUsed) *2*360;
    previous_mahUsed = mahUsed;
    previous_millis = current_millis;
  }
  return calculatedCurrent;
}

void ltm_check() {
  timer.packetcount++;
  mw_ltm.LTMreadIndex=0;
  static uint8_t GPS_fix_HOME_validation=GPSHOMEFIX;
  static uint8_t armedglitchprotect=0;
  uint32_t dummy;
#ifdef DATA_MSP
  timer.MSP_active=DATA_MSP;             // getting something on serial port
#endif
  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_GFRAME)
  {
#ifdef ALARM_GPS
    timer.GPS_active=ALARM_GPS;
#endif //ALARM_GPS
    GPS_latitude = (int32_t)ltmread_u32();
    GPS_longitude = (int32_t)ltmread_u32();
    GPS_speed = ltmread_u8() * 100;            // LTM gives m/s, we expect cm/s
    GPS_altitude = ((int32_t)ltmread_u32());   // LTM altitude in cm.
    if (GPS_fix_HOME == 0){
      GPS_reset_home_position();
    }
    GPS_altitude=GPS_altitude - mw_ltm.GPS_altitude_home;
    MwAltitude = (int32_t) GPS_altitude;       // baro alt in cm
    GPS_altitude = GPS_altitude / 100;         // gps  alt in m
    uint8_t ltm_satsfix = ltmread_u8();
    GPS_numSat = (ltm_satsfix >> 2) & 0xFF;
    GPS_fix    = ((ltm_satsfix & 0b00000011) <= 1) ? 0 : 1;
    // ipdate home distance and bearing
    if ((GPS_fix>0) && (GPS_numSat >= MINSATFIX)) {
      uint32_t dist;
      int32_t  dir;
      GPS_distance_cm_bearing(&GPS_latitude,&GPS_longitude,&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
      GPS_distanceToHome = dist/100;
      GPS_directionToHome = dir/100;
    } 
  }

  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_AFRAME)
  {
    MwAngle[1]=(int16_t)10*ltmread_u16();
    MwAngle[0]=(int16_t)10*ltmread_u16();
    MwHeading = (int16_t)ltmread_u16();
#if defined(USEGPSHEADING)
    MwHeading = GPS_ground_course/10;
#endif
#ifdef HEADINGCORRECT
    if (MwHeading >= 180) MwHeading -= 360;
#endif

  }
  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_SFRAME)
  {
    MwVBat = ltmread_u16()/100;
    dummy = ltmread_u16();
    if (Settings[S_MWAMPERAGE]){ 
      amperagesum = 360*dummy;
      MWAmperage  = 10*calculateCurrentFromConsumedCapacity(dummy);
    }
    MwRssi = ltmread_u8() * 4; // 0-255 to 0-1023 (1020, actually)
    dummy = ltmread_u8();
    uint8_t ltm_armfsmode = ltmread_u8();
    armed = (ltm_armfsmode & 0b00000001) ? 1 : 0;
    if (ltm_armfsmode & 0b00000001){
      armed=1;
      armedglitchprotect=3;
    }
    else{
      if (armedglitchprotect>0){
        armedglitchprotect--;        
      }
      else{
        armed=0;
#ifdef RESETHOMEARMED
        GPS_fix_HOME = 0;
#endif
      }
    }
    
    dummy = (ltm_armfsmode >> 1) & 0b00000001; // uavData.isFailsafe
    mw_ltm.mode = (ltm_armfsmode >> 2) & 0b00111111; // uavData.flightMode
    mw_ltm.mode = (mw_ltm.mode>15) ? 15 : mw_ltm.mode;
  }

  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_OFRAME)
  {

    if (GPS_fix && (GPS_numSat >= MINSATFIX)) {
      if (GPS_fix_HOME_validation>0){
        GPS_fix_HOME_validation--;
        GPS_numSat=1;
      }
      else{
        if (GPS_fix_HOME == 0){
          GPS_home[LAT] = (int32_t)ltmread_u32();
          GPS_home[LON] = (int32_t)ltmread_u32();
          GPS_fix_HOME=1;
        }
      }
    }
  }
}

void serialLTMreceive(uint8_t c) {
  static enum _serial_state {
    LTM_IDLE,
    LTM_HEADER_START1,
    LTM_HEADER_START2,
    LTM_HEADER_MSGTYPE,
    LTM_HEADER_DATA
  }
  c_state = LTM_IDLE;

  if (c_state == LTM_IDLE) {
    c_state = (c == '$') ? LTM_HEADER_START1 : LTM_IDLE;
  }
  else if (c_state == LTM_HEADER_START1) {
    c_state = (c == 'T') ? LTM_HEADER_START2 : LTM_IDLE;
  }
  else if (c_state == LTM_HEADER_START2) {
    switch (c) {
    case 'G': //G
      mw_ltm.LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    case 'A': //A
      mw_ltm.LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    case 'S': //S
      mw_ltm.LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    case 'O': //O
      mw_ltm.LTMframelength = LIGHTTELEMETRY_OFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    default:
      c_state = LTM_IDLE;
    }
    mw_ltm.LTMcmd = c;
    mw_ltm.LTMreceiverIndex = 0;
  }
  else if (c_state == LTM_HEADER_MSGTYPE) {
    if (mw_ltm.LTMreceiverIndex == 0) {
      mw_ltm.LTMrcvChecksum = c;
    }
    else {
      mw_ltm.LTMrcvChecksum ^= c;
    }
    if (mw_ltm.LTMreceiverIndex == mw_ltm.LTMframelength - 4) { // received checksum byte
      if (mw_ltm.LTMrcvChecksum == 0) {
        ltm_check();
        c_state = LTM_IDLE;
      }
      else {                                                   // wrong checksum, drop packet
        c_state = LTM_IDLE;
      }
    }
    else LTMserialBuffer[mw_ltm.LTMreceiverIndex++] = c;
  }
}




































