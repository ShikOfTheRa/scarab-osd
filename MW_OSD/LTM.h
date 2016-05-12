
#ifdef PROTOCOL_LTM

#define LIGHTTELEMETRY_START1 0x24 //$
#define LIGHTTELEMETRY_START2 0x54 //T
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_OFRAME 0x4F  //O OSD additionals data ( home pos, home alt, direction to home )
#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11
#define LIGHTTELEMETRY_OFRAMELENGTH 18
static uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH - 4];
static uint8_t LTMreceiverIndex;
static uint8_t LTMcmd;
static uint8_t LTMrcvChecksum;
static uint8_t LTMreadIndex;
static uint8_t LTMframelength;
static uint8_t LTMpassed = 0;
static uint8_t crlf_count = 0;

#define  LAT  0
#define  LON  1

uint8_t  GPS_fix_HOME=0;
uint16_t GPS_altitude_home;                            
float    GPS_scaleLonDown;
//static   uint8_t LTM_ok = 0;
static   uint32_t lastLTMpacket;
int32_t  GPS_home[2];


uint8_t ltmread_u8()  {
  return LTMserialBuffer[LTMreadIndex++];
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
  GPS_scaleLonDown = cos(rads);
}

void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                 // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;

  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output radians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_reset_home_position() {
  if (GPS_fix && GPS_numSat >= MINSATFIX) {
    GPS_home[LAT] = GPS_latitude;
    GPS_home[LON] = GPS_longitude;
    GPS_altitude_home = GPS_altitude;
    GPS_calc_longitude_scaling(GPS_latitude);  //need an initial value for distance and bearing calc
    GPS_fix_HOME = 1;
  }
}


// --------------------------------------------------------------------------------------
// Decoded received commands
void ltm_check() {
  static uint8_t GPS_fix_HOME_validation=GPSHOMEFIX;
  uint32_t dummy;
#ifdef MSPACTIVECHECK
  timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
#endif
  if (LTMcmd == LIGHTTELEMETRY_GFRAME)
  {
#ifdef GPSACTIVECHECK
    timer.GPS_active=GPSACTIVECHECK;
#endif //GPSACTIVECHECK
    GPS_latitude = (int32_t)ltmread_u32();
    GPS_longitude = (int32_t)ltmread_u32();
    GPS_speed = ltmread_u8() * 100;            // LTM gives m/s, we expect cm/s
    GPS_altitude = ((int32_t)ltmread_u32());      // altitude from cm to m.


    if (GPS_fix_HOME == 0){
      GPS_reset_home_position();
    }
    GPS_altitude=GPS_altitude - GPS_altitude_home;
    MwAltitude = (int32_t) GPS_altitude *100;       // m--cm gps to baro

    uint8_t ltm_satsfix = ltmread_u8();

    GPS_numSat = (ltm_satsfix >> 2) & 0xFF;
    GPS_fix    = ((ltm_satsfix & 0b00000011) <= 1) ? 0 : 1;

    // hpdate home distance and bearing
    if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX)) {
      uint32_t dist;
      int32_t  dir;
      GPS_distance_cm_bearing(&GPS_latitude,&GPS_longitude,&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
      GPS_distanceToHome = dist/100;
      GPS_directionToHome = dir/100;
    } 
  }

  if (LTMcmd == LIGHTTELEMETRY_AFRAME)
  {
    MwAngle[0]=(int16_t)ltmread_u16();
    MwAngle[1]=(int16_t)ltmread_u16();
    MwHeading = (int16_t)ltmread_u16();
#if defined(USEGPSHEADING)
    MwHeading = GPS_ground_course/10;
#endif
#ifdef HEADINGCORRECT
    if (MwHeading >= 180) MwHeading -= 360;
#endif
  }
  if (LTMcmd == LIGHTTELEMETRY_SFRAME)
  {
    MwVBat = ltmread_u16();
    dummy  = ltmread_u16(); //uavData.batUsedCapacity
    MwRssi = ltmread_u8();
    dummy  = ltmread_u8();

    uint8_t ltm_armfsmode = ltmread_u8();
    armed = (ltm_armfsmode & 0b00000001) ? 1 : 0;
    dummy = (ltm_armfsmode >> 1) & 0b00000001; // uavData.isFailsafe
    dummy = (ltm_armfsmode >> 2) & 0b00111111; // uavData.flightMode

    // uavData.batCellVoltage = detectBatteryCellVoltage(uavData.batVoltage);  // LTM does not have this info, calculate ourselves
    // uavData.batCurrent = calculateCurrentFromConsumedCapacity(uavData.batUsedCapacity);
  }

  if (LTMcmd == LIGHTTELEMETRY_OFRAME)
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

      //    dummy = (int32_t)(ltmread_u32()) / 100.0f; // altitude from cm to m.
      //    dummy = ltmread_u8();
      //    dummy = ltmread_u8();
    }
  }
}

void serialLTMreceive(uint8_t c) {

  static enum _serial_state {
    IDLE,
    HEADER_START1,
    HEADER_START2,
    HEADER_MSGTYPE,
    HEADER_DATA
  }
  c_state = IDLE;

  //    uavData.flagTelemetryOk = ((millis() - lastLTMpacket) < 500) ? 1 : 0;

  if (c_state == IDLE) {
    c_state = (c == '$') ? HEADER_START1 : IDLE;
    //Serial.println("header $" );
  }
  else if (c_state == HEADER_START1) {
    c_state = (c == 'T') ? HEADER_START2 : IDLE;
    //Serial.println("header T" );
  }
  else if (c_state == HEADER_START2) {
    switch (c) {
    case 'G':
      LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
      c_state = HEADER_MSGTYPE;
      break;
    case 'A':
      LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
      c_state = HEADER_MSGTYPE;
      break;
    case 'S':
      LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
      c_state = HEADER_MSGTYPE;
      break;
    case 'O':
      LTMframelength = LIGHTTELEMETRY_OFRAMELENGTH;
      c_state = HEADER_MSGTYPE;
      break;
    default:
      c_state = IDLE;
    }
    LTMcmd = c;
    LTMreceiverIndex = 0;
  }
  else if (c_state == HEADER_MSGTYPE) {
    if (LTMreceiverIndex == 0) {
      LTMrcvChecksum = c;
    }
    else {
      LTMrcvChecksum ^= c;
    }
    if (LTMreceiverIndex == LTMframelength - 4) { // received checksum byte
      if (LTMrcvChecksum == 0) {
        ltm_check();
        c_state = IDLE;
      }
      else {                                                   // wrong checksum, drop packet
        c_state = IDLE;

      }
    }
    else LTMserialBuffer[LTMreceiverIndex++] = c;
  }
}

#endif // PROTOCOL_LTM


































