

void mav_checksum(uint8_t val) {
  uint16_t tmp;
  tmp = val ^ mw_mav.serial_checksum &0xFF;
  tmp ^= (tmp<<4)&0xFF;
  mw_mav.serial_checksum = (mw_mav.serial_checksum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);  
}


float serialbufferfloat(uint8_t offset){
  float f_temp;
  byte * b = (byte *) &f_temp;
  for(uint8_t i=0;i<4;i++){
    b[i]=serialBuffer[offset+i];
  }
  return f_temp;
}


int32_t serialbufferint(uint8_t offset){
  int32_t i_temp;
  byte * b = (byte *) &i_temp;
  for(uint8_t i=0;i<4;i++){
    b[i]=serialBuffer[offset+i];
  }
  return i_temp;
}


void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                 // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * mw_mav.GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;

  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output radians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}


void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  mw_mav.GPS_scaleLonDown = cos(rads);
}


void GPS_reset_home_position() {
  GPS_home[LAT] = GPS_latitude;
  GPS_home[LON] = GPS_longitude;
  GPS_altitude_home = GPS_altitude;
  //  GPS_calc_longitude_scaling(GPS_home[LAT]);
}

void mav_tx_checksum_func(int val) {
  long tmp;
  tmp = val ^ mw_mav.tx_checksum &0xFF;
  tmp ^= (tmp<<4)&0xFF;
  mw_mav.tx_checksum = ( mw_mav.tx_checksum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);  
}


void mav_serialize8(uint8_t val) {
  mav_tx_checksum_func(val);
  Serial.write(val);
}


void mav_serialize16(uint16_t val) {
  mav_serialize8((val   ) & 0xFF);
  mav_serialize8((val>>8) & 0xFF);
}


void mavlink_msg_request_data_stream_send(uint8_t MAVStreams, uint16_t MAVRates){
  //head:
  mw_mav.tx_checksum=0xFFFF; //init
  Serial.write(0xFE);
  mav_serialize8(mw_mav.message_length);
  mav_serialize8(mw_mav.sequence&0xFF);
  mav_serialize8(99);
  mav_serialize8(99);
  mav_serialize8(MAVLINK_MSG_ID_REQUEST_DATA_STREAM);  
  //body:
  mav_serialize16(MAVRates); //MAVRates
  mav_serialize8(mw_mav.message_sysid);
  mav_serialize8(mw_mav.message_component);
  mav_serialize8(MAVStreams);
  mav_serialize8(1);
  //tail:
  mav_checksum(MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MAGIC);
  Serial.write((uint8_t)(mw_mav.serial_checksum&0xFF));
  Serial.write((uint8_t)(mw_mav.serial_checksum>>8&0xFF));
}


void request_mavlink_rates(){
  const int  maxStreams = 6;
  const uint8_t MAVStreams[maxStreams] = {
    MAV_DATA_STREAM_RAW_SENSORS,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    MAV_DATA_STREAM_RC_CHANNELS,
    MAV_DATA_STREAM_POSITION,
    MAV_DATA_STREAM_EXTRA1, 
    MAV_DATA_STREAM_EXTRA2
  };
  const uint16_t MAVRates[maxStreams] = {
    0x02, 0x02, 0x05, 0x02, 0x05, 0x02                  
  };
  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_send(MAVStreams[i], MAVRates[i]);
  }
}


void serialLTMCheck(){
#ifdef MSPACTIVECHECK
  timer.MSP_active=MSPACTIVECHECK; // getting valid LTM on serial port
#endif //MSPACTIVECHECK
  int16_t MwHeading360;
  uint8_t apm_mav_type=0;
  uint8_t osd_mode=serialbufferint(0);
  switch(mw_mav.message_cmd) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    mode.armed      = (1<<0);
    mode.gpshome    = (1<<4);
    mode.gpshold    = (1<<5);
    mode.gpsmission = (1<<6);
    MwSensorActive&=0xFFFFFF8E;
    apm_mav_type=serialBuffer[4];   
    mw_mav.mode=serialbufferint(0);
    if (serialBuffer[6]&(1<<7)){     //armed
      MwSensorActive|=(1<<0);
      armed=1;
    }
    else{
      armed=0;
      GPS_fix_HOME=0;
    }
    /* maybe implement MWOSD mode icons ?
     if (apm_mav_mode==11)      //RTH
     MwSensorActive|=(1<<4);
     if (apm_mav_mode==1)       //HOLD
     MwSensorActive|=(1<<5);
     if (apm_mav_mode==99)      //MISSION
     MwSensorActive|=(1<<6);
     */
#if defined MAVLINKREQ
    request_mavlink_rates();
#endif //MAVLINKREQ
    break;
  case MAVLINK_MSG_ID_VFR_HUD:
    GPS_speed=(int16_t)serialbufferfloat(4)*100;    // m/s-->cm/s 
    GPS_altitude=(int16_t)serialbufferfloat(8);     // m-->m
    if (GPS_fix_HOME == 0){
      GPS_reset_home_position();
    }
    GPS_altitude=GPS_altitude - GPS_altitude_home;
    MwAltitude = (int32_t) GPS_altitude *100;       // m--cm gps to baro
    MwHeading=serialBuffer[16]|serialBuffer[17]<<8; // deg (-->deg*10 if GPS heading)
    MwHeading360=MwHeading;
    if (MwHeading360>180)
      MwHeading360 = MwHeading360-360;
    MwHeading   = MwHeading360;
    MwVario=(int16_t)serialbufferfloat(12)*100;     // m/s-->cm/s
    break;
  case MAVLINK_MSG_ID_ATTITUDE:
    MwAngle[0]=(int16_t)(serialbufferfloat(4)*57296/10);     // rad-->0.1deg
    MwAngle[1]=(int16_t)(serialbufferfloat(8)*57296/10);     // rad-->0.1deg
    break;
  case MAVLINK_MSG_ID_GPS_RAW_INT:
#ifdef GPSACTIVECHECK
    timer.GPS_active=GPSACTIVECHECK;
#endif //GPSACTIVECHECK
    GPS_numSat=serialBuffer[29];                                                                         
    GPS_fix=serialBuffer[28];                                                                            
    GPS_ground_course = (serialBuffer[26]|(serialBuffer[27]<<8))/10;
    GPS_latitude =serialbufferint(8);
    GPS_longitude=serialbufferint(12);
    if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX)) {
      uint32_t dist;
      int32_t  dir;
      GPS_distance_cm_bearing(&GPS_latitude,&GPS_longitude,&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
      GPS_distanceToHome = dist/100;
      GPS_directionToHome = dir/100;
    } 
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    MwRssi=(uint16_t)(((103)*serialBuffer[21])/10);
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = (int16_t)(serialBuffer[4+(i*2)]|(serialBuffer[5+(i*2)]<<8));
    handleRawRC();
    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    mode.stable = 2;
    mode.baro   = 4;
    mode.mag    = 8;
    MwSensorActive&=0xFFFFFFF1;
    if ((serialbufferint(4)&(1<<1))>0) //acc
      MwSensorActive|=(1<<1);
    if ((serialbufferint(4)&(1<<3))>0) //baro1
      MwSensorActive|=(1<<2);
    if ((serialbufferint(4)&(1<<4))>0) //baro2
      MwSensorActive|=(1<<2);
    if ((serialbufferint(4)&(1<<2))>0) //mag
      MwSensorActive|=(1<<3);
    MwVBat=(serialBuffer[14]|(serialBuffer[15]<<8))/100;
    MWAmperage=serialBuffer[16]|(serialBuffer[17]<<8);
    break;
  }
  if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX) && armed){
    GPS_fix_HOME = 1;
  }
  else{
    GPS_altitude = 0 ;
    MwAltitude = 0 ;          
    GPS_distanceToHome = 0;
    GPS_directionToHome = 0;
    GPS_fix_HOME = 0;
  }
}


void serialMAVreceive(uint8_t c)
{
  static uint8_t mav_payload_index; 
  static uint16_t mav_checksum_rcv; 

  static enum _serial_state {
    MAV_IDLE,
    MAV_HEADER_START,
    MAV_HEADER_LEN,
    MAV_HEADER_SEQ,
    MAV_HEADER_SYS,
    MAV_HEADER_COMP,
    MAV_HEADER_MSG,
    MAV_PAYLOAD,
    MAV_CHECKSUM,
  }
  mav_state = MAV_IDLE;

  if ((mav_state == MAV_IDLE)||(mav_state == MAV_PAYLOAD))
  {
  }
  else
  {
    mav_checksum(c);
  }

  if (mav_state == MAV_IDLE)
  {
    if (c==0xFE)
    {
      mw_mav.serial_checksum=0xFFFF;
      mav_payload_index=0;
      mav_state = MAV_HEADER_START;
    }
    else
    {
      mav_state = MAV_IDLE;
    } 
  }
  else if (mav_state == MAV_HEADER_START)
  {
    mw_mav.message_length = c;
    mav_state = MAV_HEADER_LEN;
    if ((mav_payload_index) > SERIALBUFFERSIZE){  // too much data so reset check
      mav_state = MAV_IDLE;
    }
  }
  else if (mav_state == MAV_HEADER_LEN)
  {
    mav_state = MAV_HEADER_SEQ;
  }
  else if (mav_state == MAV_HEADER_SEQ)
  {
    mw_mav.message_sysid = c;
    mav_state = MAV_HEADER_SYS;
  }
  else if (mav_state == MAV_HEADER_SYS)
  {
    mw_mav.message_component = c;
    mav_state = MAV_HEADER_COMP;
  }
  else if (mav_state == MAV_HEADER_COMP)
  {
    mw_mav.message_cmd = c;
    mav_state = MAV_HEADER_MSG;
  }
  else if (mav_state == MAV_HEADER_MSG)
  {
    serialBuffer[mav_payload_index]=c;
    mav_payload_index++;
    if (mav_payload_index==mw_mav.message_length){  // end of data
      mav_state = MAV_PAYLOAD;
    }
  }
  else if (mav_state == MAV_PAYLOAD)
  {
    if (mav_payload_index==mw_mav.message_length){
      mav_checksum_rcv=c;
      mav_payload_index++;
    }
    else{
      mav_checksum_rcv+=(c<<8);
      int8_t mav_magic;
      switch(mw_mav.message_cmd) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        mav_magic = MAVLINK_MSG_ID_HEARTBEAT_MAGIC;
        break;
      case MAVLINK_MSG_ID_VFR_HUD:
        mav_magic = MAVLINK_MSG_ID_VFR_HUD_MAGIC;
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        mav_magic = MAVLINK_MSG_ID_ATTITUDE_MAGIC;
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        mav_magic = MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC;
        break;
      case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        mav_magic = MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC;
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        mav_magic = MAVLINK_MSG_ID_SYS_STATUS_MAGIC;
        break;
      }
      mav_checksum(mav_magic);
      if(mav_checksum_rcv == mw_mav.serial_checksum) {
        serialMAVCheck();
      }
      mav_state = MAV_IDLE;
    }
  }
}

/* #################################################################################################################
* LightTelemetry protocol (LTM)
*
* Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds).
*
* Protocol details: 3 different frames, little endian.
*   G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
*    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0
*     $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
*   A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
*     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0
*      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
*   S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
*     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0
*      $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
* ################################################################################################################# */

#ifdef PROTOCOL_LIGHTTELEMETRY

#define LIGHTTELEMETRY_START1 0x24 //$
#define LIGHTTELEMETRY_START2 0x54 //T
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_OFRAME 0x4F  //O OSD additionals data ( home pos, home alt, ddirection to home )
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

static uint8_t LTM_ok = 0;
static uint32_t lastLTMpacket;

// --------------------------------------------------------------------------------------
// Decoded received commands
void ltm_check() {
    uint32_t dummy;

    LTMreadIndex = 0;
    LTM_ok = 1;
    lastLTMpacket = millis();

    if (LTMcmd == LIGHTTELEMETRY_GFRAME)
    {
        uavData.gpsLatitude = (int32_t)ltmread_u32();
        uavData.gpsLongitude = (int32_t)ltmread_u32();
        uavData.gpsSpeed = ltmread_u8() * 100;            // LTM gives m/s, we expect cm/s
        uavData.altitude = ((int32_t)ltmread_u32());      // altitude from cm to m.
        uint8_t ltm_satsfix = ltmread_u8();

        uavData.gpsNumSat = (ltm_satsfix >> 2) & 0xFF;
        uavData.gpsFix    = ((ltm_satsfix & 0b00000011) <= 1) ? 0 : 1;

        // hpdate home distance and bearing
        if (uavData.gpsFixHome) {
            float rads = fabs((float)uavData.gpsHomeLatitude / 10000000.0f) * 0.0174532925f;
            float scaleLongDown = cos(rads);
            float dstlon, dstlat;

            //DST to Home
            dstlat = fabs(uavData.gpsHomeLatitude - uavData.gpsLatitude) * 1.113195f;
            dstlon = fabs(uavData.gpsHomeLongitude - uavData.gpsLongitude) * 1.113195f * scaleLongDown;
            uavData.gpsHomeDistance = sqrt(sq(dstlat) + sq(dstlon)) / 100.0;

            //DIR to Home
            dstlon = (uavData.gpsHomeLongitude - uavData.gpsLongitude); //OffSet_X
            dstlat = (uavData.gpsHomeLatitude - uavData.gpsLatitude) * (1.0f / scaleLongDown); //OffSet Y

            float bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); //absolut home direction
            if (bearing < 0) bearing += 360;//normalization
            bearing = bearing - 180;//absolut return direction
            if (bearing < 0) bearing += 360;//normalization
            uavData.gpsHomeBearing = bearing;
        }
        else {
            uavData.gpsHomeBearing = 0;
        }

        LTMpassed = 1;
    }

    if (LTMcmd == LIGHTTELEMETRY_AFRAME)
    {
        uavData.anglePitch = (int16_t)ltmread_u16() * 10;
        uavData.angleRoll = (int16_t)ltmread_u16() * 10 ;
        uavData.heading = (int16_t)ltmread_u16();
        if (uavData.heading < 0 ) uavData.heading = uavData.heading + 360; //convert from -180/180 to 0/360°
        LTMpassed = 1;
    }
    if (LTMcmd == LIGHTTELEMETRY_SFRAME)
    {
        uavData.batVoltage = ltmread_u16();
        uavData.batUsedCapacity = ltmread_u16();
        uavData.rssi = ltmread_u8();
        uavData.airspeed = ltmread_u8();

        uint8_t ltm_armfsmode = ltmread_u8();
        uavData.isArmed = (ltm_armfsmode & 0b00000001) ? 1 : 0;
        uavData.isFailsafe = (ltm_armfsmode >> 1) & 0b00000001;
        uavData.flightMode = (ltm_armfsmode >> 2) & 0b00111111;

        uavData.batCellVoltage = detectBatteryCellVoltage(uavData.batVoltage);  // LTM does not have this info, calculate ourselves
        uavData.batCurrent = calculateCurrentFromConsumedCapacity(uavData.batUsedCapacity);
    }

    if (LTMcmd == LIGHTTELEMETRY_OFRAME)
    {
        uavData.gpsHomeLatitude = (int32_t)ltmread_u32();
        uavData.gpsHomeLongitude = (int32_t)ltmread_u32();
        dummy = (int32_t)(ltmread_u32()) / 100.0f; // altitude from cm to m.
        dummy  = ltmread_u8();
        uavData.gpsFixHome = ltmread_u8();
        LTMpassed = 1;
    }
}

void readTelemetry() {
    uint8_t c;

    static enum _serial_state {
        IDLE,
        HEADER_START1,
        HEADER_START2,
        HEADER_MSGTYPE,
        HEADER_DATA
    }
    c_state = IDLE;

    uavData.flagTelemetryOk = ((millis() - lastLTMpacket) < 500) ? 1 : 0;

    while (Serial.available()) {
        c = char(Serial.read());
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
}



#endif





























