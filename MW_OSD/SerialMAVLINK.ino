#ifdef PROTOCOL_MAVLINK


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

void mav_serialize32(uint16_t val) {
  mav_serialize8((val   ) & 0xFF);
  mav_serialize8((val>>8) & 0xFF);
  mav_serialize8((val>>16) & 0xFF);
  mav_serialize8((val>>24) & 0xFF);
}


void mavlink_msg_request_data_stream_send(uint8_t MAVStreams, uint16_t MAVRates){
  //head:
  static int8_t tx_sequence=0;
  tx_sequence++;
  mw_mav.tx_checksum=0xFFFF; //init
  Serial.write(0xFE);
  mav_serialize8(6);
  mav_serialize8(tx_sequence);
  mav_serialize8(99);
  mav_serialize8(99);
  mav_serialize8(MAVLINK_MSG_ID_REQUEST_DATA_STREAM);  
  //body:
  mav_serialize16(MAVRates); //MAVRates
  mav_serialize8(1);
  mav_serialize8(1);
  mav_serialize8(MAVStreams);
  mav_serialize8(1);
  //tail:
  mav_tx_checksum_func(MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MAGIC);
  Serial.write((uint8_t)(mw_mav.tx_checksum&0xFF));
  Serial.write((uint8_t)(mw_mav.tx_checksum>>8&0xFF));
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


void serialMAVCheck(){
#ifdef DEBUGDPOSPACKET
  timer.packetcount++;
#endif
#ifdef DATA_MSP
  timer.MSP_active=DATA_MSP; // getting valid MAV on serial port
#endif //DATA_MSP
  int16_t MwHeading360;
  static uint8_t armedglitchprotect=0;
  switch(mw_mav.message_cmd) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    mode.armed      = (1<<0);
    mode.gpshome    = (1<<4);
    mode.gpshold    = (1<<5);
    mode.gpsmission = (1<<6);
    MwSensorActive&=0xFFFFFFFE; // set disarmed

    mw_mav.mode=serialbufferint(0);
    if (mw_mav.mode>MAV_MODE_MAX) mw_mav.mode=MAV_MODE_MAX;
    /* maybe implement MWOSD mode icons ?
     if (apm_mav_mode==11)      //RTH
     MwSensorActive|=(1<<4);
     if (apm_mav_mode==1)       //HOLD
     MwSensorActive|=(1<<5);
     if (apm_mav_mode==99)      //MISSION
     MwSensorActive|=(1<<6);
     */
 
    #ifdef ALWAYSARMED
      serialBuffer[6]|=(1<<7); 
    #endif
    if (serialBuffer[6]&(1<<7)){     //armed
      MwSensorActive|=(1<<0);
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
          GPS_fix_HOME=0;
        #endif      
      }
    }
#if defined MAVLINKREQ
     static uint8_t mavreqdone=5;
     if (mavreqdone>0){ 
       request_mavlink_rates();
       mavreqdone--;
     }
#endif //MAVLINKREQ
    break;
  case MAVLINK_MSG_ID_VFR_HUD:
    GPS_speed=(int16_t)serialbufferfloat(4)*100;    // m/s-->cm/s 
    GPS_altitude=(int16_t)serialbufferfloat(8);     // m-->m
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
    MwAngle[0]=(int16_t)(serialbufferfloat(4)*57.2958*10); // rad-->0.1deg
    MwAngle[1]=(int16_t)(serialbufferfloat(8)*57.2958*10); // rad-->0.1deg
    break;
  case MAVLINK_MSG_ID_GPS_RAW_INT:
#ifdef ALARM_GPS
    timer.GPS_active=ALARM_GPS;
#endif //ALARM_GPS
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
      MwRcData[i+1] = (int16_t)(serialBuffer[4+(i*2)]|(serialBuffer[5+(i*2)]<<8));
    handleRawRC();
    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    mode.stable = (1<<1);
    mode.baro   = (1<<2);
    mode.mag    = (1<<3);
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
  if (GPS_fix_HOME == 0){
    GPS_reset_home_position();
  }
  if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX) && armed){
    GPS_fix_HOME = 1;
  }
  else{
    GPS_altitude = 0 ;
    MwAltitude = 0 ;          
    GPS_distanceToHome = 0;
    GPS_directionToHome = 0;
  }
}


void serialMAVreceive(uint8_t c)
{
  static uint8_t  mav_payload_index; 
  static uint16_t mav_checksum_rcv; 
  static uint8_t  mav_magic=0;

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
  }
  else if (mav_state == MAV_HEADER_LEN)
  {
    mav_state = MAV_HEADER_SEQ;
  }
  else if (mav_state == MAV_HEADER_SEQ)
  {
    if (c==MAV_SYS_ID){
      mav_state = MAV_HEADER_SYS;        
    }
    else{
      mav_state = MAV_IDLE;  
    }
  }
  else if (mav_state == MAV_HEADER_SYS)
  {
    if (c==MAV_COM_ID){
      mav_state = MAV_HEADER_COMP;        
    }
    else{
      mav_state = MAV_IDLE;  
    }
  }
  else if (mav_state == MAV_HEADER_COMP)
  {
    mw_mav.message_cmd = c;
    uint8_t  mav_len; 
    switch(c) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        mav_magic = MAVLINK_MSG_ID_HEARTBEAT_MAGIC;
        mav_len = MAVLINK_MSG_ID_HEARTBEAT_LEN;
        break;
      case MAVLINK_MSG_ID_VFR_HUD:
        mav_magic = MAVLINK_MSG_ID_VFR_HUD_MAGIC;
        mav_len = MAVLINK_MSG_ID_VFR_HUD_LEN;
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        mav_magic = MAVLINK_MSG_ID_ATTITUDE_MAGIC;
        mav_len = MAVLINK_MSG_ID_ATTITUDE_LEN;
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        mav_magic = MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC;
        mav_len = MAVLINK_MSG_ID_GPS_RAW_INT_LEN;
        break;
      case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        mav_magic = MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC;
        mav_len = MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN;
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        mav_magic = MAVLINK_MSG_ID_SYS_STATUS_MAGIC;
        mav_len = MAVLINK_MSG_ID_SYS_STATUS_LEN;
        break;
      }
      if ((mw_mav.message_length) == mav_len){  // too much data so reset check
         mav_state = MAV_HEADER_MSG;
      }
      else{
        mav_state = MAV_IDLE;
      }
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
      mav_checksum(mav_magic);
      if(mav_checksum_rcv == mw_mav.serial_checksum) {
        serialMAVCheck();
      }
      mav_state = MAV_IDLE;
    }
  }
}

#endif





























