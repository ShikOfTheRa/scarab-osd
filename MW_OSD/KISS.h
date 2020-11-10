

uint8_t kissread_u8(uint8_t index) {
  return serialBuffer[index];
}

uint16_t kissread_u16(uint8_t index) {
  uint16_t t = kissread_u8(index) << 8;
  t |= (uint16_t)kissread_u8(index + 1);
  return t;
}

uint32_t kissread_u32(uint8_t index) {
  uint32_t t = (uint32_t)kissread_u8(index) << 24;
  t |= (uint32_t)kissread_u8(index + 1) << 16;
  t |= (uint32_t)kissread_u8(index + 2) << 8;
  t |= (uint32_t)kissread_u8(index + 3) << 0;
  
  return t;
}

bool kissread_bool(uint8_t index) {
  return serialBuffer[index] == 1;
}

uint32_t ESC_filter(uint32_t oldVal, uint32_t newVal) {
  return (uint32_t)((uint32_t)((uint32_t)((uint32_t)oldVal * ESC_FILTER) + (uint32_t)newVal)) / (ESC_FILTER + 1);
}

void kissWriteInBuffer_u8(uint8_t index, uint8_t value) {
   serialBuffer[index] = (uint8_t)(value);
}

void kissWriteInBuffer_u16(uint8_t index, uint16_t value) {
  serialBuffer[index] = (uint8_t)((value) >> 8);
   serialBuffer[index + 1] = (uint8_t)(value);
}


uint16_t calculateCurrentFromConsumedCapacity(uint16_t mahUsed)
{
  static unsigned long previous_millis = 0;
  static uint16_t previous_mahUsed = 0;
  static uint16_t calculatedCurrent = 0;
  unsigned long current_millis = millis();
  if ((current_millis - previous_millis) > 5000 || (previous_mahUsed > mahUsed)) { // Stalled or invalid telemetry. Reset statistics
    calculatedCurrent = 0;
    previous_mahUsed = mahUsed;
    previous_millis = current_millis;
  }
  else if ((current_millis - previous_millis) > 500 && (previous_mahUsed < mahUsed) ) {
    calculatedCurrent = uint32_t ((mahUsed - previous_mahUsed) * 360 * 2 * 500 / (current_millis - previous_millis)); // A * 10
    previous_mahUsed = mahUsed;
    previous_millis = current_millis;
  }
  return calculatedCurrent;
}

#ifdef KISSGPS

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, uint32_t* dist, int32_t* bearing) {
  float rads = (abs((float)*lat1) / 10000000.0) * 0.0174532925;
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * cos(rads);
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_reset_home_position() {
  GPS_home[LAT] = GPS_latitude;
  GPS_home[LON] = GPS_longitude;
  GPS_home_altitude=GPS_altitude;
}

void kiss_sync_gps () {
timer.packetcount++;
#ifdef DATA_MSP
  timer.MSP_active = DATA_MSP;           // getting something on serial port
#endif

    GPS_fix           = kissread_u8(KISS_INDEX_GPS_NUMSATFIX) >> 7;
    GPS_numSat        = (kissread_u8(KISS_INDEX_GPS_NUMSATFIX)) & 0x7F;
    if (GPS_fix && GPS_numSat >= MINSATFIX) { 
      GPS_latitude      = kissread_u32(KISS_INDEX_GPS_LATITUDE);
      GPS_longitude     = kissread_u32(KISS_INDEX_GPS_LONGITUDE);
      GPS_speed         = kissread_u16(KISS_INDEX_GPS_SPEED) / 100;
      GPS_ground_course = kissread_u16(KISS_INDEX_GPS_COURSE) / 10; // Conversion to Unit degree*10 (MSP_RAW_GPS)
      GPS_altitude      = kissread_u16(KISS_INDEX_GPS_ALTITUDE);
      GPS_altitude_ASL  = GPS_altitude;

      // calculation of the GPS direction of the drone on a 180 degree scale
      MwHeading = GPS_ground_course / 10;
      if (MwHeading >= 180) 
        MwHeading -= 360;
      MwAltitude = (int32_t)GPS_altitude*100;

      // checking if the home position is to be fixed
      if (GPS_fix_HOME == 0){
        GPS_reset_home_position();
        GPS_fix_HOME=1;
      }
      // distance and direction calculation from home position
      uint32_t dist;
      int32_t  dir;
      GPS_distance_cm_bearing(&GPS_latitude,&GPS_longitude,&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
      GPS_distanceToHome = dist/100;
      GPS_directionToHome = dir/100;
    }
}
#endif

void kiss_sync_telemetry() {
  timer.packetcount++;
#ifdef DATA_MSP
  timer.MSP_active = DATA_MSP;           // getting something on serial port
#endif
  armed = kissread_u8(KISS_INDEX_CURRENT_ARMED);
  MwVBat = kissread_u16(KISS_INDEX_LIPOVOLT) / 10;
  MwAngle[0] = (int16_t)kissread_u16(KISS_INDEX_ANGLE0) / 10;
  MwAngle[1] = (int16_t)kissread_u16(KISS_INDEX_ANGLE1) / 10;
  Kvar.mode = kissread_u8(KISS_INDEX_MODE);
  Kvar.mode = (Kvar.mode > KISS_mode_RTH_index) ? 0 : Kvar.mode;
  MwRcData[1] = 1000 + (int16_t)kissread_u16(KISS_INDEX_THROTTLE);
  for (uint8_t i = 1; i < 8; i++) {
    MwRcData[i + 1] = 1500 + (int16_t)kissread_u16(i * 2);
  }
  // RC8->10 not on old versions, check if the frame contains them. MWRcData starts at 1, so shift
  if (Kvar.framelength >= KISS_INDEX_RC10) {
    MwRcData[8+1] = 1500 + (int16_t)kissread_u16(KISS_INDEX_RC8);
    MwRcData[9+1] = 1500 + (int16_t)kissread_u16(KISS_INDEX_RC9);
    MwRcData[10+1] = 1500 + (int16_t)kissread_u16(KISS_INDEX_RC10);
  }
  handleRawRC();

  static uint32_t filtereddata[6];

  if (Settings[S_MWAMPERAGE]) {
    // calculate amperage using capacity method...
    // uint16_t dummy=kissread_u16(148);
    // MWAmperage = 10*calculateCurrentFromConsumedCapacity(dummy);

    // calculate amperage using ESC sum method...
    MWAmperage = 0;
    for (uint8_t i = 0; i < 6; i++) {
      filtereddata[i] = ESC_filter((uint32_t)filtereddata[i], (uint32_t)((serialBuffer[KISS_INDEX_ESC1_AMP + (i * 10)] << 8) | serialBuffer[KISS_INDEX_ESC1_AMP + 1 + (i * 10)]) << 4);
      MWAmperage     += filtereddata[i] >> 4;
    }
    //    MWAmperage/=10;
    amperagesum = (uint32_t)360 * kissread_u16(KISS_INDEX_MAH);
  }
}

void kiss_sync_settings() {
  timer.packetcount++;
#ifdef DATA_MSP
  timer.MSP_active=DATA_MSP;             // getting something on serial port
#endif

  // PIDs
  for(uint8_t i=0; i<3; i++) {
    pidP[i] = kissread_u16(KISS_SETTINGS_IDX_PID_ROLL_P + (i * 2)) / 10;
    pidI[i] = kissread_u16(KISS_SETTINGS_IDX_PID_ROLL_I + (i * 2));
    pidD[i] = kissread_u16(KISS_SETTINGS_IDX_PID_ROLL_D + (i * 2)) / 10;
  }

  // Rates
  for(uint8_t i=0; i<3; i++) {
    rateRC[i] = kissread_u16(KISS_SETTINGS_IDX_RATE_ROLL_RC + (i * 2)) / 10;
    rateRate[i] = kissread_u16(KISS_SETTINGS_IDX_RATE_ROLL_RATE + (i * 2)) / 10;
    rateCurve[i] = kissread_u16(KISS_SETTINGS_IDX_RATE_ROLL_CURVE + (i * 2)) / 10;
  }

  Kvar.version = kissread_u8(KISS_SETTINGS_IDX_VERSION);

  // Notch Filters
  nfRollEnable = kissread_bool(KISS_SETTINGS_IDX_NF_ROLL_ENABLE);
  nfRollCenter = kissread_u16(KISS_SETTINGS_IDX_NF_ROLL_CENTER);
  nfRollCutoff = kissread_u16(KISS_SETTINGS_IDX_NF_ROLL_CUTOFF);
  nfPitchEnable = kissread_bool(KISS_SETTINGS_IDX_NF_PITCH_ENABLE);
  nfPitchCenter = kissread_u16(KISS_SETTINGS_IDX_NF_PITCH_CENTER);
  nfPitchCutoff = kissread_u16(KISS_SETTINGS_IDX_NF_PITCH_CUTOFF);

  // Low Pass Filters
  rpLPF = kissread_u8(KISS_SETTINGS_IDX_RP_LPF);
  yawLPF = kissread_u8(KISS_SETTINGS_IDX_YAW_LPF);
  dtermLPF = kissread_u8(KISS_SETTINGS_IDX_DTERM_LPF);

  // Yaw C Filter
  yawCFilter = kissread_u8(KISS_SETTINGS_IDX_YAW_C_FILTER);

  // VTX
  vtxType = kissread_u8(KISS_SETTINGS_IDX_VTX_TYPE);
  vtxNChannel = kissread_u8(KISS_SETTINGS_IDX_VTX_N_CHANNEL);
  vtxBand = vtxNChannel / 8;
  vtxChannel = vtxNChannel % 8 + 1;
  vtxLowPower = kissread_u16(KISS_SETTINGS_IDX_VTX_LOW_POWER);
  vtxMaxPower = kissread_u16(KISS_SETTINGS_IDX_VTX_MAX_POWER);

  modeMSPRequests &=~ REQ_MSP_KISS_SETTINGS;
}

void kiss_message() {
  // No new message
  if (Kvar.framelength <= 1) {
    return;
  }

  uint8_t newPriorityMessage = kissread_u8(KISS_GET_MESSAGE_PRIORITY);
  // Check if new message has higher priority or if old one is still displayed
  if (newPriorityMessage >= kissMessagePriority || timer.fcMessage == 0) {
    kissMessagePriority = newPriorityMessage;
    timer.fcMessage = kissread_u16(KISS_GET_MESSAGE_DURATION) / 1000;

    fcMessageLength = Kvar.framelength - KISS_GET_MESSAGE_MESSAGE;
    if (fcMessageLength > KISS_MAX_MESSAGE_SIZE) {
      fcMessageLength = KISS_MAX_MESSAGE_SIZE;
    }
    // message loading
    for (uint8_t i = KISS_GET_MESSAGE_MESSAGE; i < Kvar.framelength && (i - KISS_GET_MESSAGE_MESSAGE) < KISS_MAX_MESSAGE_SIZE; i++) {
      // Message display start at 1
      fontData[i - KISS_GET_MESSAGE_MESSAGE + 1] = toupper(serialBuffer[i]);
    }
  }

}

uint8_t kissProtocolCRC8(const uint8_t *data, uint8_t startIndex, uint8_t stopIndex) 
{
  uint8_t crc = 0;
  for (uint8_t i = startIndex; i < stopIndex; i++) 
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) 
    {
      if ((crc & 0x80) != 0) 
      {
        crc = (uint8_t) ((crc << 1) ^ 0xD5);
      } 
      else 
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint8_t kissProtocolChecksum(const uint8_t *data, uint8_t startIndex, uint8_t stopIndex) 
{
  uint8_t checksum = 0;
  for (uint8_t i = startIndex; i < stopIndex; i++) 
  {
    checksum += data[i];
  }
  return checksum;
}

void serialKISSsendData(uint8_t request, uint8_t dataSize) {
  uint8_t crc = 0;
  if (Kvar.version > 0 && Kvar.version < 109) {
    crc = kissProtocolChecksum(serialBuffer, 0, dataSize);
  } else {
    crc = kissProtocolCRC8(serialBuffer, 0, dataSize);
  }
  KISScurrentRequest = request;
  Serial.write(request);
  if (request != KISS_GET_TELEMETRY && request != KISS_GET_SETTINGS) {
    Serial.write(dataSize);
    for (int i = 0; i < dataSize;i++) {
      Serial.write(serialBuffer[i]);
    }
    Serial.write(crc);
  }
}

void kiss_send_pids() {
  for(uint8_t i=0; i<3; i++) {
    kissWriteInBuffer_u16(KISS_SET_PID_IDX_PID_ROLL_P + (i * 6), pidP[i] * 10);
    kissWriteInBuffer_u16(KISS_SET_PID_IDX_PID_ROLL_I + (i * 6), pidI[i]);
    kissWriteInBuffer_u16(KISS_SET_PID_IDX_PID_ROLL_D + (i * 6), pidD[i] * 10);
  }

  serialKISSsendData(KISS_SET_PIDS, 18);
}

void kiss_send_rates() {
  for(uint8_t i=0; i<3; i++) {
    kissWriteInBuffer_u16(KISS_SET_RATE_IDX_ROLL_RC + (i * 6), rateRC[i] * 10);
    kissWriteInBuffer_u16(KISS_SET_RATE_IDX_ROLL_RATE + (i * 6), rateRate[i] * 10);
    kissWriteInBuffer_u16(KISS_SET_RATE_IDX_ROLL_CURVE + (i * 6), rateCurve[i] * 10);
  }

  serialKISSsendData(KISS_SET_RATES, 18);
}

void kiss_send_filters() {
  kissWriteInBuffer_u8(KISS_SET_FILTER_IDX_RP_LPF, rpLPF);
  kissWriteInBuffer_u8(KISS_SET_FILTER_IDX_YAW_C_FILTER, yawCFilter);
  kissWriteInBuffer_u8(KISS_SET_FILTER_IDX_NF_ROLL_ENABLE, nfRollEnable);
  kissWriteInBuffer_u16(KISS_SET_FILTER_IDX_NF_ROLL_CENTER, nfRollCenter);
  kissWriteInBuffer_u16(KISS_SET_FILTER_IDX_NF_ROLL_CUTOFF, nfRollCutoff);
  kissWriteInBuffer_u8(KISS_SET_FILTER_IDX_NF_PITCH_ENABLE, nfPitchEnable);
  kissWriteInBuffer_u16(KISS_SET_FILTER_IDX_NF_PITCH_CENTER, nfPitchCenter);
  kissWriteInBuffer_u16(KISS_SET_FILTER_IDX_NF_PITCH_CUTOFF, nfPitchCutoff);
  kissWriteInBuffer_u8(KISS_SET_FILTER_IDX_YAW_LPF, yawLPF);
  kissWriteInBuffer_u8(KISS_SET_FILTER_IDX_DTERM_LPF, dtermLPF);

  serialKISSsendData(KISS_SET_FILTERS, 14);
}

void kiss_send_vtx() {
  kissWriteInBuffer_u8(KISS_SET_VTX_IDX_TYPE, vtxType);
  vtxNChannel = vtxBand * 8 + vtxChannel - 1;
  kissWriteInBuffer_u8(KISS_SET_VTX_IDX_N_CHANNEL, vtxNChannel);
  kissWriteInBuffer_u16(KISS_SET_VTX_IDX_LOW_POWER, vtxLowPower);
  kissWriteInBuffer_u16(KISS_SET_VTX_IDX_MAX_POWER, vtxMaxPower);

  serialKISSsendData(KISS_SET_VTX, 6);
}

  static enum _serial_state {
    KISS_IDLE,
    KISS_HEADER_INIT,
    KISS_HEADER_SIZE,
    KISS_PAYLOAD,
  }
  c_state = KISS_IDLE;
  
void serialKISSsendRequestIfPossible(uint8_t request) {
  static unsigned long previous_millis = 0;
  unsigned long current_millis = millis();
  
  if (c_state == KISS_IDLE) {
    serialKISSsendData(request, 0);
    previous_millis = current_millis;
  } else {
    if ((current_millis - previous_millis) > 20) {
      c_state = KISS_IDLE;
    }
  }
}

void serialKISSreceive(uint8_t c) {
  if (c_state == KISS_IDLE) {
    Kvar.index=0;
    Kvar.cksumtmp=0;
    Kvar.crc8=0;
    if (KISScurrentRequest == KISS_GET_TELEMETRY || KISScurrentRequest == KISS_GET_SETTINGS){
      c_state = (c == KISSFRAMEINIT) ? KISS_HEADER_INIT : KISS_IDLE;
    } else {
      c_state = (c == KISScurrentRequest) ? KISS_HEADER_INIT : KISS_IDLE;
    }
  }
  else if (c_state == KISS_HEADER_INIT) {
    Kvar.framelength = c;
    c_state = KISS_HEADER_SIZE;
  }
  else if (c_state == KISS_HEADER_SIZE) {
    if (Kvar.index < KISSFRAMELENGTH) {
      serialBuffer[Kvar.index] = c;
    }
    Kvar.cksumtmp+=c;

    Kvar.crc8 ^= c;
    for (uint8_t j = 0; j < 8; j++) 
    {
      if ((Kvar.crc8 & 0x80) != 0) 
      {
        Kvar.crc8 = (uint8_t) ((Kvar.crc8 << 1) ^ 0xD5);
      } 
      else 
      {
        Kvar.crc8 <<= 1;
      }
    }
    Kvar.index++;
    if (Kvar.index == Kvar.framelength) {
      c_state = KISS_PAYLOAD;
    }
  }
  else if (c_state == KISS_PAYLOAD) {
    if ((Kvar.cksumtmp/Kvar.framelength) == c || Kvar.crc8 == c) {
      switch(KISScurrentRequest) {
        case KISS_GET_TELEMETRY:
          kiss_sync_telemetry();
          break;
        case KISS_GET_SETTINGS:
          kiss_sync_settings();
          break;
        case KISS_GET_MESSAGE:
          kiss_message();
          break;
#ifdef KISSGPS
        case KISS_GET_GPS:
          kiss_sync_gps();
          break;
#endif
      }
    }
    c_state = KISS_IDLE; // Go straight to idle to avoid missing every other packet
  }
  else {
    c_state = KISS_IDLE;
  }
}
