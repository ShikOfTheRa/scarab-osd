

uint8_t kissread_u8(uint8_t index) {
  return KISSserialBuffer[index];
}

uint16_t kissread_u16(uint8_t index) {
  uint16_t t = kissread_u8(index) << 8;
  t |= (uint16_t)kissread_u8(index + 1);
  return t;
}

uint32_t kissread_u32(uint8_t index) {
  uint32_t t = kissread_u16(index) << 8;
  t |= (uint32_t)kissread_u16(index + 2);
  return t;
}

uint32_t ESC_filter(uint32_t oldVal, uint32_t newVal) {
  return (uint32_t)((uint32_t)((uint32_t)((uint32_t)oldVal * ESC_FILTER) + (uint32_t)newVal)) / (ESC_FILTER + 1);
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

void kiss_sync() {
  timer.packetcount++;
#ifdef DATA_MSP
  timer.MSP_active = DATA_MSP;           // getting something on serial port
#endif

#ifdef KISSGPS
  if (Kvar.framelength == 1) { // GPS telemetry packet - need to establish correct length
#else
  if (0){ //Never use GPS telemetry
#endif    
    GPS_latitude      = kissread_u16(KISS_INDEX_GPS_SPEED);
    GPS_longitude     = kissread_u16(KISS_INDEX_GPS_SPEED);
    GPS_speed         = kissread_u16(KISS_INDEX_GPS_SPEED) / 100;
    GPS_ground_course = kissread_u16(KISS_INDEX_GPS_COURSE);
    GPS_altitude      = kissread_u16(KISS_INDEX_GPS_ALTITUDE);
    GPS_fix           = kissread_u8(KISS_INDEX_GPS_NUMSATFIX) >> 7;
    GPS_numSat        = (kissread_u8(KISS_INDEX_GPS_NUMSATFIX)) & 0x7F;
    
    MwHeading = GPS_ground_course / 10;
    if (MwHeading >= 180) 
      MwHeading -= 360;
    MwAltitude = (int32_t)GPS_altitude*100;
  }
  else { // Standard telemtry packet
    armed = kissread_u8(KISS_INDEX_CURRENT_ARMED);
    MwVBat = kissread_u16(KISS_INDEX_LIPOVOLT) / 10;
    MwAngle[0] = (int16_t)kissread_u16(KISS_INDEX_ANGLE0) / 10;
    MwAngle[1] = (int16_t)kissread_u16(KISS_INDEX_ANGLE1) / 10;
    Kvar.mode = kissread_u8(KISS_INDEX_MODE);
    Kvar.mode = (Kvar.mode > 2) ? 0 : Kvar.mode;
    MwRcData[1] = 1000 + (int16_t)kissread_u16(KISS_INDEX_THROTTLE);
    for (uint8_t i = 1; i < 8; i++) {
      MwRcData[i + 1] = 1500 + (int16_t)kissread_u16(i * 2);
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
        filtereddata[i] = ESC_filter((uint32_t)filtereddata[i], (uint32_t)((KISSserialBuffer[KISS_INDEX_ESC1_AMP + (i * 10)] << 8) | KISSserialBuffer[KISS_INDEX_ESC1_AMP + 1 + (i * 10)]) << 4);
        MWAmperage     += filtereddata[i] >> 4;
      }
      //    MWAmperage/=10;
      amperagesum = (uint32_t)360 * kissread_u16(KISS_INDEX_MAH);
    }
  }
}

void serialKISSreceive(uint8_t c) {
  static enum _serial_state {
    KISS_IDLE,
    KISS_HEADER_INIT,
    KISS_HEADER_SIZE,
    KISS_PAYLOAD,
  }
  c_state = KISS_IDLE;

  if (c_state == KISS_IDLE) {
    Kvar.index = 0;
    Kvar.cksumtmp = 0;
    c_state = (c == KISSFRAMEINIT) ? KISS_HEADER_INIT : KISS_IDLE;
  }
  else if (c_state == KISS_HEADER_INIT) {
    Kvar.framelength = c;
    c_state = KISS_HEADER_SIZE;
  }
  else if (c_state == KISS_HEADER_SIZE) {
    if (Kvar.index < KISSFRAMELENGTH) {
      KISSserialBuffer[Kvar.index] = c;
    }
    Kvar.cksumtmp += c;
    Kvar.index++;
    if (Kvar.index == Kvar.framelength) {
      c_state = KISS_PAYLOAD;
    }
  }
  else if (c_state == KISS_PAYLOAD) {
    if ((Kvar.cksumtmp / Kvar.framelength) == c) {
      kiss_sync();
    }
    c_state = KISS_IDLE; // Go straight to idle to avoid missing every other packet
  }
  else {
    c_state = KISS_IDLE;
  }
}



