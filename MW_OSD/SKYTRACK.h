struct __SL {
  uint8_t  index;
  uint8_t  checksum;
  uint8_t  SLserialBuffer[0x20];
  uint32_t 
}
SL;


uint8_t SLread_u8(uint8_t val)  {
  return SL.SLserialBuffer[val];
}


uint16_t SLread_u16(uint8_t val) {
  uint16_t t = SLread_u8(val++);
  t |= (uint16_t)SLread_u8(val) << 8;
  return t;
}


uint32_t SLread_u32(uint8_t val) {
  uint32_t t = SLread_u16(val);
  t |= (uint32_t)SLread_u16(val + 2) << 16;
  return t;
}


void SL_sync() {
  timer.packetcount++;
#ifdef DATA_MSP
  timer.MSP_active = DATA_MSP;           // getting something on serial port
#endif
  GPS_longitude = (int32_t)SLread_u32(4);
  GPS_latitude  = (int32_t)SLread_u32(8);
  GPS_numSat    = SLread_u8(21);
}


void serialSLreceive(uint8_t c) {
  static enum _serial_state {
    SL_IDLE,
    SL_HEADER_1,
    SL_HEADER_2,
    SL_CLASS,
    SL_ID,
    SL_LENGTH,
    SL_PAYLOAD,
    SL_FLAG,
    SL_CHECKSUM
  }
  c_state = SL_IDLE;

  if (c_state == SL_IDLE) {
    SL.index = 0;
    c_state = (c == 'T') ? SL_HEADER_1 : SL_IDLE;
  }
  else if (c_state == SL_HEADER_1) {
    c_state = (c == 'M') ? SL_HEADER_2 : SL_IDLE;
  }
  else if (c_state == SL_HEADER_2) {
    c_state = (c == 0x0B) ? SL_CLASS : SL_IDLE;
  }
  else if (c_state == SL_CLASS) {
    c_state = (c == 0x0B) ? SL_ID : SL_IDLE;
  }
  else if (c_state == SL_ID) {
    c_state = (c == 0x16) ? SL_LENGTH : SL_IDLE;
    //c_state = SL_LENGTH;
  }
  else if (c_state == SL_LENGTH) {
    SL.SLserialBuffer[SL.index++] = c;
    SL.index++;
    //    c_state = (SL.index == SL.SLserialBuffer[2]) ? SL_PAYLOAD : SL_LENGTH;
    c_state = (SL.index == 0x16) ? SL_PAYLOAD : SL_LENGTH;

  }
  else if (c_state == SL_PAYLOAD) {
    c_state = (c == 0) ? SL_FLAG : SL_IDLE;
  }
  else if (c_state == SL_FLAG) {
    SL.checksum=1;  /DEVVVVVVVVVVVVVVVV
    if (SL.checksum)
      SL_sync();
    c_state = SL_IDLE;
  }
  else if (c_state == SL_CHECKSUM) {
  }
}


void DrawSkytrack() {
#define SL_LAT_POS (10*30)+2
#define SL_LON_POS (11*30)+2

  for (int xx = 0; xx < MAX_screen_size; ++xx) {
    screen[xx] = ' ';
  }

  if (1) { // uh oh - seems like we no data display last known co-ordianates...
    screenBuffer[0] = SYM_LAT;
    FormatGPSCoord(GPS_latitude, screenBuffer + 1, 4, 'N', 'S');
    MAX7456_WriteString(screenBuffer, SL_LAT_POS);
    screenBuffer[0] = SYM_LON;
    FormatGPSCoord(GPS_longitude, screenBuffer + 1, 4, 'E', 'W');
    MAX7456_WriteString(screenBuffer, SL_LON_POS);
  }
  
  displayVoltage();
}



































