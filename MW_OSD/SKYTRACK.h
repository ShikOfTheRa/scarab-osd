// Vars
struct __SLvar {
  uint8_t index;
}
SLvar;

uint8_t SLserialBuffer[100];


uint8_t SLread_u8(uint8_t val)  {
  return SLserialBuffer[val];
}

uint16_t SLread_u16(uint8_t val) {
  uint16_t t = SLread_u8(val++);
  t |= (uint16_t)SLread_u8(val) << 8;
  return t;
}

uint32_t SLread_u32(uint8_t val) {
  uint32_t t = SLread_u16(val);
  t |= (uint32_t)SLread_u16(val+2) << 16;
  return t;
}


void SL_sync(){ 
#ifdef ALARM_MSP
  timer.MSP_active=ALARM_MSP;             // getting something on serial port
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
    SLvar.index=0;
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
    SLserialBuffer[SLvar.index++] = c;
    SLvar.index++; 
    c_state = (SLvar.index == SLserialBuffer[2]) ? SL_PAYLOAD : SL_LENGTH; 
  }
  else if (c_state == SL_PAYLOAD) {
    SLserialBuffer[SLvar.index++] = c;
    SLvar.index++; 
    c_state = (SLvar.index == 0x16) ? SL_FLAG : SL_IDLE;
    //c_state = (SLvar.index == SLserialBuffer[2]) ? SL_CHECKSUM : SL_IDLE;
  }
  else if (c_state == SL_FLAG) {
    c_state = (c == 0) ? SL_CHECKSUM : SL_IDLE;
  }
  else if (c_state == SL_CHECKSUM) {
//    if checksum...
    SL_sync();
    c_state = SL_IDLE;
  }
}




































