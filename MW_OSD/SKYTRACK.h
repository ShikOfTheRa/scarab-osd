struct __SL {
  uint8_t  index;
  uint8_t  checksum;
  uint8_t  SLserialBuffer[0x20];
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
  GPS_longitude = SLread_u32(8);
  GPS_latitude  = SLread_u32(4);
  GPS_numSat    = SLread_u8(21);
  GPS_altitude  = SLread_u16(12);
//  pitchAngle    = SLread_u16(0);
//  rollAngle     = SLread_u16(2); 
}


void serialSLreceive(uint8_t c) {
  static enum _serial_state {
    SL_IDLE,
    SL_HEADER_1,
    SL_HEADER_2,
    SL_CLASS,
    SL_ID,
    SL_LENGTH_LSB,
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
    c_state = SL_LENGTH_LSB;
  }
  else if (c_state == SL_LENGTH_LSB) {
    c_state = SL_LENGTH;
  }
  else if (c_state == SL_LENGTH) {
    SL.SLserialBuffer[SL.index] = c;
    SL.index++;
    c_state = (SL.index == 0x16) ? SL_PAYLOAD : SL_LENGTH; // Only cater for single ID
  }
  else if (c_state == SL_PAYLOAD) {
//      #ifdef DEBUGDPOSPACKET
        timer.packetcount++;
//      #endif
    c_state = SL_CHECKSUM; 
  }
  else if (c_state == SL_CHECKSUM) {
    SL.checksum=1;  // No checksum used at present. Visual indicator only
    if (SL.checksum){
      SL_sync();
    }
    c_state = SL_IDLE;
  }
  else{ 
    c_state = SL_IDLE;
  }
}

void DrawSkytrack(){
  #define SL_WARN_POS LINE04+10
  #define SL_VOLT_POS LINE05+10
  #define SL_LAT_POS  LINE06+10
  #define SL_LON_POS  LINE07+10
  #define SL_SAT_POS  LINE08+10
  #define SL_PKT_POS  LINE09+10

  for (int xx = 0; xx < MAX_screen_size; ++xx) {
    screen[xx] = ' ';
  }
  #ifdef DEBUG
        displayDebug();
  #else

//VOLTAGE 
  uint8_t t_cells = (voltage / (CELL_VOLTS_MAX+3)) + 1; // Detect 3s > 9.0v, 4s > 13.5v, 5s > 18.0v, 6s > 22.5v power up voltage
  if (t_cells > cells){
    cells++;
  }
  voltageWarning = cells * 36;
  screenBuffer[0]=SYM_MAIN_BATT;
  uint8_t t_offset = 1;
  ItoaPadded(voltage,screenBuffer+t_offset,4,3);  
  screenBuffer[4+1] = 0;         
  t_offset = FindNull();
  screenBuffer[t_offset++] = SYM_VOLT;
  screenBuffer[t_offset] = 0;
  if (voltage<voltageWarning){
    if (timer.Blink2hz){
      MAX7456_WriteString_P(skytracktext0, SL_WARN_POS);
      MAX7456_WriteString(screenBuffer,SL_VOLT_POS);
    }
  }
// VOLTAGE

//Choose:
  if (timer.MSP_active!=0) { //Continue display last known co-ordinates if no data...
//  if (0) { // Always display...
    return;
  }

//CO-ORDINATES
    screenBuffer[0] = SYM_LAT;
    FormatGPSCoord(GPS_latitude, screenBuffer + 1, 4, 'N', 'S');
    MAX7456_WriteString(screenBuffer, SL_LAT_POS);
    screenBuffer[0] = SYM_LON;
    FormatGPSCoord(GPS_longitude, screenBuffer + 1, 4, 'E', 'W');
    MAX7456_WriteString(screenBuffer, SL_LON_POS);
//CO-ORDINATES
  
//SATS
  MAX7456_WriteString("SAT",SL_SAT_POS);
  itoa(GPS_numSat,screenBuffer,10);
  MAX7456_WriteString(screenBuffer,SL_SAT_POS+5); 
//SATS

//SERIAL PACKET COUNT    
  MAX7456_WriteString("PKT",SL_PKT_POS);
  itoa(packetrate,screenBuffer,10);
  MAX7456_WriteString(screenBuffer,SL_PKT_POS+5);
//SERIAL PACKET COUNT      


  #endif

}







































