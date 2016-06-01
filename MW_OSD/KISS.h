

uint8_t kissread_u8(uint8_t index){
  return KISSserialBuffer[index];
}

uint16_t kissread_u16(uint8_t index) {
  uint16_t t = kissread_u8(index) << 8;
  t |= (uint16_t)kissread_u8(index+1);
  return t;
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
    calculatedCurrent = uint32_t ((mahUsed - previous_mahUsed) * 360 *2 *500 / (current_millis - previous_millis)); // A * 10
    previous_mahUsed = mahUsed;
    previous_millis = current_millis;
  }
  return calculatedCurrent;
}

void kiss_sync() {
#ifdef MSPACTIVECHECK
  timer.MSP_active=MSPACTIVECHECK;             // getting something on serial port
#endif
  armed = kissread_u8(16);
  MwVBat = kissread_u16(17)/10;
  MwAngle[0]=(int16_t)kissread_u16(31)/10;
  MwAngle[1]=(int16_t)kissread_u16(33)/10;
  Kvar.mode = kissread_u8(65); 
  MwRcData[0]=1000+(int16_t)kissread_u16(0);
  for(uint8_t i=1; i<8; i++) {
    MwRcData[i]=1500+(int16_t)kissread_u16(i*2);
  } 
  handleRawRC();
  if (Settings[S_MWAMPERAGE]){ 
    // calculate amperage using capacity method...
    // uint16_t dummy=kissread_u16(148);
    // MWAmperage = 10*calculateCurrentFromConsumedCapacity(dummy);

    // calculate amperage using ESC sum method...    
    MWAmperage=0;    
    for(uint8_t i=0; i<6; i++) {
      MWAmperage+=kissread_u16(85+(i*10));
    }  
    MWAmperage/=10;   
    amperagesum = 360* kissread_u16(150);
  }
}

void serialKISSreceive(uint8_t c) {
  static enum _serial_state {
    KISS_IDLE,
    KISS_HEADER_INIT,
    KISS_HEADER_SIZE,
    KISS_HEADER_PAYLOAD,
  }
  c_state = KISS_IDLE;

  if (c_state == KISS_IDLE) {
    Kvar.index=0;
    c_state = (c == KISSFRAMEINIT) ? KISS_HEADER_INIT : KISS_IDLE;
  }
  else if (c_state == KISS_HEADER_INIT) {
    c_state = (c == KISSFRAMELENGTH) ? KISS_HEADER_SIZE : KISS_IDLE;
  }
  else if (c_state == KISS_HEADER_SIZE) {
    if (Kvar.index == KISSFRAMELENGTH) {
      c_state = KISS_HEADER_PAYLOAD;
    }
    else {
      KISSserialBuffer[Kvar.index] = c;
      Kvar.index++;
    }
  }
  else if (c_state == KISS_HEADER_PAYLOAD) {
    kiss_sync();
    c_state = KISS_IDLE;
  }
  else {
    c_state = KISS_IDLE;
  }
}


