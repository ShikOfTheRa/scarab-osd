

uint8_t kissread_u8(uint8_t index){
  return KISSserialBuffer[index];
}

uint16_t kissread_u16(uint8_t index) {
  uint16_t t = kissread_u8(index) << 8;
  t |= (uint16_t)kissread_u8(index+1);
  return t;
}

uint32_t ESC_filter(uint32_t oldVal, uint32_t newVal){
  return (uint32_t)((uint32_t)((uint32_t)((uint32_t)oldVal*ESC_FILTER)+(uint32_t)newVal))/(ESC_FILTER+1);
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
  timer.packetcount++;
#ifdef DATA_MSP
  timer.MSP_active=DATA_MSP;             // getting something on serial port
#endif
  armed = kissread_u8(16);
  MwVBat = kissread_u16(17)/10;
  MwAngle[0]=(int16_t)kissread_u16(31)/10;
  MwAngle[1]=(int16_t)kissread_u16(33)/10;
  Kvar.mode = kissread_u8(65); 
  Kvar.mode = (Kvar.mode>2) ? 0 : Kvar.mode;
  MwRcData[1]=1000+(int16_t)kissread_u16(0);
  for(uint8_t i=1; i<8; i++) {
    MwRcData[i+1]=1500+(int16_t)kissread_u16(i*2);
  } 
  handleRawRC();

  static uint32_t filtereddata[6];

  if (Settings[S_MWAMPERAGE]){ 
    // calculate amperage using capacity method...
    // uint16_t dummy=kissread_u16(148);
    // MWAmperage = 10*calculateCurrentFromConsumedCapacity(dummy);

    // calculate amperage using ESC sum method...    
    MWAmperage=0;    
    for(uint8_t i=0; i<6; i++) {
      filtereddata[i] = ESC_filter((uint32_t)filtereddata[i],(uint32_t)((KISSserialBuffer[87+(i*10)]<<8) | KISSserialBuffer[88+(i*10)])<<4);
      MWAmperage     += filtereddata[i]>>4;
    }
//    MWAmperage/=10;   
    amperagesum = (uint32_t)360* kissread_u16(148);
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
    Kvar.index=0;
    Kvar.cksumtmp=0;
    c_state = (c == KISSFRAMEINIT) ? KISS_HEADER_INIT : KISS_IDLE;
  }
  else if (c_state == KISS_HEADER_INIT) {
    Kvar.framelength=c;
    c_state = KISS_HEADER_SIZE;
  }
  else if (c_state == KISS_HEADER_SIZE) {
    KISSserialBuffer[Kvar.index] = c;
    Kvar.cksumtmp+=c;
    Kvar.index++;
    if (Kvar.index == Kvar.framelength) {
      c_state = KISS_PAYLOAD;
    }
  }
  else if (c_state == KISS_PAYLOAD) {
    if ((Kvar.cksumtmp/Kvar.framelength)==c){
      kiss_sync();
    }
    c_state = KISS_IDLE; // Go straight to idle to avoid missing every other packet
  }
  else {
    c_state = KISS_IDLE;
  }
}



