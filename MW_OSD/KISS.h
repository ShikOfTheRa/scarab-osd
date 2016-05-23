#define KISSFRAMEINIT 5
#define KISSFRAMELENGTH 154
uint8_t KISSserialBuffer[KISSFRAMELENGTH];


const char KISS_mode_MANU[] PROGMEM   = "MANU"; //Manual
const char KISS_mode_RATE[] PROGMEM   = "RATE"; //Rate
const char KISS_mode_ACRO[] PROGMEM   = "ACRO"; //Acrobatic: rate control
const char KISS_mode_STAB[] PROGMEM   = "STAB"; //Stabilize: hold level position
const char KISS_mode_HOZN[] PROGMEM   = "HOZN"; //Horizon
const char KISS_mode_HOLD[] PROGMEM   = "HOLD"; //Hold
const char KISS_mode_HEAD[] PROGMEM   = "HEAD"; //Head
const char KISS_mode_WAYP[] PROGMEM   = "WAYP"; //Waypoint
const char KISS_mode_RTH[]  PROGMEM   = "RTH "; //Return to Launch: auto control
const char KISS_mode_FOLL[] PROGMEM   = "FOLL"; //Follow me
const char KISS_mode_CIRC[] PROGMEM   = "CIRC"; //Circle: auto control
const char KISS_mode_FBWA[] PROGMEM   = "FBWA"; //Fly-by-wire A
const char KISS_mode_FBWB[] PROGMEM   = "FBWB"; //Fly-by-wire B
const char KISS_mode_CRUI[] PROGMEM   = "CRUI"; //Cruise
const char KISS_mode_LAND[] PROGMEM   = "LAND"; //Land
const char KISS_mode_KISS[] PROGMEM   = "KISS "; //Unknown KISS mode

const PROGMEM char * const KISS_mode_index[] = 
{   
  KISS_mode_MANU, //0
  KISS_mode_RATE,
  KISS_mode_STAB,
  KISS_mode_HOZN,
  KISS_mode_ACRO,
  KISS_mode_STAB,
  KISS_mode_STAB,
  KISS_mode_STAB,
  KISS_mode_HOLD, 
  KISS_mode_HOLD, 
  KISS_mode_WAYP,
  KISS_mode_HEAD,
  KISS_mode_CIRC,
  KISS_mode_RTH , 
  KISS_mode_FOLL, 
  KISS_mode_LAND,
  KISS_mode_FBWA,
  KISS_mode_FBWB, 
  KISS_mode_CRUI, 
  KISS_mode_KISS , 
};

// Vars
struct __xKISS {
  uint8_t mode;
  uint8_t index;
  uint8_t cmd;
  uint8_t rcvChecksum;
  uint8_t readIndex;
  uint8_t framelength;
}
Kvar;

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

  if ((current_millis - previous_millis) > 5000 || (previous_mahUsed > mahUsed)) {
    // Stalled or invalid telemetry. Reset statistics
    calculatedCurrent = 0;
    previous_mahUsed = mahUsed;
    previous_millis = current_millis;
  }
  else if ((current_millis - previous_millis) > 500 && (previous_mahUsed < mahUsed)) {
    calculatedCurrent = (mahUsed - previous_mahUsed) / (current_millis - previous_millis);
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
  MwAngle[0]=(int16_t)kissread_u16(31/10);
  MwAngle[1]=(int16_t)kissread_u16(33)/10;
  Kvar.mode = kissread_u8(65); 
  for(uint8_t i=0; i<8; i++) {
    MwRcData[THROTTLESTICK]=kissread_u16(i*2);
  } 
  if (Settings[S_MWAMPERAGE]){ 
    uint16_t dummy=kissread_u16(148);
    amperagesum = 360*dummy;
    MWAmperage = 10*calculateCurrentFromConsumedCapacity(amperagesum);
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
 debug[0]++;
    c_state = (c == KISSFRAMELENGTH) ? KISS_HEADER_SIZE : KISS_IDLE;
  }
  else if (c_state == KISS_HEADER_SIZE) {
 debug[1]++;
    if (Kvar.index == KISSFRAMELENGTH) {
      c_state = KISS_HEADER_PAYLOAD;
    }
    else {
      KISSserialBuffer[Kvar.index] = c;
      Kvar.index++;
    }
  }
  else if (c_state == KISS_HEADER_PAYLOAD) {
 debug[2]++;
    kiss_sync();
    c_state = KISS_IDLE;
  }
  else {
    c_state = KISS_IDLE;
  }
}


