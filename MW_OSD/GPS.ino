// GPS protocol GGA and RMC  sentences are needed

#if defined(GPSOSD) && !defined(NAZA)

#if defined(INIT_MTK_GPS)
#define MTK_SET_BINARY          PSTR("$PGCMD,16,0,0,0,0,0*6A\r\n")
#define MTK_SET_NMEA            PSTR("$PGCMD,16,1,1,1,1,1*6B\r\n")
#define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
#define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
#define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
#define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")
#define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n") // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s  
#define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
#define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
#define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")  //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)
#endif

// GPS data buffer length
#define GPS_BUFFER_LEN 200

// moving average filter variables
#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

struct __GPS_parse{
  uint8_t  GPS_fix;
  uint8_t  GPS_numSat;
  int16_t  GPS_altitude;
  uint16_t GPS_speed;
  int16_t  GPS_ground_course;
  int32_t  GPS_coord[2];
  uint8_t  GPS_Present;
}
GPS_parse;


#if defined(INIT_MTK_GPS) || defined(UBLOX)
uint32_t init_speed[5] = {9600, 19200, 38400, 57600, 115200};
void SerialGpsPrint(const char* str) {
  //  void SerialGpsPrint(char str) {
  char b;
  while (str && (b = pgm_read_byte(str++))) {
    Serial.write(b);
#if defined(UBLOX)
    delay(5);
#endif
  }
}
#endif
//const PROGMEM char * const
#if defined(UBLOX)
const char UBLOX_INIT[] PROGMEM = {                          // PROGMEM array must be outside any function !!!
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0xF0, (const char) 0x05, (const char) 0x00, (const char) 0xFF, (const char) 0x19,                  //disable all default NMEA messages
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0xF0, (const char) 0x03, (const char) 0x00, (const char) 0xFD, (const char) 0x15,
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0xF0, (const char) 0x01, (const char) 0x00, (const char) 0xFB, (const char) 0x11,
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0xF0, (const char) 0x00, (const char) 0x00, (const char) 0xFA, (const char) 0x0F,
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0xF0, (const char) 0x02, (const char) 0x00, (const char) 0xFC, (const char) 0x13,
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0xF0, (const char) 0x04, (const char) 0x00, (const char) 0xFE, (const char) 0x17,
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0x01, (const char) 0x02, (const char) 0x01, (const char) 0x0E, (const char) 0x47,                  //set POSLLH MSG rate
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0x01, (const char) 0x03, (const char) 0x01, (const char) 0x0F, (const char) 0x49,                  //set STATUS MSG rate
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0x01, (const char) 0x06, (const char) 0x01, (const char) 0x12, (const char) 0x4F,                  //set SOL MSG rate
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0x01, (const char) 0x12, (const char) 0x01, (const char) 0x1E, (const char) 0x67,                  //set VELNED MSG rate
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x01, (const char) 0x03, (const char) 0x00, (const char) 0x01, (const char) 0x21, (const char) 0x05, (const char) 0x31, (const char) 0x89,                  //set TIMEUTC MSG rate
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x16, (const char) 0x08, (const char) 0x00, (const char) 0x03, (const char) 0x07, (const char) 0x03, (const char) 0x00, (const char) 0x51, (const char) 0x08, (const char) 0x00, (const char) 0x00, (const char) 0x8A, (const char) 0x41, //set WAAS to EGNOS
  (const char) 0xB5, (const char) 0x62, (const char) 0x06, (const char) 0x08, (const char) 0x06, (const char) 0x00, (const char) 0xC8, (const char) 0x00, (const char) 0x01, (const char) 0x00, (const char) 0x01, (const char) 0x00, (const char) 0xDE, (const char) 0x6A //set rate to 5Hz
};
#endif

void GPS_SerialInit() {
#if defined(UBLOX)
  for (uint8_t i = 0; i < 5; i++) {
    delay(100);
    Serial.begin(init_speed[i]);
    Serial.flush();
#if (GPS_BAUD==19200)
    SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
#endif
#if (GPS_BAUD==38400)
    SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
#endif
#if (GPS_BAUD==57600)
    SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
#endif
#if (GPS_BAUD==115200)
    SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
#endif
  }
  Serial.flush();
  delay(100);
  Serial.begin(GPS_BAUD);
  for (uint8_t i = 0; i < sizeof(UBLOX_INIT); i++) {                   // send configuration data in UBX protocol
    Serial.write(pgm_read_byte(UBLOX_INIT + i));
    delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
#elif defined(INIT_MTK_GPS)                                            // MTK GPS setup
  for (uint8_t i = 0; i < 5; i++) {
    Serial.flush();
    delay(100);
    Serial.begin(init_speed[i]);
#if (GPS_BAUD==19200)
    SerialGpsPrint(PSTR("$PMTK251,19200*22\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
#endif
#if (GPS_BAUD==38400)
    SerialGpsPrint(PSTR("$PMTK251,38400*27\r\n"));     // 38400 baud
#endif
#if (GPS_BAUD==57600)
    SerialGpsPrint(PSTR("$PMTK251,57600*2C\r\n"));     // 57600 baud
#endif
#if (GPS_BAUD==115200)
    SerialGpsPrint(PSTR("$PMTK251,115200*1F\r\n"));    // 115200 baud
#endif
  }
  // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
  // So now we have to set the desired mode and update rate (which depends on the NMEA or MTK_BINARYxx settings)
  Serial.flush();
  delay(100);
  Serial.begin(GPS_BAUD);

  Serial.flush();
  delay(100);
  SerialGpsPrint(MTK_NAVTHRES_OFF);
  Serial.flush();
  delay(100);
  SerialGpsPrint(SBAS_ON);
  Serial.flush();
  delay(100);
  SerialGpsPrint(WAAS_ON);
  Serial.flush();
  delay(100);
  SerialGpsPrint(SBAS_TEST_MODE);
  Serial.flush();
  delay(100);
  SerialGpsPrint(MTK_OUTPUT_5HZ);           // 5 Hz update rate
  Serial.flush();
  delay(100);

#if defined(NMEA)
  SerialGpsPrint(MTK_SET_NMEA_SENTENCES); // only GGA and RMC sentence
  Serial.flush();
  delay(100);
#endif
#if defined(MTK_BINARY19) || defined(MTK_BINARY16)
  SerialGpsPrint(MTK_SET_BINARY);
  Serial.flush();
  delay(100);
#endif
#elif defined(NMEA)                              // NMEA only
  Serial.flush();
  delay(100);
  Serial.begin(GPS_BAUD);
#endif  // init gps type 
}


void GPS_updateRMC() {
  GPS_speed = GPS_parse.GPS_speed;
  GPS_ground_course = GPS_parse.GPS_ground_course;
}


void GPS_updateGGA() {
  uint8_t GPS_fix_temp = GPS_parse.GPS_fix;
  if (GPS_fix_temp) {
    GPS_fix = 1;
  }
  GPS_numSat = GPS_parse.GPS_numSat;
  GPS_altitude_ASL = GPS_parse.GPS_altitude;
  GPS_coord[LAT] = GPS_parse.GPS_coord[LAT];
  GPS_coord[LON] = GPS_parse.GPS_coord[LON];
  GPS_Present = GPS_parse.GPS_Present;
  gpsvario();
}


void GPS_reset_home_position() { 
  if (GPS_fix && GPS_numSat >= MINSATFIX) {
    GPS_home[LAT+2] = GPS_coord[LAT];
    GPS_home[LON+2] = GPS_coord[LON];
    GPS_altitude_home_2 = GPS_altitude_ASL;
    GPS_fix_HOME = 1;
  }
}



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



////////////////////////////////////////////////////////////////////////////////////
// Utilities
//


#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for (i = 0; src[i] != 0; i++) {
    if (src[i] == '.') {
      i++;
      if (mult == 0)   break;
      else  src[i + mult] = 0;
    }
    tmp *= 10;
    if (src[i] >= '0' && src[i] <= '9') tmp += src[i] - '0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if (n > 9)  n -= 7;
  n &= 0x0F;
  return n;
}

bool GPS_newFrame(char c) {
#if defined(NMEA)
  return GPS_NMEA_newFrame(c);
#endif
#if defined(UBLOX)
  return GPS_UBLOX_newFrame(c);
#endif
#if defined(MTK_BINARY16) || defined(MTK_BINARY19)
  return GPS_MTK_newFrame(c);
#endif
}

#if defined(NMEA)
#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_NMEA_newFrame(char c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {
        GPS_parse.GPS_coord[LAT] = GPS_coord_to_degrees(string);
      }
      else if (param == 3 && string[0] == 'S') GPS_parse.GPS_coord[LAT] = -GPS_parse.GPS_coord[LAT];
      else if (param == 4)                     {
        GPS_parse.GPS_coord[LON] = GPS_coord_to_degrees(string);
      }
      else if (param == 5 && string[0] == 'W') GPS_parse.GPS_coord[LON] = -GPS_parse.GPS_coord[LON];
      else if (param == 6)                     {
        GPS_parse.GPS_fix = (string[0]  > '0');
      }
      else if (param == 7)                     {
        GPS_parse.GPS_numSat = grab_fields(string, 0);
      }
      else if (param == 9)                     {
        GPS_parse.GPS_altitude = grab_fields(string, 0); // altitude in meters added by Mis
      }
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {
        GPS_parse.GPS_speed = ((uint32_t)grab_fields(string, 1) * 5144L) / 1000L; //gps speed in cm/s will be used for navigation
      }
      else if (param == 8)                     {
        GPS_parse.GPS_ground_course = grab_fields(string, 1);  //ground course deg*10
      }
#ifdef ALARM_GPS
      timer.GPS_active = ALARM_GPS;
#endif //ALARM_GPS

    }
    param++; offset = 0;
    if (c == '*') checksum_param = 1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) {
        timer.packetcount++;
        frameOK = 1;
#ifdef SENTINELAAT
        sentinel.gpsdata = true;
#endif // SENTINELAAT          
        if (frame == FRAME_GGA) {
          GPS_updateGGA();
        }
        if (frame == FRAME_RMC) {
          GPS_updateRMC();
        }
      }
    }
    checksum_param = 0;
  } else {
    if (offset < 15) string[offset++] = c;
    if (!checksum_param) parity ^= c;
  }
  if (frame) GPS_Present = 1;
  return frameOK && (frame == FRAME_GGA);
}
#endif //NMEA

#if defined(UBLOX)
struct ubx_header {
  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t length;
};
struct ubx_nav_posllh {
  uint32_t time;  // GPS msToW
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
};
struct ubx_nav_solution {
  uint32_t time;
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};
struct ubx_nav_velned {
  uint32_t time;  // GPS msToW
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  uint32_t speed_3d;
  uint32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
};
typedef struct {
    uint32_t time;              // GPS msToW
    uint32_t tAcc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
} ubx_nav_timeutc;

enum ubs_protocol_bytes {
  PREAMBLE1 = 0xb5,
  PREAMBLE2 = 0x62,
  CLASS_NAV = 0x01,
  CLASS_ACK = 0x05,
  CLASS_CFG = 0x06,
  MSG_ACK_NACK = 0x00,
  MSG_ACK_ACK = 0x01,
  MSG_POSLLH = 0x2,
  MSG_STATUS = 0x3,
  MSG_SOL = 0x6,
  MSG_VELNED = 0x12,
  MSG_TIMEUTC = 0x21,
  MSG_CFG_PRT = 0x00,
  MSG_CFG_RATE = 0x08,
  MSG_CFG_SET_RATE = 0x01,
  MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type {
  FIX_NONE = 0,
  FIX_DEAD_RECKONING = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  FIX_GPS_DEAD_RECKONING = 4,
  FIX_TIME = 5
};
enum ubx_nav_status_bits {
  NAV_STATUS_FIX_VALID = 1
};

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

//  static bool next_fix;
static uint8_t _class;

static uint8_t _disable_counter;
static uint8_t _fix_ok;

// Receive buffer
static union {
  ubx_nav_posllh posllh;
  //    ubx_nav_status status;
  ubx_nav_solution solution;
  ubx_nav_velned velned;
  ubx_nav_timeutc timeutc;
  uint8_t bytes[GPS_BUFFER_LEN];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b) {
  while (len--) {
    ck_a += *data;
    ck_b += ck_a;
    data++;
  }
}

#if defined GPSTIME
static const uint16_t days[4][12] =
{
    {   0,  31,     60,     91,     121,    152,    182,    213,    244,    274,    305,    335},
    { 366,  397,    425,    456,    486,    517,    547,    578,    609,    639,    670,    700},
    { 731,  762,    790,    821,    851,    882,    912,    943,    974,    1004,   1035,   1065},
    {1096,  1127,   1155,   1186,   1216,   1247,   1277,   1308,   1339,   1369,   1400,   1430},
};
#endif

bool GPS_UBLOX_newFrame(uint8_t data) {
  bool parsed = false;

  switch (_step) {
    case 1:
      if (PREAMBLE2 == data) {
        _step++;
        break;
      }
      _step = 0;
    case 0:
      if (PREAMBLE1 == data) _step++;
      break;
    case 2:
      _step++;
      _class = data;
      _ck_b = _ck_a = data;  // reset the checksum accumulators
      break;
    case 3:
      _step++;
      _ck_b += (_ck_a += data);  // checksum byte
      _msg_id = data;
      break;
    case 4:
      _step++;
      _ck_b += (_ck_a += data);  // checksum byte
      _payload_length = data;  // payload length low byte
      break;
    case 5:
      _step++;
      _ck_b += (_ck_a += data);  // checksum byte
      _payload_length += (uint16_t)(data << 8);
      if ((_payload_length > GPS_BUFFER_LEN)||(_payload_length == 0)) { //*************************************************************
        _payload_length = 0;
        _step = 0;
      }
      _payload_counter = 0;  // prepare to receive payload
      break;
    case 6:
      _ck_b += (_ck_a += data);  // checksum byte
      if (_payload_counter < GPS_BUFFER_LEN) {
        _buffer.bytes[_payload_counter] = data;
      }
      if (++_payload_counter == _payload_length)
        _step++;
      break;
    case 7:
      _step++;
      if (_ck_a != data) _step = 0;  // bad checksum
      break;
    case 8:
      _step = 0;
      if (_ck_b != data)  break;  // bad checksum
      GPS_Present = 1;
      if (UBLOX_parse_gps())  {
        parsed = true;
      }
  } //end switch
  return parsed;
}

bool UBLOX_parse_gps(void) {
  timer.packetcount++;
#ifdef SENTINELAAT
  sentinel.gpsdata = true;
#endif // SENTINELAAT  

  switch (_msg_id) {
    case MSG_POSLLH:
      //i2c_dataset.time                = _buffer.posllh.time;
      if (_fix_ok) {
        GPS_coord[LON]   = _buffer.posllh.longitude;
        GPS_coord[LAT]   = _buffer.posllh.latitude;
        GPS_altitude_ASL = _buffer.posllh.altitude_msl / 1000;      //alt in m
        GPS_altitude_vario = _buffer.posllh.altitude_msl / 10;      //alt in cm
        gpsvarioublox();
      }
      GPS_fix = _fix_ok;
#ifdef ALARM_GPS
      timer.GPS_active = ALARM_GPS;
#endif //ALARM_GPS
      return true;        // POSLLH message received, allow blink GUI icon and LED
      break;
    case MSG_SOL:
      _fix_ok = 0;
      if ((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) _fix_ok = 1;
      GPS_numSat = _buffer.solution.satellites;
      GPS_dop = _buffer.solution.position_DOP;
//      if ((GPS_fix_HOME == 0) && (GPS_dop > GPSDOP)) {
//        GPS_numSat = MINSATFIX;
//      }
      break;
    case MSG_VELNED:
      GPS_speed         = _buffer.velned.speed_2d;  // cm/s
      GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);  // Heading 2D deg * 100000 rescaled to deg * 10
      break;
#if defined GPSTIME
    case MSG_TIMEUTC:
      if (GPS_numSat >= MINSATFIX) {
        datetime.year = _buffer.timeutc.year-2000;
        datetime.month = _buffer.timeutc.month;
        datetime.day = _buffer.timeutc.day;
        datetime.hours = _buffer.timeutc.hour;
        datetime.minutes = _buffer.timeutc.min;
        datetime.seconds = _buffer.timeutc.sec;
//        unsigned int second = _buffer.timeutc.sec;       // 0-59
//        unsigned int minute = _buffer.timeutc.min;       // 0-59
//        unsigned int hour   = _buffer.timeutc.hour;      // 0-23
//        unsigned int day    = _buffer.timeutc.day-1;     // 0-30
//        unsigned int month  = _buffer.timeutc.month-1;   // 0-11
//        unsigned int year   = _buffer.timeutc.year-2000; // 0-99  
//        if(!armed){ // For now to avoid uneven looking clock
//          GPS_time = ((((year) / 4 * (365 * 4 + 1) + days[(year) % 4][month] + day) * 24 + hour) * 60 + minute) * 60 + second;
//          //setDateTime();
//       }
       }
#endif //GPSTIME       
    default:
      break;
  }
  return false;
}

#endif //UBLOX

#if defined(MTK_BINARY16) || defined(MTK_BINARY19)

struct diyd_mtk_msg {
  int32_t  latitude;
  int32_t  longitude;
  int32_t  altitude;
  int32_t  ground_speed;
  int32_t  ground_course;
  uint8_t  satellites;
  uint8_t  fix_type;
  uint32_t utc_date;
  uint32_t utc_time;
  uint16_t hdop;
};
// #pragma pack(pop)
enum diyd_mtk_fix_type {
  FIX_NONE = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  FIX_2D_SBAS = 6,
  FIX_3D_SBAS = 7
};

#if defined(MTK_BINARY16)
enum diyd_mtk_protocol_bytes {
  PREAMBLE1 = 0xd0,
  PREAMBLE2 = 0xdd,
};
#endif

#if defined(MTK_BINARY19)
enum diyd_mtk_protocol_bytes {
  PREAMBLE1 = 0xd1,
  PREAMBLE2 = 0xdd,
};
#endif

// Packet checksum accumulators
uint8_t  _ck_a;
uint8_t  _ck_b;

// State machine state
uint8_t  _step;
uint8_t  _payload_counter;

// Time from UNIX Epoch offset
long  _time_offset;
bool  _offset_calculated;

// Receive buffer
union {
  diyd_mtk_msg  msg;
  uint8_t       bytes[GPS_BUFFER_LEN];
} _buffer;

inline long _swapl(const void *bytes)
{
  const uint8_t *b = (const uint8_t *)bytes;
  union {
    long    v;
    uint8_t b[4];
  } u;

  u.b[0] = b[3];
  u.b[1] = b[2];
  u.b[2] = b[1];
  u.b[3] = b[0];

  return (u.v);
}

bool GPS_MTK_newFrame(uint8_t data)
{
  bool parsed = false;

restart:
  switch (_step) {

    // Message preamble, class, ID detection
    //
    // If we fail to match any of the expected bytes, we
    // reset the state machine and re-consider the failed
    // byte as the first byte of the preamble.  This
    // improves our chances of recovering from a mismatch
    // and makes it less likely that we will be fooled by
    // the preamble appearing as data in some other message.
    //
    case 0:
      if (PREAMBLE1 == data)
        _step++;
      break;
    case 1:
      if (PREAMBLE2 == data) {
        _step++;
        break;
      }
      _step = 0;
      goto restart;
    case 2:
      if (sizeof(_buffer) == data) {
        _step++;
        _ck_b = _ck_a = data;                  // reset the checksum accumulators
        _payload_counter = 0;
      } else {
        _step = 0;                             // reset and wait for a message of the right class
        goto restart;
      }
      break;

    // Receive message data
    //
    case 3:
      _buffer.bytes[_payload_counter++] = data;
      _ck_b += (_ck_a += data);
      if (_payload_counter == GPS_BUFFER_LEN)
        _step++;
      break;

    // Checksum and message processing
    //
    case 4:
      _step++;
      if (_ck_a != data) {
        _step = 0;
      }
      break;
    case 5:
      _step = 0;
      if (_ck_b != data) {
        break;
      }

      timer.packetcount++;
      GPS_fix                   = ((_buffer.msg.fix_type == FIX_3D) || (_buffer.msg.fix_type == FIX_3D_SBAS));
#ifdef ALARM_GPS
      timer.GPS_active = ALARM_GPS;
#endif //ALARM_GPS

#if defined(MTK_BINARY16)
      GPS_coord[LAT]              = _buffer.msg.latitude * 10;    // XXX doc says *10e7 but device says otherwise
      GPS_coord[LON]              = _buffer.msg.longitude * 10;   // XXX doc says *10e7 but device says otherwise
#endif
#if defined(MTK_BINARY19)
      GPS_coord[LAT]              = _buffer.msg.latitude;         // With 1.9 now we have real 10e7 precision
      GPS_coord[LON]              = _buffer.msg.longitude;
#endif
      GPS_altitude_ASL            = _buffer.msg.altitude / 100;   // altitude in meter
      gpsvario();
      GPS_speed                   = _buffer.msg.ground_speed;     // in m/s * 100 == in cm/s
      GPS_ground_course           = _buffer.msg.ground_course / 100; //in degrees
      GPS_numSat                  = _buffer.msg.satellites;
      GPS_dop                     = _buffer.msg.hdop;
      parsed = true;
      GPS_Present = 1;
  }
  return parsed;
}
#endif //MTK


void     GPSOSDcalculate(){
  //calculate distance. bearings etc
  uint32_t dist;
  int32_t  dir;
  if (GPS_numSat < 5)
    return;
  GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
  GPS_distanceToHome = dist / 100;
  GPS_directionToHome = dir / 100;
  GPS_altitude =  GPS_altitude_ASL - GPS_altitude_home;
  MwAltitude = (int32_t)GPS_altitude * 100;
  GPS_latitude = GPS_coord[LAT];
  GPS_longitude = GPS_coord[LON];
  int16_t MwHeading360 = GPS_ground_course / 10;
  if (MwHeading360 > 180)
  MwHeading360 = MwHeading360 - 360;
  MwHeading   = MwHeading360;
}


void GPS_NewData() {

  if (GPSOSD_state>1){
    GPSOSDcalculate();
  }

  switch (GPSOSD_state) {
    case 1: // waiting for steady state fix - (enough sats for a consecutive period without glitch). Default: 6 sats for 10 seconds)
      if (GPS_numSat >= HOMESATFIX){
        GPS_reset_home_position();
        GPS_home[LAT] = GPS_home[LAT+2];
        GPS_home[LON] = GPS_home[LON+2];
        GPS_altitude_home = GPS_altitude_home_2;        
        if (millis() > GPSHOMEFIX*1000+timer.GPSOSDstate){ 
          timer.GPSOSDstate=millis();          
          GPSOSD_state=2;     
        }       
      }
      else{
        timer.GPSOSDstate=millis();
      }
      break;

    case 2: // waiting for launch. Continually reset home to improve accuracy
      armedangle = MwHeading;
      if (millis() > 10000+timer.GPSOSDstate){ //Reset home position every 10 secs ago if launch not detected to improve home accuracy.
        GPS_home[LAT] = GPS_home[LAT+2];
        GPS_home[LON] = GPS_home[LON+2];
        GPS_altitude_home = GPS_altitude_home_2;
        GPS_reset_home_position();
        timer.GPSOSDstate=millis();
      } 
      else if (GPS_distanceToHome > GPSOSDARMDISTANCE) { // To determine launch. Optional "&& (GPS_speed > 75)"
        GPS_armedangleset = 1;
        armed = 1;
        GPSOSD_state=3; 
        timer.GPSOSDstate=millis();    
      }        
      break;

    case 3: // in active flight
      if ((GPS_distanceToHome < GPSOSDHOMEDISTANCE) && (GPS_speed < 75)) { // Detected potential landed 
        if (millis() > GPSOSDLANDED*1000+timer.GPSOSDstate)  { // Confirmed landed
          configPage = 0;
          armed=0;
          timer.GPSOSDstate=millis();
          GPSOSD_state=4;     
        }
      }
      else{ // not landed - resume active OSD mode
        timer.GPSOSDstate=millis();
      }
      break;

    case 4: // confirm landed. Display stats
      if (timer.disarmed==0) { // Dispalay stats until timer or cancelled by throttle
        configExit();
        GPS_armedangleset = 0;
        GPSOSD_state=2;  
      }
      break;
     
    default: // case 0 – Waiting GPS fix
      timer.GPSOSDstate=millis();
      if ((GPS_fix) && (GPS_numSat >= MINSATFIX)){
        GPSOSD_state=1;     
      }
      break;
  }
}


void gpsvarioublox() { // Slightly laggy compared to baro. 
  if (millis() > timer.fwAltitudeTimer) { // To make vario from GPS altitude
    timer.fwAltitudeTimer += 1000;
    MwVario = (int32_t)(((GPS_altitude_vario+previousfwaltitude)>>1) - previousfwaltitude);
    previousfwaltitude = GPS_altitude_vario;
  }
}


#endif // GPS


void gpsvario() {
  if (millis() > timer.fwAltitudeTimer) { // To make vario from GPS altitude
    timer.fwAltitudeTimer += 1000;
    MwVario = (GPS_altitude - previousfwaltitude) * 100;
    previousfwaltitude = GPS_altitude;
  }
}

#ifdef SENTINELAAT
void sentinelinit() // SENTINEL GPSOSD
{
  Settings[S_AAT] = 1;
  static bool fontupdated = false;

// FONT................
if (fontupdated == false) {
  for(uint8_t x = 0; x < 3; x++){
    for(uint8_t i = 0; i < 54; i++){
      serialBuffer[1+i] = (uint8_t)pgm_read_byte(fontdata+(64*x)+i);
    }
    write_NVM(SentinelFont[x]);
    delay(20); // Shouldn't be needed due to status reg wait.
    fontupdated = true;
  }
}

// STATUS INDICATOR................
  switch (GPSOSD_state) {
    case 0: // No sats
      if (timer.Blink10hz){
        screen[3]=0x2A; 
      }
       break;        
    case 1: // Waiting for steady state fix - (enough sats for a consecutive period without glitch). Default: 6 sats for 10 seconds)
      if (timer.Blink2hz){
        screen[3]=0x2A; 
      }
      break;    
    default: //Ready / in flight / disconnected warning
      if (sentinel.gpsdetected == false){
        if (timer.Blink2hz){
          screen[3]=0x2A; 
        }
      }     
      break;
  }
 
  // Autodetect baud rate.............
    if (millis() > sentinel.timer_gpsdata){
      sentinel.gpsdetected = false;
    }
    if (sentinel.gpsdata == true){
      sentinel.gpsdetected =true;
      sentinel.timer_gpsdata=millis()+SENTINELTIMEOUT;    
    }
    sentinel.gpsdata = false; 

    if (sentinel.gpsdetected == false){
      if (sentinel.timeout > (SENTINELTIMEOUT)){
        sentinel.timeout = 0;
      }   
      if (millis() > sentinel.timer_baudchange){
        sentinel.baud++; 
        if (sentinel.baud>5){
          sentinel.baud = 0;
          sentinel.timeout +=1500;
        }          
        serialMSPreceive(1);
        Serial.begin(SentinelBaud[sentinel.baud]);
        delay(10);
        serialMSPreceive(1);     
        sentinel.timer_baudchange = millis() + sentinel.timeout;
      }
    }
}
#endif // SENTINELLAAT
