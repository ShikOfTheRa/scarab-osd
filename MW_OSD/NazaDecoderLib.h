#if defined NAZA

/*
  DJI Naza (v1, v1 Lite, V2) data decoder library
  (c) Pawelsky 20141130
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection

  The RC PWM input code taken from https://www.instructables.com/id/RC-Quadrotor-Helicopter/step12/Arduino-Demo-PWM-Input/
*/

#ifndef __NAZA_DECODER_LIB_H__
#define __NAZA_DECODER_LIB_H__

#include "Arduino.h"

// Uncomment the line below if you want to disable attitude (pitch/roll) sensing. This may be useful when you experience a conflict with another library that also uses the interrupt
// or on a board that does not support the interrupt used. Note that for Teensy boards attitude sensing will be disabled by default.
#define ATTITUDE_SENSING_DISABLED

#define NAZA_MESSAGE_NONE    0x00
#define NAZA_MESSAGE_GPS     0x10
#define NAZA_MESSAGE_COMPASS 0x20

#if defined(__MK20DX128__) || defined(__MK20DX256__)
  #define ATTITUDE_SENSING_DISABLED
#endif

class NazaDecoderLib
{
  public:
    typedef enum { NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4 } fixType_t;

    NazaDecoderLib();

    uint8_t decode(int input);
    double getLat();
    double getLon();
    double getGpsAlt();
    double getSpeed();
    fixType_t getFixType();
    uint8_t getNumSat();
    double getHeadingNc();
    double getCog();
    double getGpsVsi();
    double getHdop();
    double getVdop();
    uint8_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour(); // Note that for time between 16:00 and 23:59 the hour returned from GPS module is actually 00:00 - 7:59.
    uint8_t getMinute();
    uint8_t getSecond();

#ifndef ATTITUDE_SENSING_DISABLED
    void   pwmInterruptHandler();
    int8_t getPitch();
    int8_t getRoll();
#endif

  private:
    int payload[58];
    int seq;
    int cnt;
    int msgId;
    int msgLen;
    uint8_t cs1; // checksum #1
    uint8_t cs2; // checksum #2
    int16_t magXMin;
    int16_t magXMax;
    int16_t magYMin;
    int16_t magYMax;

    double lon;     // longitude in degree decimal
    double lat;     // latitude in degree decimal
    double gpsAlt;  // altitude in m (from GPS)
    double spd;     // speed in m/s
    fixType_t fix;   // fix type
    uint8_t sat;     // number of satellites
    double headingNc;// heading (not tilt compensated) in degrees
    double cog;     // course over ground
    double gpsVsi;  // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double hdop;    // horizontal dilution of precision
    double vdop;    // vertical dilution of precision
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    int32_t  decodeLong(uint8_t idx, uint8_t mask);
    int16_t  decodeShort(uint8_t idx, uint8_t mask);
    void     updateCS(int input);

#ifndef ATTITUDE_SENSING_DISABLED
    typedef struct
    {
      int8_t   edge;
      uint32_t riseTime;
      uint32_t fallTime;
      uint32_t lastGoodWidth;
    } tPwmData;
    tPwmData pwmData[2];

    volatile uint8_t pcIntLast;

    void     startPwmReader();
    int8_t   pwm2Deg(uint32_t pulseWidth);
#endif
};

extern NazaDecoderLib NazaDecoder;

#endif // __NAZA_DECODER_LIB_H__
#endif //NAZA
