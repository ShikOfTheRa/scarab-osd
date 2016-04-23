#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef ARDUINO
#include <HardwareSerial.h>
#endif

void mspWrite8(uint8_t t);
void mspWrite16(uint16_t t);
void mspWriteChecksum(void);
void settingsSerialRequest(void);
void settingswriteSerialRequest(void);
void handleRawRC(void);
void serialMenuCommon(void);
void setFCProfile(void);

void serialInit(HardwareSerial &_serial);
void mspWriteRequest(uint8_t mspCommand, uint8_t txDataSize);
void configExit(void);
void serialMSPreceive(uint8_t loops);
void configSave();

#if defined MAVLINK
  #define SERIALBUFFERSIZE 75
#elif defined NAZA
  #define SERIALBUFFERSIZE 75
#elif defined GPSOSD
  #define SERIALBUFFERSIZE 100
#else
  #define SERIALBUFFERSIZE 150
#endif

#endif
