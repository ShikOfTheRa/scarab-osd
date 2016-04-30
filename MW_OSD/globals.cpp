#include "platform.h"

struct Timer_s timer;
struct Flags_s flags;
struct Mode_s mode;
struct RC_s rc;
struct Mwosd_s mwosd;
struct Debug_s debug;

uint8_t Settings[EEPROM_SETTINGS];
uint16_t Settings16[EEPROM16_SETTINGS];

//static unsigned int menuSec=0;
//static uint16_t debugerror;
//static uint16_t debugval=0;
//static uint16_t cell_data[6]={0,0,0,0,0,0};
//static uint16_t cycleTime;
//static uint16_t I2CError;
//static uint8_t oldROW=0;
//
//
//
//
//
////const uint8_t screenlayoutoffset=((EEPROM_SETTINGS-EEPROM16_SETTINGS_START)>>2);
//
//
//
//
//
//
//// for analogue / PWM sensor filtering
//
//
//#ifdef HAS_ALARMS
//static uint8_t alarmState = ALARM_OK;
//static uint8_t alarmMsg[MAX_ALARM_LEN];
//#endif
//
//
//// For Heading
//static int16_t MwHeading=0;
//
//
//// Rssi
////static int rssiADC=0;
////static int rssi_Int=0;
//
//
//
