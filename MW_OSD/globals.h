// it would be awesome to get rid of most of these...

#ifndef __GLOBALS_H
#define __GLOBALS_H

#include "settings.h"

// General use variables
struct Timer_s {
  uint8_t tenthSec;
  uint8_t halfSec;
  uint8_t Blink2hz;                          // This is turing on and off at 2hz
  uint8_t Blink10hz;                         // This is turing on and off at 10hz
  uint16_t lastCallSign;                          // Callsign_timer
  uint8_t rssiTimer;
//  uint8_t accCalibrationTimer;
  uint8_t magCalibrationTimer;
  uint32_t fwAltitudeTimer;
  uint32_t seconds;
  uint8_t MSP_active;
  uint8_t GPS_active;
  uint8_t armed;
  uint32_t allSec;
};

struct Flags_s {
  uint8_t ident;
  uint8_t box;
  uint8_t reset;
};

// Mode bits
struct Mode_s {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint8_t baro;
  uint8_t mag;
  uint16_t camstab;
  uint16_t gpshome;
  uint16_t gpshold;
  uint16_t passthru;
  uint32_t air;
  uint32_t acroplus;
  uint32_t osd_switch;
  uint32_t llights;
  uint32_t gpsmission;
  uint32_t gpsland;
};

extern struct Timer_s timer;
extern struct Flags_s flags;
extern struct Mode_s mode;

//extern uint16_t debug[4];   // int32_t ?...
//extern unsigned int allSec;
//extern unsigned int menuSec;
//extern uint16_t debugerror;
//extern uint16_t debugval;
//extern uint16_t cell_data[6];
//extern uint16_t cycleTime;
//extern uint16_t I2CError;
//extern uint8_t oldROW;
//extern uint8_t cells;
//
//#if defined CORRECT_MSP_BF1
//extern uint8_t bfconfig[25];
//#endif
//
//
//// Config status and cursor location
//extern uint8_t screenlayout;
//extern uint8_t oldscreenlayout;
//extern uint8_t ROW;
//extern uint8_t COL;
//extern int8_t configPage;
//extern int8_t previousconfigPage;
//extern uint8_t configMode;
//extern uint8_t fontData[54];
//extern uint8_t nextCharToRequest;
//extern uint8_t lastCharToRequest;
//
//// For Settings Defaults
//extern uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
//
//extern uint8_t rcRate8,rcExpo8;
//extern uint8_t rollPitchRate;
//extern uint8_t rollRate;
//extern uint8_t PitchRate;
//extern uint8_t yawRate;
//extern uint8_t dynThrPID;
//extern uint8_t thrMid8;
//extern uint8_t thrExpo8;
//extern uint8_t tpa_breakpoint16;
//extern uint8_t rcYawExpo8;
//extern uint8_t FCProfile;
//extern uint8_t PreviousFCProfile;
//extern uint8_t CurrentFCProfile;
//extern uint8_t PIDController;
//extern uint16_t LoopTime;
//
//extern int32_t  MwAltitude;                         // This hold barometric value
//extern int32_t  old_MwAltitude;                     // This hold barometric value
//
//
//extern int MwAngle[2];           // Those will hold Accelerator Angle
//extern uint16_t MwRcData[8];
//
//// for analogue / PWM sensor filtering
//extern int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2];
//
//extern uint8_t MwVersion;
//extern uint8_t MwVBat;
//extern int16_t MwVario;
//extern uint8_t armed;
//extern uint8_t previousarmedstatus;  // for statistics after disarming
//extern uint16_t armedangle;           // for capturing direction at arming
//extern uint32_t GPS_distanceToHome;
//extern uint8_t GPS_fix;
//extern uint8_t GPS_frame_timer;
//extern int32_t GPS_latitude;
//extern int32_t GPS_longitude;
//extern int16_t GPS_altitude;
//extern int16_t GPS_home_altitude;
//extern int16_t previousfwaltitude;
//extern int16_t interimfwaltitude;
//extern uint16_t GPS_speed;
//extern int16_t  GPS_ground_course;
//extern uint16_t old_GPS_speed;
//extern int16_t GPS_directionToHome;
//extern uint8_t GPS_numSat;
//extern uint8_t GPS_waypoint_step;
////uint16_t I2CError;
////uint16_t cycleTime;
//extern uint16_t pMeterSum;
//extern uint16_t MwRssi;
//extern uint32_t GPS_time ;        //local time of coord calc - haydent
//
//#ifdef HAS_ALARMS
//extern uint8_t alarmState ;
//extern uint8_t alarmMsg[MAX_ALARM_LEN];
//#endif
//
//extern uint8_t MvVBatMinCellVoltage;
//extern uint8_t MvVBatMaxCellVoltage;
//extern uint8_t MvVBatWarningCellVoltage;
//
////For Current Throttle
//extern uint16_t LowT ;
//extern uint16_t HighT ;
//
//// For Heading
//extern int16_t MwHeading;
//
//// For Amperage
//extern float amperage ;                // its the real value x10
//extern float amperagesum ;
//extern uint16_t MWAmperage;
//
//// Rssi
//extern int16_t rssi ;   // uint8_t ?
//extern int16_t oldrssi;   // uint8_t ?
//extern int16_t pwmRSSI ;
////int rssiADC;
////int rssi_Int;
//
//
//// For Voltage
//extern uint16_t voltage;                      // its the value x10
//extern uint16_t vidvoltage;                   // its the value x10
//extern uint8_t voltageWarning;
//extern uint8_t vidvoltageWarning;
//
//// For temprature
//extern int16_t temperature;                  // temperature in degrees Centigrade

#endif
