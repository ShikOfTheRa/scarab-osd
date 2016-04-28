#include "platform.h"

Timer_s timer = {
  .tenthSec = 0 ,
  .halfSec = 0,
  .Blink2hz = 0,
  .Blink10hz = 0,
  .lastCallSign = 0,
  .rssiTimer = 0,
  .magCalibrationTimer = 0,
  .fwAltitudeTimer = 0,
  .seconds = 0,
  .MSP_active = 0,
  .GPS_active = 0,
  .armed = 255,
  // Total runtime in seconds
  .allSec = 0
};

struct Timer_s timer;
struct Flags_s flags;
struct Mode_s mode;

static uint16_t debug[4];   // int32_t ?...
static unsigned int menuSec=0;
static uint16_t debugerror;
static uint16_t debugval=0;
static uint16_t cell_data[6]={0,0,0,0,0,0};
static uint16_t cycleTime;
static uint16_t I2CError;
static uint8_t oldROW=0;
static uint8_t cells=0;

#if defined CORRECT_MSP_BF1
static uint8_t bfconfig[25];
#endif


// Config status and cursor location
static uint8_t ROW=10;
static uint8_t COL=3;
static int8_t configPage=1;
static int8_t previousconfigPage=1;
static uint8_t configMode=0;
static uint8_t fontData[54];
static uint8_t nextCharToRequest;
static uint8_t lastCharToRequest;


//const uint8_t screenlayoutoffset=((EEPROM_SETTINGS-EEPROM16_SETTINGS_START)>>2);


// For Settings Defaults
static uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];

static uint8_t rcRate8,rcExpo8;
static uint8_t rollPitchRate;
static uint8_t rollRate;
static uint8_t PitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t thrMid8;
static uint8_t thrExpo8;
static uint8_t tpa_breakpoint16;
static uint8_t rcYawExpo8;
static uint8_t FCProfile;
static uint8_t PreviousFCProfile;
static uint8_t CurrentFCProfile;
static uint8_t PIDController;
static uint16_t LoopTime;

static int32_t  MwAltitude=0;                         // This hold barometric value
static int32_t  old_MwAltitude=0;                     // This hold barometric value


static int MwAngle[2]={0,0};           // Those will hold Accelerator Angle
static uint16_t MwRcData[8]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500} ;

// for analogue / PWM sensor filtering
#define SENSORFILTERSIZE 8
#define SENSORTOTAL 5
static int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2];

static uint8_t MwVersion=0;

#ifdef HAS_ALARMS
static uint8_t alarmState = ALARM_OK;
static uint8_t alarmMsg[MAX_ALARM_LEN];
#endif

static uint8_t MvVBatMinCellVoltage=CELL_VOLTS_MIN;
static uint8_t MvVBatMaxCellVoltage=CELL_VOLTS_MAX;
static uint8_t MvVBatWarningCellVoltage=CELL_VOLTS_WARN;

//For Current Throttle
static uint16_t LowT = LOWTHROTTLE;
static uint16_t HighT = HIGHTHROTTLE;

// For Heading
static int16_t MwHeading=0;


// Rssi
static int16_t rssi =0;   // uint8_t ?
static int16_t oldrssi;   // uint8_t ?
static int16_t pwmRSSI = 0;
//static int rssiADC=0;
//static int rssi_Int=0;


// For Voltage
static uint16_t voltage=0;                      // its the value x10
static uint16_t vidvoltage=0;                   // its the value x10
static uint8_t voltageWarning=0;
static uint8_t vidvoltageWarning=0;

// For temprature
static int16_t temperature=0;                  // temperature in degrees Centigrade
