// mwosd.ino
#ifdef USEADXL345
#include <Wire.h>
#include "ADXL345.h"
ADXL345 accel(ADXL345_ALT);
#endif

#ifdef USEADXL345 // setup
Wire.begin();
initADXL345();
#endif

#ifdef USEADXL345 // loop
readADXL345();
#endif

// config.h #####################################################################################################################################################################
#define USEADXL345   // Enable to add AHI pitch roll to GPSOSD using an ADXL345 accelerometer via I2C
#define ACCPITCH 0   // Sensor orientation 0-2. Typically swap ACCPITCH/ACCROLL  values 
#define ACCROLL  1   // sensor orientation 0-2. Typically swap ACCPITCH/ACCROLL  values 

//also consider:
//#define REVERSE_AHI_PITCH         // Reverse pitch / roll direction of AHI
//#define REVERSE_AHI_ROLL          // Reverse pitch / roll direction of AHI


// ADXL345.h ###########################################################################################################################################################################

#ifndef ADXL345_h
#define ADXL345_h

#include "Arduino.h"

// I2C Address
#define ADXL345_STD     0x1D    // Standard address if SDO/ALT ADDRESS is HIGH.
#define ADXL345_ALT     0x53    // Alternate address if SDO/ALT ADDRESS is LOW. 
#define ADXL345_ALT8    0xA6    // Alternate address if SDO/ALT ADDRESS is LOW.
// Data Rate
#define ADXL345_RATE_3200HZ   0x0F    // 3200 Hz
#define ADXL345_RATE_1600HZ   0x0E    // 1600 Hz
#define ADXL345_RATE_800HZ    0x0D    // 800 Hz
#define ADXL345_RATE_400HZ    0x0C    // 400 Hz
#define ADXL345_RATE_200HZ    0x0B    // 200 Hz
#define ADXL345_RATE_100HZ    0x0A    // 100 Hz
#define ADXL345_RATE_50HZ     0x09    // 50 Hz
#define ADXL345_RATE_25HZ     0x08    // 25 Hz
#define ADXL345_RATE_12_5HZ   0x07    // 12.5 Hz
#define ADXL345_RATE_6_25HZ   0x06    // 6.25 Hz
#define ADXL345_RATE_3_13HZ   0x05    // 3.13 Hz
#define ADXL345_RATE_1_56HZ   0x04    // 1.56 Hz
#define ADXL345_RATE_0_78HZ   0x03    // 0.78 Hz
#define ADXL345_RATE_0_39HZ   0x02    // 0.39 Hz
#define ADXL345_RATE_0_20HZ   0x01    // 0.20 Hz
#define ADXL345_RATE_0_10HZ   0x00    // 0.10 Hz

// Range
#define ADXL345_RANGE_2G      0x00    // +-2 g
#define ADXL345_RANGE_4G      0x01    // +-4 g
#define ADXL345_RANGE_8G      0x02    // +-8 g
#define ADXL345_RANGE_16G     0x03    // +-16 g


class ADXL345 {
  private:
    class PowerCtlBits {
      public:
        uint8_t link;       // D5
        uint8_t autoSleep;  // D4
        uint8_t measure;    // D3
        uint8_t sleep;      // D2
        uint8_t wakeup;     // D1 - D0

        PowerCtlBits();

        uint8_t toByte();
    };

    class DataFormatBits {
      public:
        uint8_t selfTest;   // D7
        uint8_t spi;        // D6
        uint8_t intInvert;  // D5
        uint8_t fullRes;    // D3
        uint8_t justify;    // D2
        uint8_t range;      // D1 - D0

        DataFormatBits();

        uint8_t toByte();
    };

    class BwRateBits {
      public:
        uint8_t lowPower;   // D4
        uint8_t rate;       // D3 - D0

        BwRateBits();

        uint8_t toByte();
    };

    static const float kRatio2g;
    static const float kRatio4g;
    static const float kRatio8g;
    static const float kRatio16g;

    TwoWire *_wire;
    int8_t _i2cAddress;
    int16_t _xyz[3];
    PowerCtlBits _powerCtlBits;
    DataFormatBits _dataFormatBits;
    BwRateBits _bwRateBits;

    float convertToSI(int16_t rawValue);

    bool write(uint8_t value);
    bool write(uint8_t *values, size_t size);
    bool read(uint8_t *values, int size);
    bool readRegister(uint8_t address, uint8_t *value);
    bool readRegisters(uint8_t address, uint8_t *values, uint8_t size);
    bool writeRegister(uint8_t address, uint8_t value);

  public:
    ADXL345(uint8_t i2cAddress, TwoWire *wire = &Wire);
    bool start();
    bool stop();
    uint8_t readDeviceID();
    bool update();
    float getX();
    float getY();
    float getZ();
    int16_t getRawX();
    int16_t getRawY();
    int16_t getRawZ();

    bool writeRate(uint8_t rate);
    bool writeRateWithLowPower(uint8_t rate);
    bool writeRange(uint8_t range);
};



// ADXL345.cpp #############################################################################################################

// please refer to the sensor datasheet.
//
// EN: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
// JP: http://www.analog.com/media/jp/technical-documentation/data-sheets/ADXL345_jp.pdf

// Register Map
// Order is "Type, Reset Value, Description".
#define REG_DEVID          0x00    // R,     11100101,   Device ID
#define REG_THRESH_TAP     0x1D    // R/W,   00000000,   Tap threshold
#define REG_OFSX           0x1E    // R/W,   00000000,   X-axis offset
#define REG_OFSY           0x1F    // R/W,   00000000,   Y-axis offset
#define REG_OFSZ           0x20    // R/W,   00000000,   Z-axis offset
#define REG_DUR            0x21    // R/W,   00000000,   Tap duration
#define REG_LATENT         0x22    // R/W,   00000000,   Tap latency 
#define REG_WINDOW         0x23    // R/W,   00000000,   Tap window
#define REG_THRESH_ACT     0x24    // R/W,   00000000,   Activity threshold
#define REG_THRESH_INACT   0x25    // R/W,   00000000,   Inactivity threshold
#define REG_TIME_INACT     0x26    // R/W,   00000000,   Inactivity time
#define REG_ACT_INACT_CTL  0x27    // R/W,   00000000,   Axis enable control for activity and inactiv ity detection
#define REG_THRESH_FF      0x28    // R/W,   00000000,   Free-fall threshold
#define REG_TIME_FF        0x29    // R/W,   00000000,   Free-fall time
#define REG_TAP_AXES       0x2A    // R/W,   00000000,   Axis control for single tap/double tap
#define REG_ACT_TAP_STATUS 0x2B    // R,     00000000,   Source of single tap/double tap
#define REG_BW_RATE        0x2C    // R/W,   00001010,   Data rate and power mode control
#define REG_POWER_CTL      0x2D    // R/W,   00000000,   Power-saving features control
#define REG_INT_ENABLE     0x2E    // R/W,   00000000,   Interrupt enable control
#define REG_INT_MAP        0x2F    // R/W,   00000000,   Interrupt mapping control
#define REG_INT_SOUCE      0x30    // R,     00000010,   Source of interrupts
#define REG_DATA_FORMAT    0x31    // R/W,   00000000,   Data format control
#define REG_DATAX0         0x32    // R,     00000000,   X-Axis Data 0
#define REG_DATAX1         0x33    // R,     00000000,   X-Axis Data 1
#define REG_DATAY0         0x34    // R,     00000000,   Y-Axis Data 0
#define REG_DATAY1         0x35    // R,     00000000,   Y-Axis Data 1
#define REG_DATAZ0         0x36    // R,     00000000,   Z-Axis Data 0
#define REG_DATAZ1         0x37    // R,     00000000,   Z-Axis Data 1
#define REG_FIFO_CTL       0x38    // R/W,   00000000,   FIFO control
#define REG_FIFO_STATUS    0x39    // R,     00000000,   FIFO status

ADXL345::ADXL345(uint8_t i2cAddress, TwoWire *wire) {
  _wire = wire;
  _i2cAddress = i2cAddress;

  _xyz[0] = 0; // x
  _xyz[1] = 0; // y
  _xyz[2] = 0; // z
}

const float ADXL345::kRatio2g  = (float) (2 * 2) / 1024.0f;
const float ADXL345::kRatio4g  = (float) (4 * 2) / 1024.0f;
const float ADXL345::kRatio8g  = (float) (8 * 2) / 1024.0f;
const float ADXL345::kRatio16g = (float) (16 * 2) / 1024.0f;

bool ADXL345::start() {
  _powerCtlBits.measure = 1;
  return writeRegister(REG_POWER_CTL, _powerCtlBits.toByte());
}

bool ADXL345::stop() {
  _powerCtlBits.measure = 0;
  return writeRegister(REG_POWER_CTL, _powerCtlBits.toByte());
}

uint8_t ADXL345::readDeviceID() {
  uint8_t value = 0;
  if (readRegister(REG_DEVID, &value)) {
    return value;
  } else {
    return 0;
  }
}

bool ADXL345::update() {
  // Read all axis registers (DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1) in accordance with data sheet.
  // "It is recommended that a multiple-uint8_t read of all registers be performed to prevent a change in data between reads of sequential registers."
  uint8_t values[6];
  if (readRegisters(REG_DATAX0, values, sizeof(values))) {
    // convert endian
    _xyz[0] = (int16_t) word(values[1], values[0]);  // x
    _xyz[1] = (int16_t) word(values[3], values[2]);  // y
    _xyz[2] = (int16_t) word(values[5], values[4]);  // z
    return true;
  } else {
    return false;
  }
}

float ADXL345::getX() {
  return convertToSI(_xyz[0]);
}

float ADXL345::getY() {
  return convertToSI(_xyz[1]);
}

float ADXL345::getZ() {
  return convertToSI(_xyz[2]);
}

int16_t ADXL345::getRawX() {
  return _xyz[0];
}

int16_t ADXL345::getRawY() {
  return _xyz[1];
}

int16_t ADXL345::getRawZ() {
  return _xyz[2];
}

bool ADXL345::writeRate(uint8_t rate) {
  _bwRateBits.lowPower = 0;
  _bwRateBits.rate = rate & 0x0F;
  return writeRegister(REG_BW_RATE, _bwRateBits.toByte());
}

bool ADXL345::writeRateWithLowPower(uint8_t rate) {
  _bwRateBits.lowPower = 1;
  _bwRateBits.rate = rate & 0x0F;
  return writeRegister(REG_BW_RATE, _bwRateBits.toByte());
}

bool ADXL345::writeRange(uint8_t range) {
  _dataFormatBits.range = range & 0x03;
  return writeRegister(REG_DATA_FORMAT, _dataFormatBits.toByte());
}

float ADXL345::convertToSI(int16_t rawValue) {
  switch (_dataFormatBits.range) {
    case ADXL345_RANGE_2G:
      return rawValue * kRatio2g;

    case ADXL345_RANGE_4G:
      return rawValue * kRatio4g;

    case ADXL345_RANGE_8G:
      return rawValue * kRatio8g;

    case ADXL345_RANGE_16G:
      return rawValue * kRatio16g;

    default:
      return 0;
  }
}

bool ADXL345::write(uint8_t value) {
  uint8_t values[1] = {value};
  return write(values, 1);
}

bool ADXL345::write(uint8_t *values, size_t size) {
  _wire->beginTransmission(_i2cAddress);

  if (_wire->write(values, size) == size) {
    switch (_wire->endTransmission()) {
      case 0: // success
        return true;

      case 1: // data too long to fit in transmit buffer
        return false;

      case 2: // received NACK on transmit of address
        return false;

      case 3: // received NACK on transmit of data
        return false;

      case 4: // other error
        return false;

      default: // module unknown error
        return false;
    }
  } else {
    _wire->endTransmission();
    return false;
  }
}

bool ADXL345::read(uint8_t *values, int size) {
  _wire->requestFrom(_i2cAddress, size);
  if (_wire->available() == size) {
    for (uint8_t i = 0; i < size; i++) {
      values[i] = _wire->read();
    }
    return true;
  } else {
    return false;
  }
}

bool ADXL345::readRegister(uint8_t address, uint8_t *value) {
  return readRegisters(address, value, 1);
}

bool ADXL345::readRegisters(uint8_t address, uint8_t *values, uint8_t size) {
  if (!write(address)) {
    return false;
  }

  return read(values, size);
}

bool ADXL345::writeRegister(uint8_t address, uint8_t value) {
  uint8_t values[2] = {address, value};
  return write(values, sizeof(values));
}

ADXL345::PowerCtlBits::PowerCtlBits() {
  this->link      = 0;
  this->autoSleep = 0;
  this->measure   = 0;
  this->sleep     = 0;
  this->wakeup    = 0;
}

uint8_t ADXL345::PowerCtlBits::toByte() {
  uint8_t bits = 0x00;
  bits |= this->link      << 5;
  bits |= this->autoSleep << 4;
  bits |= this->measure   << 3;
  bits |= this->sleep     << 2;
  bits |= this->wakeup;
  return bits;
}

ADXL345::DataFormatBits::DataFormatBits() {
  this->selfTest  = 0;
  this->spi       = 0;
  this->intInvert = 0;
  this->fullRes   = 0;
  this->justify   = 0;
  this->range     = 0;
}

uint8_t ADXL345::DataFormatBits::toByte() {
  uint8_t bits = 0x00;
  bits |= this->selfTest  << 7;
  bits |= this->spi       << 6;
  bits |= this->intInvert << 5;
  bits |= this->fullRes   << 3;
  bits |= this->justify   << 2;
  bits |= this->range;
  return bits;
}

ADXL345::BwRateBits::BwRateBits() {
  this->lowPower = 0;
  this->rate     = 0x0A;
}

uint8_t ADXL345::BwRateBits::toByte() {
  uint8_t bits = 0x00;
  bits |= this->lowPower << 4;
  bits |= this->rate;
  return bits;
}



// ADXL345.ino ######################################################################################################################################

float ax, ay, az;



void updateACC(void)
{
  float aoffsetX, aoffsetY, aoffsetZ;
  float rawX, rawY, rawZ;
  float X, Y, Z;
  float rollrad, pitchrad;
  float rolldeg, pitchdeg;

  //  accel.readAccel(&ax, &ay, &az); //read the accelerometer values and store them in variables  x,y,z
  rawX = ax - aoffsetX;
  rawY = ay - aoffsetY;
  rawZ = az  - (255 - aoffsetZ);
  X = rawX / 256.00; // used for angle calculations
  Y = rawY / 256.00; // used for angle calculations
  Z = rawZ / 256.00; // used for angle calculations

  rolldeg = 180 * (atan(Y / sqrt(X * X + Z * Z))) / PI; // calculated angle in degrees
  pitchdeg = 180 * (atan(X / sqrt(Y * Y + Z * Z))) / PI; // calculated angle in degrees
  MwAngle[1] = pitchdeg * 360;
  MwAngle[0] = rolldeg * 360;
  /*
    Serial.println(" ");
    Serial.print(MwAngle[1]);
    Serial.print(", ");
    Serial.print(MwAngle[0]);
    Serial.println(" ");
    */
}

void readADXL345(void)
{
  if (accel.update()) {
    ax = accel.getX();
    ay = accel.getY();
    az = accel.getZ();
    updateACC();
  }

  Serial.println(" ");
  Serial.print(MwAngle[1]);
  Serial.print(", ");
  Serial.print(MwAngle[0]);
  Serial.println(" ");

}

void initADXL345(void)
{
  Wire.begin();
  byte deviceID = accel.readDeviceID();
  if (deviceID != 0) {
    Serial.print("0x");
    Serial.print(deviceID, HEX);
    Serial.println(" ");
    delay(3000);
  } else {
    Serial.println("read device id: failed");
    while (1) {
      delay(100);
      readADXL345();
    }
  }
}




#endif

