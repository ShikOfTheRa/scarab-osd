#ifndef __SENSORS_H
#define __SENSORS_H

/********************       For Sensors presence      *********************/
#define ACCELEROMETER  1//0b00000001
#define BAROMETER      2//0b00000010
#define MAGNETOMETER   4//0b00000100
#define GPSSENSOR      8//0b00001000
//#define SONAR         16//0b00010000

#define SENSORFILTERSIZE 8
#define SENSORTOTAL 5

class SensorsClass
{
  public:
    void Process(void);
    void Force(void);
    bool IsPresent(uint8_t sensorMask);
    bool IsActive(uint8_t sensorMask);
    
    void SetPresent(uint16_t present);
    void SetActive(uint32_t active);

    int16_t sensorfilter[SENSORTOTAL][SENSORFILTERSIZE+2];

    // For Voltage
    uint16_t voltage=0;                      // its the value x10
    uint16_t vidvoltage=0;                   // its the value x10
    uint8_t voltageWarning=0;
    uint8_t vidvoltageWarning=0;
  private:
    uint16_t  MwSensorPresent=0;
    uint32_t  MwSensorActive=0;
};

extern SensorsClass Sensors;

#endif

