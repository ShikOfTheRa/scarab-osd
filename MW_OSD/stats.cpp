#include "platform.h"

void StatsClass::Reset(void) {
  _speedMAX=0;
  _altitudeMAX=0;
  _distanceMAX=0;
  _ampMAX=0;
  _trip=0;
  _tripSum=0; 
  _flyingTime=0;
}

void StatsClass::CalculateTrip(void)
{
  if(GPS_fix && armed && (GPS_speed>0)) {
    if(Settings[S_UNITSYSTEM])
      _tripSum += GPS_speed *0.0032808;     //  100/(100*1000)*3.2808=0.0016404     cm/sec ---> ft/50msec
    else
      _tripSum += GPS_speed *0.0010;        //  100/(100*1000)=0.0005               cm/sec ---> mt/50msec (trip var is float)      
  }
  _trip = (uint32_t) _tripSum;
}

void StatsClass::CalculateAmps(void) {
  if (Eeprom.Settings[S_AMPER_HOUR]) {
    amperagesum += amperage;
  }
};

// ampAlarming returns true if the total consumed mAh is greater than
// the configured alarm value (which is stored as 100s of amps)
bool StatsClass::IsAmpAlarming() {
    int used = pMeterSum > 0 ? pMeterSum : (amperagesum / 360);
    return used > (Eeprom.Settings[S_AMPER_HOUR_ALARM]*100);
}
