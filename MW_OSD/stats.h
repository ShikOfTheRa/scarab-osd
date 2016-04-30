#ifndef __STATS_H
#define __STATS_H

class StatsClass {
  public:
    void Reset(void);
    void CalculateTrip(void);
    void CalculateAmps(void);

    bool IsAmpAlarming();

    // These fields do not get reset with Reset()
    // For Time
    uint16_t onTime=0;
    uint16_t flyTime=0;
    // For Amperage
    float amperage = 0;                // its the real value x10
    float amperagesum = 0;
    uint16_t MWAmperage=0;

    // These get Reset()
    // TODO: make private wherever possible
    uint16_t _speedMAX=0;
    int16_t _altitudeMAX=0;
    uint32_t _distanceMAX=0;
    uint16_t _ampMAX=0;
    uint32_t _trip=0;
    // TODO: $$$ can we do this w/o a float?
    float _tripSum=0; 
    uint16_t _flyingTime=0;
};

extern StatsClass Stats;

#endif
