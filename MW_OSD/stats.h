#ifndef __STATS_H
#define __STATS_H

class StatsClass {
  public:
    void Reset(void);
    void CalculateTrip(void);
    void CalculateAmps(void);

    // These fields do not get reset with Reset()
    // For Time
    uint16_t onTime=0;
    uint16_t flyTime=0;
    // For Amperage
    float amperage = 0;                // its the real value x10
    float amperagesum = 0;
    uint16_t MWAmperage=0;

    uint8_t MwVBat=0;
    int16_t MwVario=0;
    uint8_t armed=0;
    uint8_t previousarmedstatus=0;  // for statistics after disarming
    uint16_t armedangle=0;           // for capturing direction at arming
    uint32_t GPS_distanceToHome=0;
    uint8_t GPS_fix=0;
    uint8_t GPS_frame_timer=0;
    int32_t GPS_latitude;
    int32_t GPS_longitude;
    int16_t GPS_altitude;
    int16_t GPS_home_altitude;
    int16_t previousfwaltitude=0;
    int16_t interimfwaltitude=0;
    uint16_t GPS_speed;
    int16_t  GPS_ground_course;
    uint16_t old_GPS_speed;
    int16_t GPS_directionToHome=0;
    uint8_t GPS_numSat=0;
    uint8_t GPS_waypoint_step=0;
    uint16_t pMeterSum=0;
    uint16_t MwRssi=0;
    uint32_t GPS_time = 0;        //local time of coord calc - haydent

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
