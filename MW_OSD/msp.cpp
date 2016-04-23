#include "platform.h"

void setMspRequests() {
  if(Font.inFontMode()) {
    modeMSPRequests = REQ_MSP_FONT;
  }
  else if(configMode) {
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
      REQ_MSP_ATTITUDE|
      REQ_MSP_RAW_IMU|
      REQ_MSP_ALTITUDE|
      REQ_MSP_RC_TUNING|
      REQ_MSP_PID_CONTROLLER|
      REQ_MSP_PID|
      REQ_MSP_LOOP_TIME|
#ifdef CORRECT_MSP_BF1
      REQ_MSP_CONFIG|
#endif
#ifdef DEBUGMW
      REQ_MSP_DEBUG|
#endif
#ifdef SPORT
      REQ_MSP_CELLS|
#endif
#ifdef HAS_ALARMS
      REQ_MSP_ALARMS|
#endif
      REQ_MSP_RC;
  }
  else {
//wtf:?? try deleting next 4 lines and what happens to memory. is it local vs global in some way?
//    MwSensorPresent |=GPSSENSOR;
//    MwSensorPresent |=BAROMETER;
//    MwSensorPresent |=MAGNETOMETER;
//    MwSensorPresent |=ACCELEROMETER;

    modeMSPRequests = 
      REQ_MSP_STATUS|
      REQ_MSP_RC|
     #ifdef DEBUGMW
      REQ_MSP_DEBUG|
     #endif
     #ifdef SPORT      
      REQ_MSP_CELLS|
     #endif
     #ifdef HAS_ALARMS
      REQ_MSP_ALARMS|
     #endif
      REQ_MSP_ATTITUDE;
    if(MwSensorPresent&BAROMETER){ 
      modeMSPRequests |= REQ_MSP_ALTITUDE;
    }
    if(flags.ident!=1){
      modeMSPRequests |= REQ_MSP_IDENT;
    }
    if(MwSensorPresent&GPSSENSOR) 
      modeMSPRequests |= REQ_MSP_RAW_GPS| REQ_MSP_COMP_GPS;
    if(mode.armed == 0)
      modeMSPRequests |=REQ_MSP_BOX;
#if defined MULTIWII_V24
    if(MwSensorActive&mode.gpsmission)
    modeMSPRequests |= REQ_MSP_NAV_STATUS;
#endif
  }
 
  if(Settings[S_MAINVOLTAGE_VBAT] ||
    Settings[S_MWRSSI]) {
    modeMSPRequests |= REQ_MSP_ANALOG;
    
#ifdef USE_FC_VOLTS_CONFIG
    modeMSPRequests |= REQ_MSP_MISC;
#endif

  }

  queuedMSPRequests &= modeMSPRequests;   // so we do not send requests that are not needed.
}


