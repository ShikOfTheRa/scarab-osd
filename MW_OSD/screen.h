#ifndef __SCREEN_H
#define __SCREEN_H

#include "symbols.h"

enum Positions {
  GPS_numSatPosition,
  GPS_directionToHomePosition,
  GPS_distanceToHomePosition,
  speedPosition,
  GPS_angleToHomePosition,
  MwGPSAltPosition,
  sensorPosition,
  MwHeadingPosition,
  MwHeadingGraphPosition,
  MwAltitudePosition,
  MwClimbRatePosition,
  CurrentThrottlePosition,
  flyTimePosition,
  onTimePosition,
  motorArmedPosition,
  pitchAnglePosition,
  rollAnglePosition,
  MwGPSLatPositionTop,
  MwGPSLonPositionTop,
  rssiPosition,
  temperaturePosition,
  voltagePosition,
  vidvoltagePosition,
  amperagePosition,
  pMeterSumPosition,
  horizonPosition,
  SideBarPosition,
  SideBarScrollPosition,
  SideBarHeightPosition,
  SideBarWidthPosition,
  gimbalPosition,
  GPS_timePosition,
  SportPosition,
  ModePosition,
  MapModePosition,
  MapCenterPosition,
  APstatusPosition,
  wattPosition,
  glidescopePosition,
  callSignPosition,
  debugPosition,

  POSITIONS_SETTINGS
};

class ScreenClass
{
  public:
    void DisplayIntro(void);
    void DisplayConfigScreen(void);
    void DisplayHorizon(int rollAngle, int pitchAngle);
    void DisplayCallsign(void);
    void DisplayVoltage(void);
    void DisplayVidVoltage(void);
    void DisplayI2CError();
    void DisplayRSSI(void);
    void DisplayAmperage(void);
    void DisplayWatt(void);
    void DisplaypMeterSum(void);
    void DisplayTime(void);
    void DisplayArmed(void);
    void DisplayCurrentThrottle(void);
    void DisplayHeadingGraph(void);
    void DisplayHeading(void);
    void DisplayAltitude(void);
    void DisplayClimbRate(void);
    void DisplayNumberOfSat(void);
    void DisplayDirectionToHome(void);
    void DisplayCursor(void);
    void DisplayDistanceToHome(void);
    void DisplayAngleToHome(void);
    void DisplayGPSAltitude(void);
    void DisplayGPSSpeed(void);
    void DisplayGPSPosition(void);
    void DisplayGPSTime(void);
    void DisplayTemperature(void);
    void DisplayMode(void);
    void DisplayDebug(void);
    void DisplayCells(void);
    void DisplayForcedCrosshair(void);

    void ReadLayout(void);

    void MapMode(void);

    void MenuconfigOnoff(uint16_t pos, uint8_t setting);
    void UpdateLayout(void);

    // Run when a loadfont mode is # defined
    void LoadFontLoop(void);

    // Config status and cursor location
    uint8_t ROW=10;
    uint8_t COL=3;
    int8_t configPage=1;
    int8_t previousconfigPage=1;

    // int32_t ?...
    uint16_t debugBuffer[4];

  private:
    uint8_t findNull(void);
    uint16_t getPosition(uint8_t pos);
    uint8_t fieldIsVisible(uint8_t pos);

    char _screenBuffer[20]; 
    uint16_t _screenPosition[POSITIONS_SETTINGS];

    uint8_t screenlayout=0;
    uint8_t oldscreenlayout=0;
    uint8_t oldROW=0;

    // Battery
    uint8_t _cells=0;

    // For decoration
    uint8_t SYM_AH_DECORATION_LEFT = 0x10;
    uint8_t SYM_AH_DECORATION_RIGHT = 0x10;
    //static uint8_t sym_sidebartopspeed = SYM_BLANK;
    //static uint8_t sym_sidebarbottomspeed = SYM_BLANK;
    //static uint8_t sym_sidebartopalt = SYM_BLANK;
    //static uint8_t sym_sidebarbottomalt = SYM_BLANK;
};

extern ScreenClass Screen;

#endif
