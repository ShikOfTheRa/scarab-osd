#ifndef __SCREEN_H
#define __SCREEN_H

#include "symbols.h"

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
  private:
    uint8_t findNull(void);
    uint16_t getPosition(uint8_t pos);
    uint8_t fieldIsVisible(uint8_t pos);

    char _screenBuffer[20]; 
    uint16_t _screenPosition[POSITIONS_SETTINGS];

    uint8_t screenlayout=0;
    uint8_t oldscreenlayout=0;
};

extern ScreenClass Screen;

#endif
