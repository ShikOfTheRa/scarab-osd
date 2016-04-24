#include "platform.h"

char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  } 

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *ItoaUnPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  } 

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}
char *FormatGPSCoord(int32_t val, char *str, uint8_t p, char pos, char neg) {
  if(val < 0) {
    pos = neg;
    val = -val;
  }

  uint8_t bytes = p+6;
  val = val / 100;
 
  str[bytes] = 0;
  str[--bytes] = pos;
  for(;;) {
    if(bytes == p) {
      str[--bytes] = DECIMAL;
      continue;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (bytes < 3 && val == 0))
      break;   }

   while(bytes != 0)
     str[--bytes] = ' ';

   return str;
}

// Take time in Seconds and format it as 'MM:SS'
// Alternately Take time in Minutes and format it as 'HH:MM'
// If hhmmss is 1, display as HH:MM:SS
char *formatTime(uint32_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}

uint8_t ScreenClass::findNull(void)
{
  uint8_t xx;
  for(xx=0;_screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}

 
uint16_t ScreenClass::getPosition(uint8_t pos)
{
  uint16_t val = _screenPosition[pos];
  uint16_t ret = val&POS_MASK;
  return ret;
}

uint8_t ScreenClass::fieldIsVisible(uint8_t pos)
{
//  uint16_t val = (uint16_t)pgm_read_word(&_screenPosition[pos]);
  uint16_t val = _screenPosition[pos];
  if ((val & DISPLAY_MASK)==DISPLAY_ALWAYS)
    return 1;
  else
    return 0;  
/*
  switch(val & DISPLAY_MASK) {
  case DISPLAY_ALWAYS:
    return 1;
  case DISPLAY_NEVER:
    return 0;
  case DISPLAY_COND:
    return !!(MwSensorActive&mode.osd_switch);
  case DISPLAY_MIN_OFF:
    return !(MwSensorActive&mode.osd_switch);
  }
*/
}


void ScreenClass::DisplayTemperature(void)        // DEPRECATED RUSHDUINO SUPPORT
{
  int xxx;
  if (Settings[S_UNITSYSTEM])
    xxx = temperature*1.8+32;       //Fahrenheit conversion for imperial system.
  else
    xxx = temperature;

//  if(!fieldIsVisible(temperaturePosition))
//    return;

  itoa(xxx,_screenBuffer,10);
  uint8_t xx = findNull();   // find the NULL
  _screenBuffer[xx++]=temperatureUnitAdd[Settings[S_UNITSYSTEM]];
  _screenBuffer[xx]=0;  // Restore the NULL
  MAX7456.WriteString(_screenBuffer,getPosition(temperaturePosition));
}


void ScreenClass::DisplayMode(void)
{
  if(timer.MSP_active==0){ // no MSP >> mode display not valid
    return;
  }  
  uint8_t xx = 0;
  
  if((MwSensorActive&mode.camstab)&&Settings[S_GIMBAL]){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_GIMBAL;
    _screenBuffer[1]=SYM_GIMBAL1;  
    if(fieldIsVisible(gimbalPosition))
      MAX7456.WriteString(_screenBuffer,getPosition(gimbalPosition));
  }
  if(Settings[S_MODESENSOR]){
    xx = 0;
    if(MwSensorActive&mode.stable||MwSensorActive&mode.horizon){
      _screenBuffer[xx] = SYM_ACC;
      xx++;
    }
    if (MwSensorActive&mode.baro){
      _screenBuffer[xx] = SYM_BAR;
      xx++;
    }
    if (MwSensorActive&mode.mag){
      _screenBuffer[xx] = SYM_MAG;
      xx++;
    }
    _screenBuffer[xx] = 0;
    if(fieldIsVisible(sensorPosition)){
      MAX7456.WriteString(_screenBuffer,getPosition(sensorPosition));
    }
  }  

  
  if(MwSensorActive&mode.passthru){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_PASS;
    _screenBuffer[1]=SYM_PASS1;
  }
  else if(MwSensorActive&mode.gpshome){
#ifdef APINDICATOR
    _screenBuffer[0] = SYM_GHOME;
    _screenBuffer[1] = SYM_GHOME1;
    _screenBuffer[2]=0;
#else
    uint32_t dist;
    if(Settings[S_UNITSYSTEM]){
      dist = GPS_distanceToHome * 3.2808;           // mt to feet
    }
    else{
      dist = GPS_distanceToHome;                    // Mt
    }
    itoa(dist, _screenBuffer+3, 10);
    xx = findNull()+1;

    if(Settings[S_UNITSYSTEM]==METRIC){
      _screenBuffer[xx] =SYM_M;
    }
    else{
     _screenBuffer[xx] =SYM_FT;
    }
    _screenBuffer[xx] =0;
    _screenBuffer[0] = SYM_GHOME;
    _screenBuffer[1] = SYM_GHOME1;
    _screenBuffer[2] = SYM_COLON;
    _screenBuffer[8]=0;
#endif
  }
  else if(MwSensorActive&mode.gpshold){
    _screenBuffer[2]=0;
    _screenBuffer[0] = SYM_GHOLD;
    _screenBuffer[1] = SYM_GHOLD1;
  }
#if defined MULTIWII_V24
  else if(MwSensorActive&mode.gpsmission){
    itoa(GPS_waypoint_step,_screenBuffer+2,10);
    _screenBuffer[4]=0;
    _screenBuffer[0] = SYM_GMISSION;
    _screenBuffer[1] = SYM_GMISSION1;  
  }
  else if(MwSensorActive&mode.gpsland){
    _screenBuffer[2]=0;
    _screenBuffer[0] = SYM_GLAND;
    _screenBuffer[1] = SYM_GLAND1;
  }
#endif //MULTIWII_V24    
  else if(MwSensorActive&mode.stable){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_STABLE;
    _screenBuffer[1]=SYM_STABLE1;
  }
  else if(MwSensorActive&mode.horizon){
    _screenBuffer[2]=0;
    _screenBuffer[0]=SYM_HORIZON;
    _screenBuffer[1]=SYM_HORIZON1;
  }
  else{
    _screenBuffer[2]=0;
    #ifdef FIXEDWING
      _screenBuffer[0]=SYM_ACROGY;
    #else
      _screenBuffer[0]=SYM_ACRO;
    #endif
    _screenBuffer[1]=SYM_ACRO1;
#if defined ACROPLUS
    if((MwSensorActive)&(mode.acroplus)){
      _screenBuffer[1]=SYM_PLUS;
    }
#endif //ACROPLUS
  }
  if(Settings[S_MODEICON]){
    if(fieldIsVisible(ModePosition)){
      MAX7456.WriteString(_screenBuffer,getPosition(ModePosition));
    #ifdef AIRMODE
      if((MwSensorActive)&(mode.air)){
        _screenBuffer[0]=SYM_AIR;
        _screenBuffer[1]=SYM_AIR1;
        _screenBuffer[2]=0;
      MAX7456.WriteString(_screenBuffer,getPosition(ModePosition)+AIRMODE);
      }
    #endif //AIRMODE  
    }  
  }

#ifdef APINDICATOR
  if(timer.Blink2hz)
    return;
  if(!fieldIsVisible(APstatusPosition))
    return;
  uint8_t apactive=0;
  if (MwSensorActive&mode.gpshome)
    apactive=6;
  else if (MwSensorActive&mode.gpshold)
    apactive=7;
  else if (MwSensorActive&mode.gpsmission)
    apactive=8;
  else
    return;
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[apactive])));
  MAX7456.WriteString(_screenBuffer, getPosition(APstatusPosition));
#endif

}


void ScreenClass::DisplayCallsign()
{
      for(uint8_t X=0; X<10; X++) {
          _screenBuffer[X] = char(Settings[S_CS0 + X]);
     }   
       _screenBuffer[10] = 0;
       MAX7456.WriteString(_screenBuffer, getPosition(callSignPosition)); 
}


void ScreenClass::DisplayHorizon(int rollAngle, int pitchAngle)
{
  // TODO: $$$ #define guard
  static uint8_t sidebarsdir; // speed
  static uint8_t sidebaradir; // alt
  static unsigned long sidebarsMillis = 0;
  static unsigned long sidebaraMillis = 0;

#ifdef DISPLAY_PR
  _screenBuffer[0]=0x50;
  int16_t xx=abs(pitchAngle/10);
  uint8_t offset=1;
  if(pitchAngle<0) {
    _screenBuffer[1]='-';
    offset++;
  }
  itoa(xx, _screenBuffer+offset,10);     
  if(fieldIsVisible(pitchAnglePosition))
    MAX7456.WriteString(_screenBuffer,getPosition(pitchAnglePosition));
  _screenBuffer[0]=0x52;
  offset=1;
  xx=abs(rollAngle/10);
  if(rollAngle<0) {
    _screenBuffer[1]='-';
    offset++;
  }
  itoa(xx, _screenBuffer+offset,10);     
  if(fieldIsVisible(rollAnglePosition))
    MAX7456.WriteString(_screenBuffer,getPosition(rollAnglePosition));
#endif

#ifdef HORIZON
  if (Settings[S_SCROLLING]||Settings[S_SIDEBARTOPS]){
      if(!armed) GPS_speed=0;
  // Scrolling decoration
      if (GPS_speed > (old_GPS_speed+15)){
        sidebarsMillis = millis();
        sidebarsdir = 2;
        old_GPS_speed = GPS_speed;
        SYM_AH_DECORATION_LEFT--;
        if (SYM_AH_DECORATION_LEFT<0x10)
          SYM_AH_DECORATION_LEFT=0x15;
      }
      else if ((GPS_speed+15) < old_GPS_speed){
        sidebarsMillis = millis();
        sidebarsdir = 1;
       old_GPS_speed = GPS_speed;
        SYM_AH_DECORATION_LEFT++;
        if (SYM_AH_DECORATION_LEFT>0x15)
          SYM_AH_DECORATION_LEFT=0x10;
      }
 
      if (MwAltitude > old_MwAltitude+20){
        sidebaraMillis = millis();
        sidebaradir = 2;
        old_MwAltitude = MwAltitude;
        SYM_AH_DECORATION_RIGHT--;
        if (SYM_AH_DECORATION_RIGHT<0x10)
          SYM_AH_DECORATION_RIGHT=0x15;
      }
      else if (MwAltitude+20 < old_MwAltitude){
        sidebaraMillis = millis();
        sidebaradir = 1;
        old_MwAltitude = MwAltitude;
        SYM_AH_DECORATION_RIGHT++;
        if (SYM_AH_DECORATION_RIGHT>0x15)
          SYM_AH_DECORATION_RIGHT=0x10;
      }
  }

  if (!Settings[S_SCROLLING]){
     SYM_AH_DECORATION_LEFT=0x13;
     SYM_AH_DECORATION_RIGHT=0x13;
  } 
  
  uint16_t position = getPosition(horizonPosition);

  if(pitchAngle>AHIPITCHMAX) pitchAngle=AHIPITCHMAX;
  if(pitchAngle<-AHIPITCHMAX) pitchAngle=-AHIPITCHMAX;
  if(rollAngle>AHIROLLMAX) rollAngle=AHIROLLMAX;
  if(rollAngle<-AHIROLLMAX) rollAngle=-AHIROLLMAX;
  #ifndef AHICORRECT
    #define AHICORRECT 10
  #endif
  pitchAngle=pitchAngle+AHICORRECT;
  #if defined REVERSEAHI
    pitchAngle=-pitchAngle;
    rollAngle=-rollAngle;
  #endif //REVERSEAHI

  if(Settings[S_DISPLAY_HORIZON_BR]&fieldIsVisible(horizonPosition)){

  #ifdef FULLAHI
    for(uint8_t X=0; X<=12; X++) {
      if (X==6) X=7;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position -9 + LINE*(Y/9) + 3 - 4*LINE + X;
        MAX7456.WriteChar(SYM_AH_BAR9_0+(Y%9), pos);
        if (Settings[S_HORIZON_ELEVATION]){ 
          if(X >= 4 && X <= 8) {
            MAX7456.WriteChar(SYM_AH_BAR9_0+(Y%9), pos-3*LINE);
            MAX7456.WriteChar(SYM_AH_BAR9_0+(Y%9), pos+3*LINE);
          }
        }
      }
    }
  #else //FULLAHI
    for(uint8_t X=0; X<=8; X++) {
      if (X==4) X=5;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position -7 + LINE*(Y/9) + 3 - 4*LINE + X;
        MAX7456.WriteChar(SYM_AH_BAR9_0+(Y%9), pos);
        if (Settings[S_HORIZON_ELEVATION]){ 
          if(X >= 2 && X <= 6) {
            MAX7456.WriteChar(SYM_AH_BAR9_0+(Y%9), pos-3*LINE);
            MAX7456.WriteChar(SYM_AH_BAR9_0+(Y%9), pos+3*LINE);
          }
        }            
      }
    }
  #endif //FULLAHI

    if(!fieldIsVisible(MapModePosition)){
      if(Settings[S_DISPLAY_HORIZON_BR]){
        MAX7456.WriteChar(SYM_AH_CENTER_LINE, position-1);
        MAX7456.WriteChar(SYM_AH_CENTER_LINE_RIGHT, position+1);
        MAX7456.WriteChar(SYM_AH_CENTER, position);
      }
    }
  }

  if (Settings[S_WITHDECORATION]&fieldIsVisible(SideBarPosition)){
    // Draw AH sides
    int8_t hudwidth=  getPosition(SideBarWidthPosition)&0x0F;
    int8_t hudheight= getPosition(SideBarHeightPosition)&0x0F;
    for(int8_t X=-hudheight; X<=hudheight; X++) {
      MAX7456.WriteChar(SYM_AH_DECORATION_LEFT, position-hudwidth+(X*LINE));
      MAX7456.WriteChar(SYM_AH_DECORATION_RIGHT, position+hudwidth+(X*LINE));
    }
#if defined AHILEVEL
    MAX7456.WriteChar(SYM_AH_LEFT, position-hudwidth+1);
    MAX7456.WriteChar(SYM_AH_RIGHT, position+hudwidth-1);
#endif //AHILEVEL
    
  #if defined(USEGLIDESCOPE) && defined(FIXEDWING)                     
    if(Settings[S_DISPLAYGPS]){
      displayfwglidescope();
    }
  #endif //USEGLIDESCOPE  

  #ifdef SBDIRECTION

    if (Settings[S_SIDEBARTOPS]&fieldIsVisible(SideBarScrollPosition)) {
      if (millis()<(sidebarsMillis + 1000)) {
        if (sidebarsdir == 2){
          MAX7456.WriteChar(SYM_AH_DECORATION_UP, position-(hudheight*LINE)-hudwidth);
        }
        else{
          MAX7456.WriteChar(SYM_AH_DECORATION_DOWN, position+(hudheight*LINE)-hudwidth);
        }
      }
      if (millis()<(sidebaraMillis + 1000)) { 
        if (sidebaradir == 2){
          MAX7456.WriteChar(SYM_AH_DECORATION_UP, position-(hudheight*LINE)+hudwidth);
        }
        else{
          MAX7456.WriteChar(SYM_AH_DECORATION_DOWN, position+(hudheight*LINE)+hudwidth);
        }
      }
    }
  #endif //SBDIRECTION
  }
#endif //HORIZON
}


void ScreenClass::DisplayVoltage(void)
{
  if (Settings[S_MAINVOLTAGE_VBAT]){
    voltage=MwVBat;
  }
  
#ifdef AUTOCELL
  uint8_t tcells = ((voltage-3) / MvVBatMaxCellVoltage) + 1;
  if (tcells>cells){
    if (tcells<11){
      cells=tcells;
    }
  }   
  voltageWarning = cells * MvVBatWarningCellVoltage;
 #ifdef AUTOCELL_VOLTAGE
  voltageWarning = cells * Settings[S_VOLTAGEMIN];
 #endif // AUTOCELL_VOLTAGE
#else //NOT AUTOCELL
  voltageWarning = Settings[S_VOLTAGEMIN];
  cells = Settings[S_BATCELLS];
#endif //AUTOCELL


#ifdef BATTERYICONVOLTS
  if (Settings[S_SHOWBATLEVELEVOLUTION])
  {
    uint8_t battev = 0;
    int batevlow  = cells * MvVBatMinCellVoltage;
    int batevhigh = cells * MvVBatMaxCellVoltage;
    battev = constrain(voltage, batevlow, batevhigh-2);
    battev = map(battev, batevlow, batevhigh-1, 0, 7);   
    _screenBuffer[0]=(SYM_BATT_EMPTY)-battev;
  }
  else 
#endif // BATTERYICONVOLTS

  {
    _screenBuffer[0]=SYM_MAIN_BATT;
  }

  if ((voltage<voltageWarning)&&(timer.Blink2hz))
    return;

#ifdef DISP_LOW_VOLTS_WARNING
  if (voltage<=voltageWarning&&!armedtimer)
    MAX7456.WriteString_P(lowvolts_text, getPosition(motorArmedPosition));
#endif

#ifdef FORCE_DISP_LOW_VOLTS
  if(fieldIsVisible(voltagePosition)||(voltage<=voltageWarning)) 
#else
  if(fieldIsVisible(voltagePosition)) 
#endif
  {
    ItoaPadded(voltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    MAX7456.WriteString(_screenBuffer,getPosition(voltagePosition)-1);
  }
}


void ScreenClass::DisplayVidVoltage(void)
{
  if((vidvoltage<vidvoltageWarning)&&(timer.Blink2hz))
    return;
#ifdef FORCE_DISP_LOW_VID_VOLTS
  if(fieldIsVisible(vidvoltagePosition)||(vidvoltage<=vidvoltageWarning)) 
#else
  if(fieldIsVisible(vidvoltagePosition)) 
#endif  
  {
    _screenBuffer[0]=SYM_VID_BAT;
    ItoaPadded(vidvoltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;    
    MAX7456.WriteString(_screenBuffer,getPosition(vidvoltagePosition)-1);
  }
}


void ScreenClass::DisplayCurrentThrottle(void)
{
  if(!fieldIsVisible(CurrentThrottlePosition))
    return;

  #ifndef NOTHROTTLESPACE
    #define THROTTLESPACE 1
  #else
    #define THROTTLESPACE 0
  #endif  
  _screenBuffer[1]=' ';
  #ifdef AUTOTHROTTLE
    if (MwRcData[THROTTLESTICK] > HighT) HighT = MwRcData[THROTTLESTICK];
    if (MwRcData[THROTTLESTICK] < LowT) LowT = MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
    if (HighT>2050) HighT=2050;
    if (LowT<950) LowT=950;
  #else
    HighT=HIGHTHROTTLE;
    LowT=LOWTHROTTLE;
  #endif
    
  
  uint32_t Vthrottle = constrain(MwRcData[THROTTLESTICK],1000,2000);

#ifndef FIXEDWING   
  if(!armed) {
    _screenBuffer[0+THROTTLESPACE]=' ';
    _screenBuffer[1+THROTTLESPACE]=' ';
    _screenBuffer[2+THROTTLESPACE]='-';
    _screenBuffer[3+THROTTLESPACE]='-';
    _screenBuffer[4+THROTTLESPACE]=' ';

  }
  else
#endif // FIXEDWING    
  {
    int CurThrottle = map(MwRcData[THROTTLESTICK],LowT,HighT,0,100);
    ItoaPadded(CurThrottle,_screenBuffer+1+THROTTLESPACE,3,0);
    _screenBuffer[4+THROTTLESPACE]='%';
  }
    _screenBuffer[0]=SYM_THR;
    _screenBuffer[5+THROTTLESPACE]=0;
    MAX7456.WriteString(_screenBuffer,getPosition(CurrentThrottlePosition));
}


void ScreenClass::DisplayTime(void) 
{ 
  if(!Settings[S_TIMER])
    return;
  if (_screenPosition[onTimePosition]<512)
    return;

  uint32_t displaytime;
  if (armed) { 
    if(((flyTime/60)>=Settings[S_FLYTIME_ALARM])&&(timer.Blink2hz))
      return;

    if(flyTime < 3600) {
      _screenBuffer[0] = SYM_FLY_M;
      displaytime = flyTime;
    }
    else {
      _screenBuffer[0] = SYM_FLY_H;
      displaytime = flyTime/60;
    }
  }
  else {
    if(onTime < 3600) {
      _screenBuffer[0] = SYM_ON_M;
      displaytime = onTime;
    }
    else {
      _screenBuffer[0] = SYM_ON_H;
      displaytime = onTime/60;
    }
  }
  formatTime(displaytime, _screenBuffer+1, 0);
  MAX7456.WriteString(_screenBuffer,getPosition(onTimePosition));
}


void ScreenClass::DisplayAmperage(void)
{
  if(amperage > ampMAX)
    ampMAX = amperage;
  if(!fieldIsVisible(amperagePosition))
    return;
  ItoaPadded(amperage, _screenBuffer, 5, 4);     // 999.9 ampere max!
  _screenBuffer[5] = SYM_AMP;
  _screenBuffer[6] = 0;
  MAX7456.WriteString(_screenBuffer,getPosition(amperagePosition));
}


void ScreenClass::DisplayWatt(void)
{
  if(!fieldIsVisible(wattPosition))
    return;
  uint16_t WhrPosition = getPosition(wattPosition);
  uint16_t watts = amperage*voltage/100; 
  ItoaPadded(watts, _screenBuffer+1 , 5, 5);
  _screenBuffer[0] = SYM_BLANK;
  _screenBuffer[5] = SYM_WATT;
  _screenBuffer[6] = 0;
  MAX7456.WriteString(_screenBuffer,WhrPosition);
}


void ScreenClass::DisplaypMeterSum(void)
{
  if(!fieldIsVisible(pMeterSumPosition))
    return;

  #ifdef BATTERYICONAMPS
  uint16_t battev =0;
  if (Settings[S_SHOWBATLEVELEVOLUTION]){
    battev=amperagesum/(360*Settings[S_AMPER_HOUR_ALARM]);
    battev=constrain(battev,0,100);
    battev = map(100-battev, 0, 101, 0, 7);
    _screenBuffer[0]=SYM_BATT_EMPTY-battev;
    _screenBuffer[1]=SYM_MAH;
    int xx=amperagesum/360;
    itoa(xx,_screenBuffer+2,10);
  }
  else 
  #endif //BATTERYICONAMPS
  {
  _screenBuffer[0]=SYM_MAH;
  int xx=amperagesum/360;
  itoa(xx,_screenBuffer+1,10);
  }
  MAX7456.WriteString(_screenBuffer,getPosition(pMeterSumPosition));
}


void ScreenClass::DisplayI2CError(void)
{
#ifdef I2CERROR
  if (I2CError<=I2CERROR)
    return;
  _screenBuffer[0] = 0x49;
  _screenBuffer[1] = 0X3A;
  itoa(I2CError,_screenBuffer+2,10);
  MAX7456.WriteString(_screenBuffer,getPosition(temperaturePosition));
#endif
}


void ScreenClass::DisplayRSSI(void)
{
  if(!fieldIsVisible(rssiPosition))
    return;
  _screenBuffer[0] = SYM_RSSI;
  itoa(rssi,_screenBuffer+1,10);
  uint8_t xx = findNull();
  _screenBuffer[xx++] = '%';
  _screenBuffer[xx] = 0;
  MAX7456.WriteString(_screenBuffer,getPosition(rssiPosition)-1);
}


void ScreenClass::DisplayHeading(void)
{
 if(!fieldIsVisible(MwHeadingPosition))
    return;
 if (Settings[S_SHOWHEADING]) {  
   int16_t heading = MwHeading;
   if (Settings[S_HEADING360]) {
     if(heading < 0)
       heading += 360;
       ItoaPadded(heading,_screenBuffer,3,0);
       _screenBuffer[3]=SYM_DEGREES;
       _screenBuffer[4]=0;
    }
    else {
      ItoaPadded(heading,_screenBuffer,4,0);
      _screenBuffer[4]=SYM_DEGREES;
      _screenBuffer[5]=0;
    }
    MAX7456.WriteString(_screenBuffer,getPosition(MwHeadingPosition));
  }  
}


void ScreenClass::DisplayHeadingGraph(void)
{
  if (!fieldIsVisible(MwHeadingGraphPosition))
    return;
  if (!Settings[S_COMPASS])
    return;
  int xx;
  xx = MwHeading * 4;
  xx = xx + 720 + 45;
  xx = xx / 90;
  uint16_t pos = getPosition(MwHeadingGraphPosition);
  MAX7456.WriteBytes_P(headGraph+xx+1, pos, 9);
}


void ScreenClass::DisplayIntro(void)
{
  const int startCol = 4;
  int line = LINE02;

  MAX7456.WriteString_P(message0, line+startCol);
  line += LINE*2;
#ifndef GPSOSD
  MAX7456.WriteString_P(message5, line+startCol);
  MAX7456.WriteString(ItoaPadded(MwVersion, _screenBuffer, 4, 2), line+startCol+11);
  line += LINE;
#endif
#ifdef INTRO_CALLSIGN
  MAX7456.WriteString_P(message9, line+startCol);
  DisplayCallsign(line+startCol+4);
  line += LINE;
#endif
#ifdef GPSTIME
#ifdef INTRO_TIMEZONE
//timezone
  MAX7456.WriteString_P(message10, line+startCol);
  if(abs(Settings[S_GPSTZ]) >= 100)ItoaPadded(Settings[S_GPSTZ], _screenBuffer, 5, 4);
  else ItoaPadded(Settings[S_GPSTZ], _screenBuffer, 4, 3);
  if(Settings[S_GPSTZAHEAD] || Settings[S_GPSTZ] == 0)_screenBuffer[0] = '+';
  else _screenBuffer[0] = '-';
  MAX7456.WriteString(_screenBuffer, line+startCol+8);
  line += LINE;
//more settings
  MAX7456.WriteString_P(message11, line+startCol);
#endif
#endif
#ifdef INTRO_MENU
  MAX7456.WriteString_P(message6, line+startCol);
  line += LINE;
  MAX7456.WriteString_P(message7, line+startCol+10);
  line += LINE;
  MAX7456.WriteString_P(message8, line+startCol+10);
  line += LINE;
 #endif
#ifdef HAS_ALARMS
  if (alarmState != ALARM_OK) {
      line += LINE;
      MAX7456.WriteString((const char*)alarmMsg, line+startCol);
  }
#endif
}


void ScreenClass::DisplayGPSPosition(void)
{
  uint16_t position;
  if(!GPS_fix)
    return;
  DisplayGPSAltitude();
  if(!fieldIsVisible(MwGPSLatPositionTop))
    return;
  if (!MwSensorActive&mode.gpshome)
    return;
  if(Settings[S_COORDINATES]|(MwSensorActive&mode.gpshome)){
      position = getPosition(MwGPSLatPositionTop);  
      _screenBuffer[0] = SYM_LAT;
      FormatGPSCoord(GPS_latitude,_screenBuffer+1,4,'N','S');
      MAX7456.WriteString(_screenBuffer, position);  
      position = getPosition(MwGPSLonPositionTop);  
      _screenBuffer[0] = SYM_LON;
      FormatGPSCoord(GPS_longitude,_screenBuffer+1,4,'E','W');
      MAX7456.WriteString(_screenBuffer, position);  
  }
}


void ScreenClass::DisplayGPSAltitude(void){
  if(Settings[S_GPSALTITUDE]){
    if(!fieldIsVisible(MwGPSAltPosition))
    return;
    _screenBuffer[0] = MwGPSAltPositionAdd[Settings[S_UNITSYSTEM]];
    uint16_t xx;
    if(Settings[S_UNITSYSTEM])
      xx = GPS_altitude * 3.2808; // Mt to Feet
    else
      xx = GPS_altitude;          // Mt
    if(((xx/10)>=Settings[S_ALTITUDE_ALARM])&&(timer.Blink2hz))
      return;
    itoa(xx,_screenBuffer+1,10);
    MAX7456.WriteString(_screenBuffer,getPosition(MwGPSAltPosition));
  }
}


void ScreenClass::DisplayNumberOfSat(void)
{
  if((GPS_numSat<MINSATFIX)&&(timer.Blink2hz)){
    return;
  }
  if(!fieldIsVisible(GPS_numSatPosition))
    return;
  _screenBuffer[0] = SYM_SAT_L;
  _screenBuffer[1] = SYM_SAT_R;
  itoa(GPS_numSat,_screenBuffer+2,10);
  MAX7456.WriteString(_screenBuffer,getPosition(GPS_numSatPosition));
}


void ScreenClass::DisplayGPSSpeed(void)
{
  if(!GPS_fix) return;
  if(!armed) GPS_speed=0;
  uint16_t xx;
  if(!Settings[S_UNITSYSTEM])
    xx = GPS_speed * 0.036;           // From MWii cm/sec to Km/h
  else
    xx = GPS_speed * 0.02236932;      // (0.036*0.62137)  From MWii cm/sec to mph
  if(xx > speedMAX)
    speedMAX = xx;
  if(!fieldIsVisible(speedPosition))
    return;
  if((xx>Settings[S_SPEED_ALARM])&&(timer.Blink2hz))
    return;    
  _screenBuffer[0]=speedUnitAdd[Settings[S_UNITSYSTEM]];
  itoa(xx,_screenBuffer+1,10);
  MAX7456.WriteString(_screenBuffer,getPosition(speedPosition));
#ifdef SHOW_MAX_SPEED
  itoa(speedMAX,_screenBuffer+1,10);
  MAX7456.WriteString(_screenBuffer,getPosition(speedPosition)+LINE);
#endif //SHOW_MAX_SPEED
}


void ScreenClass::DisplayGPSTime(void)       //local time of coord calc - haydent
{
  if(!GPS_fix) return;
  if(!Settings[S_GPSTIME]) return;
  if(!fieldIsVisible(GPS_timePosition)) return;

//convert to local   
  int TZ_SIGN = (Settings[S_GPSTZAHEAD] ? 1 :-1);
  uint32_t local = GPS_time + (((Settings[S_GPSTZ] * 60 * TZ_SIGN / 10)) * 60000);//make correction for time zone
  local = local % 604800000;//prob not necessary but keeps day of week accurate <= 7
//convert to local

//format and display
//  uint16_t milli = local % 1000;//get milli for later
  uint32_t seconds = (local / 1000) % 86400;//remove millisonds and whole days
  
  formatTime(seconds, _screenBuffer, 1);
  if(_screenBuffer[0] == ' ')_screenBuffer[0] = '0';//put leading zero if empty space
/*
  _screenBuffer[8] = '.';//add milli indicator
  _screenBuffer[9] = '0' + (milli / 100);//only show first digit of milli due to limit of gps rate
  _screenBuffer[10] = 0;//equivalent of new line or end of buffer
*/   
  _screenBuffer[8] = 0;//equivalent of new line or end of buffer
  MAX7456.WriteString(_screenBuffer,getPosition(GPS_timePosition));
}


void ScreenClass::DisplayAltitude(void)
{
  int16_t altitude;
  if(Settings[S_UNITSYSTEM])
    altitude = MwAltitude*0.032808;    // cm to feet
  else
    altitude = MwAltitude/100;         // cm to mt

  if(armed && allSec>5 && altitude > altitudeMAX)
    altitudeMAX = altitude;
  if(!fieldIsVisible(MwAltitudePosition))
    return;
  if(!Settings[S_BAROALT])
    return;
  if(((altitude/10)>=Settings[S_ALTITUDE_ALARM])&&(timer.Blink2hz))
    return;   
  _screenBuffer[0]=MwAltitudeAdd[Settings[S_UNITSYSTEM]];
  itoa(altitude,_screenBuffer+1,10);
  MAX7456.WriteString(_screenBuffer,getPosition(MwAltitudePosition));
#ifdef SHOW_MAX_ALTITUDE
  itoa(altitudeMAX,_screenBuffer+1,10);
  MAX7456.WriteString(_screenBuffer,getPosition(MwAltitudePosition)+LINE);
#endif //SHOW_MAX_ALTITUDE

}


void ScreenClass::DisplayClimbRate(void)
{
  if(!fieldIsVisible(MwClimbRatePosition))
    return;
  if(!Settings[S_VARIO])
    return;
    uint16_t position = getPosition(MwClimbRatePosition);
    for(int8_t X=-1; X<=1; X++) {
      MAX7456.WriteChar(SYM_VARIO, position+(X*LINE));
    }
   int8_t xx=MwVario;
   if (MwVario>120) xx=120;
   if (MwVario<-120) xx=-120;
   xx=map(xx,120,-120,0,17);
   int8_t varline=(xx/6)-1;
   int8_t varsymbol=xx%6;
   MAX7456.WriteChar(0x8F-varsymbol, position+(varline*LINE));
}


void ScreenClass::DisplayDistanceToHome(void)
{
  if(!GPS_fix)
    return;
  uint16_t dist;
  if(Settings[S_UNITSYSTEM])
    dist = GPS_distanceToHome * 3.2808;           // mt to feet
  else
    dist = GPS_distanceToHome;                    // Mt
  if(dist > distanceMAX)
    distanceMAX = dist;
  if(!fieldIsVisible(GPS_distanceToHomePosition))
    return;

  if(((dist/100)>=Settings[S_DISTANCE_ALARM])&&(timer.Blink2hz))
    return;
    
  _screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
  itoa(dist, _screenBuffer+1, 10);

#if defined LONG_RANGE_DISPLAY // Change to decimal KM / Miles    
  if (dist>9999){

    if(Settings[S_UNITSYSTEM]){
      dist = int(GPS_distanceToHome * 0.0062137);           // mt to miles*10
    }
    else{
      dist = dist=dist/100;
    }
    itoa(dist, _screenBuffer+1, 10);
    uint8_t xx = findNull();
//    if (xx==2){ // if want to limit distance to 999 or less instead of 9999. This adds a leading 0 for improved display
//      _screenBuffer[2]=_screenBuffer[1];
//      _screenBuffer[1]=0x30;
//      xx++;
//    }
    _screenBuffer[xx]=_screenBuffer[xx-1];
    _screenBuffer[xx-1] = DECIMAL;
    xx++;
    _screenBuffer[xx] = 0;
  }
#endif // LONG_RANGE_DISPLAY

  _screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
  MAX7456.WriteString(_screenBuffer,getPosition(GPS_distanceToHomePosition));
}


void ScreenClass::DisplayAngleToHome(void)
{
  if(!GPS_fix)
      return;
 if(!fieldIsVisible(GPS_angleToHomePosition))
    return;
  if(Settings[S_ANGLETOHOME]){
    if((GPS_numSat < MINSATFIX) && timer.Blink2hz)
      return;
    ItoaPadded(GPS_directionToHome,_screenBuffer,3,0);
    _screenBuffer[3] = SYM_DEGREES;
    _screenBuffer[4] = 0;
    MAX7456.WriteString(_screenBuffer,getPosition(GPS_angleToHomePosition));
  }
}


void ScreenClass::DisplayDirectionToHome(void)
{
  if(!GPS_fix)
    return;
 if(!fieldIsVisible(GPS_directionToHomePosition))
    return;

  if(GPS_distanceToHome <= 2 && timer.Blink2hz)
    return;
  uint16_t position=getPosition(GPS_directionToHomePosition);
  int16_t d = MwHeading + 180 + 360 - GPS_directionToHome;
  d *= 4;
  d += 45;
  d = (d/90)%16;
  _screenBuffer[0] = SYM_ARROW_SOUTH + d;
  _screenBuffer[1] = 0;
  MAX7456.WriteString(_screenBuffer,position);
}


void ScreenClass::DisplayCursor(void)
{
  int cursorpos;
  if(ROW==10){
    if(COL==3) cursorpos=SAVEP+16-1;    // page
    if(COL==1) cursorpos=SAVEP-1;       // exit
    if(COL==2) cursorpos=SAVEP+6-1;     // save/exit
  }
  if(ROW<10)
    {
#ifdef MENU_PID
    if(configPage==MENU_PID){
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      if(COL==1) cursorpos=(ROW+2)*30+10;
      if(COL==2) cursorpos=(ROW+2)*30+10+6;
      if(COL==3) cursorpos=(ROW+2)*30+10+6+6;
     }
#endif
#ifdef MENU_RC
  #if defined CORRECT_MENU_RCT2
     if(configPage==MENU_RC){
      COL=3;
      cursorpos=(ROW+2)*30+10+6+6;
    }
  #elif defined CORRECT_MENU_RCT1
    if(configPage==MENU_RC)
    {  
      if (ROW==9){
        if (oldROW==8)
          ROW=10;
        else
          ROW=8;
      }
      oldROW=ROW;
      COL=3;
      cursorpos=(ROW+2)*30+10+6+6;
      }
  #else
    if(configPage==MENU_RC){
      COL=3;
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      cursorpos=(ROW+2)*30+10+6+6;
      }
  #endif
      
#endif
#ifdef MENU_VOLTAGE
    if(configPage==MENU_VOLTAGE){
      COL=3;
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      cursorpos=(ROW+2)*30+10+6+6;     
      }
#endif
#ifdef MENU_RSSI
    if(configPage==MENU_RSSI){
      COL=3;
      if (ROW==7) ROW=10;
      if (ROW==9) ROW=6;
      cursorpos=(ROW+2)*30+10+6+6;
      }    
#endif
#ifdef MENU_CURRENT
    if(configPage==MENU_CURRENT)
      {  
      COL=3;
      if (ROW==9) ROW=5;
      if (ROW==6) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_DISPLAY
    if(configPage==MENU_DISPLAY)
      {  
        if (ROW==9){
          if (oldROW==8)
            ROW=10;
          else
            ROW=8;
        }
        oldROW=ROW;
        COL=3;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_ADVANCED
    if(configPage==MENU_ADVANCED)
      {  
      COL=3;
      if (ROW==9) ROW=6;
      if (ROW==7) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef MENU_GPS_TIME
    if(configPage==MENU_GPS_TIME)
      {  
      COL=3;
      if (ROW==9) ROW=3;
      if (ROW==4) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif     
#ifdef MENU_ALARMS
    if(configPage==MENU_ALARMS)
      {  
      COL=3;
      if (ROW==9) ROW=6;
      if (ROW==7) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif     
#ifdef MENU_PROFILE
    if(configPage==MENU_PROFILE)
      {  
      #ifdef CORRECTLOOPTIME
        if (ROW==9) ROW=3;
        if (ROW==4) ROW=10;
      #else
        if (ROW==9) ROW=2;
        if (ROW==3) ROW=10;
      #endif

      COL=3;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif     
  }
  if(timer.Blink10hz) {
    MAX7456.WriteChar(SYM_CURSOR, cursorpos);
  }
}


void ScreenClass::DisplayConfigScreen(void)
{
  int16_t MenuBuffer[10];
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menutitle_item[configPage])));
  MAX7456.WriteString(_screenBuffer, 35);
  #ifdef MENU_PROFILE
//   MAX7456.WriteString(itoa(FCProfile,_screenBuffer,10),50); // Display Profile number
  #endif 
  MAX7456.WriteString_P(configMsgEXT, SAVEP);    //EXIT
  if(!previousarmedstatus) {
    MAX7456.WriteString_P(configMsgSAVE, SAVEP+6);  //SaveExit
    MAX7456.WriteString_P(configMsgPGS, SAVEP+16); //<Page>
  }

  if(configPage==MENU_STAT)
  {
    int xx;
//    MAX7456.WriteString_P(configMsg00, 35);

#ifdef SHORTSUMMARY
    strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_stats_item[0])));
    MAX7456.WriteString(_screenBuffer, ROLLT);
    formatTime(flyingTime, _screenBuffer, 1);
    MAX7456.WriteString(_screenBuffer,ROLLD-4);

#else // SHORTSUMMARY

 //     MenuBuffer[0]=rcRate8;
      xx=amperagesum/360;
      itoa(xx,_screenBuffer,10);
      MenuBuffer[1]=trip;
      MenuBuffer[2]=distanceMAX;
      MenuBuffer[3]=altitudeMAX;
      MenuBuffer[4]=speedMAX;
      MenuBuffer[5]=xx;
      MenuBuffer[6]=ampMAX/10;

    for(uint8_t X=0; X<=6; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_stats_item[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT + (X*30));
      MAX7456.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),110+(30*X));
    }

    formatTime(flyingTime, _screenBuffer, 1);
    MAX7456.WriteString(_screenBuffer,ROLLD-4);
#endif
#ifdef HAS_ALARMS
    if (alarmState != ALARM_OK) {
        MAX7456.WriteString((const char*)alarmMsg, LINE12 + 3);
    }
#endif

    }
#ifdef MENU_PID
  if(configPage==MENU_PID)
  {
    for(uint8_t X=0; X<=6; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_pid[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
   
    for(uint8_t Y=0; Y<=8; Y++) {      
      if (Y==5) Y=7;
      uint8_t X=Y;
      if (Y>6){
        X=X-2;
      }
      MAX7456.WriteString(itoa(P8[Y],_screenBuffer,10),ROLLP+(X*30));
      MAX7456.WriteString(itoa(I8[Y],_screenBuffer,10),ROLLI+(X*30));
      MAX7456.WriteString(itoa(D8[Y],_screenBuffer,10),ROLLD+(X*30));
    }

    MAX7456.WriteString("P",71);
    MAX7456.WriteString("I",77);
    MAX7456.WriteString("D",83);
  }
#endif
#ifdef MENU_RC
  if(configPage==MENU_RC)
  {
    #if defined CORRECT_MENU_RCT2
      MenuBuffer[0]=rcRate8;
      MenuBuffer[1]=rcExpo8;
      MenuBuffer[2]=rollRate;
      MenuBuffer[3]=PitchRate;
      MenuBuffer[4]=yawRate;
      MenuBuffer[5]=dynThrPID;
      MenuBuffer[6]=thrMid8;
      MenuBuffer[7]=thrExpo8;
      MenuBuffer[8]=tpa_breakpoint16;
       for(uint8_t X=0; X<=8; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rc[X])));
        MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
        MAX7456.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    #elif defined CORRECT_MENU_RCT1
      MenuBuffer[0]=rcRate8;
      MenuBuffer[1]=rcExpo8;
      MenuBuffer[2]=rollRate;
      MenuBuffer[3]=PitchRate;
      MenuBuffer[4]=yawRate;
      MenuBuffer[5]=dynThrPID;
      MenuBuffer[6]=thrMid8;
      MenuBuffer[7]=thrExpo8;
       for(uint8_t X=0; X<=7; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rc[X])));
        MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
        MAX7456.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    #else
      MenuBuffer[0]=rcRate8;
      MenuBuffer[1]=rcExpo8;
      MenuBuffer[2]=rollPitchRate;
      MenuBuffer[3]=yawRate;
      MenuBuffer[4]=dynThrPID;
      MenuBuffer[5]=thrMid8;
      MenuBuffer[6]=thrExpo8;
     for(uint8_t X=0; X<=6; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rc[X])));
        MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
        MAX7456.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    #endif
  }
#endif
#ifdef MENU_VOLTAGE
  if(configPage==MENU_VOLTAGE)
  {
    ProcessSensors();
    _screenBuffer[0]=SYM_MAIN_BATT;
    ItoaPadded(voltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    MAX7456.WriteString(_screenBuffer,ROLLD-LINE-LINE-1);

    _screenBuffer[0]=SYM_VID_BAT;
    ItoaPadded(vidvoltage, _screenBuffer+1, 4, 3);
    _screenBuffer[5] = SYM_VOLT;
    _screenBuffer[6] = 0;
    MAX7456.WriteString(_screenBuffer,ROLLI-LINE-LINE-3);

    for(uint8_t X=0; X<=6; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_bat[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    MAX7456.WriteString(itoa(Settings[S_VIDVOLTAGE],_screenBuffer,10),ALTD);
    MenuconfigOnoff(ROLLD,S_DISPLAYVOLTAGE);
    MAX7456.WriteString(itoa(Settings[S_DIVIDERRATIO],_screenBuffer,10),PITCHD);
    MAX7456.WriteString(itoa(Settings[S_VOLTAGEMIN],_screenBuffer,10),YAWD);
    MenuconfigOnoff(ALTD,S_VIDVOLTAGE);
    MAX7456.WriteString(itoa(Settings[S_VIDDIVIDERRATIO],_screenBuffer,10),VELD);
    MAX7456.WriteString(itoa(Settings[S_BATCELLS],_screenBuffer,10),LEVD);
    MenuconfigOnoff(MAGD,S_MAINVOLTAGE_VBAT);
  }
#endif
#ifdef MENU_RSSI
  if(configPage==MENU_RSSI)
  {
    itoa(rssi,_screenBuffer,10);
    uint8_t xx = findNull();
    _screenBuffer[xx++] = '%';
    _screenBuffer[xx] = 0;
    MAX7456.WriteString(_screenBuffer,ROLLD-LINE-LINE);
    for(uint8_t X=0; X<=5; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_rssi[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    MenuconfigOnoff(ROLLD,S_DISPLAYRSSI);
    if(timer.rssiTimer>0) {
      MAX7456.WriteString(itoa(timer.rssiTimer,_screenBuffer,10),PITCHD);
    }
    else {
    MAX7456.WriteString("-", PITCHD);
    }
    MenuconfigOnoff(YAWD,S_MWRSSI);
    MenuconfigOnoff(ALTD,S_PWMRSSI);
    MAX7456.WriteString(itoa(Settings16[S16_RSSIMAX],_screenBuffer,10),VELD);
    MAX7456.WriteString(itoa(Settings16[S16_RSSIMIN],_screenBuffer,10),LEVD);
  }
#endif
#ifdef MENU_CURRENT
  if(configPage==MENU_CURRENT)
  {
    ItoaPadded(amperage, _screenBuffer, 4, 3);     // 99.9 ampere max!
    _screenBuffer[4] = SYM_AMP;
    _screenBuffer[5] = 0;
    MAX7456.WriteString(_screenBuffer,ROLLD-LINE-LINE-1);

    for(uint8_t X=0; X<=4; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_amps[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
    MenuconfigOnoff(ROLLD,S_AMPERAGE);
    MenuconfigOnoff(PITCHD,S_AMPER_HOUR);
    MenuconfigOnoff(YAWD,S_AMPERAGE_VIRTUAL);
    MAX7456.WriteString(itoa(Settings16[S16_AMPDIVIDERRATIO],_screenBuffer,10),ALTD);
    MAX7456.WriteString(itoa(Settings16[S16_AMPZERO],_screenBuffer,10),VELD);
  }
#endif
#ifdef MENU_DISPLAY
  if(configPage==MENU_DISPLAY)
  {
    for(uint8_t X=0; X<=7; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_display[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }    
    MenuconfigOnoff(ROLLD,S_DISPLAY_HORIZON_BR);
    MenuconfigOnoff(PITCHD,S_WITHDECORATION);
    MenuconfigOnoff(YAWD,S_SCROLLING);
    MenuconfigOnoff(ALTD,S_THROTTLEPOSITION);
    MenuconfigOnoff(VELD,S_COORDINATES);
    MenuconfigOnoff(LEVD,S_MODESENSOR);
    MenuconfigOnoff(MAGD,S_GIMBAL);
    MAX7456.WriteString(itoa(Settings[S_MAPMODE],_screenBuffer,10),MAGD+LINE);
  }
#endif
#ifdef MENU_ADVANCED
  if(configPage==MENU_ADVANCED)
  {
    for(uint8_t X=0; X<=5; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_advanced[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }

//  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[apactive])));
//  MAX7456.WriteString(_screenBuffer, getPosition(APstatusPosition));


    if(!Settings[S_UNITSYSTEM]){
      MAX7456.WriteString_P(configMsg710, ROLLD);
    }
    else {
      MAX7456.WriteString_P(configMsg711, ROLLD);
      }
    if(!Settings[S_VIDEOSIGNALTYPE]){
      MAX7456.WriteString_P(configMsg720, PITCHD);
    }
    else {
      MAX7456.WriteString_P(configMsg721, PITCHD);
      }
    if(Settings[S_VREFERENCE]){
      MAX7456.WriteString_P(configMsg730, YAWD);
    }
    else {
      MAX7456.WriteString_P(configMsg731, YAWD);
      }

/*
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[Settings[S_UNITSYSTEM]])));
  MAX7456.WriteString(_screenBuffer, ROLLD);
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[Settings[S_VIDEOSIGNALTYPE]])));
  MAX7456.WriteString(_screenBuffer, PITCHD);
  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[Settings[S_VREFERENCE]])));
  MAX7456.WriteString(_screenBuffer, YAWD);
*/
    MenuconfigOnoff(ALTD,S_DEBUG);    
    if(timer.magCalibrationTimer>0)
      MAX7456.WriteString(itoa(timer.magCalibrationTimer,_screenBuffer,10),VELD);
    else
      MAX7456.WriteString("-",VELD);
    MAX7456.WriteString(itoa(Settings[S_RCWSWITCH_CH],_screenBuffer,10),LEVD);
   }
#endif
#ifdef MENU_GPS_TIME
  if(configPage==MENU_GPS_TIME)
  {
    for(uint8_t X=0; X<=2; X++) {
      strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_gps_time[X])));
      MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
    }
  MenuconfigOnoff(ROLLD,S_GPSTIME);    
  MenuconfigOnoff(PITCHD,S_GPSTZAHEAD);    
  MAX7456.WriteString(itoa(Settings[S_GPSTZ],_screenBuffer,10),YAWD);
  }    
#endif  
#ifdef MENU_ALARMS
    if(configPage==MENU_ALARMS){
      MenuBuffer[0]=Settings[S_DISTANCE_ALARM];
      MenuBuffer[1]=Settings[S_ALTITUDE_ALARM];
      MenuBuffer[2]=Settings[S_SPEED_ALARM];
      MenuBuffer[3]=Settings[S_FLYTIME_ALARM];
      MenuBuffer[4]=Settings[S_AMPER_HOUR_ALARM];
      MenuBuffer[5]=Settings[S_AMPERAGE_ALARM];
      for(uint8_t X=0; X<=5; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_alarm_item[X])));
        MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
        MAX7456.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    }
#endif  
#ifdef MENU_PROFILE
    if(configPage==MENU_PROFILE){
      #ifdef CORRECTLOOPTIME
        #define MENU10MAX 2
      #else
        #define MENU10MAX 1
      #endif
      MenuBuffer[0]=FCProfile;
      MenuBuffer[1]=PIDController;
      MenuBuffer[2]=LoopTime;
      for(uint8_t X=0; X<=MENU10MAX; X++) {
        strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_profile[X])));
        MAX7456.WriteString(_screenBuffer, ROLLT+ (X*30));
        MAX7456.WriteString(itoa(MenuBuffer[X],_screenBuffer,10),113+(30*X));
      }
    }
#endif  
    if(configPage > MAXPAGE)configPage=MINPAGE;

  DisplayCursor();
}



void ScreenClass::DisplayDebug(void)
{
#if defined (DEBUG)||defined (DEBUGMW)
  if(!Settings[S_DEBUG])
    return;
 
  for(uint8_t X=0; X<4; X++) {
    ItoaPadded(debug[X], _screenBuffer+2,7,0);     
    _screenBuffer[0] = 0x30+X;
    _screenBuffer[1] = 0X3A;
    MAX7456.WriteString(_screenBuffer,getPosition(debugPosition)+(X*LINE));
  } 
 
#endif
}

void ScreenClass::DisplayCells(void){

  #ifndef MIN_CELL
    #define MIN_CELL 320
  #endif
  uint16_t sum = 0;
  uint16_t low = 0;
  uint8_t cells = 0;   
  
  for(uint8_t i=0; i<6; i++) {
    uint16_t volt = cell_data[i];
    if(!volt)continue;//empty cell
    ++cells;
    sum += volt;
    if(volt < low || !low)low = volt;
    if((volt>MIN_CELL)||(timer.Blink2hz)){
      int tempvolt=constrain(volt,300,415);
      tempvolt = map(tempvolt,300,415,0,14);
      _screenBuffer[i]=SYM_CELL0+tempvolt;
      }
      else _screenBuffer[i]=' ';      
    }
    
    if(cells){
      _screenBuffer[cells] = 0;       
      MAX7456.WriteString(_screenBuffer,getPosition(SportPosition)+(6-cells));//bar chart

      ItoaPadded(low, _screenBuffer+1,4,2);
      _screenBuffer[0] = SYM_MIN;
      _screenBuffer[5] = SYM_VOLT;
      _screenBuffer[6] = 0;
    
    if((low>MIN_CELL)||(timer.Blink2hz))
      MAX7456.WriteString(_screenBuffer,getPosition(SportPosition)+LINE);//lowest
    
      uint16_t avg = 0;
      if(cells)avg = sum / cells;
      ItoaPadded( avg, _screenBuffer+1,4,2);
      _screenBuffer[0] = SYM_AVG;
      _screenBuffer[5] = SYM_VOLT;
      _screenBuffer[6] = 0;
    
    if((avg>MIN_CELL)||(timer.Blink2hz))
      MAX7456.WriteString(_screenBuffer,getPosition(SportPosition)+(2*LINE));//average     
  }
}


void ScreenClass::MapMode(void) {

#ifdef MAPMODE

  int mapstart=0;
  int mapend=0;

  switch(Settings[S_MAPMODE]) {
    case 1:
      mapstart=0;mapend=1;
      break;
    case 2:
      mapstart=1;mapend=2;
      break;
    case 3:
      mapstart=0;mapend=2;
      break;
    case 4:
      mapstart=1;mapend=2;
      break;
    default:
      return;
  }

  if(!GPS_fix)
    return;
  if(!fieldIsVisible(MapModePosition))
    return;
    
  int8_t xdir=0;
  int8_t ydir=0;
  int16_t targetx;
  int16_t targety;
  int16_t range=200;
  int16_t angle;
  int16_t targetpos;
  int16_t centerpos;
  uint32_t maxdistance;
  uint8_t mapsymbolcenter;
  uint8_t mapsymboltarget;
  uint8_t mapsymbolrange;
  int16_t tmp;


for(uint8_t maptype=mapstart; maptype<mapend; maptype++) {

  if (maptype==1) {
    angle=(180+360+GPS_directionToHome-armedangle)%360;
  }
  else {
    angle=(360+GPS_directionToHome-MwHeading)%360;  
  }
  
  tmp = angle/90;
  switch (tmp) {
    case 0:
      xdir=+1;
      ydir=-1;
      break;
    case 1:    
      xdir=+1;
      ydir=+1;
      angle=180-angle;
      break;
    case 2:    
      xdir=-1;
      ydir=+1;
      angle=angle-180;
      break;
    case 3: 
      xdir=-1;
      ydir=-1;
      angle=360-angle;
      break;   
    }  

  float rad  = angle * PI / 180;    // convert to radians  
  uint16_t x = (uint16_t)(GPS_distanceToHome * sin(rad));
  uint16_t y = (uint16_t)(GPS_distanceToHome * cos(rad));

  if (y > x) maxdistance=y;
  else maxdistance=x;
  if (maxdistance < 100) {
    range = 100;
    mapsymbolrange=SYM_RANGE_100;
  }
  else if (maxdistance < 500) {
    range = 500;
    mapsymbolrange=SYM_RANGE_500;
  }
  else if (maxdistance < 2500) {
    range = 2500;
    mapsymbolrange=SYM_RANGE_2500;
  }
  else {
    range = maxdistance;
    mapsymbolrange=SYM_RANGE_MAX;
  }

  targetx = xdir*map(x, 0, range, 0, 16);
  targety = ydir*map(y, 0, range, 0, 15);

  if (maxdistance<20) {
    targetx = 0;
    targety = 0;  
  }
    
  centerpos=getPosition(MapCenterPosition);
  targetpos= centerpos + (targetx/2) + (LINE*(targety/3)); 

  if (maptype==1) {
    mapsymbolcenter = SYM_HOME;
    mapsymboltarget = SYM_AIRCRAFT;
  }
  else {
    mapsymbolcenter = SYM_AIRCRAFT;
    mapsymboltarget = SYM_HOME;
  }

    int8_t symx = (int8_t)abs(targetx)%2;
    int8_t symy = (int8_t)abs(targety)%3;
    if (ydir==1)
      symy=2-symy;
    if (xdir==-1)
      symx=1-symx;
    if (abs(targety)<3)
      symy = 1 - ydir;
    if (abs(targetx)<2){
      if (targetx<0)
        symx=0;
      else
        symx=1;
    }

  if (maptype==0) 
    mapsymboltarget = 0xD6;
  else
    mapsymboltarget = 0xD0;

  mapsymboltarget = uint8_t( mapsymboltarget + symy + (symx*3));


  if (Settings[S_MAPMODE]==4) {
    tmp=(360+382+MwHeading-armedangle)%360/45;
    mapsymboltarget = SYM_DIRECTION + tmp;
  }

  _screenBuffer[0] = mapsymbolrange;
  _screenBuffer[1] = 0;
  MAX7456.WriteString(_screenBuffer,getPosition(MapModePosition));

  _screenBuffer[0] = mapsymboltarget;
  _screenBuffer[1] = 0;
  MAX7456.WriteString(_screenBuffer,targetpos);

  _screenBuffer[0] = mapsymbolcenter;
  _screenBuffer[1] = 0;
  MAX7456.WriteString(_screenBuffer,centerpos);
  }
 
#endif
}

#ifdef USEGLIDESCOPE
void ScreenClass::Displayfwglidescope(void){
  if(!fieldIsVisible(glidescopePosition))
    return;  

  int8_t GS_deviation_scale   = 0;
  if (GPS_distanceToHome>0){ //watch div 0!!
    int16_t gs_angle          =(573*atan((float)MwAltitude/10/GPS_distanceToHome));
    int16_t GS_target_delta   = gs_angle-USEGLIDESCOPE;
    GS_target_delta           = constrain(GS_target_delta,-400,400); 
    GS_deviation_scale        = map(GS_target_delta,400,-400,0,8);
  }

  int8_t varline              = (GS_deviation_scale/3)-1;
  int8_t varsymbol            = GS_deviation_scale%3;
    
  uint16_t position = getPosition(glidescopePosition);
    for(int8_t X=-1; X<=1; X++) {
      MAX7456.WriteChar(SYM_GLIDESCOPE, position+(X*LINE));
    }
    MAX7456.WriteChar(SYM_GLIDESCOPE+3-varsymbol, position+(varline*LINE));
}
#endif //USEGLIDESCOPE

void ScreenClass::MenuconfigOnoff(uint16_t pos, uint8_t setting){
    strcpy_P(_screenBuffer, (char*)pgm_read_word(&(menu_on_off[(Settings[setting])])));
    MAX7456.WriteString(_screenBuffer, pos);
}

void ScreenClass::DisplayArmed(void)
{
  if(!fieldIsVisible(motorArmedPosition))
    return;  
  uint8_t message_no = 0;

#ifdef HAS_ALARMS
  if (alarmState != ALARM_OK) {
      // There's an alarm, let it have this space.
      return;
  }
#endif

  if(!armed){
    message_no=1;
    armedtimer=30;
  } 

  if(MwSensorPresent&GPSSENSOR){
    #ifdef SATACTIVECHECK
    if (GPS_numSat<MINSATFIX){ // below minimum preferred value
      message_no=5;
    }
    #endif //SATACTIVECHECK

    #ifdef GPSACTIVECHECK
    if(timer.GPS_active==0){
      message_no=4;
    }
    #endif //GPSACTIVECHECK
  }

  #ifdef MSPACTIVECHECK
  if(timer.MSP_active==0){
    message_no=3;
  }
  #endif //MSPACTIVECHECK
  if(armedtimer&&armed){
    if (timer.Blink10hz){
      armedtimer--;
      message_no=2;
    }
    else{
      message_no=0;
    }
  }

   if(message_no>2){
      if(!armed&&timer.Blink2hz){
        message_no=1;
      }
      else if(!armed){
      }
      else if(timer.Blink2hz){
        return;
      }
    }

  #ifdef HIDEARMEDSTATUS
  if(message_no<3){
    return;
  }
  #endif //HIDEARMEDSTATUS

  strcpy_P(_screenBuffer, (char*)pgm_read_word(&(message_item[message_no])));
  if (message_no>0){
    MAX7456.WriteString(_screenBuffer, getPosition(motorArmedPosition));
  }
}

void ScreenClass::DisplayForcedCrosshair()
{
  uint16_t position = getPosition(horizonPosition);
  MAX7456.WriteChar(SYM_AH_CENTER_LINE, position-1);
  MAX7456.WriteChar(SYM_AH_CENTER_LINE_RIGHT, position+1);
  MAX7456.WriteChar(SYM_AH_CENTER, position);
}

// TODO: $$$ this should be in EEPROM, and return a struct or something that gets assigned to _screenPosition
void ScreenClass::ReadLayout(EEPROMClass* EEPROM)
{
  // $$$ screenlayout is global
  uint16_t EEPROMscreenoffset=EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(screenlayout*POSITIONS_SETTINGS*2);
  for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
    uint16_t pos = (en*2)+EEPROMscreenoffset;
    _screenPosition[en] = EEPROM->read(pos);
    uint16_t xx = (uint16_t)EEPROM->read(pos+1)<<8;
    _screenPosition[en] = _screenPosition[en] + xx;

    if(Settings[S_VIDEOSIGNALTYPE]){
      uint16_t x = _screenPosition[en]&0x1FF;
      if (x>LINE06) _screenPosition[en] = _screenPosition[en] + LINE;
      if (x>LINE09) _screenPosition[en] = _screenPosition[en] + LINE;
    }
#ifdef SHIFTDOWN
    if ((_screenPosition[en]&0x1FF)<LINE04) _screenPosition[en] = _screenPosition[en] + LINE;
#endif
  }
}



#ifdef HAS_ALARMS
void ScreenClass::DisplayAlarms()
{
    if (alarmState == ALARM_OK) {
        return;
    }
    if (alarmState == ALARM_CRIT || alarmState == ALARM_ERROR || timer.Blink2hz) {
        MAX7456.WriteString((const char*)alarmMsg, getPosition(motorArmedPosition));
    }
}
#endif
