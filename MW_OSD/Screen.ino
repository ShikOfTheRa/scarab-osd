
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

uint8_t FindNull(void)
{
  uint8_t xx;
  for(xx=0;screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}

 
uint16_t getPosition(uint8_t pos) {
  uint16_t val = screenPosition[pos];
  uint16_t ret = val&POS_MASK;
  return ret;
}

uint8_t fieldIsVisible(uint8_t pos) {
//  uint16_t val = (uint16_t)pgm_read_word(&screenPosition[pos]);
  uint16_t val = screenPosition[pos];
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
}


void displayTemperature(void)        // WILL WORK ONLY WITH V1.2
{
  int xxx;
  if (Settings[S_UNITSYSTEM])
    xxx = temperature*1.8+32;       //Fahrenheit conversion for imperial system.
  else
    xxx = temperature;

  if(!fieldIsVisible(temperaturePosition))
    return;

  itoa(xxx,screenBuffer,10);
  uint8_t xx = FindNull();   // find the NULL
  screenBuffer[xx++]=temperatureUnitAdd[Settings[S_UNITSYSTEM]];
  screenBuffer[xx]=0;  // Restore the NULL
  MAX7456_WriteString(screenBuffer,getPosition(temperaturePosition));
}


void displayMode(void)
{
  
  int16_t dist;
  if(Settings[S_UNITSYSTEM])
    dist = GPS_distanceToHome * 3.2808;           // mt to feet
  else
    dist = GPS_distanceToHome;                    // Mt

  if(dist > distanceMAX)
    distanceMAX = dist;
  itoa(dist, screenBuffer+3, 10);
  uint8_t xx = FindNull();
  if(Settings[S_UNITSYSTEM]==METRIC)
    screenBuffer[xx++] =SYM_M;
  else
   screenBuffer[xx++] =SYM_FT;
   screenBuffer[xx++] =0;

    if(MwSensorActive&mode.passthru){ // Remove All mode Icons Create a Manual.
      screenBuffer[2]=0;
      screenBuffer[0] = 0;
      screenBuffer[1] = 0;
    }
 
    if(MwSensorActive&mode.gpshome){
      screenBuffer[0] = SYM_GHOME;
      screenBuffer[1] = SYM_GHOME1;
#ifdef APINDICATOR
      screenBuffer[2]=0;
#else
      screenBuffer[2] = SYM_COLON;
      screenBuffer[8]=0;
#endif
    }
    else if(MwSensorActive&mode.gpshold){
      screenBuffer[2]=0;
      screenBuffer[0] = SYM_GHOLD;
      screenBuffer[1] = SYM_GHOLD1;
    }
    else if(MwSensorActive&mode.gpsmission){
      itoa(GPS_waypoint_step,screenBuffer+2,10);
      screenBuffer[4]=0;
      screenBuffer[0] = SYM_GMISSION;
      screenBuffer[1] = SYM_GMISSION1;
    }
    else if(MwSensorActive&mode.gpsland){
      screenBuffer[2]=0;
      screenBuffer[0] = SYM_GLAND;
      screenBuffer[1] = SYM_GLAND1;
    }
    else if(MwSensorActive&mode.stable){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_STABLE;
      screenBuffer[1]=SYM_STABLE1;
    }
    else if(MwSensorActive&mode.horizon){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_HORIZON;
      screenBuffer[1]=SYM_HORIZON1;
    }
   else{
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_ACRO;
      screenBuffer[1]=SYM_ACRO1;
    }
#ifdef APINDICATOR
      displayAPstatus();
#endif
    if(Settings[S_MODEICON]){
//    if (!screenPosition[ModePosition]<512)
    if(fieldIsVisible(ModePosition))
      MAX7456_WriteString(screenBuffer,getPosition(ModePosition));
    }
    if((MwSensorActive&mode.camstab)&&Settings[S_GIMBAL]){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_GIMBAL;
      screenBuffer[1]=SYM_GIMBAL1;  
//    if (!screenPosition[gimbalPosition]<512)
    if(fieldIsVisible(gimbalPosition))
      MAX7456_WriteString(screenBuffer,getPosition(gimbalPosition));
    }

  if(Settings[S_MODESENSOR]){
    xx = 0;
    if(MwSensorActive&mode.passthru){
      screenBuffer[xx] = SYM_MAN;  xx++;
      screenBuffer[xx] = SYM_MAN1;  xx++;
      screenBuffer[xx] = SYM_MAN2;  xx++;
    }
  else {
    if(MwSensorActive&mode.stable||MwSensorActive&mode.horizon){
      screenBuffer[xx] = SYM_ACC;
      xx++;
    }
    if (MwSensorActive&mode.baro){
      screenBuffer[xx] = SYM_BAR;
      xx++;
    }
    if (MwSensorActive&mode.mag){
      screenBuffer[xx] = SYM_MAG;
      xx++;
    }
  }
  screenBuffer[xx] = 0;
//  if (!screenPosition[sensorPosition]<512)
  if(fieldIsVisible(sensorPosition))
    MAX7456_WriteString(screenBuffer,getPosition(sensorPosition));
  }
}

void displayArmed(void)
{
#ifndef HIDEARMEDSTATUS
  if(!fieldIsVisible(motorArmedPosition))
    return;
  if(!armed){
    MAX7456_WriteString_P(disarmed_text, getPosition(motorArmedPosition));
    armedtimer=20;
  }
  else if(timer.Blink10hz&&armedtimer){
    armedtimer--;
    MAX7456_WriteString_P(armed_text, getPosition(motorArmedPosition));
  }
#endif  
}

void displayCallsign(int cposition)
{
//  uint16_t  = getPosition(callSignPosition);
      for(uint8_t X=0; X<10; X++) {
          screenBuffer[X] = char(Settings[S_CS0 + X]);
     }   
       screenBuffer[10] = 0;
       MAX7456_WriteString(screenBuffer, cposition); 
}

//************************************************************************************************************************************************************************************
void displayHorizon(int rollAngle, int pitchAngle)
{

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

#endif

  if (!Settings[S_SCROLLING]){
     SYM_AH_DECORATION_LEFT=0x13;
     SYM_AH_DECORATION_RIGHT=0x13;
  } 
  
  uint16_t position = getPosition(horizonPosition)-(2*LINE);

  if(pitchAngle>200) pitchAngle=200;
  if(pitchAngle<-250) pitchAngle=-250;
  if(rollAngle>400) rollAngle=400;
  if(rollAngle<-400) rollAngle=-400;
  pitchAngle=pitchAngle+10;

  if(Settings[S_DISPLAY_HORIZON_BR]&fieldIsVisible(horizonPosition)){
#ifdef FULLAHI
    for(uint8_t X=0; X<=10; X++) {
      if (X==4) X=7;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position + LINE*(Y/9) + 3 - 2*LINE + X;
        screen[pos] = SYM_AH_BAR9_0+(Y%9);
      }
    }
#else
    for(uint8_t X=0; X<=8; X++) {
      if (X==3) X=6;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position + LINE*(Y/9) + 3 - 2*LINE + X;
        screen[pos] = SYM_AH_BAR9_0+(Y%9);
      }
    }
#endif

    if (Settings[S_HORIZON_ELEVATION]){                   
      for(int X=2; X<=6; X++) { 
        if (X==4) X=5;
        int Y = (rollAngle * (4-X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if(Y >= 0 && Y <= 81) {
          uint16_t pos = position + LINE*(Y/9) + 3 - 2*LINE + X;
        pos = pos - 3*LINE;
        if(pos >= 60 && pos <= 360) 
          screen[pos] = SYM_AH_BAR9_0+(Y%9);
        pos = pos + 2*3*LINE;
        if(pos >= 60 && pos <= 330) 
          screen[pos] = SYM_AH_BAR9_0+(Y%9);
        }
      }
    }
    if(!fieldIsVisible(MapModePosition)){
      if(Settings[S_DISPLAY_HORIZON_BR]){
        screen[position+2*LINE+7-1] = SYM_AH_CENTER_LINE;
        screen[position+2*LINE+7+1] = SYM_AH_CENTER_LINE_RIGHT;
        screen[position+2*LINE+7] =   SYM_AH_CENTER;
      }
    }
  }

  if (Settings[S_WITHDECORATION]&fieldIsVisible(SideBarPosition)){
    // Draw AH sides
    screen[position+2*LINE+1] =   SYM_AH_LEFT;
    screen[position+2*LINE+13] =  SYM_AH_RIGHT;
    for(int X=0; X<=4; X++) {
      screen[position+X*LINE] =     SYM_AH_DECORATION_LEFT;
      screen[position+X*LINE+14] =  SYM_AH_DECORATION_RIGHT;
    }

#ifdef SBDIRECTION

    if (Settings[S_SIDEBARTOPS]&fieldIsVisible(SideBarScrollPosition)) {
      if (millis()<(sidebarsMillis + 1000)) {
        if (sidebarsdir == 2){
          screen[position-LINE] = SYM_AH_DECORATION_UP;
        }
        else{
          screen[position+5*LINE] = SYM_AH_DECORATION_DOWN;
        }
      }
      if (millis()<(sidebaraMillis + 1000)) { 
        if (sidebaradir == 2){
          screen[position+14-LINE] = SYM_AH_DECORATION_UP;
        }
        else{
          screen[position+5*LINE+14] = SYM_AH_DECORATION_DOWN;
        }
      }
    }
#endif
  }

}


void displayVoltage(void)
{
 
 
  if (Settings[S_VIDVOLTAGE_VBAT]){
    vidvoltage=MwVBat;
  }
  if (Settings[S_MAINVOLTAGE_VBAT]){
    voltage=MwVBat;
  }

  if (Settings[S_SHOWBATLEVELEVOLUTION]){

    int battev=voltage/Settings[S_BATCELLS];
    battev=constrain(battev,34,42);
    battev = map(battev, 34, 42, 0, 6);
    screenBuffer[0]=SYM_BATT_EMPTY-battev;
  }
  else {
    screenBuffer[0]=SYM_MAIN_BATT;
  }

  if(fieldIsVisible(voltagePosition)) {
    ItoaPadded(voltage, screenBuffer+1, 4, 3);
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;
    MAX7456_WriteString(screenBuffer,getPosition(voltagePosition)-1);
  }
  if(!fieldIsVisible(vidvoltagePosition))
    return;

  if (Settings[S_VIDVOLTAGE]){
    screenBuffer[0]=SYM_VID_BAT;
    ItoaPadded(vidvoltage, screenBuffer+1, 4, 3);
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;    
    MAX7456_WriteString(screenBuffer,getPosition(vidvoltagePosition)-1);
  }
}

void displayCurrentThrottle(void)
{
  if(!fieldIsVisible(CurrentThrottlePosition))
    return;

  if (MwRcData[THROTTLESTICK] > HighT) HighT = MwRcData[THROTTLESTICK] -5;
  if (MwRcData[THROTTLESTICK] < LowT) LowT = MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
  if(!armed) {
    screenBuffer[2]=' ';
    screenBuffer[3]=' ';
    screenBuffer[4]='-';
    screenBuffer[5]='-';
  }
  else
  {
    int CurThrottle = map(MwRcData[THROTTLESTICK],LowT,HighT,0,100);
    ItoaPadded(CurThrottle,screenBuffer+2,3,0);
    screenBuffer[5]='%';
  }
    screenBuffer[0]=SYM_THR;
    screenBuffer[1]=' ';
    screenBuffer[6]=0;
    MAX7456_WriteString(screenBuffer,getPosition(CurrentThrottlePosition));
}


void displayTime(void) 
{ 
  if(!Settings[S_TIMER])
    return;
  if (screenPosition[onTimePosition]<512)
    return;

  uint32_t displaytime;
  if (armed) { 
    if(flyTime < 3600) {
      screenBuffer[0] = SYM_FLY_M;
      displaytime = flyTime;
    }
    else {
      screenBuffer[0] = SYM_FLY_H;
      displaytime = flyTime/60;
    }
  }
  else {
    if(onTime < 3600) {
      screenBuffer[0] = SYM_ON_M;
      displaytime = onTime;
    }
    else {
      screenBuffer[0] = SYM_ON_H;
      displaytime = onTime/60;
    }
  }
  formatTime(displaytime, screenBuffer+1, 0);
  MAX7456_WriteString(screenBuffer,getPosition(onTimePosition));
}


void displayAmperage(void)
{
  if(!fieldIsVisible(amperagePosition))
    return;
  ItoaPadded(amperage, screenBuffer, 5, 4);     // 999.9 ampere max!
  screenBuffer[5] = SYM_AMP;
  screenBuffer[6] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(amperagePosition));
}


void displaypMeterSum(void)
{
  if(!fieldIsVisible(pMeterSumPosition))
    return;
  screenBuffer[0]=SYM_MAH;
  int xx=amperagesum/36;
  itoa(xx,screenBuffer+1,10);
  MAX7456_WriteString(screenBuffer,getPosition(pMeterSumPosition));
}


void displayI2CError(void)
{
#ifdef I2CERROR
  if (I2CError<=I2CERROR)
    return;
  screenBuffer[0] = 0x49;
  screenBuffer[1] = 0X3A;
  itoa(I2CError,screenBuffer+2,7);
  MAX7456_WriteString(screenBuffer,getPosition(temperaturePosition));
#endif
}


void displayRSSI(void)
{
  if(!fieldIsVisible(rssiPosition))
    return;
  screenBuffer[0] = SYM_RSSI;
  itoa(rssi,screenBuffer+1,10);
  uint8_t xx = FindNull();
  screenBuffer[xx++] = '%';
  screenBuffer[xx] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(rssiPosition)-1);
}


void displayAPstatus()
{
  if(timer.Blink2hz)
    return;
  if(!fieldIsVisible(APstatusPosition))
    return;
  if (MwSensorActive&mode.gpshome)
    MAX7456_WriteString_P(APRTHtext,getPosition(APstatusPosition));
  else if (MwSensorActive&mode.gpshold)
    MAX7456_WriteString_P(APHOLDtext,getPosition(APstatusPosition));
  else if (MwSensorActive&mode.gpsmission)
    MAX7456_WriteString_P(APWAYPOINTtext,getPosition(APstatusPosition));
//  else if (MwSensorActive&mode.gpsland)
//    MAX7456_WriteString_P(APLANDtext,getPosition(APstatusPosition));
 

}

void displayHeading(void)
{
 if(!fieldIsVisible(MwHeadingPosition))
    return;
 if (Settings[S_SHOWHEADING]) {  
   int16_t heading = MwHeading;
   if (Settings[S_HEADING360]) {
     if(heading < 0)
       heading += 360;
       ItoaPadded(heading,screenBuffer,3,0);
       screenBuffer[3]=SYM_DEGREES;
       screenBuffer[4]=0;
    }
    else {
      ItoaPadded(heading,screenBuffer,4,0);
      screenBuffer[4]=SYM_DEGREES;
      screenBuffer[5]=0;
    }
    MAX7456_WriteString(screenBuffer,getPosition(MwHeadingPosition));
  }  
}


void displayHeadingGraph(void)
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
  memcpy_P(screen+pos, headGraph+xx+1, 9);
}


void displayIntro(void)
{
  MAX7456_WriteString_P(message0, MWOSDVersionPosition);
  MAX7456_WriteString_P(message5, MWOSDVersionPosition+LINE+LINE);
  MAX7456_WriteString(ItoaPadded(MwVersion, screenBuffer, 4, 2),MWOSDVersionPosition+11+LINE+LINE+1);
#ifdef INTRO_CALLSIGN
  MAX7456_WriteString_P(message9, MWOSDVersionPosition+LINE+LINE+LINE);
  displayCallsign(MWOSDVersionPosition+LINE+LINE+LINE+4);
#endif   
#ifdef GPSTIME
#ifdef INTRO_TIMEZONE
//timezone
  MAX7456_WriteString_P(message10, MWOSDVersionPosition+LINE+LINE+LINE+LINE); 
  if(abs(Settings[S_GPSTZ]) >= 100)ItoaPadded(Settings[S_GPSTZ], screenBuffer, 5, 4);
  else ItoaPadded(Settings[S_GPSTZ], screenBuffer, 4, 3);
  if(Settings[S_GPSTZAHEAD] || Settings[S_GPSTZ] == 0)screenBuffer[0] = '+';
  else screenBuffer[0] = '-';   
  MAX7456_WriteString(screenBuffer, MWOSDVersionPosition+LINE+LINE+LINE+LINE+8); 
//more settings
  MAX7456_WriteString_P(message11, MWOSDVersionPosition+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE);
#endif
#endif
#ifdef INTRO_MENU
  MAX7456_WriteString_P(message6, MWOSDVersionPosition+LINE+LINE+LINE+LINE+LINE+LINE);
  MAX7456_WriteString_P(message7,  MWOSDVersionPosition+LINE+LINE+LINE+LINE+LINE+LINE+LINE+10);
  MAX7456_WriteString_P(message8,  MWOSDVersionPosition+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE+10); 
 #endif 
}


void displayGPSPosition(void)
{
  uint16_t position;
  if(!GPS_fix)
    return;
  if(!fieldIsVisible(MwGPSLatPosition))
    return;
  if (!MwSensorActive&mode.gpshome)
    return;
  if(Settings[S_COORDINATES]|MwSensorActive&mode.gpshome){
    if(fieldIsVisible(MwGPSLatPosition)|MwSensorActive&mode.gpshome) {
      if(!Settings[S_GPSCOORDTOP])
        position = getPosition(MwGPSLatPosition);
      else
        position = getPosition(MwGPSLatPositionTop);  
      screenBuffer[0] = SYM_LAT;
      FormatGPSCoord(GPS_latitude,screenBuffer+1,4,'N','S');
      MAX7456_WriteString(screenBuffer, position);  
      screenBuffer[0] = SYM_LON;
      FormatGPSCoord(GPS_longitude,screenBuffer+1,4,'E','W');
      MAX7456_WriteString(screenBuffer, position+13);  
    }
  }
  if(Settings[S_GPSALTITUDE]){
    if(!fieldIsVisible(MwGPSAltPosition))
    return;
    screenBuffer[0] = MwGPSAltPositionAdd[Settings[S_UNITSYSTEM]];
    uint16_t xx;
    if(Settings[S_UNITSYSTEM])
      xx = GPS_altitude * 3.2808; // Mt to Feet
    else
      xx = GPS_altitude;          // Mt
    itoa(xx,screenBuffer+1,10);
    MAX7456_WriteString(screenBuffer,getPosition(MwGPSAltPosition));
  }
}


void displayNumberOfSat(void)
{
  if(!GPS_fix)
    return;
  if((GPS_numSat<5)&&(timer.Blink2hz))
    return;
  if(!fieldIsVisible(GPS_numSatPosition))
    return;
  screenBuffer[0] = SYM_SAT_L;
  screenBuffer[1] = SYM_SAT_R;
  itoa(GPS_numSat,screenBuffer+2,10);
  MAX7456_WriteString(screenBuffer,getPosition(GPS_numSatPosition));
}


void displayGPS_speed(void)
{

  if(!GPS_fix) return;
  if(!armed) GPS_speed=0;
  int xx;
  if(!Settings[S_UNITSYSTEM])
    xx = GPS_speed * 0.036;           // From MWii cm/sec to Km/h
  else
    xx = GPS_speed * 0.02236932;      // (0.036*0.62137)  From MWii cm/sec to mph
  if(xx > speedMAX)
    speedMAX = xx;
  if(!fieldIsVisible(speedPosition))
    return;
  screenBuffer[0]=speedUnitAdd[Settings[S_UNITSYSTEM]];
  itoa(xx,screenBuffer+1,10);
  MAX7456_WriteString(screenBuffer,getPosition(speedPosition));
}


void displayGPS_time(void)       //local time of coord calc - haydent
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
  uint16_t milli = local % 1000;//get milli for later
  uint32_t seconds = (local / 1000) % 86400;//remove millisonds and whole days
  
  formatTime(seconds, screenBuffer, 1);
  if(screenBuffer[0] == ' ')screenBuffer[0] = '0';//put leading zero if empty space
/*
  screenBuffer[8] = '.';//add milli indicator
  screenBuffer[9] = '0' + (milli / 100);//only show first digit of milli due to limit of gps rate
  screenBuffer[10] = 0;//equivalent of new line or end of buffer
*/   
  screenBuffer[8] = 0;//equivalent of new line or end of buffer
  MAX7456_WriteString(screenBuffer,getPosition(GPS_timePosition));
}


void displayAltitude(void)
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
  screenBuffer[0]=MwAltitudeAdd[Settings[S_UNITSYSTEM]];
  itoa(altitude,screenBuffer+1,10);
  MAX7456_WriteString(screenBuffer,getPosition(MwAltitudePosition));
}


void displayClimbRate(void)
{
  if(!fieldIsVisible(MwClimbRatePosition))
    return;
  if(!Settings[S_VARIO])
    return;
    uint16_t position = getPosition(MwClimbRatePosition);
    for(int8_t X=-1; X<=1; X++) {
      screen[position+(X*LINE)] =  SYM_VARIO;
    }
   int8_t xx=MwVario;
   if (MwVario>120) xx=120;
   if (MwVario<-120) xx=-120;
   xx=map(xx,120,-120,0,17);
   int8_t varline=(xx/6)-1;
   int8_t varsymbol=xx%6;
   screen[position+(varline*LINE)] = 0x8F-varsymbol;
}


void displayDistanceToHome(void)
{
  if(!GPS_fix)
    return;

  int16_t dist;
  if(Settings[S_UNITSYSTEM])
    dist = GPS_distanceToHome * 3.2808;           // mt to feet
  else
    dist = GPS_distanceToHome;                    // Mt
  if(dist > distanceMAX)
    distanceMAX = dist;
  if(!fieldIsVisible(GPS_distanceToHomePosition))
    return;
  screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
  itoa(dist, screenBuffer+1, 10);
  MAX7456_WriteString(screenBuffer,getPosition(GPS_distanceToHomePosition));
}


void displayAngleToHome(void)
{
  if(!GPS_fix)
      return;
 if(!fieldIsVisible(GPS_angleToHomePosition))
    return;
  if(Settings[S_ANGLETOHOME]){
    if(GPS_distanceToHome <= 2 && timer.Blink2hz)
      return;
    ItoaPadded(GPS_directionToHome,screenBuffer,3,0);
    screenBuffer[3] = SYM_DEGREES;
    screenBuffer[4] = 0;
    MAX7456_WriteString(screenBuffer,getPosition(GPS_angleToHomePosition));
  }
}


void displayDirectionToHome(void)
{
  if(!GPS_fix)
    return;
  if (screenPosition[GPS_directionToHomePosition]<512)
    return;

  if(GPS_distanceToHome <= 2 && timer.Blink2hz)
    return;
  uint16_t position=getPosition(GPS_directionToHomePosition);
  int16_t d = MwHeading + 180 + 360 - GPS_directionToHome;
  d *= 4;
  d += 45;
  d = (d/90)%16;
  screenBuffer[0] = SYM_ARROW_SOUTH + d;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer,position);
}


void displayCursor(void)
{
  int cursorpos;

  if(ROW==10){
    if(COL==3) cursorpos=SAVEP+16-1;    // page
    if(COL==1) cursorpos=SAVEP-1;       // exit
    if(COL==2) cursorpos=SAVEP+6-1;     // save/exit
  }
  if(ROW<10)
    {
#ifdef PAGE0      
    if(configPage==0)
      {
      ROW=10;
      }
#endif      
#ifdef PAGE1
    if(configPage==1){
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      if(COL==1) cursorpos=(ROW+2)*30+10;
      if(COL==2) cursorpos=(ROW+2)*30+10+6;
      if(COL==3) cursorpos=(ROW+2)*30+10+6+6;
      if(ROW==7) {cursorpos=(ROW+2)*30+10;COL=1;}
     }
#endif
#ifdef PAGE2
    if(configPage==2){
      COL=3;
      if (ROW==6) ROW=10;
      if (ROW==9) ROW=5;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef PAGE3      
    if(configPage==3){
      COL=3;
      if (ROW==8) ROW=10;
      if (ROW==9) ROW=7;
      cursorpos=(ROW+2)*30+10+6+6;     
      }
#endif
#ifdef PAGE4      
    if(configPage==4){
      COL=3;
      if (ROW==7) ROW=10;
      if (ROW==9) ROW=6;
      cursorpos=(ROW+2)*30+10+6+6;
      }    
#endif
#ifdef PAGE5      
    if(configPage==5)
      {  
      COL=3;
      if (ROW==9) ROW=5;
      if (ROW==6) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef PAGE6      
    if(configPage==6)
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
#ifdef PAGE7      
    if(configPage==7)
      {  
      COL=3;
      if (ROW==9) ROW=5;
      if (ROW==6) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif
#ifdef PAGE8      
    if(configPage==8)
      {  
      COL=3;
      if (ROW==9) ROW=3;
      if (ROW==4) ROW=10;
       cursorpos=(ROW+2)*30+10+6+6;
      }
#endif     
  }
  if(timer.Blink10hz)
    screen[cursorpos] = SYM_CURSOR;
}


void displayConfigScreen(void)
{
  MAX7456_WriteString_P(configMsgEXT, SAVEP);    //EXIT
  if(!previousarmedstatus) {
    MAX7456_WriteString_P(configMsgSAVE, SAVEP+6);  //SaveExit
    MAX7456_WriteString_P(configMsgPGS, SAVEP+16); //<Page>
  }

  if(configPage==0)
  {
    int xx;
    MAX7456_WriteString_P(configMsg00, 35);

    MAX7456_WriteString_P(configMsg01, ROLLT);
    MAX7456_WriteString(itoa(trip,screenBuffer,10),ROLLD-3);

    MAX7456_WriteString_P(configMsg02, PITCHT);
    MAX7456_WriteString(itoa(distanceMAX,screenBuffer,10),PITCHD-3);

    MAX7456_WriteString_P(configMsg03, YAWT);
    MAX7456_WriteString(itoa(altitudeMAX,screenBuffer,10),YAWD-3);

    MAX7456_WriteString_P(configMsg04, ALTT);
    MAX7456_WriteString(itoa(speedMAX,screenBuffer,10),ALTD-3);

    xx=amperagesum/36;
    itoa(xx,screenBuffer,10);
    MAX7456_WriteString_P(configMsg06, VELT);
    MAX7456_WriteString(itoa(xx,screenBuffer,10),VELD-3);

    MAX7456_WriteString_P(configMsg05, LEVT);

    formatTime(flyingTime, screenBuffer, 1);
    MAX7456_WriteString(screenBuffer,LEVD-4);
    }
#ifdef PAGE1
  if(configPage==1)
  {
    MAX7456_WriteString_P(configMsg10, 35);
    MAX7456_WriteString_P(configMsg11, ROLLT);
    MAX7456_WriteString(itoa(P8[0],screenBuffer,10),ROLLP);
    MAX7456_WriteString(itoa(I8[0],screenBuffer,10),ROLLI);
    MAX7456_WriteString(itoa(D8[0],screenBuffer,10),ROLLD);

    MAX7456_WriteString_P(configMsg12, PITCHT);
    MAX7456_WriteString(itoa(P8[1],screenBuffer,10), PITCHP);
    MAX7456_WriteString(itoa(I8[1],screenBuffer,10), PITCHI);
    MAX7456_WriteString(itoa(D8[1],screenBuffer,10), PITCHD);

    MAX7456_WriteString_P(configMsg13, YAWT);
    MAX7456_WriteString(itoa(P8[2],screenBuffer,10),YAWP);
    MAX7456_WriteString(itoa(I8[2],screenBuffer,10),YAWI);
    MAX7456_WriteString(itoa(D8[2],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg14, ALTT);
    MAX7456_WriteString(itoa(P8[3],screenBuffer,10),ALTP);
    MAX7456_WriteString(itoa(I8[3],screenBuffer,10),ALTI);
    MAX7456_WriteString(itoa(D8[3],screenBuffer,10),ALTD);

    MAX7456_WriteString_P(configMsg15, VELT);
    MAX7456_WriteString(itoa(P8[4],screenBuffer,10),VELP);
    MAX7456_WriteString(itoa(I8[4],screenBuffer,10),VELI);
    MAX7456_WriteString(itoa(D8[4],screenBuffer,10),VELD);

    MAX7456_WriteString_P(configMsg16, LEVT);
    MAX7456_WriteString(itoa(P8[7],screenBuffer,10),LEVP);
    MAX7456_WriteString(itoa(I8[7],screenBuffer,10),LEVI);
    MAX7456_WriteString(itoa(D8[7],screenBuffer,10),LEVD);

    MAX7456_WriteString_P(configMsg17, MAGT);
    MAX7456_WriteString(itoa(P8[8],screenBuffer,10),MAGP);

    MAX7456_WriteString("P",71);
    MAX7456_WriteString("I",77);
    MAX7456_WriteString("D",83);
  }
#else
    if(configPage == 1)configPage+=menudir;
#endif
#ifdef PAGE2
  if(configPage==2)
  {
    MAX7456_WriteString_P(configMsg20, 35);
    MAX7456_WriteString_P(configMsg21, ROLLT);
    MAX7456_WriteString(itoa(rcRate8,screenBuffer,10),ROLLD);
    MAX7456_WriteString_P(configMsg22, PITCHT);
    MAX7456_WriteString(itoa(rcExpo8,screenBuffer,10),PITCHD);
    MAX7456_WriteString_P(configMsg23, YAWT);
    MAX7456_WriteString(itoa(rollPitchRate,screenBuffer,10),YAWD);
    MAX7456_WriteString_P(configMsg24, ALTT);
    MAX7456_WriteString(itoa(yawRate,screenBuffer,10),ALTD);
    MAX7456_WriteString_P(configMsg25, VELT);
    MAX7456_WriteString(itoa(dynThrPID,screenBuffer,10),VELD);

  }
 #else
    if(configPage == 2)configPage+=menudir; 
#endif
#ifdef PAGE3
  if(configPage==3)
  {
    ProcessSensors();
    MAX7456_WriteString_P(configMsg30, 35);

    screenBuffer[0]=SYM_MAIN_BATT;
    ItoaPadded(voltage, screenBuffer+1, 4, 3);
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;
    MAX7456_WriteString(screenBuffer,ROLLD-LINE-LINE-1);

    screenBuffer[0]=SYM_VID_BAT;
    ItoaPadded(vidvoltage, screenBuffer+1, 4, 3);
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;
    MAX7456_WriteString(screenBuffer,ROLLI-LINE-LINE-3);
 //R1:    
    MAX7456_WriteString_P(configMsg31, ROLLT);
    if(Settings[S_DISPLAYVOLTAGE]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else {
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }
//R2:     
    MAX7456_WriteString_P(configMsg32, PITCHT);
    MAX7456_WriteString(itoa(Settings[S_DIVIDERRATIO],screenBuffer,10),PITCHD);
//R3:
    MAX7456_WriteString_P(configMsg33, YAWT);
    MAX7456_WriteString(itoa(Settings[S_VOLTAGEMIN],screenBuffer,10),YAWD);
//R4:     
    MAX7456_WriteString_P(configMsg34, ALTT);
    if(Settings[S_VIDVOLTAGE]){
      MAX7456_WriteString_P(configMsgON, ALTD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ALTD);
    }
//R5:
    MAX7456_WriteString_P(configMsg35, VELT);   
    MAX7456_WriteString(itoa(Settings[S_BATCELLS],screenBuffer,10),VELD);
//R6:
    MAX7456_WriteString_P(configMsgMWII, LEVT);   
    if(Settings[S_MAINVOLTAGE_VBAT]){
      MAX7456_WriteString_P(configMsgON, LEVD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, LEVD);
  }
//R7:     
    MAX7456_WriteString_P(configMsg32, MAGT);
    MAX7456_WriteString(itoa(Settings[S_VIDDIVIDERRATIO],screenBuffer,10),MAGD);
  }
#else
    if(configPage == 3)configPage+=menudir;  
#endif
#ifdef PAGE4
  if(configPage==4)
  {
    MAX7456_WriteString_P(configMsg40, 35);
    itoa(rssi,screenBuffer,10);
    uint8_t xx = FindNull();
    screenBuffer[xx++] = '%';
    screenBuffer[xx] = 0;
    MAX7456_WriteString(screenBuffer,ROLLD-LINE-LINE);
//    MAX7456_WriteString(itoa(rssi,screenBuffer,10),ROLLD-LINE-LINE);

//R1:
    MAX7456_WriteString_P(configMsg42, ROLLT);
    if(Settings[S_DISPLAYRSSI]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }
//R2:
    MAX7456_WriteString_P(configMsg43, PITCHT);
    if(timer.rssiTimer>0) {
      MAX7456_WriteString(itoa(timer.rssiTimer,screenBuffer,10),PITCHD);
    }
    else {
    MAX7456_WriteString("-", PITCHD);
    }
//R3:
    MAX7456_WriteString_P(configMsgMWII, YAWT);
    if(Settings[S_MWRSSI]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);
  }
//R4:
    MAX7456_WriteString_P(configMsg46, ALTT);
    if(Settings[S_PWMRSSI]){
      MAX7456_WriteString_P(configMsgON, ALTD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ALTD);
  }
//R5:
    MAX7456_WriteString_P(configMsg44, VELT);   
    MAX7456_WriteString(itoa(Settings[S_RSSIMAX],screenBuffer,10),VELD);
//R6:
    MAX7456_WriteString_P(configMsg45, LEVT);   
    MAX7456_WriteString(itoa(Settings[S_RSSIMIN],screenBuffer,10),LEVD);

  }
#else
    if(configPage == 4)configPage+=menudir;  
#endif
#ifdef PAGE5
  if(configPage==5)
  {
    MAX7456_WriteString_P(configMsg50, 35);
    ItoaPadded(amperage, screenBuffer, 4, 3);     // 99.9 ampere max!
    screenBuffer[4] = SYM_AMP;
    screenBuffer[5] = 0;
    MAX7456_WriteString(screenBuffer,ROLLD-LINE-LINE-1);
//    MAX7456_WriteString(itoa(amperage,screenBuffer,10),ROLLD-LINE-LINE);
//R1:
    MAX7456_WriteString_P(configMsg51, ROLLT);
    if(Settings[S_AMPERAGE]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }
//R2:
    MAX7456_WriteString_P(configMsg52, PITCHT);
    if(Settings[S_AMPER_HOUR]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);
    }
//R3:
    MAX7456_WriteString_P(configMsg53, YAWT);
    if(Settings[S_AMPERAGE_VIRTUAL]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);
    }
//R4:
    MAX7456_WriteString_P(configMsg54, ALTT);   
    MAX7456_WriteString(itoa(S16_AMPMAX,screenBuffer,10),ALTD);
//    MAX7456_WriteString(itoa(amperage,screenBuffer,10),ROLLD-LINE-LINE);
//R5:
    MAX7456_WriteString_P(configMsg55, VELT);   
    MAX7456_WriteString(itoa(Settings[S_AMPMIN],screenBuffer,10),VELD);
//    MAX7456_WriteString(itoa(amperage,screenBuffer,10),ROLLD-LINE-LINE);
  }
#else
    if(configPage == 5)configPage+=menudir;  
#endif
#ifdef PAGE6
  if(configPage==6)
  {
    MAX7456_WriteString_P(configMsg60, 35);
    
//R1:     
    MAX7456_WriteString_P(configMsg61, ROLLT);
    if(Settings[S_DISPLAY_HORIZON_BR]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }
//R2:     
    MAX7456_WriteString_P(configMsg62, PITCHT);
    if(Settings[S_WITHDECORATION]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);
    }
//R3:
    MAX7456_WriteString_P(configMsg63, YAWT);
    if(Settings[S_SCROLLING]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);
    }
//R4:     
    MAX7456_WriteString_P(configMsg64, ALTT);
    if(Settings[S_THROTTLEPOSITION]){
      MAX7456_WriteString_P(configMsgON, ALTD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ALTD);
    }
//R5:     
    MAX7456_WriteString_P(configMsg65, VELT);
    if(Settings[S_COORDINATES]){
      MAX7456_WriteString_P(configMsgON, VELD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, VELD);
    }
//R6:     
    MAX7456_WriteString_P(configMsg66, LEVT);
    if(Settings[S_MODESENSOR]){
      MAX7456_WriteString_P(configMsgON, LEVD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, LEVD);
    }
//R7:     
    MAX7456_WriteString_P(configMsg67, MAGT);
    MAX7456_WriteString(itoa(Settings[S_GIMBAL],screenBuffer,10),MAGD);
    if(Settings[S_GIMBAL]){
      MAX7456_WriteString_P(configMsgON, MAGD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, MAGD);
    }
//R8:     
    MAX7456_WriteString_P(configMsg69, MAGT+LINE);
    MAX7456_WriteString(itoa(Settings[S_MAPMODE],screenBuffer,10),MAGD+LINE);
    if(Settings[S_MAPMODE]){
      MAX7456_WriteString_P(configMsgON, MAGD+LINE);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, MAGD+LINE);
    }   
  }
#else
    if(configPage == 6)configPage+=menudir;  
#endif
#ifdef PAGE7
  if(configPage==7)
  {
    MAX7456_WriteString_P(configMsg70, 35);
//R1:
    MAX7456_WriteString_P(configMsg71, ROLLT);
    if(!Settings[S_UNITSYSTEM]){
      MAX7456_WriteString_P(configMsg710, ROLLD);
    }
    else {
      MAX7456_WriteString_P(configMsg711, ROLLD);
      }
//R2:
    MAX7456_WriteString_P(configMsg72, PITCHT);
    if(!Settings[S_VIDEOSIGNALTYPE]){
      MAX7456_WriteString_P(configMsg720, PITCHD);
    }
    else {
      MAX7456_WriteString_P(configMsg721, PITCHD);
      }
//R3:
    MAX7456_WriteString_P(configMsg73, YAWT);
    if(Settings[S_VREFERENCE]){
      MAX7456_WriteString_P(configMsg730, YAWD);
    }
    else {
      MAX7456_WriteString_P(configMsg731, YAWD);
      }
//R5:
    MAX7456_WriteString_P(configMsg74, ALTT);
    if(Settings[S_DEBUG]){
      MAX7456_WriteString_P(configMsgON, ALTD);
    }
    else {
      MAX7456_WriteString_P(configMsgOFF, ALTD);
      }
//R6:
    MAX7456_WriteString_P(configMsg75, VELT);
    if(timer.magCalibrationTimer>0)
      MAX7456_WriteString(itoa(timer.magCalibrationTimer,screenBuffer,10),VELD);
    else
      MAX7456_WriteString("-",VELD);
   }
#else
    if(configPage == 7)configPage+=menudir;
#endif
#ifdef PAGE8
  if(configPage==8)
  {
    MAX7456_WriteString_P(configMsg80, 35);
    
//R1:     
    MAX7456_WriteString_P(configMsg81, ROLLT);
    if(Settings[S_GPSTIME]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }
//R2:     
    MAX7456_WriteString_P(configMsg82, PITCHT);
    if(Settings[S_GPSTZAHEAD]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);
    }
//R3:
    MAX7456_WriteString_P(configMsg83, YAWT);
    MAX7456_WriteString(itoa(Settings[S_GPSTZ],screenBuffer,10),YAWD);
  }    
#else
    if(configPage == 8)configPage+=menudir;
    if(configPage > MAXPAGE)configPage=MINPAGE;
#endif  
  displayCursor();
}

void mapmode(void) {

#ifdef MAPMODE

  if(!GPS_fix)
    return;
  if(!fieldIsVisible(MapModePosition))
    return;
//  if(!Settings[S_MAPMODE]) 
//    return;
    
  int8_t xdir;
  int8_t ydir;
  int16_t targetx;
  int16_t targety;
  int16_t range=200;
  int16_t angle;
  int16_t targetpos;
  int16_t centerpos;
  uint16_t maxdistance;
  uint8_t mapsymbolcenter;
  uint8_t mapsymboltarget;
  uint8_t mapsymbolrange;
  int16_t tmp;
  if (MAPTYPE==1) {
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


  if (MAPTYPE==1) {
    mapsymbolcenter = SYM_HOME;
    mapsymboltarget = SYM_AIRCRAFT;
  }
  else {
    mapsymbolcenter = SYM_AIRCRAFT;
    mapsymboltarget = SYM_HOME;
  }

  screenBuffer[0] = mapsymbolrange;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(MapModePosition));

#ifdef MAPRESLOW
  if (MAPTYPE==1) {
    tmp=(360+382+MwHeading-armedangle)%360/45;
    mapsymboltarget = SYM_DIRECTION + tmp;
  }
#else
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
    mapsymboltarget = 0xD0 + symy + (symx*3);
#endif

  screenBuffer[0] = mapsymboltarget;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer,targetpos);

  screenBuffer[0] = mapsymbolcenter;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer,centerpos);
 
#endif
}

void displayDebug(void)
{
#if defined (DEBUG)||defined (DEBUGMW)
  if(!Settings[S_DEBUG])
    return;
 
//  check_mem();
//  debug[0]=(int16_t)heapptr;
//  debug[1]=(int16_t)stackptr;

//  debug[0]=I2CError;
//  debug[1]=MWOSDVER;
//  debug[2]=MwVersion;
//  debug[3]=MWOSDVER;

  for(uint8_t X=0; X<4; X++) {
    ItoaPadded(debug[X], screenBuffer+2,7,0);     
    screenBuffer[0] = 0x30+X;
    screenBuffer[1] = 0X3A;
    MAX7456_WriteString(screenBuffer,getPosition(debugPosition)+(X*LINE));
  } 
 
#endif
}

void displayCells(void){

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
      screenBuffer[i]=SYM_CELL0+tempvolt;
      }
      else screenBuffer[i]=' ';      
    }
    
    if(cells){
      screenBuffer[cells] = 0;       
      MAX7456_WriteString(screenBuffer,getPosition(SportPosition)+(6-cells));//bar chart

      ItoaPadded(low, screenBuffer+1,4,2);
      screenBuffer[0] = SYM_MIN;
      screenBuffer[5] = SYM_VOLT;
      screenBuffer[6] = 0;
    
    if((low>MIN_CELL)||(timer.Blink2hz))
      MAX7456_WriteString(screenBuffer,getPosition(SportPosition)+LINE);//lowest
    
      uint16_t avg = 0;
      if(cells)avg = sum / cells;
      ItoaPadded( avg, screenBuffer+1,4,2);
      screenBuffer[0] = SYM_AVG;
      screenBuffer[5] = SYM_VOLT;
      screenBuffer[6] = 0;
    
    if((avg>MIN_CELL)||(timer.Blink2hz))
      MAX7456_WriteString(screenBuffer,getPosition(SportPosition)+(2*LINE));//average     
  }
}
