

char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
  // Val to convert
  // Return String
  // Length
  // Decimal position
  uint8_t neg = 0;
  if (val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for (;;) {
    if (bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if (bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if (neg && bytes > 0)
    str[--bytes] = '-';

  while (bytes != 0)
    str[--bytes] = ' ';
  return str;
}


uint8_t FindNull(void)
{
  uint8_t xx;
  for (xx = 0; screenBuffer[xx] != 0; xx++)
    ;
  return xx;
}


uint16_t getPosition(uint8_t pos) {
  uint16_t val = screenPosition[pos];
  uint16_t ret = val & POS_MASK;
  return ret;
}

uint8_t fieldIsVisible(uint8_t pos) {
  //  uint16_t val = (uint16_t)pgm_read_word(&screenPosition[pos]);
  uint16_t val = screenPosition[pos];
  if ((val & DISPLAY_MASK) == DISPLAY_ALWAYS)
    return 1;
  else
    return 0;
}

void FormatGPSCoord(uint16_t t_position, int32_t val, uint8_t t_cardinalaxis) {  // lat = 0 or lon = 2

  uint8_t t_leadicon = SYM_LAT;
  if (t_cardinalaxis>0) 
    t_leadicon++;  
  uint8_t t_cardinal = 0;
  if (val < 0) {
      t_cardinal ++;
    val = -val;
  }
  t_cardinal+=t_cardinalaxis;

  uint8_t bytes = 11;
  val = val / 100;

  screenBuffer[bytes] = 0;
  screenBuffer [--bytes] = compass[t_cardinal];
  for (;;) {
    if (bytes == 5) {
      screenBuffer [--bytes] = DECIMAL;
      continue;
    }
    screenBuffer [--bytes] = '0' + (val % 10);
    val = val / 10;
    if (bytes == 0 || (bytes < 4 && val == 0))
      break;
  }

  while (bytes != 0)
    screenBuffer [--bytes] = ' ';
  screenBuffer[0] = t_leadicon;
  if (Settings[S_GPS_MASK]) {
    screenBuffer[1] = '0';
    screenBuffer[2] = '6';
    screenBuffer[3] = '3';
  }

   MAX7456_WriteString(screenBuffer, t_position);
}

#ifdef VIRTUAL_NOSE
void displayVirtualNose(void)
{
#define HTCENTER 14
  screenBuffer[0] = 0xC9;
  screenBuffer[1] = 0;
  uint16_t htpos = map(MwRcData[HTCHANNEL], 1000, 2000, -HTSCALE, HTSCALE);
  MAX7456_WriteString(screenBuffer, (30 * HTLINE) + HTCENTER HTDIRECTION htpos);
}
#endif


void displayTemperature(void)
{
#ifdef SUBMERSIBLE
  if ((((temperature / 10) > Settings[TEMPERATUREMAX]) && (timer.Blink2hz)))
    return;
#else
  if (((temperature > Settings[TEMPERATUREMAX]) && (timer.Blink2hz)))
    return;
#endif
  int xxx;
  if (Settings[S_UNITSYSTEM])
    xxx = temperature * 1.8 + 32;   //Fahrenheit conversion for imperial system.
  else
    xxx = temperature;
#ifdef SUBMERSIBLE
  displayItem(temperaturePosition, xxx, SYM_TMP, temperatureUnitAdd[Settings[S_UNITSYSTEM]], 1 ); // *10 value, 1 DP
#else
  displayItem(temperaturePosition, xxx, SYM_TMP, temperatureUnitAdd[Settings[S_UNITSYSTEM]], 0 );
#endif
}


void displayMode(void)
{
#ifdef NAZAMODECONTROL
  if (timer.MSP_active == 0) { // no MSP >> mode display not valid
    return;
  }
#elif defined NAZA
  return;
#else
  if (timer.MSP_active == 0) { // no MSP >> mode display not valid
    return;
  }
#endif

  if ((MwSensorActive & mode.camstab)) {
    screenBuffer[2] = 0;
    screenBuffer[0] = SYM_GIMBAL;
    screenBuffer[1] = SYM_GIMBAL1;
    if (fieldIsVisible(gimbalPosition))
      MAX7456_WriteString(screenBuffer, getPosition(gimbalPosition));
  }

  uint8_t xx = 0;
  if (MwSensorActive & mode.stable || MwSensorActive & mode.horizon) {
    screenBuffer[xx] = SYM_ACC;
    xx++;
  }
  if (MwSensorActive & mode.baro) {
    screenBuffer[xx] = SYM_BAR;
    xx++;
  }
  if (MwSensorActive & mode.mag) {
    screenBuffer[xx] = SYM_MAG;
    xx++;
  }
  screenBuffer[xx] = 0;
  if (fieldIsVisible(sensorPosition)) {
    MAX7456_WriteString(screenBuffer, getPosition(sensorPosition));
  }
#ifdef PROTOCOL_MAVLINK // override MWOSD mode icons
  strcpy_P(screenBuffer, (char*)pgm_read_word(&(mav_mode_index[mw_mav.mode])));
#elif defined PROTOCOL_LTM // override MWOSD mode icons
  strcpy_P(screenBuffer, (char*)pgm_read_word(&(ltm_mode_index[mw_ltm.mode])));
#elif defined PROTOCOL_KISS// override MWOSD mode icons
  strcpy_P(screenBuffer, (char*)pgm_read_word(&(KISS_mode_index[Kvar.mode])));
#elif defined (NAZAMODECONTROL) && defined (NAZA)// override MWOSD mode icons
  strcpy_P(screenBuffer, (char*)pgm_read_word(&(NAZA_mode_index[Naza.mode])));
#elif defined (PROTOCOL_MSP) // MWOSD mode icons
  uint8_t flightmode = 0;
  if (MwSensorActive & mode.passthru) {
    flightmode = 1;
  }
  else if (MwSensorActive & mode.failsafe) {
    flightmode = 2;
  }
  else if (MwSensorActive & mode.gpshome) {
    flightmode = 3;
  }
  else if (MwSensorActive & mode.gpsmission) {
    flightmode = 5;
  }
  else if (MwSensorActive & mode.gpshold) {
    flightmode = 6;
  }
  else if (MwSensorActive & mode.cruise){
    flightmode = 4;
  }
  else if (MwSensorActive & mode.stable) {
    flightmode = 7;
  }
  else if (MwSensorActive & mode.horizon) {
    flightmode = 8;
  }
#if defined AIRMODE
  else if (MwSensorActive & mode.air) {
    flightmode = 9;
  }
#endif //AIRMODE 
#if defined ACROPLUS
  else if ((MwSensorActive) & (mode.acroplus)) {
    flightmode = 9;
  }
#endif //ACROPLUS
  else {
    flightmode = 10;
  }
  uint8_t t_flightmodetext = 0;
  if (Settings[S_FLIGHTMODETEXT]) {
    t_flightmodetext = 11;
  }
  strcpy_P(screenBuffer, (char*)pgm_read_word(&(msp_mode_index[flightmode + t_flightmodetext])));
#endif  //PROTOCOL selection

  if (fieldIsVisible(ModePosition)) {
    MAX7456_WriteString(screenBuffer, getPosition(ModePosition));
  }

#ifdef APINDICATOR
  if (!fieldIsVisible(APstatusPosition))
    return;
  uint8_t apactive = 0;
#ifdef PROTOCOL_MAVLINK
#ifdef PX4 // within MAVLINK
#define MAVMISSIONID 5
#define MAVRTLID 6
#elif defined FIXEDWING // within MAVLINK
#undef MAVMISSIONID
#undef MAVRTLID
#define MAVMISSIONID 10
#define MAVRTLID 11
#else
#undef MAVMISSIONID
#undef MAVRTLID
#define MAVMISSIONID 3
#define MAVRTLID 6
#endif
  if (mw_mav.mode == MAVRTLID) { // RTL
    apactive = 2;
    MAX7456_WriteString_P(PGMSTR(&(message_text[apactive])), getPosition(APstatusPosition));
  }
  else if ((mw_mav.mode == MAVMISSIONID) & (GPS_waypoint_step > 0)) // Mission
    apactive = 4;
  if ((mw_mav.mode == MAVMISSIONID) & (GPS_waypoint_step > 0)) {
    screenBuffer[0] = 0x57;
    screenBuffer[1] = 0x50;
    screenBuffer[2] = SYM_COLON;
    itoa(GPS_waypoint_step, screenBuffer + 3, 10);
    MAX7456_WriteString(screenBuffer, getPosition(APstatusPosition));
    xx = FindNull() + 1;
    uint16_t dist = GPS_waypoint_dist;
    if (Settings[S_UNITSYSTEM])
      dist = GPS_waypoint_dist * 3.2808;           // mt to feet
    formatDistance(dist, 1, 2, 0);
    MAX7456_WriteString(screenBuffer, getPosition(APstatusPosition) + xx);
  }
#else  // NOT PROTOCOL_MAVLINK       
  if (timer.Blink2hz)
    return;
  if (MwSensorActive & mode.failsafe)
    apactive = 1;
  else if (MwSensorActive & mode.gpshome)
    apactive = 2;
  else if (MwSensorActive & mode.gpshold)
    apactive = 3;
  else if (MwSensorActive & mode.gpsmission)
    apactive = 4;
#ifdef EXTENDEDMODESUPPORT  
  else if (MwSensorActive & mode.launch)
    apactive = 5;
  else if (MwSensorActive & mode.autotrim)
    apactive = 6;
  else if (MwSensorActive & mode.autotune)
    apactive = 7;
#endif // EXTENDEDMODESUPPORT
  else
    return;
  MAX7456_WriteString_P(PGMSTR(&(message_text[apactive])), getPosition(APstatusPosition));
#endif // PROTOCOL_MAVLINK     
#endif
}


void displayCallsign(int cposition)
{
  for (uint8_t X = 0; X < 10; X++) {
    screenBuffer[X] = char(Settings[S_CS0 + X]);
  }
  screenBuffer[10] = 0;
  MAX7456_WriteString(screenBuffer, cposition);
}

void displayIcon(int cposition)
{
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t pos = cposition + (LINE * (i / 3)) + (i % 3);
    screen[pos] = PILOT_ICON + i;
  }
}

#ifdef MAV_STATUS
void displayMAVstatustext(void)
{
  uint16_t pos;
  if (timer.MAVstatustext == 0)
    return;
  pos = (30 * (getPosition(motorArmedPosition) / 30));
  for (uint8_t i = 0; i < 60; i++) {
    screen[pos+i] = SYM_BLANK;
  }
  for (uint8_t i = 1; i <= MAVstatuslength; i++) {
    pos = 2 + (30 * (getPosition(motorArmedPosition) / 30));
      if (MAVstatuslength<=25){ // single line
        pos +=  (25 - (MAVstatuslength)) /2;
      }
      else if (i<25) { // first row of multi line       
      }
      else{ // second row of multi line
        pos +=LINE;
        pos +=  (25 - (MAVstatuslength%25)) /2;
      }
      pos += i % 25;  
      screen[pos] = char(fontData[i]);
  }
}
#endif

void displayHorizon(int rollAngle, int pitchAngle)
{
#ifdef DISPLAY_PR
  screenBuffer[0] = SYM_PITCH;
  int16_t xx = abs(pitchAngle / 10);
  uint8_t offset = 1;
#ifdef INVERT_PITCH_SIGN
  if (pitchAngle > 0)
#else
  if (pitchAngle < 0)
#endif
  {
    screenBuffer[1] = '-';
    offset++;
  }
  itoa(xx, screenBuffer + offset, 10);
  if (fieldIsVisible(pitchAnglePosition))
    MAX7456_WriteString(screenBuffer, getPosition(pitchAnglePosition));
  screenBuffer[0] = SYM_ROLL;
  offset = 1;
  xx = abs(rollAngle / 10);
#ifdef INVERT_ROLL_SIGN
  if (rollAngle > 0)
#else
  if (rollAngle < 0)
#endif
  {
    screenBuffer[1] = '-';
    offset++;
  }
  itoa(xx, screenBuffer + offset, 10);
  if (fieldIsVisible(rollAnglePosition))
    MAX7456_WriteString(screenBuffer, getPosition(rollAnglePosition));
#endif

#ifdef HORIZON
  if (!armed) GPS_speed = 0;
  // Scrolling decoration
  if ((GPS_speed + 15) < old_GPS_speed) {
    sidebarsMillis = millis();
    sidebarsdir = 2;
    old_GPS_speed = GPS_speed;
    SYM_AH_DECORATION_LEFT--;
    if (SYM_AH_DECORATION_LEFT < 0x10)
      SYM_AH_DECORATION_LEFT = 0x15;
  }
  else if (GPS_speed > (old_GPS_speed + 15)) {
    sidebarsMillis = millis();
    sidebarsdir = 1;
    old_GPS_speed = GPS_speed;
    SYM_AH_DECORATION_LEFT++;
    if (SYM_AH_DECORATION_LEFT > 0x15)
      SYM_AH_DECORATION_LEFT = 0x10;
  }

  if (MwAltitude + 20 < old_MwAltitude) {
    sidebaraMillis = millis();
    sidebaradir = 2;
    old_MwAltitude = MwAltitude;
    SYM_AH_DECORATION_RIGHT--;
    if (SYM_AH_DECORATION_RIGHT < 0x10)
      SYM_AH_DECORATION_RIGHT = 0x15;
  }
  else if (MwAltitude > old_MwAltitude + 20) {
    sidebaraMillis = millis();
    sidebaradir = 1;
    old_MwAltitude = MwAltitude;
    SYM_AH_DECORATION_RIGHT++;
    if (SYM_AH_DECORATION_RIGHT > 0x15)
      SYM_AH_DECORATION_RIGHT = 0x10;
  }

  if (!getPosition(SideBarPosition)) {
    SYM_AH_DECORATION_LEFT = 0x13;
    SYM_AH_DECORATION_RIGHT = 0x13;
  }

  uint16_t position = getPosition(horizonPosition);

#ifdef AHIINVERTSUPPORT
  if (rollAngle < -900) {
    rollAngle += 1800;
    pitchAngle = -pitchAngle;
  } else if (rollAngle > 900) {
    rollAngle -= 1800;
    pitchAngle = -pitchAngle;
  }
#endif

#ifdef AHIPITCHMAX
  if (pitchAngle > AHIPITCHMAX) pitchAngle = AHIPITCHMAX;
  if (pitchAngle < -AHIPITCHMAX) pitchAngle = -AHIPITCHMAX;
#endif //AHIPITCHMAX
#ifdef AHIROLLMAX
  if (rollAngle > AHIROLLMAX) rollAngle = AHIROLLMAX;
  if (rollAngle < -AHIROLLMAX) rollAngle = -AHIROLLMAX;
#endif //AHIROLLMAX

#ifdef AHIPITCHSCALE
  pitchAngle = pitchAngle * AHIPITCHSCALE / 100;
#endif
#ifdef AHIROLLSCALE
  rollAngle = rollAngle * AHIROLLSCALE / 100;
#endif

#if defined REVERSE_AHI_PITCH
  pitchAngle = -pitchAngle;
#endif //REVERSE_AHI_PITCH
#if defined REVERSE_AHI_ROLL
  rollAngle = -rollAngle;
#endif //REVERSE_AHI_ROLL

#ifndef AHICORRECT
#define AHICORRECT 10
#endif
  pitchAngle = pitchAngle + AHICORRECT;

  if (fieldIsVisible(horizonPosition)) {
    if (MwSensorPresent & ACCELEROMETER) {

#ifdef NOAHI
#elif defined FULLAHI
      for (uint8_t X = 0; X <= 12; X++) {
        if (X == 6) X = 7;
        int Y = (rollAngle * (4 - X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if (Y >= 0 && Y <= 81) {
          uint16_t pos = position - 9 + LINE * (Y / 9) + 3 - 4 * LINE + X;
          if (pos < 480)
            screen[pos] = SYM_AH_BAR9_0 + (Y % 9);
          if (Settings[S_ELEVATIONS]) {
            if (X >= 4 && X <= 8) {
              if ((pos - 3 * LINE) < 480)
                screen[pos - 3 * LINE] = SYM_AH_BAR9_0 + (Y % 9);
              if ((pos + 3 * LINE) < 480)
                screen[pos + 3 * LINE] = SYM_AH_BAR9_0 + (Y % 9);
            }
          }
        }
      }
#else //Normal AHI
      for (uint8_t X = 0; X <= 8; X++) {
        if (X == 4) X = 5;
        int Y = (rollAngle * (4 - X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if (Y >= 0 && Y <= 81) {
          uint16_t pos = position - 7 + LINE * (Y / 9) + 3 - 4 * LINE + X;
          if (pos < 480)
            screen[pos] = SYM_AH_BAR9_0 + (Y % 9);
          if (Settings[S_ELEVATIONS]) {
            if (X >= 2 && X <= 6) {
              if ((pos - 3 * LINE) < 480)
                screen[pos - 3 * LINE] = SYM_AH_BAR9_0 + (Y % 9);
              if ((pos + 3 * LINE) < 480)
                screen[pos + 3 * LINE] = SYM_AH_BAR9_0 + (Y % 9);
            }
          }
        }
      }
#endif //FULLAHI
    }
    if (!fieldIsVisible(MapModePosition)) {
      screen[position - 1] = SYM_AH_CENTER_LINE;
      screen[position + 1] = SYM_AH_CENTER_LINE_RIGHT;
      screen[position] =   SYM_AH_CENTER;
    }
  }

  if (fieldIsVisible(SideBarPosition)) {
    // Draw AH sides
    int8_t hudwidth = Settings[S_SIDEBARWIDTH];
    int8_t hudheight = Settings[S_SIDEBARHEIGHT];
    for (int8_t X = -hudheight; X <= hudheight; X++) {
      screen[position - hudwidth + (X * LINE)] =  SYM_AH_DECORATION_LEFT;
      screen[position + hudwidth + (X * LINE)] =  SYM_AH_DECORATION_RIGHT;
    }
#if defined AHILEVEL
    screen[position - hudwidth + 1] =  SYM_AH_LEFT;
    screen[position + hudwidth - 1] =  SYM_AH_RIGHT;
#endif //AHILEVEL

#if defined (USEGLIDESCOPE) || defined (USEGLIDEANGLE) // && defined(FIXEDWING)                     
    displayfwglidescope();
#endif //USEGLIDESCOPE  

#ifdef SBDIRECTION

    if (fieldIsVisible(SideBarScrollPosition)) {
      if (millis() < (sidebarsMillis + 1000)) {
        if (sidebarsdir == 2) {
          screen[position - (hudheight * LINE) - hudwidth] = SYM_AH_DECORATION_UP;
        }
        else {
          screen[position + (hudheight * LINE) - hudwidth] = SYM_AH_DECORATION_DOWN;
        }
      }
      if (millis() < (sidebaraMillis + 1000)) {
        if (sidebaradir == 2) {
          screen[position - (hudheight * LINE) + hudwidth] = SYM_AH_DECORATION_UP;
        }
        else {
          screen[position + (hudheight * LINE) + hudwidth] = SYM_AH_DECORATION_DOWN;
        }
      }
    }
#endif //SBDIRECTION
  }
#endif //HORIZON
}


void displayVoltage(void)
{
  uint8_t t_lead_icon;
  if (voltage < voltageMIN)
    voltageMIN = voltage;

  if (Settings[S_AUTOCELL]) {
    uint8_t t_cells = (uint16_t)(voltage / (CELL_VOLTS_MAX + 3)) + 1; // Detect 3s > 9.0v, 4s > 13.5v, 5s > 18.0v, 6s > 22.5v power up voltage
    if (t_cells > cells) {
      cells++;
    }
    voltageWarning = (uint16_t)cells * Settings[S_AUTOCELL_ALARM];
  }
  else {
    cells = Settings[S_BATCELLS];
    voltageWarning = Settings[S_VOLTAGEMIN];
  }
#ifdef BATTERYICONVOLTS
  if (Settings[S_SHOWBATLEVELEVOLUTION])
  {
    uint16_t battev = 0;
    uint16_t batevlow  = (uint16_t)cells * CELL_VOLTS_MIN;
    uint16_t batevhigh = (uint16_t)cells * CELL_VOLTS_MAX;
    battev = constrain(voltage, batevlow, batevhigh - 2);
    battev = map(battev, batevlow, batevhigh - 1, 0, 7);
    t_lead_icon = (SYM_BATT_EMPTY) - battev;
  }
  else
  {
    t_lead_icon = SYM_MAIN_BATT;
  }
#else
  t_lead_icon = SYM_MAIN_BATT;
#endif // BATTERYICONVOLTS
  if ((voltage < voltageWarning) && (timer.Blink2hz))
    return;
  displayItem(voltagePosition, voltage, t_lead_icon, SYM_VOLT, 1 );
#ifdef SHOW_CELL_VOLTAGE
  displayItem(vidvoltagePosition, voltage / cells, t_lead_icon, SYM_VOLT, 1 ); // individual cell voltage avg.
#endif

}


void displayVidVoltage(void)
{
#ifndef SHOW_CELL_VOLTAGE
  if ((vidvoltage < Settings[S_VIDVOLTAGEMIN]) && (timer.Blink2hz))
    return;
  displayItem(vidvoltagePosition, vidvoltage, SYM_VID_BAT, SYM_VOLT, 1 );
#endif
}


void displayCurrentThrottle(void){

#ifdef AUTOTHROTTLE
  if (MwRcData[THROTTLESTICK] > HighT) HighT = MwRcData[THROTTLESTICK];
  if (MwRcData[THROTTLESTICK] < LowT) LowT = MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
  if (HighT > 2050) HighT = 2050;
  if (LowT < 950) LowT = 950;
#else
  HighT = HIGHTHROTTLE;
  LowT = LOWTHROTTLE;
#endif
  uint16_t t_throttle = MwRcData[THROTTLESTICK];
  uint8_t t_symbol=0;
  if (Settings[S_THROTTLE_PWM] == 0) {
    t_throttle = map(MwRcData[THROTTLESTICK], LowT, HighT, 0, 100);
    t_symbol=SYM_PERCENT;
  }
  if (!armed){
    t_throttle = 0;
    t_symbol   = SYM_ZERO;
  }
  displayItem(CurrentThrottlePosition, t_throttle, SYM_THR, t_symbol, 0 );  
}

void OLDdisplayCurrentThrottle(void)
{
  if (!fieldIsVisible(CurrentThrottlePosition))
    return;

#ifndef NOTHROTTLESPACE
#define THROTTLESPACE 1
#endif
  screenBuffer[1] = ' ';
#ifdef AUTOTHROTTLE
  if (MwRcData[THROTTLESTICK] > HighT) HighT = MwRcData[THROTTLESTICK];
  if (MwRcData[THROTTLESTICK] < LowT) LowT = MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
  if (HighT > 2050) HighT = 2050;
  if (LowT < 950) LowT = 950;
#else
  HighT = HIGHTHROTTLE;
  LowT = LOWTHROTTLE;
#endif


#ifndef FIXEDWING
  if (!armed) {
    screenBuffer[0 + THROTTLESPACE] = ' ';
    screenBuffer[1 + THROTTLESPACE] = ' ';
    screenBuffer[2 + THROTTLESPACE] = '-';
    screenBuffer[3 + THROTTLESPACE] = '-';
    screenBuffer[4 + THROTTLESPACE] = ' ';

  }
  else
#endif // FIXEDWING    
  {
    if (Settings[S_THROTTLE_PWM] > 0) {
      ItoaPadded(MwRcData[THROTTLESTICK], screenBuffer + 1 + THROTTLESPACE, 4, 0);
    }
    else {
      int CurThrottle = map(MwRcData[THROTTLESTICK], LowT, HighT, 0, 100);
      ItoaPadded(CurThrottle, screenBuffer + 1 + THROTTLESPACE, 3, 0);
      screenBuffer[4 + THROTTLESPACE] = '%';
    }
  }
  screenBuffer[0] = SYM_THR;
  screenBuffer[5 + THROTTLESPACE] = 0;
  MAX7456_WriteString(screenBuffer, getPosition(CurrentThrottlePosition));
}


void displayTimer(uint32_t t_time, uint16_t t_pos, uint8_t t_leadsymbol)
{
  if (t_time>=3600){
    t_time /=60;
  }
  uint32_t digit0 = t_time/60;
  uint32_t digit1 = t_time%60;
  if (t_leadsymbol>0){
    screenBuffer[0]=t_leadsymbol;
    screenBuffer[1]=0;
    MAX7456_WriteString(screenBuffer, t_pos);  
    t_pos++;
  }
  formatDateTime(digit0, digit1, 0, ':', 0);
  MAX7456_WriteString(screenBuffer, t_pos);  
}


void displayRemainingTime(void){
  int32_t t_remaining;
  int32_t t_used = 100 * Settings[S_AMPER_HOUR_ALARM]- (amperagesum/(360));  
  if (screenPosition[remainingTimePosition] < 512)
    return;
  if (t_used < 0){
    t_used = 0;
  }
#ifdef EFFICIENCYTIMEINST  
  if (amperage>1){
    t_remaining = (uint32_t) 60 * 60 *(t_used)/(amperage * 100);
  }
#else
  if (amperagesum>100){
    t_remaining = (uint32_t) flyTime *(t_used)/(amperagesum/360);
  }
#endif
  else{ 
    t_remaining = 0;
  }
  displayTimer(t_remaining,getPosition(remainingTimePosition), 0x2A);
}


void displayFlightTime(void){
  if (screenPosition[onTimePosition] < 512)
    return;

  uint32_t displaytime;
  uint8_t t_leadsymbol = 0;
  
  if (armed) {
    if (Settings[S_FLYTIME_ALARM] > 0) {
      if (((flyTime / 60) >= Settings[S_FLYTIME_ALARM]) && (timer.Blink2hz))
        return;
    }
      t_leadsymbol +=2;
      displaytime = flyTime;
  }
  else {
      screenBuffer[0] = SYM_ON_H;
      displaytime = onTime;
  }
  if (displaytime>=3600){
    t_leadsymbol+=1;
  }
  displayTimer(displaytime,getPosition(onTimePosition), flightUnitAdd[t_leadsymbol]);
}


void displayAmperage(void)
{
  if (amperage > ampMAX)
    ampMAX = amperage;
  displayItem(amperagePosition, amperage, 0, SYM_AMP, 1 );
}


void displayWatt(void)
{
  uint16_t watts = amperage * voltage / 100; // Watts
  displayItem(wattPosition, watts, SYM_POWER, SYM_WATT, 0 );
}


void displayEfficiency(void)
{
  uint16_t t_xx;
  uint16_t t_efficiency;
  if (!Settings[S_UNITSYSTEM])
    t_xx = GPS_speed * GPS_CONVERSION_UNIT_TO_KM_H;
  else
    t_xx = GPS_speed * GPS_CONVERSION_UNIT_TO_M_H;      
  if (t_xx > 0) {
    t_efficiency = amperage * voltage / (10 * t_xx); // Watts/Speed}
  }
  else {
    t_efficiency = 999;
  }
  if (t_efficiency < 999)
    displayItem(efficiencyPosition, t_efficiency, SYM_EFF, 0, 0 );
}


void displayAverageEfficiency(void)
{
  uint16_t t_efficiency;
  if (flyTime > 0){
    t_efficiency = (uint32_t) amperagesum /(6 * flyTime) ;
  if (t_efficiency < 999)
    displayItem(avgefficiencyPosition, t_efficiency, SYM_AVG_EFF, 0, 0 );
  }
}


void displaypMeterSum(void)
{
  int xx = amperagesum / 360;

#ifdef BATTERYICONAMPS
  uint16_t battev = 0;
  if (Settings[S_SHOWBATLEVELEVOLUTION]) {
    battev = amperagesum / (360 * Settings[S_AMPER_HOUR_ALARM]);
    battev = constrain(battev, 0, 100);
    battev = map(100 - battev, 0, 101, 0, 7);
    uint8_t t_lead_icon = SYM_BATT_EMPTY - battev;
    displayItem(pMeterSumPosition, xx, t_lead_icon, SYM_MAH, 0 );
  }
  else
    displayItem(pMeterSumPosition, xx, 0, SYM_MAH, 0 );
#else
  displayItem(pMeterSumPosition, xx, 0, SYM_MAH, 0 );
#endif //BATTERYICONAMPS
}


void displayI2CError(void)
{
#ifdef I2CERROR
  if (I2CError <= I2CERROR)
    return;
  screenBuffer[0] = 0x49;
  screenBuffer[1] = 0X3A;
  itoa(I2CError, screenBuffer + 2, 10);
  MAX7456_WriteString(screenBuffer, getPosition(temperaturePosition));
#endif
}


void displayRSSI(void)
{
  if (rssi < rssiMIN && rssi > 0)
    rssiMIN = rssi;
#ifdef DUALRSSI
  displayItem(rssiPosition, rssi, SYM_RSSI, '%', 0 );
  if (!fieldIsVisible(rssiPosition))
    return;
  screenBuffer[0] = SYM_RSSI;
  uint8_t t_FCRssi = map(FCRssi, 0, DUALRSSI, 0, 100);
  itoa(t_FCRssi, screenBuffer + 1, 10);
  uint8_t xx = FindNull();
  screenBuffer[xx++] = '%';
  screenBuffer[xx] = 0;
  MAX7456_WriteString(screenBuffer, getPosition(rssiPosition) + 30);
#else
  displayItem(rssiPosition, rssi, SYM_RSSI, '%', 0 );
#endif
}


void displayIntro(void)
{
  for (uint8_t X = 0; X <= 8; X++) {
    MAX7456_WriteString_P(PGMSTR(&(intro_item[X])), 64 + (X * 30));
  }
#ifdef INTRO_CALLSIGN
  displayCallsign(64 + (30 * 6) + 4);
#endif
#ifdef INTRO_SIGNALTYPE
  MAX7456_WriteString_P(PGMSTR(&(signal_type[flags.signalauto])), 64 + (30 * 7) + 4);
#endif
#ifdef INTRO_FC
  itoa(FC.verMajor, screenBuffer, 10);
  uint8_t xx = FindNull();
  screenBuffer[xx] = 0x2E;
  xx++;
  itoa(FC.verMinor, screenBuffer + xx, 10);
  xx = FindNull();
  screenBuffer[xx] = 0x2E;
  xx++;
  itoa(FC.verMinor, screenBuffer + xx, 10);
  MAX7456_WriteString(screenBuffer, 64 + (30 * 8) + 4);

#endif
#ifdef HAS_ALARMS
  if (alarmState != ALARM_OK) {
    MAX7456_WriteString((const char*)alarmMsg, 64 + (30 * 9) + 4);
  }
#endif
}


void displayAltitude(int32_t t_alt10, int16_t t_pos, uint8_t t_icon) { // alt sent as dm
  uint8_t t_dp = 0;
#if defined SUBMARINE
  t_alt10 = abs(t_alt10);
#endif // SUBMARINE  
  if (Settings[S_UNITSYSTEM]) {
    t_alt10 = (float) (3.32808 * t_alt10); // convert to imperial dm
  }
  int32_t t_alt = t_alt10 / 10; // alt in meters or feet
  if (armed && (allSec > 5) && ((t_alt) > altitudeMAX)) { // not sure why 5 secs...
    altitudeMAX = t_alt;
  }
  if (Settings[S_ALTITUDE_ALARM] > 0) {
    if (((t_alt / 1000) >= Settings[S_ALTITUDE_ALARM]) && (timer.Blink2hz)) {
      return;
    }
  }
  if (t_alt < Settings[S_ALTRESOLUTION]) {
    t_dp = 1;
    if (t_icon==SYM_ALT){
      t_icon = SYM_AGL;
    }
  }
  displayItem(t_pos, t_alt, t_icon, UnitsIcon[Settings[S_UNITSYSTEM] + 0], t_dp ); 

/*  
  if (t_alt < Settings[S_ALTRESOLUTION]) {
    displayItem(t_pos, t_alt10, SYM_AGL, UnitsIcon[Settings[S_UNITSYSTEM] + 0], 1 );
  }
  else {
    displayItem(t_pos, t_alt, t_icon, UnitsIcon[Settings[S_UNITSYSTEM] + 0], 0 );
  }
*/  
}


void displayNumberOfSat(void)
{
  if ((GPS_numSat < MINSATFIX) && (timer.Blink2hz)) {
    return;
  }
#if defined ICON_SAT
  if (!fieldIsVisible(GPS_numSatPosition))
    return;
  screenBuffer[0] = SYM_SAT_L;
  screenBuffer[1] = SYM_SAT_R;
  itoa(GPS_numSat, screenBuffer + 2, 10);
  MAX7456_WriteString(screenBuffer, getPosition(GPS_numSatPosition));
#else
  displayItem(GPS_numSatPosition, GPS_numSat, SYM_SAT, 0 , 0 );
#endif
}


void displayGPSdop(void)
{
  uint16_t t_dop = GPS_dop / 10;
  if (t_dop > 99)
    t_dop = 99;
  displayItem(DOPposition, t_dop, SYM_DOP, 0 , 1 );
}


void display_speed(int16_t t_value, uint8_t t_position, uint8_t t_leadicon)
{
  uint16_t t_speed;
  if (!Settings[S_UNITSYSTEM])
    t_speed = t_value * GPS_CONVERSION_UNIT_TO_KM_H;
  else
    t_speed = t_value * GPS_CONVERSION_UNIT_TO_M_H;
  if (t_speed > (speedMAX + 20)) // simple GPS glitch limit filter
    speedMAX += 20;
  else if (t_speed > speedMAX)
    speedMAX = t_speed;
  if (Settings[S_SPEED_ALARM] > 0) {
    if ((t_speed > Settings[S_SPEED_ALARM]) && (timer.Blink2hz))
      return;
  }
#ifdef DISPLAYSPEEDMS
  t_speed = t_value * 0.01;           // From MWii cm/sec to m/sec
  displayItem(t_position, t_speed, t_leadicon, SYM_MS, 0 );
#else
  displayItem(t_position, t_speed, t_leadicon, speedUnitAdd[Settings[S_UNITSYSTEM]], 0 );
#endif
}


void displayVario(void)
{
  if (!fieldIsVisible(MwVarioPosition))
    return;
  uint16_t position = getPosition(MwVarioPosition);

#ifndef VARIOSCALE
#define VARIOSCALE 150 // max 127 8 bit
#endif

#if defined VARIOENHANCED // multi char slider representation of climb rate
#define VARIOICONCOUNT 3
#define VARIOROWS Settings[S_VARIO_SCALE]
  int16_t t_vario = MwVario;
  if (MwVario > VARIOSCALE) t_vario = VARIOSCALE;
  if (MwVario < -VARIOSCALE) t_vario = -VARIOSCALE;
  int8_t t_vario_rows = (int16_t)t_vario / (VARIOSCALE / VARIOROWS);
  int8_t t_vario_icon = ((int16_t)t_vario % (VARIOSCALE / VARIOROWS)) / (VARIOSCALE / (VARIOROWS * VARIOICONCOUNT));
  for (uint8_t X = 0; X < abs(t_vario_rows); X++) {
    if (MwVario > 0)
      screen[position - (LINE * X)] = SYM_VARIO + VARIOICONCOUNT; // need -ve too
    else
      screen[position + (LINE * X)] = SYM_VARIO - VARIOICONCOUNT; // need -ve too
  }
  if (t_vario_icon != 0)
    screen[position - (LINE * t_vario_rows)] = SYM_VARIO + t_vario_icon; // need -ve too
#elif defined VARIOSTANDARD // single char icon representation of climb rate
#define VARIOICONCOUNT 3
#undef VARIOROWS
#define VARIOROWS 1
  int16_t t_vario = MwVario;
  if (MwVario > VARIOSCALE) t_vario = VARIOSCALE;
  if (MwVario < -VARIOSCALE) t_vario = -VARIOSCALE;
  t_vario = t_vario / (VARIOSCALE / VARIOICONCOUNT);
  screen[position] = SYM_VARIO + t_vario;
#else
#endif

#ifdef AUDIOVARIO
  //test when using non RSSI output
  //MwVario = map(rssi,0,100,-VARIOMAXCLIMB,VARIOMAXCLIMB);

  if (MwRcData[THROTTLESTICK] > (20 * Settings[S_AUDVARIO_TH_CUT])) {
    return;
  }
#ifdef AUDIOVARIOSWITCH
  if (!fieldIsVisible(MwClimbRatePosition))
    return;
#endif //AUDIOVARIOSWITCH

#define AUDIOVARIOMINFREQ    200  // Minimum audio frequency
#define AUDIOVARIOMAXFREQ    1600 // Maximum audio frequency
#define AUDIOVARIOMAXCLIMB   200  // Maximum climb/sink rate
  int16_t ConstrainedVario   = constrain(MwVario, -AUDIOVARIOMAXCLIMB, AUDIOVARIOMAXCLIMB);
  int16_t AudioVarioTone     = map(ConstrainedVario, -AUDIOVARIOMAXCLIMB, AUDIOVARIOMAXCLIMB, AUDIOVARIOMINFREQ, AUDIOVARIOMAXFREQ);
  int16_t MarkSpaceRatio     = map(ConstrainedVario, -AUDIOVARIOMAXCLIMB, AUDIOVARIOMAXCLIMB, 2000, 50);

  if (millis() > timer.vario) {
    if (flags.vario) { // notone
      timer.vario = millis() + (100);
      flags.vario = 0;
    }
    else { // tone
      timer.vario = millis() + (MarkSpaceRatio);
      flags.vario = 1;
    }
  }

  if (MwVario > AUDIOVARIOTHRESHOLDCLIMB) {
    if (flags.vario == 0) {
      AudioVarioTone = 0;
    }
  }
  else if (MwVario < AUDIOVARIOTHRESHOLDSINK) {
    if (flags.vario == 0) {
      AudioVarioTone = 0;
    }
  }
  else {
#ifdef AUDIOVARIOSILENTDEADBAND
    AudioVarioTone = 0;
#endif // AUDIOVARIOSILENTDEADBAND
  }

  if (AudioVarioTone > 0) {
    tone(AUDIOVARIO, AudioVarioTone);
  }
  else {
    noTone(AUDIOVARIO);
  }
#endif // AUDIOVARIO
}


void displayClimbRate(void)
{
  int16_t climbrate;
  if (Settings[S_UNITSYSTEM])
    climbrate = MwVario * 0.32808;       // ft/sec *10 for DP
  else
    climbrate = MwVario / 10;            // mt/sec *10 for DP
#ifdef SHOWNEGATIVECLIMBRATE
  displayItem(climbratevaluePosition, climbrate, SYM_CLIMBRATE, varioUnitAdd[Settings[S_UNITSYSTEM]], 1 ); //Show +/-
#else
  displayItem(climbratevaluePosition, climbrate, SYM_CLIMBRATE, varioUnitAdd[Settings[S_UNITSYSTEM]], 1 ); //neater look
#endif
}


void displayDistanceToHome(void)
{
  //  if(!GPS_fix)
  //    return;
  uint32_t dist;
  if (Settings[S_UNITSYSTEM])
    dist = GPS_distanceToHome * 3.2808;           // mt to feet
  else
    dist = GPS_distanceToHome;                    // Mt
  if (dist > distanceMAX) {
    if (dist > distanceMAX + 100)
      dist = distanceMAX + 100; // constrain for data errors
    distanceMAX = dist;
  }
  if (!fieldIsVisible(GPS_distanceToHomePosition))
    return;

  if (Settings[S_DISTANCE_ALARM] > 0) {
    if (((dist / 100) >= Settings[S_DISTANCE_ALARM]) && (timer.Blink2hz))
      return;
  }

  //  formatDistance(dist,1,2,0);
  formatDistance(dist, 1, 2, SYM_DTH);
  MAX7456_WriteString(screenBuffer, getPosition(GPS_distanceToHomePosition));
}


void displayDistanceTotal(void)
{
#if defined SHOW_TOTAL_DISTANCE
  if (!fieldIsVisible(TotalDistanceposition))
    return;
  formatDistance(trip, 1, 2, SYM_TOTAL);
  MAX7456_WriteString(screenBuffer, getPosition(TotalDistanceposition));
#endif //SHOW_TOTAL_DISTANCE
}


void displayDistanceMax(void)
{
#if defined SHOW_MAX_DISTANCE
  if (!fieldIsVisible(MaxDistanceposition))
    return;
  formatDistance(distanceMAX, 1, 2, SYM_MAX);
  MAX7456_WriteString(screenBuffer, getPosition(MaxDistanceposition));
#endif //SHOW_MAX_DISTANCE
}


void displayHeadingGraph(void)
{
  if (!fieldIsVisible(MwHeadingGraphPosition))
    return;
  int xx;
  xx = MwHeading * 4;
  xx = xx + 720 + 45;
  xx = xx / 90;
  uint16_t pos = getPosition(MwHeadingGraphPosition);
  memcpy_P(screen + pos, headGraph + xx + 1, 9);
}


void displayHeading(void)
{
  /*
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
  */
  int16_t heading = MwHeading;
  if (heading < 0)
    heading += 360;
  displayItem(MwHeadingPosition, heading, SYM_ANGLE_HDG, SYM_DEGREES, 0 );
}


void displayAngleToHome(void)
{
  /*
    if(!GPS_fix)
      return;
    if(!fieldIsVisible(GPS_angleToHomePosition))
      return;
      ItoaPadded(GPS_directionToHome,screenBuffer,3,0);
      screenBuffer[3] = SYM_DEGREES;
      screenBuffer[4] = 0;
      MAX7456_WriteString(screenBuffer,getPosition(GPS_angleToHomePosition));
  */
  displayItem(GPS_angleToHomePosition, GPS_directionToHome, SYM_ANGLE_RTH, SYM_DEGREES, 0 );

}


void displayDirectionToHome(void)
{
  if (!fieldIsVisible(GPS_directionToHomePosition))
    return;
  //if(GPS_distanceToHome <= 2 && timer.Blink2hz)
  //  return;
  uint16_t position = getPosition(GPS_directionToHomePosition);
  int16_t d = MwHeading + 180 + 360 - GPS_directionToHome;
  d *= 4;
  d += 45;
  d = (d / 90) % 16;
  screenBuffer[0] = SYM_ARROW_HOME + d;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer, position);
}


void displayWindSpeed(void)
{
  if (!fieldIsVisible(WIND_speedPosition))
    return;
  int16_t d;
#ifdef PROTOCOL_MAVLINK
#ifdef MAV_WIND_DIR_REVERSE
  d = WIND_direction + 180;
#else
  d = WIND_direction;
#endif
  d *= 4;
  d += 45;
  d = (d / 90) % 16;
  uint16_t t_WIND_speed;
  // if (WIND_speed > 0){
  if (!Settings[S_UNITSYSTEM])
    t_WIND_speed = WIND_speed;             // Km/h
  else
    t_WIND_speed = WIND_speed * 0.62137;   // Mph
  displayItem(WIND_speedPosition, t_WIND_speed, SYM_ARROW_DIR + d, speedUnitAdd[Settings[S_UNITSYSTEM]], 0 );
  // }
#else
  d = (MwHeading + 360 + 360 + 180) - GPS_ground_course / 10 ;
  d *= 4;
  d += 45;
  d = (d / 90) % 16;
  screenBuffer[0] = SYM_ARROW_DIR + d;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer, getPosition(WIND_speedPosition));
#endif
}


void displayCursor(void)
{

  uint16_t cursorpos = 0;
  if (ROW == 10) {
    if (COL == 3) cursorpos = SAVEP + 16 - 1; // page
    if (COL == 1) cursorpos = SAVEP - 1; // exit
    if (COL == 2) cursorpos = SAVEP + 6 - 1; // save/exit
  }
  if (ROW < 10)
  {
#ifdef MENU_KISS
    if (configPage == MENU_KISS) {
      if (subConfigPage <0) {
        if (ROW == 9) {
          ROW = SUBMENU_KISS_SIZE;
          COL = 1;
        } else {
          if (ROW > SUBMENU_KISS_SIZE) {
            ROW = 10;
          }
        }
        oldROW = ROW;
        cursorpos = (ROW-1) * 30 + LINE05 + 7;
      } else {
        switch (subConfigPage)
        {
          case SUBMENU_KISS_PID:
          case SUBMENU_KISS_RATE:
            if (ROW == 9) {
              ROW = 5;
            } else {
              if (ROW > 5) {
                ROW = 10;
                if (COL == 2) {
                  COL = 3;
                }
              } else {
                if (oldROW > ROW) {
                  ROW--;
                } else if (oldROW < ROW) {
                  ROW++;
                }
              }
            }
          
            oldROW = ROW;
            cursorpos = (ROW + 2) * 30 + 10 + (COL - 1) * 6;

            if (ROW < 10 && COL == 3) {
              cursorpos += 1;  
            }
            break;
          case SUBMENU_KISS_NOTCH_FILTERS:
            if (ROW == 9) {
              ROW = 2;
            } else {
              if (ROW > 2) {
                ROW = 10;
                COL = 3;
              }
            }
            cursorpos = (ROW + 2) * 30;
            switch (COL)
            {
              case 1: cursorpos += 8; break;
              case 2: cursorpos += 15; break;
              case 3: cursorpos += 22; break;
            }
            break;
          case SUBMENU_KISS_LPF:
            if (ROW == 9) {
              ROW = 4;
            } else {
              if (ROW > 4) {
                ROW = 10;
                if (COL == 2) {
                  COL = 3;
                }
              } else {
                COL = 3;
              }
            }
            cursorpos = (ROW + 2) * 30 + 18;
            break;
          case SUBMENU_KISS_VTX:
            if (ROW == 9) {
              ROW = 5;
            } else {
              if (ROW > 5) {
                ROW = 10;
                if (COL == 2) {
                  COL = 3;
                }
              } else {
                COL = 3;
              }
            }
            cursorpos = (ROW + 2) * 30 + 12;
            break;
        }
      }
    }
#endif // MENU_KISS
#ifdef USE_MENU_VTX
    if (configPage == MENU_VTX) {
      if (ROW == 5) ROW = 10;
      if (ROW == 9) ROW = 4;
      cursorpos = (ROW + 2) * 30 + 16;
    }
#endif
#ifdef MENU_PID
    if (configPage == MENU_PID) {
#ifdef MENU_PID_VEL
      if (ROW == 9) {
        if (oldROW == 8)
          ROW = 10;
        else
          ROW = 8;
      }
      oldROW = ROW;
#else
      if (ROW == 8) ROW = 10;
      if (ROW == 9) ROW = 7;
#endif
      cursorpos = (ROW + 2) * 30 + 10 + (COL - 1) * 6;
    }
#endif

#ifdef MENU_2RC
    if (configPage == MENU_2RC) {
      if (ROW == 3) ROW = 10;
      if (ROW == 9) ROW = 2;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif


#ifdef MENU_RC
#if defined CORRECT_MENU_RCT2
    if (configPage == MENU_RC) {
      if (ROW == 9) {
        if (oldROW == 8)
          ROW = 10;
        else
          ROW = 8;
      }
      oldROW = ROW;
      COL = 3;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#elif defined CORRECT_MENU_RCT1
    if (configPage == MENU_RC)
    {
      if (ROW == 9) {
        if (oldROW == 8)
          ROW = 10;
        else
          ROW = 8;
      }
      oldROW = ROW;
      COL = 3;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#else
    if (configPage == MENU_RC) {
      COL = 3;
      if (ROW == 8) ROW = 10;
      if (ROW == 9) ROW = 7;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif

#endif
#ifdef MENU_SERVO
    if (configPage == MENU_SERVO) {
      if (ROW == 9) {
        if (oldROW == 8)
          ROW = 10;
        else
          ROW = 8;
      }
      oldROW = ROW;
      if (COL == 1) cursorpos = (ROW + 2) * 30 + 6;
      if (COL == 2) cursorpos = (ROW + 2) * 30 + 6 + 7;
      if (COL == 3) cursorpos = (ROW + 2) * 30 + 6 + 7 + 7;
    }
#endif
#ifdef MENU_FIXEDWING
    if (configPage == MENU_FIXEDWING) {
      COL = 3;
      if (ROW == 9) {
        if (oldROW == 8)
          ROW = 10;
        else
          ROW = 8;
      }
      oldROW = ROW;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_VOLTAGE
    if (configPage == MENU_VOLTAGE) {
      COL = 3;
      if (ROW == 5) ROW = 10;
      if (ROW == 9) ROW = 4;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_RSSI
    if (configPage == MENU_RSSI) {
      COL = 3;
      if (ROW == 4) ROW = 10;
      if (ROW == 9) ROW = 3;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_CURRENT
    if (configPage == MENU_CURRENT)
    {
      COL = 3;
      if (ROW == 9) ROW = 2;
      if (ROW == 3) ROW = 10;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_DISPLAY
    if (configPage == MENU_DISPLAY)
    {
      if (ROW == 9) ROW = 1;
      if (ROW == 2) ROW = 10;
      COL = 3;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_ADVANCED
    if (configPage == MENU_ADVANCED)
    {
      COL = 3;
      if (ROW == 9) ROW = 2;
      if (ROW == 3) ROW = 10;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_GPS_TIME
    if (configPage == MENU_GPS_TIME)
    {
      COL = 3;
      if (ROW == 9) ROW = 3;
      if (ROW == 4) ROW = 10;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_ALARMS
    if (configPage == MENU_ALARMS)
    {
      COL = 3;
      if (ROW == 9) ROW = 7;
      if (ROW == 8) ROW = 10;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_INFO
    if (configPage == MENU_INFO)
    {
      COL = 3;
      ROW = 10;
    }
#endif
#ifdef MENU_PROFILE
    if (configPage == MENU_PROFILE)
    {
#ifdef CORRECTLOOPTIME
      if (ROW == 9) ROW = 3;
      if (ROW == 4) ROW = 10;
#else
      if (ROW == 9) ROW = 2;
      if (ROW == 3) ROW = 10;
#endif
      COL = 3;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
#ifdef MENU_DEBUG
    if (configPage == MENU_DEBUG)
    {
      COL = 3;
      ROW = 10;
    }
#endif
#ifdef USE_MENUVTX
    if (configPage == MENUVTX)
    {
      COL = 3;
      if (ROW == 9) ROW = 3;
      if (ROW == 4) ROW = 10;
      cursorpos = (ROW + 2) * 30 + 10 + 6 + 6;
    }
#endif
  }
  if (timer.Blink10hz)
    screen[cursorpos] = SYM_CURSOR;
}


void displayConfigScreen(void)
{
  int16_t MenuBuffer[10];
  uint32_t MaxMenuBuffer[9];
#ifdef MENU_KISS
  if(configPage == MENU_KISS) {
    displaySubMenuConfig();
    return;
  }
#endif // MENU_KISS

  MAX7456_WriteString_P(PGMSTR(&(menutitle_item[configPage])), 35);
#ifdef MENU_PROFILE
  //   MAX7456_WriteString(itoa(FCProfile,screenBuffer,10),50); // Display Profile number
#endif
  MAX7456_WriteString_P(configMsgEXT, SAVEP);    //EXIT
  if (!previousarmedstatus) {
    MAX7456_WriteString_P(configMsgSAVE, SAVEP + 6); //SaveExit
    MAX7456_WriteString_P(configMsgPGS, SAVEP + 16); //<Page>
  }

  if (configPage == MENU_STAT)
  {

#ifdef SHORTSUMMARY
    MAX7456_WriteString_P(PGMSTR(&(menu_stats_item[0])), ROLLT);
    displayTimer(flyingTime, ROLLD - 3, 0);
#else // SHORTSUMMARY

#ifdef MINSUMMARY
    if (!fieldIsVisible(rssiPosition)) {
      rssiMIN = 0;
    }
#endif //MINSUMMARY      
    MaxMenuBuffer[0] = 1;
    MaxMenuBuffer[1] = trip;
    MaxMenuBuffer[2] = distanceMAX;
    MaxMenuBuffer[3] = altitudeMAX;
    MaxMenuBuffer[4] = speedMAX;
    MaxMenuBuffer[5] = amperagesum / 360;
    MaxMenuBuffer[6] = ampMAX;
    MaxMenuBuffer[7] = voltageMIN;
    MaxMenuBuffer[8] = rssiMIN;

    uint8_t Y = -1;
    for (uint8_t X = 0; X <= 8; ++X) {
#ifdef MINSUMMARY
      if (MaxMenuBuffer[X] <= 0) {
        continue;
      } else  {
        Y++;
      }
#else
      Y++;
#endif

      MAX7456_WriteString_P(PGMSTR(&(menu_stats_item[X])), ROLLT + (Y * 30));
      if (( X == 6) | ( X == 7))  {
        ItoaPadded(MaxMenuBuffer[X], screenBuffer, 4, 3);
        MAX7456_WriteString(screenBuffer, 110 + (30 * Y));
      }
      else  {
#ifdef LONG_RANGE_DISPLAY
        formatDistance(MaxMenuBuffer[X], 0, 2, 0);
        MAX7456_WriteString(screenBuffer, 110 + (30 * Y));
#else
        MAX7456_WriteString(itoa(MaxMenuBuffer[X], screenBuffer, 10), 110 + (30 * Y));
#endif
      }
    }

    displayTimer(flyingTime, ROLLD - 3, 0);
#endif
#ifdef HAS_ALARMS
    if (alarmState != ALARM_OK) {
      MAX7456_WriteString((const char*)alarmMsg, LINE12 + 3);
    }
#endif

  }
#ifdef MENU_PID
  if (configPage == MENU_PID)
  {

#ifdef MENU_PID_VEL
    for (uint8_t X = 0; X <= 7; X++)
#else
    for (uint8_t X = 0; X <= 6; X++)
#endif
    {
#ifdef USE_MSP_PIDNAMES
      MAX7456_WriteString(menu_pid[X], ROLLT + (X * 30));
#else
      MAX7456_WriteString_P(PGMSTR(&(menu_pid[X])), ROLLT + (X * 30));
#endif
    }

#ifdef MENU_PID_VEL
    for (uint8_t Y = 0; Y <= 9; Y++)
#else
    for (uint8_t Y = 0; Y <= 8; Y++)
#endif
    {
      if (Y == 5) Y = 7;
      uint8_t X = Y;
      if (Y > 6) {
        X = X - 2;
      }
      MAX7456_WriteString(itoa(pidP[Y], screenBuffer, 10), ROLLP + (X * 30));
      MAX7456_WriteString(itoa(pidI[Y], screenBuffer, 10), ROLLI + (X * 30));
      MAX7456_WriteString(itoa(pidD[Y], screenBuffer, 10), ROLLD + (X * 30));
    }

    MAX7456_WriteString("P", 71);
    MAX7456_WriteString("I", 77);
    MAX7456_WriteString("D", 83);
  }
#endif
#ifdef MENU_2RC
  if (configPage == MENU_2RC)
  {
    MenuBuffer[0] = tpa_breakpoint16;
    MenuBuffer[1] = rcYawExpo8;
    for (uint8_t X = 0; X <= 1; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_rc_2[X])), ROLLT + (X * 30));
      MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
  }
#endif
#ifdef MENU_RC
  if (configPage == MENU_RC)
  {
#if defined CORRECT_MENU_RCT2
    MenuBuffer[0] = rcRate8;
    MenuBuffer[1] = rcExpo8;
    MenuBuffer[2] = rollRate;
    MenuBuffer[3] = PitchRate;
    MenuBuffer[4] = yawRate;
    MenuBuffer[5] = dynThrPID;
    MenuBuffer[6] = thrMid8;
    MenuBuffer[7] = thrExpo8;
    for (uint8_t X = 0; X <= 7; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_rc[X])), ROLLT + (X * 30));
      MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
#elif defined CORRECT_MENU_RCT1
    MenuBuffer[0] = rcRate8;
    MenuBuffer[1] = rcExpo8;
    MenuBuffer[2] = rollRate;
    MenuBuffer[3] = PitchRate;
    MenuBuffer[4] = yawRate;
    MenuBuffer[5] = dynThrPID;
    MenuBuffer[6] = thrMid8;
    MenuBuffer[7] = thrExpo8;
    for (uint8_t X = 0; X <= 7; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_rc[X])), ROLLT + (X * 30));
      MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
#else
    MenuBuffer[0] = rcRate8;
    MenuBuffer[1] = rcExpo8;
    MenuBuffer[2] = rollPitchRate;
    MenuBuffer[3] = yawRate;
    MenuBuffer[4] = dynThrPID;
    MenuBuffer[5] = thrMid8;
    MenuBuffer[6] = thrExpo8;
    for (uint8_t X = 0; X <= 6; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_rc[X])), ROLLT + (X * 30));
      MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
#endif
  }
#endif
#ifdef MENU_SERVO
  if (configPage == MENU_SERVO)
  {
    MAX7456_WriteString("MIN", 67);
    MAX7456_WriteString("MAX", 67 + 7);
    MAX7456_WriteString("MID", 67 + 7 + 7);
    for (uint8_t X = 0; X < 8; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_servo[X])), 93 + (X * 30));
      for (uint8_t YY = 0; YY < 3; YY++) {
        MAX7456_WriteString(itoa(servo.settings[YY][X], screenBuffer, 10), 97 + (X * 30) + (YY * 7));
      }
    }
  }
#endif
#ifdef MENU_FIXEDWING
  if (configPage == MENU_FIXEDWING)
  {
    MenuBuffer[0] = cfg.fw_gps_maxcorr;
    MenuBuffer[1] = cfg.fw_gps_rudder;
    MenuBuffer[2] = cfg.fw_gps_maxclimb;
    MenuBuffer[3] = cfg.fw_gps_maxdive;
    MenuBuffer[4] = cfg.fw_climb_throttle;
    MenuBuffer[5] = cfg.fw_cruise_throttle;
    MenuBuffer[6] = cfg.fw_idle_throttle;
    MenuBuffer[7] = cfg.fw_rth_alt;
    for (uint8_t X = 0; X <= 7; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_fixedwing_bf[X])), ROLLT + (X * 30));
      MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
  }
#endif
#ifdef MENU_VOLTAGE
  if (configPage == MENU_VOLTAGE)
  {
    ProcessSensors();
    screenBuffer[0] = SYM_MAIN_BATT;
    ItoaPadded(voltage, screenBuffer + 1, 4, 3);
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;
    MAX7456_WriteString(screenBuffer, ROLLD - LINE - LINE - 1);

    screenBuffer[0] = SYM_VID_BAT;
    ItoaPadded(vidvoltage, screenBuffer + 1, 4, 3);
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;
    MAX7456_WriteString(screenBuffer, ROLLI - LINE - LINE - 3);

    for (uint8_t X = 0; X <= 3; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_bat[X])), ROLLT + (X * 30));
    }
    MAX7456_WriteString(itoa(Settings[S_VOLTAGEMIN], screenBuffer, 10), ROLLD);
    MAX7456_WriteString(itoa(Settings[S_DIVIDERRATIO], screenBuffer, 10), PITCHD);
    MAX7456_WriteString(itoa(Settings[S_VIDDIVIDERRATIO], screenBuffer, 10), YAWD);
    MAX7456_WriteString(itoa(Settings[S_BATCELLS], screenBuffer, 10), ALTD);
  }
#endif
#ifdef MENU_RSSI
  if (configPage == MENU_RSSI)
  {
    itoa(rssi, screenBuffer, 10);
    uint8_t xx = FindNull();
    screenBuffer[xx++] = '%';
    screenBuffer[xx] = 0;
    MAX7456_WriteString(screenBuffer, ROLLD - LINE - LINE);
    for (uint8_t X = 0; X <= 2; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_rssi[X])), ROLLT + (X * 30));
    }
    if (timer.rssiTimer > 0) {
      MAX7456_WriteString(itoa(timer.rssiTimer, screenBuffer, 10), ROLLD);
    }
    else {
      MAX7456_WriteString("-", ROLLD);
    }
    MAX7456_WriteString(itoa(Settings16[S16_RSSIMAX], screenBuffer, 10), PITCHD);
    MAX7456_WriteString(itoa(Settings16[S16_RSSIMIN], screenBuffer, 10), YAWD);
  }
#endif
#ifdef MENU_CURRENT
  if (configPage == MENU_CURRENT)
  {
    ItoaPadded(amperage, screenBuffer, 4, 3);     // 99.9 ampere max!
    screenBuffer[4] = SYM_AMP;
    screenBuffer[5] = 0;
    MAX7456_WriteString(screenBuffer, ROLLD - LINE - LINE - 1);

    for (uint8_t X = 0; X <= 1; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_amps[X])), ROLLT + (X * 30));
    }
    MAX7456_WriteString(itoa(Settings16[S16_AMPDIVIDERRATIO], screenBuffer, 10), ROLLD);
    MAX7456_WriteString(itoa(Settings16[S16_AMPZERO], screenBuffer, 10), PITCHD);
  }
#endif
#ifdef MENU_DISPLAY
  if (configPage == MENU_DISPLAY)
  {
    for (uint8_t X = 0; X <= 0; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_display[X])), ROLLT + (X * 30));
    }
    MAX7456_WriteString(itoa(Settings[S_MAPMODE], screenBuffer, 10), ROLLD);
  }
#endif
#ifdef MENU_ADVANCED
  if (configPage == MENU_ADVANCED)
  {
    for (uint8_t X = 0; X <= 1; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_advanced[X])), ROLLT + (X * 30));
    }

    if (timer.magCalibrationTimer > 0)
      MAX7456_WriteString(itoa(timer.magCalibrationTimer, screenBuffer, 10), ROLLD);
    else
      MAX7456_WriteString("-", ROLLD);
    Menuconfig_onoff(PITCHD, S_THROTTLE_PWM);
  }
#endif
#ifdef MENU_GPS_TIME
  if (configPage == MENU_GPS_TIME)
  {
    for (uint8_t X = 0; X <= 2; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_gps_time[X])), ROLLT + (X * 30));
    }
    Menuconfig_onoff(ROLLD, S_GPSTIME);
    Menuconfig_onoff(PITCHD, S_GPSTZAHEAD);
    MAX7456_WriteString(itoa(Settings[S_GPSTZ], screenBuffer, 10), YAWD);
  }
#endif
#ifdef MENU_ALARMS
  if (configPage == MENU_ALARMS) {
    MenuBuffer[0] = Settings[S_DISTANCE_ALARM];
    MenuBuffer[1] = Settings[S_ALTITUDE_ALARM];
    MenuBuffer[2] = Settings[S_SPEED_ALARM];
    MenuBuffer[3] = Settings[S_FLYTIME_ALARM];
    MenuBuffer[4] = Settings[S_AMPER_HOUR_ALARM];
    MenuBuffer[5] = Settings[S_AMPERAGE_ALARM];
    for (uint8_t X = 0; X <= 6; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_alarm_item[X])), ROLLT + (X * 30));
      if (X <= 5)
        MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
    Menuconfig_onoff(MAGD, S_ALARMS_TEXT);
  }
#endif

#ifdef MENU_PROFILE
  if (configPage == MENU_PROFILE) {
#ifdef CORRECTLOOPTIME
#undef MENU10MAX
#define MENU10MAX 2
#else
#undef MENU10MAX
#define MENU10MAX 1
#endif
    MenuBuffer[0] = FCProfile;
    MenuBuffer[1] = PIDController;
    MenuBuffer[2] = LoopTime;
    for (uint8_t X = 0; X <= MENU10MAX; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_profile[X])), ROLLT + (X * 30));
      MAX7456_WriteString(itoa(MenuBuffer[X], screenBuffer, 10), 113 + (30 * X));
    }
  }
#endif

#ifdef MENU_INFO
  if (configPage == MENU_INFO) {
    for (uint8_t X = 0; X <= 4; X++) {
      MAX7456_WriteString_P(PGMSTR(&(menu_info[X])), ROLLT + (X * 30));
    }
  }
#endif

#ifdef USE_MENU_VTX
  if (configPage == MENU_VTX) {

    for (uint8_t X = 0; X <= 2; X++)
      MAX7456_WriteString_P(PGMSTR(&(menu_vtx[X])), ROLLT + (X * 30));
    // Power
    MAX7456_WriteString_P(PGMSTR(&(vtxPowerNames[Settings[S_VTX_POWER]])) , ROLLI);
    // Band
    MAX7456_WriteString_P(PGMSTR(&(vtxBandNames[Settings[S_VTX_BAND]])) , PITCHI);
    // Channel
    MAX7456_WriteString(itoa(Settings[S_VTX_CHANNEL] + 1, screenBuffer, 10), YAWI);
    // Set
    MAX7456_WriteString("SET", ALTI);

#ifdef DISPLAY_VTX_INFO
    updateVtxStatus();
#endif //DISPLAY_VTX_INFO

  }
#endif

#ifdef MENU_DEBUG
  if (configPage == MENU_DEBUG) {
    displayDebug();
  }
#endif

  if (configPage > MAXPAGE)configPage = MINPAGE;
  displayCursor();
}


#ifdef USE_MENU_VTX
void updateVtxStatus(void)
{
  if (configPage == MENU_VTX) {
    char tmp[2];
    tmp[0] = (char)pgm_read_byte(&vtxBandLetters[vtxBand]);
    tmp[1] = 0;
    MAX7456_WriteString(tmp, 12 + 30);
    MAX7456_WriteString(itoa(vtxChannel + 1, screenBuffer, 10), 14 + 30);
    MAX7456_WriteString_P(PGMSTR(&(vtxPowerNames[vtxPower])), 16 + 30);
    MAX7456_WriteString(itoa((uint16_t)pgm_read_word(&vtx_frequencies[vtxBand][vtxChannel]), screenBuffer, 10), 20 + 30);
  }
}
#endif

#ifdef MENU_KISS
void displaySubMenuConfig(void) {
  if (configPage == MENU_KISS) {
    MAX7456_WriteString_P(PGMSTR(&(menutitle_item[configPage])), 38);
    if (subConfigPage < 0) {
      MAX7456_WriteString_P(configMsgEXT, SAVEP);    //EXIT
      if (!previousarmedstatus) {
        MAX7456_WriteString_P(configMsgSAVE, SAVEP + 6); //SaveExit
        MAX7456_WriteString_P(configMsgPGS, SAVEP + 16); //<Page>
      }
    } else {
      MAX7456_WriteString_P(configMsgBack, SAVEP);    //BACK
      MAX7456_WriteString_P(configMsgSAVEAndBack, SAVEP + 16); //Save+Back
    }
    switch (subConfigPage)
    {
    case SUBMENU_KISS_PID:
      for (uint8_t X = 0; X < PIDITEMS; X++)
      {
        MAX7456_WriteString_P(PGMSTR(&(menu_pid[X])), ROLLT + (X * 60));
	    }
      for (uint8_t Y = 0; Y < PIDITEMS; Y++)
      {
        if (Y == 5) Y = 7;
         uint8_t X = Y;
        if (Y > 6) {
         X = X - 2;
        }
        ItoaPadded(pidP[Y], screenBuffer, 5,3);
        MAX7456_WriteString(screenBuffer, ROLLP + (X * 60)-3);
        ItoaPadded(pidI[Y], screenBuffer, 6,3);
        MAX7456_WriteString(screenBuffer, ROLLI + (X * 60)-3);
        ItoaPadded(pidD[Y], screenBuffer, 5,3);
        MAX7456_WriteString(screenBuffer, ROLLD + (X * 60)-2);
      }

      MAX7456_WriteString("P", 71);
      MAX7456_WriteString("I", 77);
      MAX7456_WriteString("D", 83);
      break;
    case SUBMENU_KISS_RATE:
      // The rows of rates correspond to the pid
      for (uint8_t X = 0; X < PIDITEMS; X++)
      {
        MAX7456_WriteString_P(PGMSTR(&(menu_pid[X])), ROLLT + (X * 60));
      }
      for (uint8_t Y = 0; Y < PIDITEMS; Y++)
      {
        if (Y == 5) Y = 7;
        uint8_t X = Y;
        if (Y > 6) {
          X = X - 2;
        }

        ItoaPadded(rateRC[Y], screenBuffer, 5,3);
        MAX7456_WriteString(screenBuffer, ROLLRC + (X * 60)-3);
        ItoaPadded(rateRate[Y], screenBuffer, 5,3);
        MAX7456_WriteString(screenBuffer, ROLLRATE + (X * 60)-3);
        ItoaPadded(rateCurve[Y], screenBuffer, 5,3);
        MAX7456_WriteString(screenBuffer, ROLLCURVE + (X * 60)-2);
      }

      MAX7456_WriteString_P(menu_kiss_rates[0], 69);
      MAX7456_WriteString_P(menu_kiss_rates[1], 75);
      MAX7456_WriteString_P(menu_kiss_rates[2], 82);
      break;
    case SUBMENU_KISS_NOTCH_FILTERS:
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_notch_filters[0])), 68);
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_notch_filters[1])), 75);
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_notch_filters[2])), 82);
      // ROLL
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_notch_filters[3])), 92);
      MAX7456_WriteString_P(PGMSTR(&(menu_on_off[nfRollEnable])), 92 + 7);
      MAX7456_WriteString(itoa(nfRollCenter, screenBuffer, 10), 92 + 14);
      MAX7456_WriteString(itoa(nfRollCutoff, screenBuffer, 10), 92 + 21);
      // PITCH
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_notch_filters[4])), 122);
      MAX7456_WriteString_P(PGMSTR(&(menu_on_off[nfPitchEnable])), 122 + 7);
      MAX7456_WriteString(itoa(nfPitchCenter, screenBuffer, 10), 122 + 14);
      MAX7456_WriteString(itoa(nfPitchCutoff, screenBuffer, 10), 122 + 21);

      break;
    case SUBMENU_KISS_LPF:
      for (uint8_t Y = 0; Y < 4; Y++) {
        MAX7456_WriteString_P(PGMSTR(&(menu_kiss_lpf[Y])), (Y + 3) * 30 + 2);
      }
      
      MAX7456_WriteString(itoa(yawCFilter, screenBuffer, 10), 90 + 19);
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_lpf[rpLPF + 4])), 120 + 19);
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_lpf[yawLPF + 4])), 150 + 19);
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_lpf[dtermLPF + 4])), 180 + 19);
      break;
    case SUBMENU_KISS_VTX:
      for (uint8_t Y = 0; Y < 5; Y++) {
        MAX7456_WriteString_P(PGMSTR(&(menu_kiss_vtx[Y])), (Y + 3) * 30 + 2);
      }
      
      MAX7456_WriteString_P(PGMSTR(&(menu_kiss_vtx_type[vtxType])), 90 + 13);
      MAX7456_WriteString(itoa(vtxLowPower, screenBuffer, 10), 120 + 13);
      MAX7456_WriteString(itoa(vtxMaxPower, screenBuffer, 10), 150 + 13);
      MAX7456_WriteString_P(PGMSTR(&(vtxBandLetters[vtxBand])), 180 + 13);
      MAX7456_WriteString(itoa(vtxChannel, screenBuffer, 10), 210 + 13);
      break;
    default:
      for (uint8_t subMenu = 0; subMenu < SUBMENU_KISS_SIZE; subMenu++) {
        MAX7456_WriteString_P(menu_kiss[subMenu], KISS_LINEFIRSTSUBMENU + (subMenu * 30));
      }
      break;
    }

    displayCursor();
  }
}
#endif


void displayDebug(void)
{
#if defined (DEBUG)

  switch (DEBUG) {
    case 4:
      break;
    default:
      if (screenlayout != DEBUG)
        return;
      break;
  }

  for (uint16_t xx = 0; xx < MAX_screen_size; ++xx) { // clear screen
    screen[xx] = ' ';
  }
#endif //(DEBUG)||defined (DEBUGMW)||defined (FORCEDEBUG)

#ifdef DEBUGDPOSMENU
  MAX7456_WriteString("DEBUGGING INFORMATION", DEBUGDPOSMENU);
#endif
#ifdef DEBUGDPOSRCDATA
  MAX7456_WriteString("RC", DEBUGDPOSRCDATA);
  for (uint8_t X = 1; X <= 8; X++) {
    itoa(MwRcData[X], screenBuffer, 10);
    MAX7456_WriteString(screenBuffer, DEBUGDPOSRCDATA + (X * LINE));
  }
#endif
#ifdef DEBUGDPOSANAL
  MAX7456_WriteString("ANAL", DEBUGDPOSANAL - 30);
  for (uint8_t sensor = 0; sensor < SENSORTOTAL; sensor++) {
    itoa(sensorfilter[sensor][SENSORFILTERSIZE], screenBuffer, 10);
    MAX7456_WriteString(screenBuffer, DEBUGDPOSANAL + (sensor * LINE));
  }
#endif
#ifdef DEBUGDPOSVAL
  //  MAX7456_WriteString("DEBUG",DEBUGDPOSVAL-30);
  for (uint8_t X = 0; X < 4; X++) {
    ItoaPadded(debug[X], screenBuffer + 2, 7, 0);
    screenBuffer[0] = 0x44;
    screenBuffer[1] = 0x30 + X;
    screenBuffer[2] = 0X3A;
    MAX7456_WriteString(screenBuffer, DEBUGDPOSVAL + (X * LINE));
  }
#endif
#ifdef DEBUGDPOSPWM
  MAX7456_WriteString("PWM", DEBUGDPOSPWM);
  itoa(pwmval1, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSPWM + (1 * LINE));
  itoa(pwmval2, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSPWM + (2 * LINE));
#endif
#ifdef DEBUGDPOSLOOP
  MAX7456_WriteString("LOOP", DEBUGDPOSLOOP);
  itoa(framerate, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSLOOP + 5);
#endif
#ifdef DEBUGDPOSSAT
  MAX7456_WriteString("SAT", DEBUGDPOSSAT);
  itoa(GPS_numSat, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSSAT + 5);
#endif
#ifdef DEBUGDPOSARMED
  MAX7456_WriteString("ARM", DEBUGDPOSARMED);
  itoa(armed, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSARMED + 5);
#endif
#ifdef DEBUGDPOSPACKET
  MAX7456_WriteString("PKT", DEBUGDPOSPACKET);
  itoa(packetrate, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSPACKET + 5);
#endif
#ifdef DEBUGDPOSRX
  MAX7456_WriteString("RX", DEBUGDPOSRX);
  itoa(serialrxrate, screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSRX + 5);
#endif
#ifdef DEBUGDPOSMSPID
  MAX7456_WriteString("MSP ID", DEBUGDPOSMSPID);
  for (uint8_t id_row = 0; id_row <= 6; id_row++) {
    for (uint8_t id_col = 0; id_col <= 4; id_col++) {
      itoa(boxidarray[(id_row * 5) + (id_col)], screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, DEBUGDPOSMSPID + 5 + LINE + (id_row * LINE) + (id_col * 3));
    }
  }
  for (uint8_t id_bit_2 = 0; id_bit_2 <= 1; id_bit_2++) {
    itoa(id_bit_2, screenBuffer, 10);
    MAX7456_WriteString(screenBuffer, DEBUGDPOSMSPID + 3 + (LINE * 9 + (id_bit_2 * LINE)));
    for (uint8_t id_bit = 0; id_bit <= 16; id_bit++) {
      uint8_t active = (MwSensorActive & (1UL << (id_bit + (id_bit_2 * 16)))) != 0;
      itoa(active, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, DEBUGDPOSMSPID + 5 + (LINE * 9 + (id_bit_2 * LINE)) + (id_bit));
    }
  }
#endif
#if defined (MEMCHECK)
#ifdef DEBUGDPOSMEMORY
  MAX7456_WriteString("MEM", DEBUGDPOSMEMORY);
  itoa(UntouchedStack(), screenBuffer, 10);
  MAX7456_WriteString(screenBuffer, DEBUGDPOSMEMORY + 5);
#endif
#endif

}

void displayCells(void) {

#ifndef MIN_CELL
#define MIN_CELL 320
#endif
  uint16_t sum = 0;
  uint16_t low = 0;
  uint8_t cells = 0;

  for (uint8_t i = 0; i < 6; i++) {
    uint16_t volt = cell_data[i];
    if (!volt)continue; //empty cell
    ++cells;
    sum += volt;
    if (volt < low || !low)low = volt;
    if ((volt > MIN_CELL) || (timer.Blink2hz)) {
      int tempvolt = constrain(volt, 300, 415);
      tempvolt = map(tempvolt, 300, 415, 0, 14);
      screenBuffer[i] = SYM_CELL0 + tempvolt;
    }
    else screenBuffer[i] = ' ';
  }

  if (cells) {
    screenBuffer[cells] = 0;
    MAX7456_WriteString(screenBuffer, getPosition(SportPosition) + (6 - cells)); //bar chart

    ItoaPadded(low, screenBuffer + 1, 4, 2);
    screenBuffer[0] = SYM_MIN;
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;

    if ((low > MIN_CELL) || (timer.Blink2hz))
      MAX7456_WriteString(screenBuffer, getPosition(SportPosition) + LINE); //lowest

    uint16_t avg = 0;
    if (cells)avg = sum / cells;
    ItoaPadded( avg, screenBuffer + 1, 4, 2);
    screenBuffer[0] = SYM_AVG;
    screenBuffer[5] = SYM_VOLT;
    screenBuffer[6] = 0;

    if ((avg > MIN_CELL) || (timer.Blink2hz))
      MAX7456_WriteString(screenBuffer, getPosition(SportPosition) + (2 * LINE)); //average
  }
}


void mapmode(void) {

#ifdef MAPMODE

  int mapstart = 0;
  int mapend = 0;

  switch (Settings[S_MAPMODE]) {
    case 1:
      mapstart = 0;
      mapend = 1;
      break;
    case 2:
      mapstart = 1;
      mapend = 2;
      break;
    case 3:
      mapstart = 0;
      mapend = 2;
      break;
    case 4:
      mapstart = 1;
      mapend = 2;
      break;
    default:
      return;
  }

  //  if(!GPS_fix)
  //    return;
  if (!fieldIsVisible(MapModePosition))
    return;

  int8_t xdir = 0;
  int8_t ydir = 0;
  int16_t targetx;
  int16_t targety;
  int16_t range = 200;
  int16_t angle;
  int16_t targetpos;
  int16_t centerpos;
  uint32_t maxdistance;
  uint8_t mapsymbolcenter;
  uint8_t mapsymboltarget;
  uint8_t mapsymbolrange;
  int16_t tmp;


  for (uint8_t maptype = mapstart; maptype < mapend; maptype++) {

    if (maptype == 1) {
      angle = (180 + 360 + GPS_directionToHome - armedangle) % 360;
    }
    else {
      angle = (360 + GPS_directionToHome - MwHeading) % 360;
    }

    tmp = angle / 90;
    switch (tmp) {
      case 0:
        xdir = +1;
        ydir = -1;
        break;
      case 1:
        xdir = +1;
        ydir = +1;
        angle = 180 - angle;
        break;
      case 2:
        xdir = -1;
        ydir = +1;
        angle = angle - 180;
        break;
      case 3:
        xdir = -1;
        ydir = -1;
        angle = 360 - angle;
        break;
    }

    float rad  = angle * PI / 180;    // convert to radians
    uint16_t x = (uint16_t)(GPS_distanceToHome * sin(rad));
    uint16_t y = (uint16_t)(GPS_distanceToHome * cos(rad));

    if (y > x) maxdistance = y;
    else maxdistance = x;
    if (maxdistance < 100) {
      range = 100;
      mapsymbolrange = SYM_RANGE_100;
    }
    else if (maxdistance < 500) {
      range = 500;
      mapsymbolrange = SYM_RANGE_500;
    }
    else if (maxdistance < 2500) {
      range = 2500;
      mapsymbolrange = SYM_RANGE_2500;
    }
    else {
      range = maxdistance;
      mapsymbolrange = SYM_RANGE_MAX;
    }

    targetx = xdir * map(x, 0, range, 0, 16);
    targety = ydir * map(y, 0, range, 0, 15);

    if (maxdistance < 20) {
      targetx = 0;
      targety = 0;
    }

    centerpos = getPosition(MapCenterPosition);
    targetpos = centerpos + (targetx / 2) + (LINE * (targety / 3));

    if (maptype == 1) {
      mapsymbolcenter = SYM_HOME;
      mapsymboltarget = SYM_AIRCRAFT;
    }
    else {
      mapsymbolcenter = SYM_AIRCRAFT;
      mapsymboltarget = SYM_HOME;
    }

    int8_t symx = (int8_t)abs(targetx) % 2;
    int8_t symy = (int8_t)abs(targety) % 3;
    if (ydir == 1)
      symy = 2 - symy;
    if (xdir == -1)
      symx = 1 - symx;
    if (abs(targety) < 3)
      symy = 1 - ydir;
    if (abs(targetx) < 2) {
      if (targetx < 0)
        symx = 0;
      else
        symx = 1;
    }

    if (maptype == 0)
      mapsymboltarget = 0xD6;
    else
      mapsymboltarget = 0xD0;

    mapsymboltarget = uint8_t( mapsymboltarget + symy + (symx * 3));


    if (Settings[S_MAPMODE] == 4) {
      tmp = (360 + 382 + MwHeading - armedangle) % 360 / 45;
      mapsymboltarget = SYM_DIRECTION + tmp;
    }

    screenBuffer[0] = mapsymbolrange;
    screenBuffer[1] = 0;
    MAX7456_WriteString(screenBuffer, getPosition(MapModePosition));

    screenBuffer[0] = mapsymboltarget;
    screenBuffer[1] = 0;
    MAX7456_WriteString(screenBuffer, targetpos);

    screenBuffer[0] = mapsymbolcenter;
    screenBuffer[1] = 0;
    MAX7456_WriteString(screenBuffer, centerpos);
  }

#endif
}


#if defined (USEGLIDESCOPE) || defined (USEGLIDEANGLE)
void displayfwglidescope(void) {
  if (!fieldIsVisible(glidescopePosition))
    return;
  int8_t GS_deviation_scale   = 0;
  int16_t gs_angle = 0;
  int8_t varsymbol = 3;

  if (GPS_distanceToHome > 0) { //watch div 0!!
    gs_angle          = (double)570 * atan2(MwAltitude / 100, GPS_distanceToHome);
    int16_t GS_target_delta   = GLIDEANGLE - gs_angle;
    GS_target_delta           = constrain(GS_target_delta, -GLIDEWINDOW, GLIDEWINDOW);
    GS_deviation_scale        = map(GS_target_delta, -GLIDEWINDOW, GLIDEWINDOW, 0, 8);
  }

#if defined USEGLIDEANGLE
  gs_angle /= 10;
  constrain(gs_angle, -90, 90);
  if (gs_angle < USEGLIDEANGLE)
    displayItem(glidescopePosition, gs_angle / 10, 0, SYM_GA, 0 );
#else
  int8_t varline              = (GS_deviation_scale / 3) - 1;
  varsymbol            = GS_deviation_scale % 3;
  uint16_t position = getPosition(glidescopePosition);
  for (int8_t X = -1; X <= 1; X++) {
    screen[position + (X * LINE)] =  SYM_GLIDESCOPE;
  }
  screen[position + (varline * LINE)] = SYM_GLIDESCOPE + 3 - varsymbol;
#endif
}


#endif //USEGLIDESCOPE


void Menuconfig_onoff(uint16_t pos, uint8_t setting) {
  MAX7456_WriteString_P(PGMSTR(&(menu_on_off[(Settings[setting])])), pos);
}


void displayArmed(void)
{
  if (!fieldIsVisible(motorArmedPosition)) {
    return;
  }
  if (!Settings[S_ALARMS_TEXT]) {
    return;
  }


#ifdef HAS_ALARMS
  if (alarmState != ALARM_OK) {
    // There's an alarm, let it have this space.
    return;
  }
#endif

  if (!armed) {
#ifndef GPSOSD
    alarms.active |= (1 << 1);
#endif
    timer.armedstatus = 4;
  }
  else {
    if (timer.armedstatus > 0) {
#ifdef GPSTIME
      if (Settings[S_GPSTIME] > 0) {
        displayDateTime();
      }
#endif //GPSTIME      
      if (timer.Blink10hz)
        return;
      alarms.active |= (1 << 2);
      armedtimer--;
    }
    else {
      alarms.active |= B00000001;
    }
  }

#if defined GPSOSD
  if (GPSOSD_state == 1) {
    alarms.active |= (1 << 3) | (1 << 0);
  }
  if ((GPSOSD_state == 2) && (GPS_numSat >= MINSATFIX)) {
    alarms.active |= (1 << 1);
  }
#endif

#ifdef ENABLEDEBUGTEXT
  if (debugtext == 1)
    alarms.active |= (1 << 7);
#endif

#ifdef ALARM_VOLTAGE
  if (voltage < voltageWarning)
    alarms.active |= (1 << 6);
#endif

  if (MwSensorPresent & GPSSENSOR) {

#ifdef ALARM_SATS
    if (GPS_numSat < MINSATFIX) { // below minimum preferred value
      alarms.active |= (1 << 5);
    }
#endif //ALARM_SATS

#ifdef ALARM_GPS
    if (timer.GPS_active == 0) {
      alarms.active |= (1 << 4);
    }
#endif //ALARM_GPS

#ifdef ALARM_MSP
    if (timer.MSP_active == 0) {
      alarms.active |= (1 << 3);
      alarms.active &= B11001111; // No need for sats/gps warning
    }
#endif //ALARM_MSP
  }

#ifndef ALARM_ARMED
  alarms.active &= B11111000;
#endif //ALARM_ARMED

  if (alarms.queue == 0)
    alarms.queue = alarms.active;
  uint8_t queueindex = alarms.queue & -alarms.queue;
  if (millis() > 500 + timer.alarms) {
    if (alarms.queue > 0)
      alarms.queue &= ~queueindex;
    timer.alarms = millis();
  }

  uint8_t queueindexbit = 0;
  for (uint8_t i = 0; i <= 7; i++) {
    if  (queueindex & (1 << i))
      queueindexbit = i;
  }
  if (alarms.active > 1) {
    MAX7456_WriteString_P(PGMSTR(&(alarm_text[queueindexbit])), getPosition(motorArmedPosition));
  }
}


void displayForcedCrosshair() {
  uint16_t position = getPosition(horizonPosition);
  screen[position - 1] = SYM_AH_CENTER_LINE;
  screen[position + 1] = SYM_AH_CENTER_LINE_RIGHT;
  screen[position] =   SYM_AH_CENTER;
}


#ifdef HAS_ALARMS
void displayAlarms() {
  if (alarmState == ALARM_OK) {
    return;
  }
  if (alarmState == ALARM_CRIT || alarmState == ALARM_ERROR || timer.Blink2hz) {
    MAX7456_WriteString((const char*)alarmMsg, getPosition(motorArmedPosition));
  }
}
#endif

void formatDistance(int32_t t_d2f, uint8_t t_units, uint8_t t_type, uint8_t t_icon ) {
  //void formatDistance(int32_t t_d2f, uint8_t t_units, uint8_t t_type) {
  // t_d2f = integer to format into string
  // t_type 0=alt, 2=dist , 4=LD alt, 6=LD dist NOTE DO NOT SEND USING LD
  // t_units 0=none, 1 show units symbol at end
  // t_licon 0=none, other = hex char of lead icon
  int32_t tmp;
  uint8_t xx = 0;
  if (t_icon > 1) {
    xx = 1;
    screenBuffer[0] = t_icon;
  }
#ifdef LONG_RANGE_DISPLAY
  if (t_d2f > 9999) {
    if (Settings[S_UNITSYSTEM]) {
      tmp = (264 + (t_d2f)) / 528;
    }
    else {
      tmp = (50 + t_d2f) / 100;
    }
    itoa(tmp, screenBuffer + xx, 10);
    xx = FindNull();
    screenBuffer[xx] = screenBuffer[xx - 1];
    screenBuffer[xx - 1] = DECIMAL;
    xx++;
    screenBuffer[xx] = 0;
    t_type += 4; // to make LD font
  }
  else {
    itoa(t_d2f, screenBuffer + xx, 10);
  }
#else
  itoa(t_d2f, screenBuffer + xx, 10);
#endif
  if (t_units == 1) {
    xx = FindNull();
    screenBuffer[xx] = UnitsIcon[Settings[S_UNITSYSTEM] + t_type];
    xx++;
    screenBuffer[xx] = 0;
  }
  else {
  }
}


void displayItem(uint16_t t_position, int16_t t_value, uint8_t t_leadicon, uint8_t t_trailicon, uint8_t t_pdec )
{
  /*
   *  t_position  = screen position
   *  t_value     = numerical value
   *  t_leadicon  = hex position of leading character. 0 = no leading character.
   *  t_trailicon = hex position of trailing character. 0 = no trailing character.
   *  t_psize     = number of characters to right justify value into. 0 = left justified with no padding.
   *  t_pdec      = decimal precision or char position of decimal point within right justified psize. e.g for 16.1 use t_psize=4,t_pdec=3
  */

  uint8_t t_offset = 0;
  uint8_t t_decsize = 0;
  if (!fieldIsVisible(t_position))
    return;
  if (t_leadicon > 0) { // has a lead icon
    screenBuffer[0] = t_leadicon;
    t_offset = 1;
  }
  else {
    t_offset = 0;
  }
  itoa(t_value, screenBuffer + t_offset, 10);
  if (t_pdec > 0) {
    t_pdec++;
    t_decsize = FindNull();
    uint8_t singlevalue = 2 + t_offset;
    if (t_decsize == 1 + t_offset ) {
      screenBuffer[singlevalue] = 0;
      screenBuffer[singlevalue - 1] = screenBuffer[t_offset];
      screenBuffer[singlevalue - 2] = 0x30;
      t_decsize++;
    }
    while (t_pdec != 0) {
      if (t_decsize > 0)
        screenBuffer[t_decsize + 1] = screenBuffer[t_decsize];
      t_decsize--;
      t_pdec--;
    }
    screenBuffer[t_decsize + 1] = 0x2E;
  }
  if (t_trailicon > 0) { // has a trailing icon
    t_offset = FindNull();
    screenBuffer[t_offset++] = t_trailicon;
    screenBuffer[t_offset] = 0;
  }
  MAX7456_WriteString(screenBuffer, getPosition(t_position));
}


void displayDateTime(void)
{
  formatDateTime(datetime.hours, datetime.minutes, datetime.seconds, ':', 1);
  MAX7456_WriteString(screenBuffer, getPosition(GPS_timePosition));
  formatDateTime(datetime.day, datetime.month, datetime.year, '/', 1);
  MAX7456_WriteString(screenBuffer, LINE + getPosition(GPS_timePosition));
}

void formatDateTime(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t seperator, uint8_t dtsize) {
  ItoaPadded(digit1, screenBuffer, 2, 0);
  screenBuffer[2] = seperator;
  ItoaPadded(digit2, screenBuffer + 3, 2, 0);
  if (dtsize > 0) {
    screenBuffer[5] = seperator;
    ItoaPadded(digit3, screenBuffer + 6, 2, 0);
    screenBuffer[8] = 0;
  }
  else {
    screenBuffer[5] = 0;
  }
  for (int i = 0; i < 10; ++i)
    if (screenBuffer[i] == 0x20) screenBuffer[i] = 0x30; // replace leading 0
}

void setDateTime(void)
{
  if (GPS_numSat >= MINSATFIX) {
    datetime.unixtime = GPS_time;
  }
}


void updateDateTime(uint32_t t_time)
{
  //datetime.unixtime=1527712200; // 30/05/2018 @ 20:30 UTC for testing

  t_time -= 946684800;
  uint8_t  t_year=0;
  uint8_t  t_month=0;
  uint8_t  t_monthsize=0;
  uint32_t t_days=0;
  static const uint8_t daysinmonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define LEAP_YEAR(Y) !(((Y))%4) 
#ifndef DATEFORMAT_UTC
  int32_t t_tzhours = 3600 * (128 - Settings[S_GPSTZ]);
  t_time = t_time - t_tzhours;
#endif // DATEFORMAT_UTC 

  datetime.seconds = uint32_t (t_time % 60); t_time /= 60;
  datetime.minutes = uint32_t (t_time % 60); t_time /= 60;
  datetime.hours = uint32_t (t_time % 24);   t_time /= 24;

  while ((unsigned)(t_days += (LEAP_YEAR(t_year) ? 366 : 365)) <= t_time) {
    t_year++;
  }
  t_days -= LEAP_YEAR(t_year) ? 366 : 365;
  t_time  -= t_days;

  t_days = 0;
  t_month = 0;
  t_monthsize = 0;
  for (t_month = 0; t_month < 12; t_month++) {
    if (t_month == 1) { // february
      if (LEAP_YEAR(t_year)) {
        t_monthsize = 29;
      } else {
        t_monthsize = 28;
      }
    } else {
      t_monthsize = daysinmonth[t_month];
    }

    if (t_time >= t_monthsize) {
      t_time -= t_monthsize;
    } else {
      break;
    }
  }
#ifdef  DATEFORMAT_US
  datetime.day   = t_month + 1;
  datetime.month = t_time + 1;
#else
  datetime.day   = t_time + 1;
  datetime.month = t_month + 1;
#endif
  datetime.year  = t_year;
}

void displayGPSPosition(void)
{
  if (!fieldIsVisible(MwGPSLatPositionTop)){
    return;
  }
  uint16_t t_position;
  t_position = getPosition(MwGPSLatPositionTop);
  FormatGPSCoord(t_position,GPS_latitude, 0) ;
  t_position = getPosition(MwGPSLonPositionTop);
  FormatGPSCoord(t_position,GPS_longitude, 2) ;
}
