#include "platform.h"

MSPClass::MSPClass(HardwareSerial &serial) {
  _serial = &serial;
}


uint32_t MSPClass::read32() {
  uint32_t t = (uint32_t) read16();
  t |= (uint32_t) read16() << 16;
  return t;
}

uint16_t MSPClass::read16() {
  uint16_t t = read8();
  t |= (uint16_t) read8() << 8;
  return t;
}

uint8_t MSPClass::read8()  {
  return _serialBuffer[_readIndex++];
}


void MSPClass::write8(uint8_t t){
  _serial->write(t);
  _txChecksum ^= t;
}

void MSPClass::write16(uint16_t t){
  write8(t);
  write8(t>>8);
}

void MSPClass::writeChecksum(){
  _serial->write(_txChecksum);
}

void MSPClass::WriteRequest(uint8_t mspCommand, uint8_t txDataSize){
  _serial->write('$');
  _serial->write('M');
  _serial->write('<');
  _txChecksum = 0;
  write8(txDataSize);
  write8(mspCommand);
  if(txDataSize == 0) {
    writeChecksum();
  }
}

void MSPClass::BuildRequests() {
  _MSPcmdsend=0;
  if(_queuedMSPRequests == 0) {
    _queuedMSPRequests = _modeMSPRequests;
  }
  uint32_t req = _queuedMSPRequests & -_queuedMSPRequests;
  _queuedMSPRequests &= ~req;
  switch(req) {
  case REQ_MSP_IDENT:
   _MSPcmdsend = MSP_IDENT;
    break;
  case REQ_MSP_STATUS:
    _MSPcmdsend = MSP_STATUS;
    break;
  case REQ_MSP_RC:
    _MSPcmdsend = MSP_RC;
    break;
  case REQ_MSP_RAW_GPS:
    _MSPcmdsend = MSP_RAW_GPS;
    break;
  case REQ_MSP_COMP_GPS:
    _MSPcmdsend = MSP_COMP_GPS;
    break;
#ifdef MSP_SPEED_LOW
  case REQ_MSP_ATTITUDE:
    _MSPcmdsend = MSP_ATTITUDE;
    break;
#endif //MSP_SPEED_LOW
  case REQ_MSP_ALTITUDE:
    _MSPcmdsend = MSP_ALTITUDE;
    break;
  case REQ_MSP_ANALOG:
    _MSPcmdsend = MSP_ANALOG;
    break;
  case REQ_MSP_MISC:
    _MSPcmdsend = MSP_MISC;
    break;
  case REQ_MSP_RC_TUNING:
    _MSPcmdsend = MSP_RC_TUNING;
    break;
  case REQ_MSP_PID_CONTROLLER:
    _MSPcmdsend = MSP_PID_CONTROLLER;
    break;
  case REQ_MSP_PID:
    _MSPcmdsend = MSP_PID;
    break;
  case REQ_MSP_LOOP_TIME:
    _MSPcmdsend = MSP_LOOP_TIME;
    break;
  case REQ_MSP_BOX:
#ifdef BOXNAMES
    _MSPcmdsend = MSP_BOXNAMES;
#else
    _MSPcmdsend = MSP_BOXIDS;
#endif
     break;
  case REQ_MSP_FONT:
     _MSPcmdsend = MSP_OSD;
     break;
#if defined DEBUGMW
  case REQ_MSP_DEBUG:
     _MSPcmdsend = MSP_DEBUG;
     break;
#endif
#if defined SPORT
  case REQ_MSP_CELLS:
     _MSPcmdsend = MSP_CELLS;
     break;
#endif
#ifdef MULTIWII_V24
  case REQ_MSP_NAV_STATUS:
       if(Sensors.IsActive(mode.gpsmission))
     _MSPcmdsend = MSP_NAV_STATUS;
  break;
#endif
#ifdef CORRECT_MSP_BF1
  case REQ_MSP_CONFIG:
     _MSPcmdsend = MSP_CONFIG;
  break;
#endif
#ifdef HAS_ALARMS
  case REQ_MSP_ALARMS:
      _MSPcmdsend = MSP_ALARMS;
  break;
#endif
  }
}

void MSPClass::SendRequests() {
  WriteRequest(_MSPcmdsend, 0);
};

void MSPClass::SetRequests() {
  if(Font.inFontMode()) {
    _modeMSPRequests = REQ_MSP_FONT;
  }
  else if(configMode) {
    _modeMSPRequests =
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
    _modeMSPRequests =
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
    if(Sensors.IsPresent(BAROMETER)){
      _modeMSPRequests |= REQ_MSP_ALTITUDE;
    }
    if(flags.ident!=1){
      _modeMSPRequests |= REQ_MSP_IDENT;
    }
    if(Sensors.IsPresent(GPSSENSOR))
      _modeMSPRequests |= REQ_MSP_RAW_GPS| REQ_MSP_COMP_GPS;
    if(mode.armed == 0)
      _modeMSPRequests |=REQ_MSP_BOX;
#if defined MULTIWII_V24
    if(Sensors.Active(mode.gpsmission))
    _modeMSPRequests |= REQ_MSP_NAV_STATUS;
#endif
  }

  if(Settings[S_MAINVOLTAGE_VBAT] ||
    Settings[S_MWRSSI]) {
    _modeMSPRequests |= REQ_MSP_ANALOG;

#ifdef USE_FC_VOLTS_CONFIG
    _modeMSPRequests |= REQ_MSP_MISC;
#endif

  }

  _queuedMSPRequests &= _modeMSPRequests;   // so we do not send requests that are not needed.
}


// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void MSPClass::serialMSPCheck()
{
  _readIndex = 0;
  #ifdef MSPACTIVECHECK
    timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
  #endif

  if (_cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();

    if (cmd == OSD_READ_CMD_EE) {
      _eeaddress = read8();
      _eeaddress = _eeaddress+read8();
      _eedata = read8();
      _settingsMode=1;
//      MSP_OSD_timer=3000+millis();
      settingsSerialRequest();
    }

    if (cmd == OSD_WRITE_CMD_EE) {
      for(uint8_t i=0; i<10; i++) {
        _eeaddress = read8();
        _eeaddress = _eeaddress+(read8()<<8);
        _eedata = read8();
        _settingsMode=1;
//        MSP_OSD_timer=3000+millis();
        Eeprom.Write(_eeaddress,_eedata);
//        if (_eeaddress==0){
          Eeprom.Write(0,MWOSDVER);
//        }
        if ((_eeaddress==EEPROM_SETTINGS+(EEPROM16_SETTINGS*2))||(_eeaddress==EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(3*2*POSITIONS_SETTINGS))){
          Eeprom.ReadSettings();
        }
      }
      _eeaddress++;
    settingswriteSerialRequest();
    }
#ifdef GUISENSORS
    if (cmd == OSD_SENSORS) {
      WriteRequest(MSP_OSD,1+10);
      write8(OSD_SENSORS);
      for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++) {
//        uint16_t sensortemp = analogRead(sensorpinarray[sensor]);
        uint16_t sensortemp = (uint16_t)Sensors.sensorfilter[sensor][SENSORFILTERSIZE]/SENSORFILTERSIZE;
        write16(sensortemp);
      }
       writeChecksum();
    }
#endif

    if(cmd == OSD_GET_FONT) {
      if(_dataSize == 5) {
        if(read16() == 7456) {
          nextCharToRequest = read8();
          lastCharToRequest = read8();
          Font.enterFontMode();
        }
      }
      else if(_dataSize == 56) {
        for(uint8_t i = 0; i < 54; i++)
          Max.fontData[i] = read8();

	uint8_t c = read8();
        Max.WriteNvm(c);
	//fontCharacterReceived(c);
        if (c==255)
          Max.Setup();
      }
    }
    if(cmd == OSD_DEFAULT) {
      Eeprom.ClearSettings();
      Eeprom.CheckSettings();
      flags.reset=1;
    }
    if(cmd == OSD_RESET) {
        flags.reset=1;
    }

  }

#ifndef GPSOSD
  if (_cmdMSP==MSP_IDENT)
  {
    flags.ident=1;
    MwVersion= read8();                             // MultiWii Firmware version
  }

  if (_cmdMSP==MSP_STATUS)
  {
    cycleTime=read16();
    I2CError=read16();
    Sensors.SetPresent(read16());
    Sensors.SetActive(read32());
    #if defined FORCESENSORS
      Sensors.Force();
    #endif
    mwosd.armed = (Sensors.IsActive(mode.armed)) != 0;
    FCProfile = read8();
    if (!configMode){
      CurrentFCProfile=FCProfile;
      PreviousFCProfile=FCProfile;
     }
  }

  if (_cmdMSP==MSP_RC)
  {
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = read16();
    handleRawRC();
  }

  if (_cmdMSP==MSP_RAW_GPS)
  {
    #ifdef GPSACTIVECHECK
     timer.GPS_active=GPSACTIVECHECK;
    #endif //GPSACTIVECHECK
    uint8_t Gps_fix_temp=read8();
    if (Gps_fix_temp){
      Gps.fix=1;
    }
    Gps.numSat = read8();
    Gps.latitude = read32();
    Gps.longitude = read32();
    Gps.altitude = read16();
    #if defined RESETGPSALTITUDEATARM
      if (!mwosd.armed){
        Gps.home_altitude=Gps.altitude;
      }
      Gps.altitude=Gps.altitude-Gps.home_altitude;
    #endif // RESETGPSALTITUDEATARM
    #if defined I2CGPS_SPEED
      Gps.speed = read16()*10;
      //gpsfix(); untested
    #else
      Gps.speed = read16();
    #endif // I2CGPS_SPEED
    Gps.ground_course = read16();
  }

  if (_cmdMSP==MSP_COMP_GPS)
  {
    Gps.distanceToHome=read16();
#ifdef I2CGPS_DISTANCE
    gpsdistancefix();
#endif

    Gps.directionToHome=read16();
#ifdef GPSTIME
    read8(); //missing
    Gps.time = read32();        //local time of coord calc - haydent
#endif
  }

#if defined MULTIWII_V24
  if (_cmdMSP==MSP_NAV_STATUS)
  {
     read8();
     read8();
     read8();
     Gps.waypoint_step=read8();
  }
#endif //MULTIWII_V24

  if (_cmdMSP==MSP_ATTITUDE)
  {
    for(uint8_t i=0;i<2;i++){
      MwAngle[i] = read16();
    }
      Gps.MwHeading = read16();
    #if defined(USEGPSHEADING)
      Gps.MwHeading = Gps.ground_course/10;
    #endif
    #ifdef HEADINGCORRECT
      if (Gps.MwHeading >= 180) Gps.MwHeading -= 360;
    #endif
  }

#if defined DEBUGMW
  if (_cmdMSP==MSP_DEBUG)
  {
    for(uint8_t i=0;i<4;i++)
      Screen.debugBuffer[i] = read16();
 }
#endif
#ifdef SPORT
  if (_cmdMSP==MSP_CELLS)
  {
    for(uint8_t i=0;i<6;i++)
      cell_data[i] = read16();
  }
#endif //SPORT
  if (_cmdMSP==MSP_ALTITUDE)
  {
    #if defined(USEGPSALTITUDE)
      Gps.MwAltitude = (int32_t)Gps.altitude*100;
      gpsvario();
    #else
      Gps.MwAltitude =read32();
      MwVario = read16();
    #endif
  }

  if (_cmdMSP==MSP_ANALOG)
  {
    MwVBat=read8();
    mwosd.pMeterSum=read16();
    rc.MwRssi = read16();
    Stats.MWAmperage = read16();
 }

#ifdef USE_FC_VOLTS_CONFIG
  if (_cmdMSP==MSP_MISC)
  {
    read16(); //ignore: midrc

    read16(); //ignore: minthrottle
    read16(); //ignore: maxthrottle
    read16(); //ignore: mincommand

    read16(); //ignore: failsafe_throttle

    read8(); //ignore: gps_type
    read8(); //ignore: gps_baudrate
    read8(); //ignore: gps_ubx_sbas

    read8(); //ignore: multiwiiCurrentMeterOutput
    read8(); //ignore: rssi_channel
    read8(); //ignore: 0

    read16(); //ignore: mag_declination

    read8(); //ignore: vbatscale
    MvVBatMinCellVoltage = read8(); //vbatmincellvoltage
    MvVBatMaxCellVoltage = read8(); //vbatmaxcellvoltage
    MvVBatWarningCellVoltage = read8(); //vbatwarningcellvoltage

  }
#endif //USE_FC_VOLTS_CONFIG

#if defined (CORRECT_MSP_BF1)
  if (_cmdMSP==MSP_CONFIG)
  {
    for(uint8_t i=0; i<25; i++) {
      _bfconfig[i]=read8();
    }
    rollRate = _bfconfig[18];
    PitchRate = _bfconfig[19];
    _modeMSPRequests &=~ REQ_MSP_CONFIG;
  }
#endif

  if (_cmdMSP==MSP_RC_TUNING)
  {
    #ifdef CORRECT_MSP_CF2
      rc.rcRate8 = read8();
      rc.rcExpo8 = read8();
      rc.rollRate = read8();
      rc.PitchRate = read8();
      rc.yawRate = read8();
      rc.dynThrPID = read8();
      rc.thrMid8 = read8();
      rc.thrExpo8 = read8();
      rc.tpa_breakpoint16 = read16();
      rc.rcYawExpo8 = read8();
      _modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #elif defined CORRECT_MSP_CF1
      rc.rcRate8 = read8();
      rc.rcExpo8 = read8();
      rc.rollRate = read8();
      rc.PitchRate = read8();
      rc.yawRate = read8();
      rc.dynThrPID = read8();
      rc.thrMid8 = read8();
      rc.thrExpo8 = read8();
      rc.tpa_breakpoint16 = read16();
      _modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #else
      rc.rcRate8 = read8();
      rc.rcExpo8 = read8();
      rc.rollPitchRate = read8();
      rc.yawRate = read8();
      rc.dynThrPID = read8();
      rc.thrMid8 = read8();
      rc.thrExpo8 = read8();
      _modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #endif
  }

  if (_cmdMSP==MSP_PID)
  {
    for(uint8_t i=0; i<PIDITEMS; i++) {
      mwosd.P8[i] = read8();
      mwosd.I8[i] = read8();
      mwosd.D8[i] = read8();
    }
    _modeMSPRequests &=~ REQ_MSP_PID;

  }

#ifdef ENABLE_MSP_SAVE_ADVANCED
  if (_cmdMSP == MSP_PID_CONTROLLER)
  {
    PIDController = read8();
    _modeMSPRequests &=~ REQ_MSP_PID_CONTROLLER;
  }

  if (_cmdMSP == MSP_LOOP_TIME)
  {
    LoopTime = read16();
    _modeMSPRequests &=~ REQ_MSP_LOOP_TIME;
  }
#endif

#ifdef HAS_ALARMS
  if (_cmdMSP == MSP_ALARMS)
  {
      alarmState = read8();
      alarmMsg[min(_dataSize-1, MAX_ALARM_LEN-1)] = 0;
      for(uint8_t i = 0; i < _dataSize-1; i++) {
          alarmMsg[min(i, MAX_ALARM_LEN-1)] = read8();
      }
  }
#endif /* HAS_ALARMS */

#ifdef BOXNAMES
  if(_cmdMSP==MSP_BOXNAMES) {
    flags.box=1;
    uint32_t bit = 1;
    uint8_t remaining = _dataSize;
    uint8_t len = 0;

    mode.armed = 0;
    mode.stable = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.llights = 0;
    mode.camstab = 0;
    mode.osd_switch = 0;
    mode.air = 0;
    mode.acroplus = 0;

    char boxname[20];

    while(remaining > 0) {
      char c = read8();
      if(c != ';') {
        boxname[len] = c;
        len++;
      }
      else {
        if(strncmp("ARM", boxname, len) == 0)
          mode.armed |= bit;
        if(strncmp("ANGLE", boxname, len) == 0)
          mode.stable |= bit;
        if(strncmp("HORIZON", boxname, len) == 0)
          mode.horizon |= bit;
        if(strncmp("MAG", boxname, len) == 0)
          mode.mag |= bit;
        if(strncmp("BARO", boxname, len) == 0)
          mode.baro |= bit;
        if(strncmp("LLIGHTS", boxname, len) == 0)
          mode.llights |= bit;
        if(strncmp("CAMSTAB", boxname, len) == 0)
          mode.camstab |= bit;
        if(strncmp("AIR MODE", boxname, len) == 0)
          mode.air |= bit;
        if(strncmp("ACRO PLUS", boxname, len) == 0)
          mode.acroplus |= bit;
        if(strncmp("GPS HOME", boxname, len) == 0)
          mode.gpshome |= bit;
        if(strncmp("GPS HOLD", boxname, len) == 0)
          mode.gpshold |= bit;
        if(strncmp("PASSTHRU", boxname, len) == 0)
          mode.passthru |= bit;
        if(strncmp("OSD SW", boxname, len) == 0)
          mode.osd_switch |= bit;

        len = 0;
        bit <<= 1L;
      }
      --remaining;
    }
  }
#else
  if(_cmdMSP==MSP_BOXIDS) {
    flags.box=1;
    uint32_t bit = 1;
    uint8_t remaining = _dataSize;

    mode.armed = 0;
    mode.stable = 0;
    mode.horizon = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.gpsmission = 0;
    mode.gpsland = 0;
    mode.llights = 0;
    mode.passthru = 0;
    mode.osd_switch = 0;
    mode.camstab = 0;
    mode.air = 0;
    mode.acroplus = 0;

    while(remaining > 0) {
      char c = read8();
      switch(c) {
      case 0:
        mode.armed |= bit;
        break;
      case 1:
        mode.stable |= bit;
        break;
      case 2:
        mode.horizon |= bit;
        break;
      case 3:
        mode.baro |= bit;
        break;
      case 5:
        mode.mag |= bit;
        break;
      case 8:
        mode.camstab |= bit;
       break;
      case 10:
        mode.gpshome |= bit;
        break;
      case 11:
        mode.gpshold |= bit;
        break;
      case 12:
        mode.passthru  |= bit;
        break;
      case 16:
        mode.llights |= bit;
        break;
      case 19:
        mode.osd_switch |= bit;
        break;
      case 20:
        mode.gpsmission |= bit;
        break;
      case 21:
        mode.gpsland |= bit;
        break;
      case 28:
        mode.air |= bit;
        break;
#if defined ACROPLUS
      case 29:
        mode.acroplus |= bit;
        break;
#endif //ACROPLUS
      }
      bit <<= 1;
      --remaining;
    }
  }
#endif
#endif // GPSOSD
}
// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------

void MSPClass::Receive(uint8_t loops)
{
  uint8_t c;
  uint8_t loopserial=0;

  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  }
  c_state = IDLE;

  if (_serial->available()) loopserial=1;
  while(loopserial==1)
  {
    c = _serial->read();

    #ifdef GPSOSD
      timer.armed = 0;
      #if defined (NAZA)
        NAZA_NewData(c);
      #else
        if (Gps.newFrame(c)) Gps.NewData();
      #endif //NAZA
    #endif //GPSOSD

    #if defined (MAVLINK)
       MAVLINK_NewData(c);
    #endif //MAVLINK

    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if (c_state == HEADER_ARROW)
    {
      if (c > SERIALBUFFERSIZE)
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        _dataSize = c;
        c_state = HEADER_SIZE;
        _rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      _cmdMSP = c;
      _rcvChecksum ^= c;
      _receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      _rcvChecksum ^= c;
      if(_receiverIndex == _dataSize) // received checksum byte
      {
        if(_rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        _serialBuffer[_receiverIndex++]=c;
    }
    if (loops==0) loopserial=0;
    if (!_serial->available()) loopserial=0;
  }
}


void MSPClass::serialMenuCommon()
  {
    if((Screen.ROW==10)&&(Screen.COL==3)) {
      if (_menudir>1){
        _menudir=1;
      }
      if (_menudir<-1){
        _menudir=-1;
      }
//      constrain(_menudir,-1,1);
      Screen.configPage=Screen.configPage+_menudir;
    }
    if(Screen.configPage<MINPAGE) Screen.configPage = MAXPAGE;
    if(Screen.configPage>MAXPAGE) Screen.configPage = MINPAGE;
#ifdef MENU_PID
	if(Screen.configPage == MENU_PID) {
	  if(Screen.ROW >= 1 && Screen.ROW <= 7) {
            uint8_t MODROW=Screen.ROW-1;
            if (Screen.ROW>5){
              MODROW=Screen.ROW+1;
            }
  	    if(Screen.COL==1) mwosd.P8[MODROW]=mwosd.P8[MODROW]+_menudir;
	    if(Screen.COL==2) mwosd.I8[MODROW]=mwosd.I8[MODROW]+_menudir;
	    if(Screen.COL==3) mwosd.D8[MODROW]=mwosd.D8[MODROW]+_menudir;
	  }
	}
#endif
#ifdef MENU_RC
        #if defined CORRECT_MENU_RCT2
          if(Screen.configPage == MENU_RC && Screen.COL == 3) {
	    if(Screen.ROW==1) rc.rcRate8=rc.rcRate8+_menudir;
	    if(Screen.ROW==2) rc.rcExpo8=rc.rcExpo8+_menudir;
	    if(Screen.ROW==3) rc.rollRate=rc.rollRate+_menudir;
	    if(Screen.ROW==4) rc.PitchRate=rc.PitchRate+_menudir;
	    if(Screen.ROW==5) rc.yawRate=rc.yawRate+_menudir;
	    if(Screen.ROW==6) rc.dynThrPID=rc.dynThrPID+_menudir;
	    if(Screen.ROW==7) rc.thrMid8=rc.thrMid8+_menudir;
	    if(Screen.ROW==8) rc.thrExpo8=rc.thrExpo8+_menudir;
	    if(Screen.ROW==9) rc.tpa_breakpoint16=rc.tpa_breakpoint16+_menudir;
          }
        #elif defined CORRECT_MENU_RCT1
          if(Screen.configPage == MENU_RC && Screen.COL == 3) {
	    if(Screen.ROW==1) rc.rcRate8=rc.rcRate8+_menudir;
	    if(Screen.ROW==2) rc.rcExpo8=rc.rcExpo8+_menudir;
	    if(Screen.ROW==3) rc.rollRate=rc.rollRate+_menudir;
	    if(Screen.ROW==4) rc.PitchRate=rc.PitchRate+_menudir;
	    if(Screen.ROW==5) rc.yawRate=rc.yawRate+_menudir;
	    if(Screen.ROW==6) rc.dynThrPID=rc.dynThrPID+_menudir;
	    if(Screen.ROW==7) rc.thrMid8=rc.thrMid8+_menudir;
	    if(Screen.ROW==8) rc.thrExpo8=rc.thrExpo8+_menudir;
         }
        #else
          if(Screen.configPage == MENU_RC && Screen.COL == 3) {
	    if(Screen.ROW==1) rc.rcRate8=rc.rcRate8+_menudir;
	    if(Screen.ROW==2) rc.rcExpo8=rc.rcExpo8+_menudir;
	    if(Screen.ROW==3) rc.rollPitchRate=rc.rollPitchRate+_menudir;
	    if(Screen.ROW==4) rc.yawRate=rc.yawRate+_menudir;
	    if(Screen.ROW==5) rc.dynThrPID=rc.dynThrPID+_menudir;
	    if(Screen.ROW==6) rc.thrMid8=rc.thrMid8+_menudir;
	    if(Screen.ROW==7) rc.thrExpo8=rc.thrExpo8+_menudir;
	  }
        #endif
#endif
#ifdef MENU_VOLTAGE
	if(Screen.configPage == MENU_VOLTAGE && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_DISPLAYVOLTAGE]=!Settings[S_DISPLAYVOLTAGE];
	  if(Screen.ROW==2) Settings[S_DIVIDERRATIO]=Settings[S_DIVIDERRATIO]+_menudir;
	  if(Screen.ROW==3) Settings[S_VOLTAGEMIN]=Settings[S_VOLTAGEMIN]+_menudir;
	  if(Screen.ROW==4) Settings[S_VIDVOLTAGE]=!Settings[S_VIDVOLTAGE];
	  if(Screen.ROW==5) Settings[S_VIDDIVIDERRATIO]=Settings[S_VIDDIVIDERRATIO]+_menudir;
	  if(Screen.ROW==6) Settings[S_BATCELLS]=Settings[S_BATCELLS]+_menudir;
	  if(Screen.ROW==7) Settings[S_MAINVOLTAGE_VBAT]=!Settings[S_MAINVOLTAGE_VBAT];
	}
#endif
#ifdef MENU_RSSI
	if(Screen.configPage == MENU_RSSI && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_DISPLAYRSSI]=!Settings[S_DISPLAYRSSI];
	  if(Screen.ROW==2) timer.rssiTimer=15; // 15 secs to turn off tx anwait to read min RSSI
	  if(Screen.ROW==3) Settings[S_MWRSSI]=!Settings[S_MWRSSI];
	  if(Screen.ROW==4) Settings[S_PWMRSSI]=!Settings[S_PWMRSSI];
	  if(Screen.ROW==5) Settings16[S16_RSSIMAX]=Settings16[S16_RSSIMAX]+_menudir;
	  if(Screen.ROW==6) Settings16[S16_RSSIMIN]=Settings16[S16_RSSIMIN]+_menudir;
	}
#endif
#ifdef MENU_CURRENT
	if(Screen.configPage == MENU_CURRENT && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_AMPERAGE]=!Settings[S_AMPERAGE];
	  if(Screen.ROW==2) Settings[S_AMPER_HOUR]=!Settings[S_AMPER_HOUR];
	  if(Screen.ROW==3) Settings[S_AMPERAGE_VIRTUAL]=!Settings[S_AMPERAGE_VIRTUAL];
	  if(Screen.ROW==4) Settings16[S16_AMPDIVIDERRATIO]=Settings16[S16_AMPDIVIDERRATIO]+_menudir;
	  if(Screen.ROW==5) Settings16[S16_AMPZERO]=Settings16[S16_AMPZERO]+_menudir;
	}
#endif
#ifdef MENU_DISPLAY
	if(Screen.configPage == MENU_DISPLAY && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_DISPLAY_HORIZON_BR]=!Settings[S_DISPLAY_HORIZON_BR];
	  if(Screen.ROW==2) Settings[S_WITHDECORATION]=!Settings[S_WITHDECORATION];
	  if(Screen.ROW==3) Settings[S_SCROLLING]=!Settings[S_SCROLLING];
	  if(Screen.ROW==4) Settings[S_THROTTLEPOSITION]=!Settings[S_THROTTLEPOSITION];
	  if(Screen.ROW==5) Settings[S_COORDINATES]=!Settings[S_COORDINATES];
	  if(Screen.ROW==6) Settings[S_MODESENSOR]=!Settings[S_MODESENSOR];
	  if(Screen.ROW==7) Settings[S_GIMBAL]=!Settings[S_GIMBAL];
	  if(Screen.ROW==8) Settings[S_MAPMODE]=Settings[S_MAPMODE]+_menudir;
	}
#endif
#ifdef MENU_ADVANCED
	if(Screen.configPage == MENU_ADVANCED && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_UNITSYSTEM]=!Settings[S_UNITSYSTEM];
	  if(Screen.ROW==2) {
	    Settings[S_VIDEOSIGNALTYPE]=!Settings[S_VIDEOSIGNALTYPE];
	    Max.Setup();
	    }
	  if(Screen.ROW==3) Settings[S_VREFERENCE]=!Settings[S_VREFERENCE];
	  if(Screen.ROW==4) Settings[S_DEBUG]=!Settings[S_DEBUG];
	  if(Screen.ROW==5) timer.magCalibrationTimer=CALIBRATION_DELAY;
	  if(Screen.ROW==6) Settings[S_RCWSWITCH_CH]=Settings[S_RCWSWITCH_CH]+_menudir;	}
#endif
#ifdef MENU_GPS_TIME
	if(Screen.configPage == MENU_GPS_TIME && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_GPSTIME]=!Settings[S_GPSTIME];
	  if(Screen.ROW==2) Settings[S_GPSTZAHEAD]=!Settings[S_GPSTZAHEAD];
	  if(Screen.ROW==3) if((_menudir == 1 && Settings[S_GPSTZ] < 130) || (_menudir == -1 && Settings[S_GPSTZ] > 0))Settings[S_GPSTZ]=Settings[S_GPSTZ]+_menudir*5;
	}
#endif
#ifdef MENU_ALARMS
	if(Screen.configPage == MENU_ALARMS && Screen.COL == 3) {
	  if(Screen.ROW==1) Settings[S_DISTANCE_ALARM]=Settings[S_DISTANCE_ALARM]+_menudir;
	  if(Screen.ROW==2) Settings[S_ALTITUDE_ALARM]=Settings[S_ALTITUDE_ALARM]+_menudir;
	  if(Screen.ROW==3) Settings[S_SPEED_ALARM]=Settings[S_SPEED_ALARM]+_menudir;
	  if(Screen.ROW==4) Settings[S_FLYTIME_ALARM]=Settings[S_FLYTIME_ALARM]+_menudir;
	  if(Screen.ROW==5) Settings[S_AMPER_HOUR_ALARM]=Settings[S_AMPER_HOUR_ALARM]+_menudir;
	  if(Screen.ROW==6) Settings[S_AMPERAGE_ALARM]=Settings[S_AMPERAGE_ALARM]+_menudir;
	}
#endif
#ifdef MENU_PROFILE
	if(Screen.configPage == MENU_PROFILE && Screen.COL == 3) {
	  if(Screen.ROW==1) FCProfile=FCProfile+_menudir;
	  if(Screen.ROW==2) PIDController=PIDController+_menudir;
        #ifdef CORRECTLOOPTIME
	  if(Screen.ROW==3) LoopTime=LoopTime+_menudir;
        #endif
	};
  #ifdef ENABLE_MSP_SAVE_ADVANCED
        if (FCProfile>2)
          FCProfile=0;
        if (FCProfile!=PreviousFCProfile){
          setFCProfile();
          PreviousFCProfile=FCProfile;
        }
  #endif
#endif
	if(Screen.ROW==10) {
          Screen.previousconfigPage=Screen.configPage;
	  if(Screen.COL==1) configExit();
	  if(Screen.COL==2) configSave();
        }
}

void MSPClass::handleRawRC() {
  static uint8_t waitStick;
  static uint32_t stickTime;
  static uint32_t timeout;

  if(MwRcData[PITCHSTICK] > 1300 && MwRcData[PITCHSTICK] < 1700 &&
     MwRcData[ROLLSTICK] > 1300 && MwRcData[ROLLSTICK] < 1700 &&
     MwRcData[YAWSTICK] > 1300 && MwRcData[YAWSTICK] < 1700) {
	waitStick = 0;
        timeout = 1000;
  }
  else if(waitStick == 1) {
    if((millis() - stickTime) > timeout)
      waitStick = 0;
      timeout = 300;
  }

  if(!waitStick)
  {
    if((MwRcData[PITCHSTICK]>MAXSTICK)&&(MwRcData[YAWSTICK]>MAXSTICK)&&(MwRcData[THROTTLESTICK]>MINSTICK)){
      if (!configMode&&(timer.allSec>5)&&!mwosd.armed){
          // Enter config mode using stick combination
          waitStick =  2;	// Sticks must return to center before continue!
          configMode = 1;
          Screen.configPage = Screen.previousconfigPage;
          SetRequests();
      }
    }
    else if(configMode) {
      int8_t old_menudir=constrain(_menudir,-5,5);
      _menudir=0;
      if(mwosd.previousarmedstatus&&(MwRcData[THROTTLESTICK]>1300))
      {
	// EXIT from SHOW STATISTICS AFTER DISARM (push throttle up)
	waitStick = 2;
	configExit();
      }
#ifdef MODE1
      if(configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // MOVE RIGHT
#else
      if(configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // MOVE RIGHT
#endif
      {
	waitStick = 1;
	Screen.COL++;
	if(Screen.COL>3) Screen.COL=3;
      }
#ifdef MODE1
      else if(configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // MOVE LEFT
#else
      else if(configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // MOVE LEFT
#endif
      {
	waitStick = 1;
	Screen.COL--;
	if(Screen.COL<1) Screen.COL=1;
      }
      else if(configMode&&(MwRcData[PITCHSTICK]>MAXSTICK)) // MOVE UP
      {
	waitStick = 1;
	Screen.ROW--;
	if(Screen.ROW<1)
	  Screen.ROW=1;
        if(Screen.configPage == 0) {
          Screen.ROW=10;
        }
      }
      else if(configMode&&(MwRcData[PITCHSTICK]<MINSTICK)) // MOVE DOWN
      {
	waitStick = 1;
	Screen.ROW++;
	if(Screen.ROW>10)
	  Screen.ROW=10;
      }
#ifdef MODE1
      else if(!mwosd.previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // DECREASE
#else
      else if(!mwosd.previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // DECREASE
#endif
      {
	waitStick = 1;
        _menudir=-1+old_menudir;
        serialMenuCommon();
      }
#ifdef MODE1
      else if(!mwosd.previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // INCREASE
#else
      else if(!mwosd.previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // INCREASE
#endif
      {
	waitStick =1;
        _menudir=1+old_menudir;
        #ifdef MENU_ALARMS
	if(Screen.configPage == MENU_ALARMS && Screen.COL == 3) {
	  if(Screen.ROW==5) timer.magCalibrationTimer=0;
        }
        #endif //MENU_ALARMS
        serialMenuCommon();
      }
    }
    if(waitStick == 1)
      stickTime = millis();
  }
}

void MSPClass::ConfigEnter()
{
  timer.armed=20;
  Screen.configPage=0;
  Screen.ROW=10;
  Screen.COL=1;
  configMode=1;
  SetRequests();
}

void MSPClass::configExit()
{
  Screen.configPage=1;
  Screen.ROW=10;
  Screen.COL=3;
  configMode=0;
  //waitStick=3;
  mwosd.previousarmedstatus = 0;
  if (Settings[S_RESETSTATISTICS]){
    Stats.Reset();
  }
  #ifdef ENABLE_MSP_SAVE_ADVANCED
    if (FCProfile!=CurrentFCProfile){
      FCProfile=CurrentFCProfile;
      setFCProfile();
    }
  #endif
  SetRequests();
}

void MSPClass::configSave()
{
  CurrentFCProfile=FCProfile;

#if defined ENABLE_MSP_SAVE_ADVANCED
  WriteRequest(MSP_SET_PID_CONTROLLER, 1);
  write8(PIDController);
  writeChecksum();

  WriteRequest(MSP_SET_LOOP_TIME, 2);
  write16(LoopTime);
  writeChecksum();
#endif

  WriteRequest(MSP_SET_PID, PIDITEMS*3);
  for(uint8_t i=0; i<PIDITEMS; i++) {
    write8(mwosd.P8[i]);
    write8(mwosd.I8[i]);
    write8(mwosd.D8[i]);
  }
  writeChecksum();

#if defined CORRECT_MSP_CF2
  WriteRequest(MSP_SET_RC_TUNING,11);
  write8(rc.rcRate8);
  write8(rc.rcExpo8);
  write8(rc.rollRate);
  write8(rc.PitchRate);
  write8(rc.yawRate);
  write8(rc.dynThrPID);
  write8(rc.thrMid8);
  write8(rc.thrExpo8);
  write16(rc.tpa_breakpoint16);
  write8(rc.rcYawExpo8);
  writeChecksum();
#elif defined CORRECT_MSP_CF1
  WriteRequest(MSP_SET_RC_TUNING,10);
  write8(rc.rcRate8);
  write8(rc.rcExpo8);
  write8(rc.rollRate);
  write8(rc.PitchRate);
  write8(rc.yawRate);
  write8(rc.dynThrPID);
  write8(rc.thrMid8);
  write8(rc.thrExpo8);
  write16(rc.tpa_breakpoint16);
  writeChecksum();
#else
  WriteRequest(MSP_SET_RC_TUNING,7);
  write8(rc.rcRate8);
  write8(rc.rcExpo8);
  write8(rc.rollPitchRate);
  write8(rc.yawRate);
  write8(rc.dynThrPID);
  write8(rc.thrMid8);
  write8(rc.thrExpo8);
  writeChecksum();
 #endif

#if defined CORRECT_MSP_BF1
  WriteRequest(MSP_SET_CONFIG,25);
  _bfconfig[18] =rollRate;
  _bfconfig[19] =PitchRate;
  for(uint8_t i=0; i<25; i++) {
    write8(_bfconfig[i]);
  }
  writeChecksum();
#endif

  Eeprom.WriteSettings();
  WriteRequest(MSP_EEPROM_WRITE,0);
  configExit();
}

// TODO: $$$ unused?
/*
void MSPClass::fontSerialRequest() {
  WriteRequest(MSP_OSD,3);
  write8(OSD_GET_FONT);
  write16(Font.getNextCharToRequest());
  writeChecksum();
}
*/

void MSPClass::settingsSerialRequest() {
  WriteRequest(MSP_OSD,1+30);
  write8(OSD_READ_CMD_EE);
  for(uint8_t i=0; i<10; i++) {
    _eedata = Eeprom.Read(_eeaddress);
    write16(_eeaddress);
    write8(_eedata);
    _eeaddress++;
  }
  writeChecksum();
}

void MSPClass::settingswriteSerialRequest() {
  WriteRequest(MSP_OSD,3);
  write8(OSD_READ_CMD_EE);
  write16(_eeaddress);
  writeChecksum();
}

void MSPClass::setFCProfile()
{
  WriteRequest(MSP_SELECT_SETTING, 1);
  write8(FCProfile);
  writeChecksum();
  WriteRequest(MSP_EEPROM_WRITE, 0);
  SetRequests();
  delay(100);
}

void MSPClass::Countdown()
{
  if (timer.MSP_active>0){
    timer.MSP_active--;
  }
}

void MSPClass::ConfigExitIfArmed()
{
  if(mwosd.armed){
    mwosd.previousarmedstatus=1;
    if (configMode==1) {
      configExit();
    }
  }
}
