 
#if defined NAZA
  #define SERIALBUFFERSIZE 75
#elif defined GPSOSD
  #define SERIALBUFFERSIZE 100
#else
  #define SERIALBUFFERSIZE 150
#endif

static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;
uint8_t txChecksum;

uint32_t read32() {
  uint32_t t = read16();
  t |= (uint32_t)read16()<<16;
  return t;
}

uint16_t read16() {
  uint16_t t = read8();
  t |= (uint16_t)read8()<<8;
  return t;
}

uint8_t read8()  {
  return serialBuffer[readIndex++];
}

void mspWriteRequest(uint8_t mspCommand, uint8_t txDataSize){
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  txChecksum = 0;
  mspWrite8(txDataSize);
  mspWrite8(mspCommand);
  if(txDataSize == 0)
    mspWriteChecksum();
}

void mspWrite8(uint8_t t){
  Serial.write(t);
  txChecksum ^= t;
}

void mspWrite16(uint16_t t){
  mspWrite8(t);
  mspWrite8(t>>8);
}

void mspWriteChecksum(){
  Serial.write(txChecksum);
}

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;
  #ifdef MSPACTIVECHECK
    timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
  #endif

  if (cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();

    if (cmd == OSD_READ_CMD_EE) {
      eeaddress = read8();
      eeaddress = eeaddress+read8();
      eedata = read8();
      settingsMode=1;
      MSP_OSD_timer=3000+millis();
      settingsSerialRequest();
    }

    if (cmd == OSD_WRITE_CMD_EE) {
      for(uint8_t i=0; i<10; i++) {
        eeaddress = read8();
        eeaddress = eeaddress+(read8()<<8);
        eedata = read8();
        settingsMode=1;
        MSP_OSD_timer=3000+millis();
        EEPROM.write(eeaddress,eedata);
//        if (eeaddress==0){
          EEPROM.write(0,MWOSDVER);
//        }
        if ((eeaddress==EEPROM_SETTINGS+(EEPROM16_SETTINGS*2))||(eeaddress==EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(3*2*POSITIONS_SETTINGS))){
          readEEPROM();
        }
      }
      eeaddress++;
    settingswriteSerialRequest();
    }
#ifdef GUISENSORS
    if (cmd == OSD_SENSORS) {
      mspWriteRequest(MSP_OSD,1+10);
      mspWrite8(OSD_SENSORS);
      for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++) {
//        uint16_t sensortemp = analogRead(sensorpinarray[sensor]);
        uint16_t sensortemp = (uint16_t)sensorfilter[sensor][SENSORFILTERSIZE]/SENSORFILTERSIZE;
        mspWrite16(sensortemp);
      }
       mspWriteChecksum();
    }
#endif

    if(cmd == OSD_GET_FONT) {
      if(dataSize == 5) {
        if(read16() == 7456) {
          nextCharToRequest = read8();
          lastCharToRequest = read8();
          initFontMode();
        }
      }
      else if(dataSize == 56) {
        for(uint8_t i = 0; i < 54; i++)
          fontData[i] = read8();
      
	uint8_t c = read8();
        write_NVM(c);
	//fontCharacterReceived(c);
        if (c==255)
          MAX7456Setup();
      }
    }
    if(cmd == OSD_DEFAULT) {
      EEPROM_clear(); 
      checkEEPROM();
      flags.reset=1;
    }
    if(cmd == OSD_RESET) {
        flags.reset=1;
    }
                    
  }

#ifndef GPSOSD
  if (cmdMSP==MSP_IDENT)
  {
    flags.ident=1;
    MwVersion= read8();                             // MultiWii Firmware version
  }

  if (cmdMSP==MSP_STATUS)
  {
    cycleTime=read16();
    I2CError=read16();
    MwSensorPresent = read16();
    MwSensorActive = read32();
    #if defined FORCESENSORS
      MwSensorPresent=GPSSENSOR|BAROMETER|MAGNETOMETER|ACCELEROMETER;
    #endif  
    armed = (MwSensorActive & mode.armed) != 0;
    FCProfile = read8();
    if (!configMode){
      CurrentFCProfile=FCProfile;
      PreviousFCProfile=FCProfile;
     }
  }

  if (cmdMSP==MSP_RC)
  {
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = read16();
    handleRawRC();
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    #ifdef GPSACTIVECHECK
     timer.GPS_active=GPSACTIVECHECK;
    #endif //GPSACTIVECHECK
    uint8_t GPS_fix_temp=read8();
    if (GPS_fix_temp){
      GPS_fix=1;
    }
    GPS_numSat=read8();
    GPS_latitude = read32();
    GPS_longitude = read32();
    GPS_altitude = read16();
    #if defined RESETGPSALTITUDEATARM
      if (!armed){
        GPS_home_altitude=GPS_altitude;
      } 
      GPS_altitude=GPS_altitude-GPS_home_altitude;
    #endif // RESETGPSALTITUDEATARM  
    #if defined I2CGPS_SPEED
      GPS_speed = read16()*10;
      //gpsfix(); untested
    #else
      GPS_speed = read16();
    #endif // I2CGPS_SPEED
    GPS_ground_course = read16();
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    GPS_distanceToHome=read16();
#ifdef I2CGPS_DISTANCE
    gpsdistancefix();
#endif
    
    GPS_directionToHome=read16();
#ifdef GPSTIME
    read8(); //missing
    GPS_time = read32();        //local time of coord calc - haydent
#endif
  }

#if defined MULTIWII_V24
  if (cmdMSP==MSP_NAV_STATUS)
  {
     read8();
     read8();
     read8();
     GPS_waypoint_step=read8();
  }
#endif //MULTIWII_V24

  if (cmdMSP==MSP_ATTITUDE)
  {
    for(uint8_t i=0;i<2;i++){
      MwAngle[i] = read16();
    }
      MwHeading = read16();
    #if defined(USEGPSHEADING)
      MwHeading = GPS_ground_course/10;
    #endif
    #ifdef HEADINGCORRECT
      if (MwHeading >= 180) MwHeading -= 360;
    #endif
  }

#if defined DEBUGMW
  if (cmdMSP==MSP_DEBUG)
  {
    for(uint8_t i=0;i<4;i++)
      debug[i] = read16();
 }
#endif
#ifdef SPORT
  if (cmdMSP==MSP_CELLS)
  {
    for(uint8_t i=0;i<6;i++)
      cell_data[i] = read16();
  }
#endif //SPORT
  if (cmdMSP==MSP_ALTITUDE)
  {
    #if defined(USEGPSALTITUDE)
      MwAltitude = (int32_t)GPS_altitude*100;
      gpsvario();
    #else    
      MwAltitude =read32();
      MwVario = read16();
    #endif
  }

  if (cmdMSP==MSP_ANALOG)
  {
    MwVBat=read8();
    pMeterSum=read16();
    MwRssi = read16();
    MWAmperage = read16();
 }

#ifdef USE_FC_VOLTS_CONFIG
  if (cmdMSP==MSP_MISC)
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
  if (cmdMSP==MSP_CONFIG)
  {
    for(uint8_t i=0; i<25; i++) {
      bfconfig[i]=read8();
    }
    rollRate = bfconfig[18];
    PitchRate = bfconfig[19];
    modeMSPRequests &=~ REQ_MSP_CONFIG;    
  }
#endif  
  
  if (cmdMSP==MSP_RC_TUNING)
  {
    #ifdef CORRECT_MSP_CF2
      rcRate8 = read8();
      rcExpo8 = read8();
      rollRate = read8();
      PitchRate = read8();
      yawRate = read8();
      dynThrPID = read8();
      thrMid8 = read8();
      thrExpo8 = read8();
      tpa_breakpoint16 = read16();
      rcYawExpo8 = read8();
      modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #elif defined CORRECT_MSP_CF1
      rcRate8 = read8();
      rcExpo8 = read8();
      rollRate = read8();
      PitchRate = read8();
      yawRate = read8();
      dynThrPID = read8();
      thrMid8 = read8();
      thrExpo8 = read8();
      tpa_breakpoint16 = read16();
      modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #else
      rcRate8 = read8();
      rcExpo8 = read8();
      rollPitchRate = read8();
      yawRate = read8();
      dynThrPID = read8();
      thrMid8 = read8();
      thrExpo8 = read8();
      modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #endif
  }

  if (cmdMSP==MSP_PID)
  {
    for(uint8_t i=0; i<PIDITEMS; i++) {
      P8[i] = read8();
      I8[i] = read8();
      D8[i] = read8();
    }
    modeMSPRequests &=~ REQ_MSP_PID;

  }

#ifdef ENABLE_MSP_SAVE_ADVANCED
  if (cmdMSP == MSP_PID_CONTROLLER)
  {
    PIDController = read8();
    modeMSPRequests &=~ REQ_MSP_PID_CONTROLLER;
  }

  if (cmdMSP == MSP_LOOP_TIME)
  {
    LoopTime = read16();
    modeMSPRequests &=~ REQ_MSP_LOOP_TIME;
  }
#endif

#ifdef HAS_ALARMS
  if (cmdMSP == MSP_ALARMS)
  {
      alarmState = read8();
      alarmMsg[min(dataSize-1, MAX_ALARM_LEN-1)] = 0;
      for(uint8_t i = 0; i < dataSize-1; i++) {
          alarmMsg[min(i, MAX_ALARM_LEN-1)] = read8();
      }
  }
#endif /* HAS_ALARMS */

#ifdef BOXNAMES
  if(cmdMSP==MSP_BOXNAMES) {
    flags.box=1;
    uint32_t bit = 1;
    uint8_t remaining = dataSize;
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
  if(cmdMSP==MSP_BOXIDS) {
    flags.box=1;
    uint32_t bit = 1;
    uint8_t remaining = dataSize;

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
#if defined BETAFLIGHT
      case 29:
        mode.acroplus |= bit;
        break;
#endif //BETAFLIGHT        
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

void handleRawRC() {
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
      if (!configMode&&(allSec>5)&&!armed){
          // Enter config mode using stick combination
          waitStick =  2;	// Sticks must return to center before continue!
          configMode = 1;
          setMspRequests();
      }
    }
    else if(configMode) {
      int8_t oldmenudir=constrain(menudir,-5,5);
      menudir=0;
      if(previousarmedstatus&&(MwRcData[THROTTLESTICK]>1300))
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
	COL++;
	if(COL>3) COL=3;
      }
#ifdef MODE1
      else if(configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // MOVE LEFT
#else
      else if(configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // MOVE LEFT
#endif
      {
	waitStick = 1;
	COL--;
	if(COL<1) COL=1;
      }
      else if(configMode&&(MwRcData[PITCHSTICK]>MAXSTICK)) // MOVE UP
      {
	waitStick = 1;
	ROW--;
	if(ROW<1)
	  ROW=1;
        if(configPage == 0) {
          ROW=10;
        }
      }
      else if(configMode&&(MwRcData[PITCHSTICK]<MINSTICK)) // MOVE DOWN
      {
	waitStick = 1;
	ROW++;
	if(ROW>10)
	  ROW=10;
      }
#ifdef MODE1
      else if(!previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // DECREASE
#else
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // DECREASE
#endif
      {
	waitStick = 1;
        menudir=-1+oldmenudir;
        serialMenuCommon();  
      }
#ifdef MODE1
      else if(!previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // INCREASE
#else
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // INCREASE
#endif
      { 
	waitStick =1;
        menudir=1+oldmenudir;
        #ifdef MENU9
	if(configPage == MENU9 && COL == 3) {
	  if(ROW==5) timer.magCalibrationTimer=0;
        }
        #endif //MENU9
        serialMenuCommon();  
      }      
    }
    if(waitStick == 1)
      stickTime = millis();
  }
}

void serialMenuCommon()
  {
    if((ROW==10)&&(COL==3)) {
      if (menudir>1){
        menudir=1;
      }
      if (menudir<-1){
        menudir=-1;
      }
//      constrain(menudir,-1,1);
      configPage=configPage+menudir;
    }
    if(configPage<MINPAGE) configPage = MAXPAGE;
    if(configPage>MAXPAGE) configPage = MINPAGE;
#ifdef MENU1
	if(configPage == MENU1) {
	  if(ROW >= 1 && ROW <= 7) {
            uint8_t MODROW=ROW-1;
            if (ROW>5){
              MODROW=ROW+1;
            }
  	    if(COL==1) P8[MODROW]=P8[MODROW]+menudir;
	    if(COL==2) I8[MODROW]=I8[MODROW]+menudir;
	    if(COL==3) D8[MODROW]=D8[MODROW]+menudir;
	  }
	}
#endif
#ifdef MENU2
        #if defined CORRECT_MENU_RCT2
          if(configPage == MENU2 && COL == 3) {
	    if(ROW==1) rcRate8=rcRate8+menudir;
	    if(ROW==2) rcExpo8=rcExpo8+menudir;
	    if(ROW==3) rollRate=rollRate+menudir;
	    if(ROW==4) PitchRate=PitchRate+menudir;
	    if(ROW==5) yawRate=yawRate+menudir;
	    if(ROW==6) dynThrPID=dynThrPID+menudir;
	    if(ROW==7) thrMid8=thrMid8+menudir;
	    if(ROW==8) thrExpo8=thrExpo8+menudir;
	    if(ROW==9) tpa_breakpoint16=tpa_breakpoint16+menudir;
          }
        #elif defined CORRECT_MENU_RCT1
          if(configPage == MENU2 && COL == 3) {
	    if(ROW==1) rcRate8=rcRate8+menudir;
	    if(ROW==2) rcExpo8=rcExpo8+menudir;
	    if(ROW==3) rollRate=rollRate+menudir;
	    if(ROW==4) PitchRate=PitchRate+menudir;
	    if(ROW==5) yawRate=yawRate+menudir;
	    if(ROW==6) dynThrPID=dynThrPID+menudir;
	    if(ROW==7) thrMid8=thrMid8+menudir;
	    if(ROW==8) thrExpo8=thrExpo8+menudir;
         }
        #else
          if(configPage == MENU2 && COL == 3) {
	    if(ROW==1) rcRate8=rcRate8+menudir;
	    if(ROW==2) rcExpo8=rcExpo8+menudir;
	    if(ROW==3) rollPitchRate=rollPitchRate+menudir;
	    if(ROW==4) yawRate=yawRate+menudir;
	    if(ROW==5) dynThrPID=dynThrPID+menudir;
	    if(ROW==6) thrMid8=thrMid8+menudir;
	    if(ROW==7) thrExpo8=thrExpo8+menudir;
	  }
        #endif
#endif
#ifdef MENU3
	if(configPage == MENU3 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAYVOLTAGE]=!Settings[S_DISPLAYVOLTAGE];  
	  if(ROW==2) Settings[S_DIVIDERRATIO]=Settings[S_DIVIDERRATIO]+menudir;
	  if(ROW==3) Settings[S_VOLTAGEMIN]=Settings[S_VOLTAGEMIN]+menudir;
	  if(ROW==4) Settings[S_VIDVOLTAGE]=!Settings[S_VIDVOLTAGE];
	  if(ROW==5) Settings[S_VIDDIVIDERRATIO]=Settings[S_VIDDIVIDERRATIO]+menudir;
	  if(ROW==6) Settings[S_BATCELLS]=Settings[S_BATCELLS]+menudir;
	  if(ROW==7) Settings[S_MAINVOLTAGE_VBAT]=!Settings[S_MAINVOLTAGE_VBAT];
	}
#endif
#ifdef MENU4
	if(configPage == MENU4 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAYRSSI]=!Settings[S_DISPLAYRSSI];
	  if(ROW==2) timer.rssiTimer=15; // 15 secs to turn off tx anwait to read min RSSI
	  if(ROW==3) Settings[S_MWRSSI]=!Settings[S_MWRSSI];
	  if(ROW==4) Settings[S_PWMRSSI]=!Settings[S_PWMRSSI];
	  if(ROW==5) Settings16[S16_RSSIMAX]=Settings16[S16_RSSIMAX]+menudir;
	  if(ROW==6) Settings16[S16_RSSIMIN]=Settings16[S16_RSSIMIN]+menudir;
	}
#endif
#ifdef MENU5
	if(configPage == MENU5 && COL == 3) {
	  if(ROW==1) Settings[S_AMPERAGE]=!Settings[S_AMPERAGE];
	  if(ROW==2) Settings[S_AMPER_HOUR]=!Settings[S_AMPER_HOUR];
	  if(ROW==3) Settings[S_AMPERAGE_VIRTUAL]=!Settings[S_AMPERAGE_VIRTUAL];
	  if(ROW==4) Settings16[S16_AMPDIVIDERRATIO]=Settings16[S16_AMPDIVIDERRATIO]+menudir;
	  if(ROW==5) Settings16[S16_AMPZERO]=Settings16[S16_AMPZERO]+menudir;
	}
#endif
#ifdef MENU6
	if(configPage == MENU6 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAY_HORIZON_BR]=!Settings[S_DISPLAY_HORIZON_BR];
	  if(ROW==2) Settings[S_WITHDECORATION]=!Settings[S_WITHDECORATION];
	  if(ROW==3) Settings[S_SCROLLING]=!Settings[S_SCROLLING];
	  if(ROW==4) Settings[S_THROTTLEPOSITION]=!Settings[S_THROTTLEPOSITION];
	  if(ROW==5) Settings[S_COORDINATES]=!Settings[S_COORDINATES];
	  if(ROW==6) Settings[S_MODESENSOR]=!Settings[S_MODESENSOR];
	  if(ROW==7) Settings[S_GIMBAL]=!Settings[S_GIMBAL];
	  if(ROW==8) Settings[S_MAPMODE]=Settings[S_MAPMODE]+menudir;
	}
#endif
#ifdef MENU7
	if(configPage == MENU7 && COL == 3) {
	  if(ROW==1) Settings[S_UNITSYSTEM]=!Settings[S_UNITSYSTEM];
	  if(ROW==2) {
	    Settings[S_VIDEOSIGNALTYPE]=!Settings[S_VIDEOSIGNALTYPE];
	    MAX7456Setup();
	    }
	  if(ROW==3) Settings[S_VREFERENCE]=!Settings[S_VREFERENCE];
	  if(ROW==4) Settings[S_DEBUG]=!Settings[S_DEBUG];
	  if(ROW==5) timer.magCalibrationTimer=CALIBRATION_DELAY;
	  if(ROW==6) Settings[S_RCWSWITCH_CH]=Settings[S_RCWSWITCH_CH]+menudir;	}
#endif
#ifdef MENU8
	if(configPage == MENU8 && COL == 3) {
	  if(ROW==1) Settings[S_GPSTIME]=!Settings[S_GPSTIME];
	  if(ROW==2) Settings[S_GPSTZAHEAD]=!Settings[S_GPSTZAHEAD];
	  if(ROW==3) if((menudir == 1 && Settings[S_GPSTZ] < 130) || (menudir == -1 && Settings[S_GPSTZ] > 0))Settings[S_GPSTZ]=Settings[S_GPSTZ]+menudir*5;
	}
#endif
#ifdef MENU9
	if(configPage == MENU9 && COL == 3) {
	  if(ROW==1) Settings[S_DISTANCE_ALARM]=Settings[S_DISTANCE_ALARM]+menudir;
	  if(ROW==2) Settings[S_ALTITUDE_ALARM]=Settings[S_ALTITUDE_ALARM]+menudir;
	  if(ROW==3) Settings[S_SPEED_ALARM]=Settings[S_SPEED_ALARM]+menudir;
	  if(ROW==4) Settings[S_FLYTIME_ALARM]=Settings[S_FLYTIME_ALARM]+menudir;
	  if(ROW==5) Settings[S_AMPER_HOUR_ALARM]=Settings[S_AMPER_HOUR_ALARM]+menudir;
	  if(ROW==6) Settings[S_AMPERAGE_ALARM]=Settings[S_AMPERAGE_ALARM]+menudir;
	}
#endif
#ifdef MENU10
	if(configPage == MENU10 && COL == 3) {
	  if(ROW==1) FCProfile=FCProfile+menudir;
	  if(ROW==2) PIDController=PIDController+menudir;
        #ifdef CORRECTLOOPTIME
	  if(ROW==3) LoopTime=LoopTime+menudir;
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
	if((ROW==10)&&(COL==1)) configExit();
	if((ROW==10)&&(COL==2)) configSave();
}

void serialMSPreceive(uint8_t loops)
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

  if (Serial.available()) loopserial=1;
  while(loopserial==1)
  {
    c = Serial.read();

    #ifdef GPSOSD    
      armedtimer = 0;
      #if defined (NAZA)
        NAZA_NewData(c);
      #else
        if (GPS_newFrame(c)) GPS_NewData();  
      #endif //NAZA  
    #endif //GPSOSD   

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
        dataSize = c;
        c_state = HEADER_SIZE;
        rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      rcvChecksum ^= c;
      receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
    if (loops==0) loopserial=0;
    if (!Serial.available()) loopserial=0;
  }
}

void configExit()
{
  configPage=1;
  ROW=10;
  COL=3;
  configMode=0;
  //waitStick=3;
  previousarmedstatus = 0;
  if (Settings[S_RESETSTATISTICS]){
    trip=0;
    distanceMAX=0;
    altitudeMAX=0;
    speedMAX=0;
    ampMAX=0;
    flyingTime=0;
  }
  #ifdef ENABLE_MSP_SAVE_ADVANCED
    if (FCProfile!=CurrentFCProfile){
      FCProfile=CurrentFCProfile;
      setFCProfile();
    }
  #endif
  setMspRequests();
}

void configSave()
{
  CurrentFCProfile=FCProfile;

#if defined ENABLE_MSP_SAVE_ADVANCED
  mspWriteRequest(MSP_SET_PID_CONTROLLER, 1);
  mspWrite8(PIDController);
  mspWriteChecksum();

  mspWriteRequest(MSP_SET_LOOP_TIME, 2);
  mspWrite16(LoopTime);
  mspWriteChecksum();  
#endif

  mspWriteRequest(MSP_SET_PID, PIDITEMS*3);
  for(uint8_t i=0; i<PIDITEMS; i++) {
    mspWrite8(P8[i]);
    mspWrite8(I8[i]);
    mspWrite8(D8[i]);
  }
  mspWriteChecksum();
  
#if defined CORRECT_MSP_CF2
  mspWriteRequest(MSP_SET_RC_TUNING,11);
  mspWrite8(rcRate8);
  mspWrite8(rcExpo8);
  mspWrite8(rollRate);
  mspWrite8(PitchRate);
  mspWrite8(yawRate);
  mspWrite8(dynThrPID);
  mspWrite8(thrMid8);
  mspWrite8(thrExpo8);
  mspWrite16(tpa_breakpoint16);
  mspWrite8(rcYawExpo8);
  mspWriteChecksum();
#elif defined CORRECT_MSP_CF1
  mspWriteRequest(MSP_SET_RC_TUNING,10);
  mspWrite8(rcRate8);
  mspWrite8(rcExpo8);
  mspWrite8(rollRate);
  mspWrite8(PitchRate);
  mspWrite8(yawRate);
  mspWrite8(dynThrPID);
  mspWrite8(thrMid8);
  mspWrite8(thrExpo8);
  mspWrite16(tpa_breakpoint16);
  mspWriteChecksum();
#else
  mspWriteRequest(MSP_SET_RC_TUNING,7);
  mspWrite8(rcRate8);
  mspWrite8(rcExpo8);
  mspWrite8(rollPitchRate);
  mspWrite8(yawRate);
  mspWrite8(dynThrPID);
  mspWrite8(thrMid8);
  mspWrite8(thrExpo8);
  mspWriteChecksum();
 #endif

#if defined CORRECT_MSP_BF1
  mspWriteRequest(MSP_SET_CONFIG,25);
  bfconfig[18] =rollRate;
  bfconfig[19] =PitchRate;
  for(uint8_t i=0; i<25; i++) {
    mspWrite8(bfconfig[i]);
  }
  mspWriteChecksum();
#endif

  writeEEPROM();
  mspWriteRequest(MSP_EEPROM_WRITE,0);
  configExit();
}

void fontSerialRequest() {
  mspWriteRequest(MSP_OSD,3);
  mspWrite8(OSD_GET_FONT);
  mspWrite16(getNextCharToRequest());
  mspWriteChecksum();
}

void settingsSerialRequest() {
  mspWriteRequest(MSP_OSD,1+30);
  mspWrite8(OSD_READ_CMD_EE);
  for(uint8_t i=0; i<10; i++) {
    eedata = EEPROM.read(eeaddress);
    mspWrite16(eeaddress);
    mspWrite8(eedata);
    eeaddress++;
  }
  mspWriteChecksum();
}

void settingswriteSerialRequest() {
  mspWriteRequest(MSP_OSD,3);
  mspWrite8(OSD_READ_CMD_EE);
  mspWrite16(eeaddress);
  mspWriteChecksum();
}

void setFCProfile()
{
  mspWriteRequest(MSP_SELECT_SETTING, 1);
  mspWrite8(FCProfile);
  mspWriteChecksum();
  mspWriteRequest(MSP_EEPROM_WRITE, 0);
  setMspRequests();
  delay(100);
}

