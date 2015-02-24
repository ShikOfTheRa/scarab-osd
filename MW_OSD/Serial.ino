
#define SERIALBUFFERSIZE 250
static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;

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

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;

  if (cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();

    if(cmd == OSD_READ_CMD) {
      uint8_t txCheckSum, txSize;
      headSerialRequest();
      txCheckSum=0;
      txSize = EEPROM_SETTINGS + 1;
      Serial.write(txSize);
      txCheckSum ^= txSize;
      Serial.write(MSP_OSD);
      txCheckSum ^= MSP_OSD;
      Serial.write(cmd);
      txCheckSum ^= cmd;
      for(uint8_t i=0; i<EEPROM_SETTINGS; i++) {
        Serial.write(Settings[i]);
	txCheckSum ^= Settings[i];
      }
      Serial.write(txCheckSum);    
    }

/*
    if (cmd == OSD_SENSORS) {
      uint8_t txCheckSum, txSize;
      uint16_t osd_voltage,osd_vidvoltage,osd_RSSI,osd_amperage;
      headSerialRequest();
      txCheckSum=0;
      txSize = 8; // no. bytes to tx
      Serial.write(txSize);
      txCheckSum ^= txSize;
      Serial.write(OSD_SENSORS);
      txCheckSum ^= OSD_SENSORS;
      Serial.write(cmd);
      txCheckSum ^= cmd;

      Serial.write(osd_voltage);
      txCheckSum ^= osd_voltage;
      Serial.write(osd_vidvoltage);
      txCheckSum ^= osd_vidvoltage;
      Serial.write(osd_RSSI);
      txCheckSum ^= osd_RSSI;
      Serial.write(osd_amperage);
      txCheckSum ^= osd_amperage;

      Serial.write(txCheckSum);
    }
*/

    if (cmd == OSD_WRITE_CMD) {
      for(uint8_t en=0;en<EEPROM_SETTINGS; en++){
	uint8_t inSetting = read8();
        EEPROM.write(en,inSetting);
      }

      for(uint8_t en=0;en<(POSITIONS_SETTINGS*2*2); en++){ // 2 Huds, 2 * 8 bit settings
	uint8_t inSetting = read8();
	EEPROM.write(EEPROM_SETTINGS+en,inSetting);
      }
      EEPROM.write(0,MWOSDVER);
      readEEPROM();
      setMspRequests();
     }


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
      }
    }
    if(cmd == OSD_DEFAULT) {
        EEPROM.write(0,0);
        resetFunc();
    }
    if(cmd == OSD_RESET) {
        resetFunc();
    }
    if(cmd == OSD_SERIAL_SPEED) {
    
    }
                    
  }

#ifndef GPSOSD
  if (cmdMSP==MSP_IDENT)
  {
    MwVersion= read8();                             // MultiWii Firmware version
    modeMSPRequests &=~ REQ_MSP_IDENT;
  }

  if (cmdMSP==MSP_STATUS)
  {
    cycleTime=read16();
    I2CError=read16();
    MwSensorPresent = read16();
    MwSensorActive = read32();
    armed = (MwSensorActive & mode.armed) != 0;
  }

  if (cmdMSP==MSP_RAW_IMU)
  {
    for(uint8_t i=0;i<3;i++)
      MwAccSmooth[i] = read16();
  }

  if (cmdMSP==MSP_RC)
  {
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = read16();
    handleRawRC();
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    GPS_fix=read8();
    GPS_numSat=read8();
    GPS_latitude = read32();
    GPS_longitude = read32();
    GPS_altitude = read16();

#if defined I2CGPS_SPEED
    GPS_speed = read16()*10;
//    gpsfix(); untested
#else
    GPS_speed = read16();
#endif
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    GPS_distanceToHome=read16();
#ifdef I2CGPS_DISTANCE
    gpsdistancefix();
#endif
    
    GPS_directionToHome=read16();
    read8(); //missing
#ifdef GPSTIME
    GPS_time = read32();        //local time of coord calc - haydent
#endif
  }

  if (cmdMSP==MSP_NAV_STATUS)
  {
     read8();
     read8();
     read8();
     GPS_waypoint_step=read8();
     read8();
     read8();
     read8();
  }

  if (cmdMSP==MSP_ATTITUDE)
  {
    for(uint8_t i=0;i<2;i++)
      MwAngle[i] = read16();
    MwHeading = read16();
#ifdef HEADINGCORRECT
    if (MwHeading >= + 180) MwHeading -= 360;
#endif
    read16();
  }

#if defined DEBUGMW
  if (cmdMSP==MSP_DEBUG)
  {
    for(uint8_t i=0;i<4;i++)
      debug[i] = read16();
 }
#endif
  if (cmdMSP==MSP_CELLS)
  {
    for(uint8_t i=0;i<6;i++)
      cell_data[i] = read16();
  }
  if (cmdMSP==MSP_ALTITUDE)
  {
    MwAltitude =read32();
    MwVario = read16();
  }

  if (cmdMSP==MSP_ANALOG)
  {
    MwVBat=read8();
    pMeterSum=read16();
    MwRssi = read16();
    MWAmperage = read16();
 #ifdef AMPERAGECORRECT
    MWAmperage = MWAmperage * 10;
#endif
 }

  if (cmdMSP==MSP_RC_TUNING)
  {
    rcRate8 = read8();
    rcExpo8 = read8();
    rollPitchRate = read8();
    yawRate = read8();
    dynThrPID = read8();
    thrMid8 = read8();
    thrExpo8 = read8();
    modeMSPRequests &=~ REQ_MSP_RC_TUNING;
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

#ifdef BOXNAMES
  if(cmdMSP==MSP_BOXNAMES) {
    uint32_t bit = 1;
    uint8_t remaining = dataSize;
    uint8_t len = 0;
    char firstc, lastc;

    mode.armed = 0;
    mode.stable = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.llights = 0;
    mode.camstab = 0;
    mode.osd_switch = 0;

    while(remaining > 0) {
      char c = read8();
      if(len == 0)
        firstc = c;
      len++;
      if(c == ';') {
        // Found end of name; set bit if first and last c matches.
        if(firstc == 'A') {
          if(lastc == 'M') // "ARM;"
            mode.armed |= bit;
          if(lastc == 'E') // "ANGLE;"
            mode.stable |= bit;
        }
        if(firstc == 'H' && lastc == 'N') // "HORIZON;"
          mode.horizon |= bit;
        if(firstc == 'M' && lastc == 'G') // "MAG;"
           mode.mag |= bit;
        if(firstc == 'B' && lastc == 'O') // "BARO;"
          mode.baro |= bit;
        if(firstc == 'L' && lastc == 'S') // "LLIGHTS;"
          mode.llights |= bit;
        if(firstc == 'C' && lastc == 'B') // "CAMSTAB;"
          mode.camstab |= bit;
        if(firstc == 'G') {
          if(lastc == 'E') // "GPS HOME;"
            mode.gpshome |= bit;
          if(lastc == 'D') // "GPS HOLD;"
            mode.gpshold |= bit;
        }
        if(firstc == 'P' && lastc == 'U')  mode.passthru |= bit; // "Passthru;"
        if(firstc == 'O' && lastc == 'W') // "OSD SW;"
          mode.osd_switch |= bit;

        len = 0;
        bit <<= 1L;
      }
      lastc = c;
      --remaining;
    }
    modeMSPRequests &=~ REQ_MSP_BOX;
  }
#else  
  if(cmdMSP==MSP_BOXIDS) {
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
      }
      bit <<= 1;
      --remaining;
    }
    modeMSPRequests &=~ REQ_MSP_BOX;
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
      if ((allSec-menuSec)>5) {
        armed=0;
      }
      else if (!configMode&&(allSec>5)&&!armed){
          // Enter config mode using stick combination
          waitStick =  2;	// Sticks must return to center before continue!
          configMode = 1;
          setMspRequests();
      }
    }
    else if(configMode) {
      int8_t oldmenudir=constrain(menudir,-5,5);
      menudir=0;
      if(previousarmedstatus&&(MwRcData[THROTTLESTICK]>MINSTICK))
      {
	// EXIT from SHOW STATISTICS AFTER DISARM (push throttle up)
	waitStick = 2;
	configExit();
      }
      if(configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // MOVE RIGHT
      {
	waitStick = 1;
	COL++;
	if(COL>3) COL=3;
      }
      else if(configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // MOVE LEFT
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
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // DECREASE
      {
	waitStick = 1;
        menudir=-1+oldmenudir;
        serialMenuCommon();  
      }
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // INCREASE
      { 
	waitStick =1;
        menudir=1+oldmenudir;
	if(configPage == 9 && COL == 3) {
	  if(ROW==5) timer.magCalibrationTimer=0;
        }
        serialMenuCommon();  
      }      
    }
    else{
      menuSec=allSec;
    }
    if(waitStick == 1)
      stickTime = millis();
  }
}

void serialMenuCommon()
  {
    if((ROW==10)&&(COL==3)) {
      constrain(menudir,-1,1);
      configPage=configPage+menudir;
    }
    if(configPage<MINPAGE) configPage = MAXPAGE;
    if(configPage>MAXPAGE) configPage = MINPAGE;
#ifdef PAGE1
	if(configPage == 1) {
	  if(ROW >= 1 && ROW <= 7) {
	    if(COL==1) P8[ROW-1]=P8[ROW-1]+menudir;
	    if(COL==2) I8[ROW-1]=I8[ROW-1]+menudir;
	    if(COL==3) D8[ROW-1]=D8[ROW-1]+menudir;
	  }
	}
#endif
#ifdef PAGE2
	if(configPage == 2 && COL == 3) {
	  if(ROW==1) rcRate8=rcRate8+menudir;
	  if(ROW==2) rcExpo8=rcExpo8+menudir;
	  if(ROW==3) rollPitchRate=rollPitchRate+menudir;
	  if(ROW==4) yawRate=yawRate+menudir;
	  if(ROW==5) dynThrPID=dynThrPID+menudir;
	}
#endif
#ifdef PAGE3
	if(configPage == 3 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAYVOLTAGE]=!Settings[S_DISPLAYVOLTAGE];  
	  if(ROW==2) Settings[S_DIVIDERRATIO]=Settings[S_DIVIDERRATIO]+menudir;
	  if(ROW==3) Settings[S_VOLTAGEMIN]=Settings[S_VOLTAGEMIN]+menudir;
	  if(ROW==4) Settings[S_VIDVOLTAGE]=!Settings[S_VIDVOLTAGE];
	  if(ROW==5) Settings[S_VIDDIVIDERRATIO]=Settings[S_VIDDIVIDERRATIO]+menudir;
	  if(ROW==6) Settings[S_BATCELLS]=Settings[S_BATCELLS]+menudir;
	  if(ROW==7) Settings[S_MAINVOLTAGE_VBAT]=!Settings[S_MAINVOLTAGE_VBAT];
	}
#endif
#ifdef PAGE4
	if(configPage == 4 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAYRSSI]=!Settings[S_DISPLAYRSSI];
	  if(ROW==2) timer.rssiTimer=15; // 15 secs to turn off tx anwait to read min RSSI
	  if(ROW==3) Settings[S_MWRSSI]=!Settings[S_MWRSSI];
	  if(ROW==4) Settings[S_PWMRSSI]=!Settings[S_PWMRSSI];
	  if(ROW==5) Settings[S_RSSIMAX]=Settings[S_RSSIMAX]+menudir;
	  if(ROW==6) Settings[S_RSSIMIN]=Settings[S_RSSIMIN]+menudir;
	}
#endif
#ifdef PAGE5
	if(configPage == 5 && COL == 3) {
	  if(ROW==1) Settings[S_AMPERAGE]=!Settings[S_AMPERAGE];
	  if(ROW==2) Settings[S_AMPER_HOUR]=!Settings[S_AMPER_HOUR];
	  if(ROW==3) Settings[S_AMPERAGE_VIRTUAL]=!Settings[S_AMPERAGE_VIRTUAL];
	  if(ROW==4) S16_AMPMAX=S16_AMPMAX+menudir;
	  if(ROW==5) Settings[S_AMPMIN]=Settings[S_AMPMIN]+menudir;
	}
#endif
#ifdef PAGE6
	if(configPage == 6 && COL == 3) {
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
#ifdef PAGE7
	if(configPage == 7 && COL == 3) {
	  if(ROW==1) Settings[S_UNITSYSTEM]=!Settings[S_UNITSYSTEM];
	  if(ROW==2) {
	    Settings[S_VIDEOSIGNALTYPE]=!Settings[S_VIDEOSIGNALTYPE];
	    MAX7456Setup();
	    }
	  if(ROW==3) Settings[S_VREFERENCE]=!Settings[S_VREFERENCE];
	  if(ROW==4) Settings[S_DEBUG]=!Settings[S_DEBUG];
	  if(ROW==5) timer.magCalibrationTimer=CALIBRATION_DELAY;
	}
#endif
#ifdef PAGE8
	if(configPage == 8 && COL == 3) {
	  if(ROW==1) Settings[S_GPSTIME]=!Settings[S_GPSTIME];
	  if(ROW==2) Settings[S_GPSTZAHEAD]=!Settings[S_GPSTZAHEAD];
	  if(ROW==3) if((menudir == 1 && Settings[S_GPSTZ] < 130) || (menudir == -1 && Settings[S_GPSTZ] > 0))Settings[S_GPSTZ]=Settings[S_GPSTZ]+menudir*5;
	}
#endif
#ifdef PAGE9
	if(configPage == 9 && COL == 3) {
	  if(ROW==1) Settings[S_DISTANCE_ALARM]=Settings[S_DISTANCE_ALARM]+menudir;
	  if(ROW==2) Settings[S_ALTITUDE_ALARM]=Settings[S_ALTITUDE_ALARM]+menudir;
	  if(ROW==3) Settings[S_SPEED_ALARM]=Settings[S_SPEED_ALARM]+menudir;
	  if(ROW==4) Settings[S_FLYTIME_ALARM]=Settings[S_FLYTIME_ALARM]+menudir;
	  if(ROW==5) Settings[S_AMPER_HOUR_ALARM]=Settings[S_AMPER_HOUR_ALARM]+menudir;
	  if(ROW==6) Settings[S_AMPERAGE_ALARM]=Settings[S_AMPERAGE_ALARM]+menudir;
	}
#endif
  	if((ROW==10)&&(COL==1)) configExit();
	if((ROW==10)&&(COL==2)) configSave();
}

void serialMSPreceive()
{
  uint8_t c;

  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  }
  c_state = IDLE;

  while(Serial.available())
  {
    c = Serial.read();

    #ifdef GPSOSD    
      armedtimer = 0;
      if (GPS_newFrame(c)) GPS_NewData();   
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
    flyingTime=0;
  }
  setMspRequests();
}

void configSave()
{
  uint8_t txCheckSum;
  uint8_t txSize;

  headSerialRequest();
  txCheckSum=0;
  txSize=30;
  Serial.write(txSize);
  txCheckSum ^= txSize;
  Serial.write(MSP_SET_PID);
  txCheckSum ^= MSP_SET_PID;
  for(uint8_t i=0; i<PIDITEMS; i++) {
    Serial.write(P8[i]);
    txCheckSum ^= P8[i];
    Serial.write(I8[i]);
    txCheckSum ^= I8[i];
    Serial.write(D8[i]);
    txCheckSum ^= D8[i];
  }
  Serial.write(txCheckSum);

  headSerialRequest();
  txCheckSum=0;
  txSize=7;
  Serial.write(txSize);
  txCheckSum ^= txSize;
  Serial.write(MSP_SET_RC_TUNING);
  txCheckSum ^= MSP_SET_RC_TUNING;
  Serial.write(rcRate8);
  txCheckSum ^= rcRate8;
  Serial.write(rcExpo8);
  txCheckSum ^= rcExpo8;
  Serial.write(rollPitchRate);
  txCheckSum ^= rollPitchRate;
  Serial.write(yawRate);
  txCheckSum ^= yawRate;
  Serial.write(dynThrPID);
  txCheckSum ^= dynThrPID;
  Serial.write(thrMid8);
  txCheckSum ^= thrMid8;
  Serial.write(thrExpo8);
  txCheckSum ^= thrExpo8;
  Serial.write(txCheckSum);

  writeEEPROM();
  blankserialRequest(MSP_EEPROM_WRITE);
  configExit();
}

void blankserialRequest(uint8_t requestMSP)
{
  if(requestMSP == MSP_OSD && fontMode) {
    fontSerialRequest();
    return;
  }
  headSerialRequest();
  Serial.write((uint8_t)0x00);
  Serial.write(requestMSP);
  Serial.write(requestMSP);
}

void fontSerialRequest() {
  int16_t cindex = getNextCharToRequest();
  uint8_t txCheckSum;
  uint8_t txSize;
  headSerialRequest();
  txCheckSum=0;
  txSize=3;
  Serial.write(txSize);
  txCheckSum ^= txSize;
  Serial.write(MSP_OSD);
  txCheckSum ^= MSP_OSD;
  Serial.write(OSD_GET_FONT);
  txCheckSum ^= OSD_GET_FONT;
  Serial.write(cindex);
  txCheckSum ^= cindex;
  Serial.write(cindex>>8);
  txCheckSum ^= cindex>>8;
  Serial.write(txCheckSum);
}

void headSerialRequest (void) {
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  
}
