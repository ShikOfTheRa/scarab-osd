 
#if defined PROTOCOL_MAVLINK
  #define SERIALBUFFERSIZE 150
#elif defined NAZA
  #define SERIALBUFFERSIZE 250
#elif defined GPSOSD
  #define SERIALBUFFERSIZE 250
#else
  #define SERIALBUFFERSIZE 150
#endif

static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;
static uint8_t txChecksum;


#if defined PROTOCOL_LTM
  #include "LTM.h"
#endif 

#if defined PROTOCOL_KISS
  #include "KISS.h"
#endif 

#if defined PROTOCOL_SKYTRACK
  #include "SKYTRACK.h"
#endif 

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

#define skip8() {readIndex++;}
#define skip16() {readIndex+=2;}
#define skip32() {readIndex+=4;}
#define skipn(n) {readIndex+=n;}

#ifndef I2C_UB_SUPPORT

//
// Legacy mspWrite*()
//

void mspWriteRequest(uint8_t mspCommand, uint8_t txDataSize){
  //return;
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

void mspWrite32(uint32_t t){
  mspWrite8(t);
  mspWrite8(t>>8);
  mspWrite8(t>>16);
  mspWrite8(t>>24);
}

void mspWriteChecksum(){
  Serial.write(txChecksum);
}

// Writes to GUI (OSD_xxx) is distinguished from writes to FC (MSP_xxx) by
// cfgWrite*() and mspWrite*().
//
// If I2C is not used, then all cfgWrite*() will be mspWrite*().

# define cfgWriteRequest mspWriteRequest
# define cfgWrite8 mspWrite8
# define cfgWrite16 mspWrite16
# define cfgWrite32 mspWrite32
# define cfgWriteChecksum mspWriteChecksum

#else // I2C_UB_SUPPORT

//
// streamWrite*()
//
// With I2C_UB_SUPPORT, we have two message i/o streams; one being a tradional
// serial, another being an I2C. Here, we generalize writes to these streams
// with streamWrite*().
// mspWrite*() and cfgWrite*() will eventually call streamWrite*() with
// appropriate stream handle (port).

void streamWriteRequest(Stream *port, uint8_t mspCommand, uint8_t txDataSize){
  port->write("$M<");
  txChecksum = 0;
  streamWrite8(port, txDataSize);
  streamWrite8(port, mspCommand);
  if(txDataSize == 0)
    streamWriteChecksum(port);
}

void streamWrite8(Stream *port, uint8_t t){
  port->write(t);
  txChecksum ^= t;
}

void streamWrite16(Stream *port, uint16_t t){
  streamWrite8(port, t);
  streamWrite8(port, t>>8);
}

void streamWrite32(Stream *port, uint32_t t){
  streamWrite16(port, t);
  streamWrite16(port, t>>16);
}

void streamWriteChecksum(Stream *port){
  port->write(txChecksum);
}

//
// mspWrite*(): Use streamWrite*(), duplicate MSP to config port if specified
//

void mspWriteRequest(uint8_t mspCommand, uint8_t txDataSize){
# ifdef MSP2CFG
  streamWriteRequest(&Serial, mspCommand, txDataSize);
#endif
  streamWriteRequest(&WireUB, mspCommand, txDataSize);
}

void mspWrite8(uint8_t t){
# ifdef MSP2CFG
  streamWrite8(&Serial, t);
# endif
  streamWrite8(&WireUB, t);
}

void mspWrite16(uint16_t t){
# ifdef MSP2CFG
  streamWrite16(&Serial, t);
# endif
  streamWrite16(&WireUB, t);
}

void mspWrite32(uint32_t t){
# ifdef MSP2CFG
  streamWrite32(&Serial, t);
# endif
  streamWrite32(&WireUB, t);
}

void mspWriteChecksum(){
# ifdef MSP2CFG
  streamWriteChecksum(&Serial);
# endif
  streamWriteChecksum(&WireUB);
}

//
// cfgWrite*(): Explicit write to config (GUI) port
//

void cfgWriteRequest(uint8_t mspCommand, uint8_t txDataSize){
  streamWriteRequest(&Serial, mspCommand, txDataSize);
}

void cfgWrite8(uint8_t t){
  streamWrite8(&Serial, t);
}

void cfgWrite16(uint16_t t){
  streamWrite16(&Serial, t);
}

void cfgWrite32(uint32_t t){
  streamWrite32(&Serial, t);
}

void cfgWriteChecksum(){
  streamWriteChecksum(&Serial);
}
#endif // I2C_UB_SUPPORT

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  #ifdef DEBUGDPOSPACKET
    timer.packetcount++;
  #endif
  readIndex = 0;
  #ifdef DATA_MSP
    timer.MSP_active=DATA_MSP; // getting something on serial port
  #endif

  if (cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();

    if (cmd == OSD_READ_CMD_EE) {
      timer.GUI_active=2;
      eeaddress = read8();
      eeaddress = eeaddress+read8();
      eedata = read8();
      settingsMode=1;
      MSP_OSD_timer=3000+millis();
      settingsSerialRequest();
    }

    if (cmd == OSD_WRITE_CMD_EE) {
      timer.GUI_active=2;
      for(uint8_t i=0; i<10; i++) {
        eeaddress = read8();
        eeaddress = eeaddress+(read8()<<8);
        eedata = read8();
        settingsMode=1;
        MSP_OSD_timer=3000+millis();
        EEPROM.write(eeaddress,eedata);
//        if (eeaddress==0){
          EEPROM.write(0,EEPROMVER);
//        }
        if ((eeaddress==(EEPROM_SETTINGS-1)+(EEPROM16_SETTINGS*2))||(eeaddress==(EEPROM_SETTINGS-1)+(EEPROM16_SETTINGS*2)+(3*2*POSITIONS_SETTINGS))){
          readEEPROM();
        }
      }
      eeaddress++;
    settingswriteSerialRequest();
    }
#ifdef GUISENSORS
    if (cmd == OSD_SENSORS) {
      timer.GPS_initdelay=3; 
      cfgWriteRequest(MSP_OSD,1+10);
      cfgWrite8(OSD_SENSORS);
      for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++) {
//        uint16_t sensortemp = analogRead(sensorpinarray[sensor]);
        uint16_t sensortemp = (uint16_t)sensorfilter[sensor][SENSORFILTERSIZE]/SENSORFILTERSIZE;
        cfgWrite16(sensortemp);
      }
       cfgWriteChecksum();
       cfgWriteRequest(MSP_OSD,1+12);
       cfgWrite8(OSD_INFO);
       cfgWrite16(INFO_CONTROLLER); 
       cfgWrite16(INFO_HARDWARE);
       cfgWrite16(INFO_VERSION); 
       cfgWrite16(INFO_AIRCRAFT);
       cfgWrite16(INFO_OPTIONS1);
       cfgWrite16(INFO_VENDOR);
       cfgWriteChecksum();
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


#ifdef PROTOCOL_MSP

  #ifdef CANVAS_SUPPORT
  if (cmdMSP == MSP_DISPLAYPORT) {
    // Don't go into canvas mode when armed or in other special mode
    if (armed || fontMode)
        return;
/*
Notes on MSP_DISPLAYPORT protocol
(should go into the protocol document... where is that?)

byte: description
0: MSP_DISPLAYPORT
1: sub-command
   0: Enter/hold canvas mode
       Sender must periodically send the enter/hold message:
        FC may exist menu mode without notifying for many reasons,
        including manual reset or power off/cycle.
        We expect the FC to sustain canvasMode by sending non-exit
        message within CANVAS_TIMO while the menu mode is active.

   1: Exit canvas mode and resume normal OSD operation
   2: Clear canvas
   3: Draw string at (row,col) with attribute attr
        Automagically wraps to next row.

For sub-command 3 (draw string):
2: row
3: col
4: attr (0=normal, 1=inverted)
5...len-1: string to draw
           Zero prematurely terminates the string.
           (Charset compat problem is ignored)
*/

    lastCanvas = millis();

    if (configMode)
      configExit(); // exits MWOSD menu so never be in menu when CMS mode active

    switch(read8()) {
    case 0: // Enter / hold canvas mode
      canvasMode = true;
      break;

    case 1: // Exit canvas mode
      canvasMode = false;
      canvasFirst = true;
      break;

    case 2: // Clear canvas
      MAX7456_ClearScreen();
      break;

    case 3: // Draw string at (row,col) with attribute (if supported)
      uint8_t canvasy = read8();
      uint8_t canvasx = read8();
      uint8_t canvasa = read8();
      for (int i = 5; i <= dataSize ; i++) {
        char canvasc[2];
        canvasc[0] = read8();
        if (canvasc[0] == 0)
          break;
        canvasc[1] = 0;
#ifdef INVERTED_CHAR_SUPPORT
        MAX7456_WriteStringWithAttr(canvasc, canvasy * LINE + canvasx, canvasa);
#else
        MAX7456_WriteString(canvasc, canvasy * LINE + canvasx);
#endif
        ++canvasx;
      }
      break;
    }
    return;
  }
  #endif // CANVAS_SUPPORT

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
    for(uint8_t i=1;i<=TX_CHANNELS;i++)
      MwRcData[i] = read16();
    handleRawRC();
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    #ifdef ALARM_GPS
     timer.GPS_active=ALARM_GPS;
    #endif //ALARM_GPS
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
        MSP_home_set=1;
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
    GPS_distanceToHome=(uint32_t)read16();
#ifdef I2CGPS_DISTANCE
    gpsdistancefix();
#endif
    
    GPS_directionToHome=read16();
    
#ifdef GPSTIME
    read8(); //missing
    GPS_time = read32();        //local time of coord calc - haydent
#endif
  }

  if (cmdMSP==MSP_NAV_STATUS)
  {
     read8();
     read8();
     read8();
     GPS_waypoint_step=read8();
  }

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
    MWAmperage = (int16_t)read16();
 }

#ifdef MENU_SERVO  
  if (cmdMSP==MSP_SERVO_CONF)
  {
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
      for (uint8_t ii = 0; ii < 5; ii++) {
        if (ii==3)
          servo.settings[ii][i] =read8();
        else
          servo.settings[ii][i] =read16();
      }
   }
   modeMSPRequests &=~ REQ_MSP_SERVO_CONF;
 }
#endif //MENU_SERVO   

#ifdef MENU_FIXEDWING
  if (cmdMSP==MSP_FW_CONFIG)
  {
    cfg.fw_althold_dir=read8();
    cfg.fw_gps_maxcorr=read16();
    cfg.fw_gps_rudder=read16();
    cfg.fw_gps_maxclimb=read16();
    cfg.fw_gps_maxdive=read16();
    cfg.fw_climb_throttle=read16();
    cfg.fw_cruise_throttle=read16();
    cfg.fw_idle_throttle=read16();
    cfg.fw_scaler_throttle=read16();
    cfg.fw_roll_comp=read32();
    cfg.fw_rth_alt=read8();
    for(uint8_t i = 0; i < 4; i++) {
      read32();
    }
    modeMSPRequests &=~ REQ_MSP_FW_CONFIG;
  }
#endif // MENU_FIXEDWING

#ifdef USE_FC_VOLTS_CONFIG
  if (cmdMSP==MSP_MISC)
  {
    // Multiple skip8/skip16 (readIndex increments) seems to be
    // collapsed into single addition by the compiler.

    skip16(); //ignore: midrc

    skip16(); //ignore: minthrottle
    skip16(); //ignore: maxthrottle
    skip16(); //ignore: mincommand

    skip16(); //ignore: failsafe_throttle
    
    skip8(); //ignore: gps_type
    skip8(); //ignore: gps_baudrate
    skip8(); //ignore: gps_ubx_sbas

    skip8(); //ignore: multiwiiCurrentMeterOutput
    skip8(); //ignore: rssi_channel
    skip8(); //ignore: 0

    skip16(); //ignore: mag_declination

    skip8(); //ignore: vbatscale

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
#ifdef USE_MSP_PIDNAMES
  if (cmdMSP==MSP_PIDNAMES)
  {
      // parse buffer and fill menu_pid[]. We need to receive all bytes, but store only ones that we need
      
      uint8_t pn_index = 0, avail = (PIDNAME_BUFSIZE - 1), c;
      uint8_t *out = (uint8_t *)menu_pid;

      for(uint8_t i = 0; i<dataSize; i++) {
        c = read8();

#ifdef MENU_PID_VEL
        if((pn_index != 5) && (pn_index != 6) && (pn_index <= 9)) // 5, 6 and >9 are skipped
#else
        if((pn_index != 5) && (pn_index != 6) && (pn_index <= 8)) // 5, 6 and >8 are skipped
#endif
        {
          if(c == ';')
          {
             *out = 0;

              out += avail + 1;
              
              avail = PIDNAME_BUFSIZE - 1;
          }
          else if(avail > 0)
          {
             *out++ = c;
             --avail;
          }
        }

        if(c == ';')
        {
           ++pn_index;
        }
      
      }
      modeMSPRequests &= ~REQ_MSP_PIDNAMES;
  }
#endif
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
  #ifdef CORRECTLOOPTIME
    if (cmdMSP == MSP_LOOP_TIME)
    {
      LoopTime = read16();
      modeMSPRequests &=~ REQ_MSP_LOOP_TIME;
    }
  #endif
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

    struct boxes {
        void    *var;
        uint8_t size;
        const char *name;
    } boxnames[] = {
        { &mode.armed,      8,  PSTR("ARM") },
        { &mode.stable,     8,  PSTR("ANGLE") },
        { &mode.horizon,    8,  PSTR("HORIZON") },
        { &mode.mag,        8,  PSTR("MAG") },
        { &mode.baro,       8,  PSTR("BARO") },
        { &mode.llights,    16, PSTR("LLIGHTS") },
        { &mode.camstab,    16, PSTR("CAMSTAB") },
        { &mode.air,        32, PSTR("AIR MODE") },
        { &mode.acroplus,   32, PSTR("ACRO PLUS") },
        { &mode.gpshome,    16, PSTR("GPS HOME") },
        { &mode.gpshold,    16, PSTR("GPS HOLD") },
        { &mode.passthru,   16, PSTR("PASSTHRU") },
        { &mode.osd_switch, 32, PSTR("OSD SW") },
        { NULL,             0,  NULL },
    };

    memset(&mode, 0, sizeof(mode));

    char boxname[20];

    while(remaining > 0) {
      char c = read8();
      if(c != ';') {
        boxname[len] = c;
        len++;
      }
      else {
          for (int i = 0; boxnames[i].name; i++) {
              if (strncmp_P(boxname, boxnames[i].name, len) == 0) {
                  switch (boxnames[i].size) {
                  case 8:
                      *(uint8_t*)boxnames[i].var |= bit;
                      break;
                  case 16:
                      *(uint16_t*)boxnames[i].var |= bit;
                      break;
                  case 32:
                      *(uint32_t*)boxnames[i].var |= bit;
                      break;
                  }
                  break;
              }
          }

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

    memset(&mode, 0, sizeof(mode));

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
      case IDBOXWP:
        mode.gpsmission |= bit;
        break;
      case 21:
        mode.gpsland |= bit;
        break;
      case IDBOXAIR:
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
#ifdef CANVAS_SUPPORT
      if (!configMode && (allSec > 5) && !armed && !canvasMode)
#else
      if (!configMode&&(allSec>5)&&!armed)
#endif
      {
          // Enter config mode using stick combination
          waitStick =  2;	// Sticks must return to center before continue!
          configMode = 1;
          configPage = previousconfigPage;
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
#ifdef TX_MODE1
      if(configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // MOVE RIGHT
#else
      if(configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // MOVE RIGHT
#endif
      {
	waitStick = 1;
	COL++;
	if(COL>3) COL=3;
      }
#ifdef TX_MODE1
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
#ifdef TX_MODE1
      else if(!previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // DECREASE
#else
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // DECREASE
#endif
      {
	waitStick = 1;
        menudir=-1+oldmenudir;
        serialMenuCommon();  
      }
#ifdef TX_MODE1
      else if(!previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // INCREASE
#else
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // INCREASE
#endif
      { 
	waitStick =1;
        menudir=1+oldmenudir;
        #ifdef MENU_ALARMS
	if(configPage == MENU_ALARMS && COL == 3) {
	  if(ROW==5) timer.magCalibrationTimer=0;
        }
        #endif //MENU_ALARMS
        serialMenuCommon();  
      }      
    }
    if(waitStick == 1)
      stickTime = millis();
  }
}

// Some cute macros to use use within the switch-case to make code look clean
// Note that it has a break, and it is enclosed in a braces.
// It may be clever to omit the trailing semi-colon in use to signify that
// the line is special.

#define ReverseSetting(name) {Settings[name] = !Settings[name]; break;}
#define ModifySetting(name) {Settings[name] += menudir; break;}
#define ModifySetting16(name) {Settings16[name] += menudir; break;}

void serialMenuCommon()
{
  if((ROW==10)&&(COL==3)) {
    if (menudir > 1){
      menudir = 1;
    }
    if (menudir < -1){
      menudir = -1;
    }
//      constrain(menudir,-1,1);
    configPage += menudir;
  }

  if(configPage < MINPAGE) configPage = MAXPAGE;
  if(configPage > MAXPAGE) configPage = MINPAGE;

#ifdef MENU_PID
  if(configPage == MENU_PID) {
#ifdef MENU_PID_VEL
    if(ROW >= 1 && ROW <= 8) {
#else
    if(ROW >= 1 && ROW <= 7) {
#endif
      uint8_t MODROW = ROW - 1;
      if (ROW > 5) {
        MODROW = ROW + 1;
      }
      switch(COL) {
      case 1: P8[MODROW] += menudir; break;
      case 2: I8[MODROW] += menudir; break;
      case 3: D8[MODROW] += menudir; break;
      }
    }
  }
#endif

#ifdef MENU_RC_2
  if(configPage == MENU_RC_2 && COL == 3) {    
    switch(ROW) {
      case 1: tpa_breakpoint16 += menudir; break;
      case 2: rcYawExpo8 += menudir; break;
    }
  }
#endif
  
#ifdef MENU_SERVO
  if(configPage == MENU_SERVO) {
    switch(COL) {
      case 1: servo.settings[0][ROW-1]+= menudir; break;
      case 2: servo.settings[1][ROW-1]+= menudir; break;
      case 3: servo.settings[2][ROW-1]+= menudir; break;
    }
  }
#endif

#ifdef MENU_RC
  #if defined CORRECT_MENU_RCT2
    if (configPage == MENU_RC && COL == 3) {
      switch(ROW) {
      case 1: rcRate8 += menudir; break;
      case 2: rcExpo8 += menudir; break;
      case 3: rollRate += menudir; break;
      case 4: PitchRate += menudir; break;
      case 5: yawRate += menudir; break;
      case 6: dynThrPID += menudir; break;
      case 7: thrMid8 += menudir; break;
      case 8: thrExpo8 += menudir; break;
      }
    }
  #elif defined CORRECT_MENU_RCT1
    if (configPage == MENU_RC && COL == 3) {
      switch(ROW) {
      case 1: rcRate8 += menudir; break;
      case 2: rcExpo8 += menudir; break;
      case 3: rollRate += menudir; break;
      case 4: PitchRate += menudir; break;
      case 5: yawRate += menudir; break;
      case 6: dynThrPID += menudir; break;
      case 7: thrMid8 += menudir; break;
      case 8: thrExpo8 += menudir; break;
      }
    }
  #else
    if (configPage == MENU_RC && COL == 3) {
      switch(ROW) {
      case 1: rcRate8 += menudir; break;
      case 2: rcExpo8 += menudir; break;
      case 3: rollPitchRate += menudir; break;
      case 4: yawRate += menudir; break;
      case 5: dynThrPID += menudir; break;
      case 6: thrMid8 += menudir; break;
      case 7: thrExpo8 += menudir; break;
      }
    }
  #endif
#endif

#ifdef MENU_FIXEDWING
  if (configPage == MENU_FIXEDWING && COL == 3) {
    switch(ROW) {
    case 1: cfg.fw_gps_maxcorr += menudir; break;
    case 2: cfg.fw_gps_rudder += menudir; break;
    case 3: cfg.fw_gps_maxclimb += menudir; break;
    case 4: cfg.fw_gps_maxdive += menudir; break;
    case 5: cfg.fw_climb_throttle += menudir; break;
    case 6: cfg.fw_cruise_throttle += menudir; break;
    case 7: cfg.fw_idle_throttle += menudir; break;
    case 8: cfg.fw_rth_alt += menudir; break;
    }
  }
#endif

#ifdef MENU_VOLTAGE
  if (configPage == MENU_VOLTAGE && COL == 3) {
    switch(ROW) {
    case 1: ReverseSetting(S_DISPLAYVOLTAGE)
    case 2: ModifySetting(S_DIVIDERRATIO)
    case 3: ModifySetting(S_VOLTAGEMIN)
    case 4: ReverseSetting(S_VIDVOLTAGE)
    case 5: ModifySetting(S_VIDDIVIDERRATIO)
    case 6: ModifySetting(S_BATCELLS)
    case 7: ReverseSetting(S_MAINVOLTAGE_VBAT)
    }
  }
#endif

#ifdef MENU_RSSI
  if (configPage == MENU_RSSI && COL == 3) {
    switch(ROW) {
    case 1: ReverseSetting(S_DISPLAYRSSI)
    case 2: timer.rssiTimer=15; break; // 15 secs to turn off tx anwait to read min RSSI
    case 3: ReverseSetting(S_MWRSSI)
    case 4: ReverseSetting(S_PWMRSSI)
    case 5: ModifySetting16(S16_RSSIMAX)
    case 6: ModifySetting16(S16_RSSIMIN)
    }
  }
#endif

#ifdef MENU_CURRENT
  if (configPage == MENU_CURRENT && COL == 3) {
    switch(ROW) {
    case 1: ReverseSetting(S_AMPERAGE)
    case 2: ReverseSetting(S_AMPER_HOUR)
    case 3: ReverseSetting(S_AMPERAGE_VIRTUAL)
    case 4: ModifySetting16(S16_AMPDIVIDERRATIO)
    case 5: ModifySetting16(S16_AMPZERO)
    }
  }
#endif

#ifdef MENU_DISPLAY
  if (configPage == MENU_DISPLAY && COL == 3) {
    switch(ROW) {
    case 1: ReverseSetting(S_DISPLAY_HORIZON_BR)
    case 2: ReverseSetting(S_WITHDECORATION)
    case 3: ReverseSetting(S_SCROLLING)
    case 4: ReverseSetting(S_THROTTLEPOSITION)
    case 5: ReverseSetting(S_COORDINATES)
    case 6: ReverseSetting(S_MODESENSOR)
    case 7: ReverseSetting(S_GIMBAL)
    case 8: ModifySetting(S_MAPMODE)
    }
  }
#endif

#ifdef MENU_ADVANCED
  if (configPage == MENU_ADVANCED && COL == 3) {
    switch(ROW) {
    case 1: ReverseSetting(S_UNITSYSTEM)
    case 2: ReverseSetting(S_VREFERENCE)
    case 3: ReverseSetting(S_DEBUG)
    case 4: timer.magCalibrationTimer=CALIBRATION_DELAY; break;
    case 5: ModifySetting(S_RCWSWITCH_CH)
    case 6: ReverseSetting(S_THROTTLE_PWM)
    }
  }
#endif

#ifdef MENU_GPS_TIME
  if (configPage == MENU_GPS_TIME && COL == 3) {
    switch(ROW) {
    case 1: ReverseSetting(S_GPSTIME);
    case 2: ReverseSetting(S_GPSTZAHEAD);
    case 3:
      if (   (menudir == 1 && Settings[S_GPSTZ] < 130)
          || (menudir == -1 && Settings[S_GPSTZ] > 0))
        Settings[S_GPSTZ] = Settings[S_GPSTZ] + menudir * 5;
      break;
    }
  }
#endif

#ifdef MENU_ALARMS
  if (configPage == MENU_ALARMS && COL == 3) {
    switch(ROW) {
    case 1: ModifySetting(S_DISTANCE_ALARM)
    case 2: ModifySetting(S_ALTITUDE_ALARM)
    case 3: ModifySetting(S_SPEED_ALARM)
    case 4: ModifySetting(S_FLYTIME_ALARM)
    case 5: ModifySetting(S_AMPER_HOUR_ALARM)
    case 6: ModifySetting(S_AMPERAGE_ALARM)
    case 7: ReverseSetting(S_ALARMS_TEXT)
    }
  }
#endif

#ifdef MENU_PROFILE
  #ifdef ADVANCEDSAVE
    if (configPage == MENU_PROFILE && COL == 3) {
      switch(ROW) {
      case 1: FCProfile += menudir; break;
      case 2: PIDController += +menudir; break;
      #ifdef CORRECTLOOPTIME
        case 3: LoopTime += menudir; break;
      #endif //CORRECTLOOPTIME
      }
    }

    #ifdef ENABLE_MSP_SAVE_ADVANCED
      if (FCProfile > 2)
        FCProfile=0;

      if (FCProfile != PreviousFCProfile){
        setFCProfile();
        PreviousFCProfile = FCProfile;
      }        
    #endif ENABLE_MSP_SAVE_ADVANCED
  #endif //ADVANCEDSAVE
#endif  

  if (ROW == 10) {
    previousconfigPage = configPage;
    switch(COL) {
    case 1: configExit(); break;
    case 2: configSave(); break;
    }
  }
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

#ifdef I2C_UB_SUPPORT
  // The focusPort is a message oriented multiplexer that switches
  // between serial and i2c ports.
  //
  // Once a first character of a new message is read from either port,
  // then the focusPort is fixed to that port and will not change
  // until the state goes back to IDLE.

  static Stream *focusPort = NULL;

  // Always reset focusPort if IDLE
  if (c_state == IDLE)
    focusPort = NULL;

  if (focusPort && focusPort->available()) {
    // Message is actively read.
    loopserial=1;
  } else {
    // Beginning of a new message. Fix the focusPort.
    if (Serial.available()) {
      focusPort = &Serial;
      loopserial = 1;
    }
    else if (WireUB.available()) {
      focusPort = &WireUB;
      loopserial = 1;
    }
  }
#else
  if (Serial.available()) loopserial=1;
#endif

  while(loopserial==1)
  {
#ifdef I2C_UB_SUPPORT
    // Read from the active port.
    c = focusPort->read();
#else
    c = Serial.read();
  #ifdef DEBUGDPOSRX    
    timer.serialrxrate++;
  #endif
#endif

    #ifdef GPSOSD    
      armedtimer = 0;
      #if defined (NAZA)
        NAZA_NewData(c);
      #else
        if (GPS_newFrame(c)) GPS_NewData();  
      #endif //NAZA  
    #endif //GPSOSD   

    #if defined (PROTOCOL_MAVLINK)
       serialMAVreceive(c);
    #endif //PROTOCOL_MAVLINK   
    #if defined (PROTOCOL_LTM)
       serialLTMreceive(c);
    #endif // PROTOCOL_LTM   
    #if defined (PROTOCOL_KISS)
       serialKISSreceive(c);
    #endif // PROTOCOL_KISS   

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
#ifdef I2C_UB_SUPPORT
    if (!focusPort->available()) loopserial=0;
#else
    if (!Serial.available()) loopserial=0;
#endif
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
  #if defined ADVANCEDSAVE
    mspWriteRequest(MSP_SET_PID_CONTROLLER, 1);
    mspWrite8(PIDController);
    mspWriteChecksum();
    #if defined CORRECTLOOPTIME
      mspWriteRequest(MSP_SET_LOOP_TIME, 2);
      mspWrite16(LoopTime);
      mspWriteChecksum();  
    #endif //CORRECTLOOPTIME
  #endif //ADVANCEDSAVE
#endif //ENABLE_MSP_SAVE_ADVANCED

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

#if defined MENU_FIXEDWING
  mspWriteRequest(MSP_SET_FW_CONFIG,38);
  mspWrite8(cfg.fw_althold_dir);
  mspWrite16(cfg.fw_gps_maxcorr);
  mspWrite16(cfg.fw_gps_rudder);
  mspWrite16(cfg.fw_gps_maxclimb);
  mspWrite16(cfg.fw_gps_maxdive);
  mspWrite16(cfg.fw_climb_throttle);
  mspWrite16(cfg.fw_cruise_throttle);
  mspWrite16(cfg.fw_idle_throttle);
  mspWrite16(cfg.fw_scaler_throttle);
  mspWrite32(cfg.fw_roll_comp); // Float is Not compatible with Gui. Change to mspWrite8
  mspWrite8(cfg.fw_rth_alt);
  for(uint8_t i=0; i<8; i++) {
    mspWrite16(0);
  }
  mspWriteChecksum();  
#endif // MENU_FIXEDWING

#ifdef MENU_SERVO  
  mspWriteRequest(MSP_SET_SERVO_CONF,(9*MAX_SERVOS));
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
      for (uint8_t ii = 0; ii < 5; ii++) {
        if (ii==3)
          mspWrite8(servo.settings[ii][i]&0xFF);
        else
          mspWrite16(servo.settings[ii][i]);
      }
   }
  mspWriteChecksum();
#endif

  writeEEPROM();
  mspWriteRequest(MSP_EEPROM_WRITE,0);
  configExit();
}

void fontSerialRequest() {
  cfgWriteRequest(MSP_OSD,3);
  cfgWrite8(OSD_GET_FONT);
  cfgWrite16(getNextCharToRequest());
  cfgWriteChecksum();
}

void settingsSerialRequest() {
  cfgWriteRequest(MSP_OSD,1+30);
  cfgWrite8(OSD_READ_CMD_EE);
  for(uint8_t i=0; i<10; i++) {
    eedata = EEPROM.read(eeaddress);
    cfgWrite16(eeaddress);
    cfgWrite8(eedata);
    eeaddress++;
  }
  cfgWriteChecksum();
}

void settingswriteSerialRequest() {
  cfgWriteRequest(MSP_OSD,3);
  cfgWrite8(OSD_READ_CMD_EE);
  cfgWrite16(eeaddress);
  cfgWriteChecksum();
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

