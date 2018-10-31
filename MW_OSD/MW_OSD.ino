

/*
Scarab NG OSD ...

 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see http://www.gnu.org/licenses/

This work is based on the following open source work :-
 Rushduino                 http://code.google.com/p/rushduino-osd/
 Rush OSD Development      https://code.google.com/p/rush-osd-development/
 Minim OSD                 https://code.google.com/p/arducam-osd/wiki/minimosd

 Its base is taken from "Rush OSD Development" R370

 All credit and full acknowledgement to the incredible work and hours from the many developers, contributors and testers that have helped along the way.
 Jean Gabriel Maurice. He started the revolution. He was the first....

 Please refer to credits.txt for list of individual contributions

 Exceptions:
 Where there are exceptions, these take precedence over the genereric GNU licencing
 Elements of the code provided by Pawelsky (DJI specific) are not for commercial use.
 See headers in individual files for further details.
 Libraries used and typically provided by compilers may have licening terms stricter than that of GNU 3
*/

// travis test 1
//------------------------------------------------------------------------
#define MEMCHECK   // to enable memory checking.
#if 1
__asm volatile ("nop");
#endif
#ifdef MEMCHECK
extern uint8_t _end;  //end of program variables
extern uint8_t __stack; //start of stack (highest RAM address)

void PaintStack(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));    //Make sure this is executed at the first time

void PaintStack(void)
{
  //using asm since compiller could not be trusted here
  __asm volatile ("    ldi r30,lo8(_end)\n"
                  "    ldi r31,hi8(_end)\n"
                  "    ldi r24,lo8(0xa5)\n" /* Paint color = 0xa5 */
                  "    ldi r25,hi8(__stack)\n"
                  "    rjmp .cmp\n"
                  ".loop:\n"
                  "    st Z+,r24\n"
                  ".cmp:\n"
                  "    cpi r30,lo8(__stack)\n"
                  "    cpc r31,r25\n"
                  "    brlo .loop\n"
                  "    breq .loop"::);
}

uint16_t UntouchedStack(void)
{
  const uint8_t *ptr = &_end;
  uint16_t       count = 0;

  while (*ptr == 0xa5 && ptr <= &__stack)
  {
    ptr++; count++;
  }

  return count;
}
#endif


// Frequently used expressions
#define PGMSTR(p) (char *)pgm_read_word(p)

//------------------------------------------------------------------------
#define MWVERS "MW-OSD - R1.9.0.6"
//#define MWVERS "MW-OSD - R1.9"
#define MWOSDVERSION 1906 // 1660=1.6.6.0 for GUI
#define EEPROMVER 16      // for eeprom layout verification

#include <avr/pgmspace.h>
#undef   PROGMEM
#define  PROGMEM __attribute__(( section(".progmem.data") )) // Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#include <EEPROM.h>
#include <util/atomic.h> // For ATOMIC_BLOCK
#include "Config.h"
#include "Def.h"
#include "symbols.h"
#include "GlobalVariables.h"
#include "math.h"

#if defined LOADFONT_LARGE
#include "fontL.h"
#elif defined LOADFONT_DEFAULT
#include "fontD.h"
#elif defined LOADFONT_BOLD
#include "fontB.h"
#endif
#ifdef I2C_UB_SUPPORT
#include "WireUB.h"
#endif
#ifdef I2C_SUPPORT
#include <Wire.h>
#endif
#if defined USEMS5837
#include "MS5837.h"
MS5837 MS5837sensor;
#endif //USEMS5837
#if defined PROTOCOL_SKYTRACK
#include "SKYTRACK.h"
#endif



//------------------------------------------------------------------------
void setup()
{
#ifdef HARDRESET
  MCUSR &= (0xFF & (0 << WDRF));
  WDTCSR |= (1 << WDCE) | (1 << WDE) | (0 << WDIE);
  WDTCSR =  (0 << WDE) | (0 << WDIE);
#endif

  Serial.begin(BAUDRATE);
#ifndef PROTOCOL_MAVLINK //use double speed asynch mode (multiwii compatible)
  uint8_t h = ((F_CPU  / 4 / (BAUDRATE) - 1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / (BAUDRATE) - 1) / 2);
  UCSR0A  |= (1 << U2X0); UBRR0H = h; UBRR0L = l;
#endif

#ifdef I2C_UB_SUPPORT
  // I2C initialization
  WireUB.begin(I2C_UB_ADDR, -1);
  TWBR = 2; // Probably has no effect.
  // Compute rx queue timeout in microseconds
  //   = 3 character time at current bps (10bits/char)
  WireUB.setWriteTimo((1000000UL) / (I2C_UB_BREQUIV / 10) * 3);
#endif

  MAX7456SETHARDWAREPORTS
  ATMEGASETHARDWAREPORTS
  LEDINIT

#if defined EEPROM_CLEAR
  EEPROM_clear();
#endif
  checkEEPROM();
  readEEPROM();

#ifndef STARTUPDELAY
#define STARTUPDELAY 1000
#endif
#ifndef INTRO_DELAY
#define INTRO_DELAY 5
#endif
  delay(STARTUPDELAY);

#ifdef VTX_RTC6705
  vtx_init();
#ifdef IMPULSERC_HELIX
  vtx_flash_led(5);
# endif
#endif

  MAX7456Setup();
#if defined GPSOSD
  timer.GPS_initdelay = INTRO_DELAY;
#else
#endif
#if defined FORECSENSORACC
  MwSensorPresent |= ACCELEROMETER;
#endif
#if defined FORCESENSORS
  MwSensorPresent |= GPSSENSOR;
  MwSensorPresent |= BAROMETER;
  MwSensorPresent |= MAGNETOMETER;
  MwSensorPresent |= ACCELEROMETER;
#endif
  setMspRequests();

#ifdef ALWAYSARMED
  armed = 1;
#endif //ALWAYSARMED

#ifdef KKAUDIOVARIO
  Wire.begin();
  setupSensor();
  pressure = getPressure();
  lowpassFast = lowpassSlow = pressure;
#endif //KKAUDIOVARIO
#if defined USEMS5837
  Wire.begin();
  MS5837sensor.init();
  MS5837sensor.setFluidDensity(FLUID_DENSITY); // kg/m^3
#endif // USE MS_5837
  GPS_time=946684801; // Set to Y2K as default.
  datetime.unixtime = GPS_time;
  Serial.flush();
  for (uint8_t i = 0; i < (1+16); i++) {
    MwRcData[i]=1500;
  }
}

//------------------------------------------------------------------------
#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
void loop()
{
  switch (fontStatus) {
    case 0:
      MAX7456_WriteString_P(messageF0, 32);
      MAX7456_DrawScreen();
      delay(3000);
      displayFont();
      MAX7456_WriteString_P(messageF1, 32);
      MAX7456_DrawScreen();
      fontStatus++;
      delay(3000);
      break;
    case 1:
      updateFont();
      MAX7456Setup();
      MAX7456_WriteString_P(messageF2, 32);
      displayFont();
      MAX7456_DrawScreen();
      fontStatus++;
      break;
  }
  LEDOFF
}
#elif defined DISPLAYFONTS
void loop()
{
  MAX7456Setup();
  displayFont();
  MAX7456_DrawScreen();
}
#else

// ampAlarming returns true if the total consumed mAh is greater than
// the configured alarm value (which is stored as 100s of amps)
bool ampAlarming() {
  int used = pMeterSum > 0 ? pMeterSum : (amperagesum / 360);
  return used > (Settings[S_AMPER_HOUR_ALARM] * 100);
}


//------------------------------------------------------------------------
void loop()
{

#if defined TX_GUI_CONTROL   //PITCH,YAW,THROTTLE,ROLL order controlled by GUI for GPSOSD and MAVLINK
  switch (Settings[S_TX_TYPE]) {
    case 1: //RPTY
      tx_roll     = 1;
      tx_pitch    = 2;
      tx_yaw      = 4;
      tx_throttle = 3;
      break;
    case 2: //TRPY
      tx_roll     = 2;
      tx_pitch    = 3;
      tx_yaw      = 4;
      tx_throttle = 1;
      break;
    default: //RPYT - default xxxflight FC
      tx_roll     = 1;
      tx_pitch    = 2;
      tx_yaw      = 3;
      tx_throttle = 4;
      break;
  }
#endif // TX_GUI_CONTROL   //PITCH,YAW,THROTTLE,ROLL order controlled by GUI   

  alarms.active = 0;
  timer.loopcount++;
  if (flags.reset) {
    resetFunc();
  }
#if defined (KISS)
  if (Kvar.mode == 1)
    screenlayout = 1;
  else
    screenlayout = 0;
#elif defined (OSD_SWITCH)
  if (MwSensorActive & mode.osd_switch)
    screenlayout = 1;
  else
    screenlayout = 0;
#elif defined (OSD_SWITCH_RC)
  rcswitch_ch = Settings[S_RCWSWITCH_CH];
  screenlayout = 0;
  if (Settings[S_RCWSWITCH] == 1) {
    if (MwRcData[rcswitch_ch] > TX_CHAN_HIGH) {
      screenlayout = 2;
    }
    else if (MwRcData[rcswitch_ch] > TX_CHAN_MID) {
      screenlayout = 1;
    }
  }
  else {
    if (MwSensorActive & mode.osd_switch)
      screenlayout = 1;
  }
#else
  screenlayout = 0;
#endif

  if (screenlayout != oldscreenlayout) {
    readEEPROM();
  }
  oldscreenlayout = screenlayout;

  // Blink Basic Sanity Test Led at 0.5hz
  if (timer.Blink2hz)
    LEDON
    else
      LEDOFF

      //---------------  Start Timed Service Routines  ---------------------------------------
      unsigned long currentMillis = millis();

#ifdef IMPULSERC_HELIX
  vtx_process_state(currentMillis, vtxBand, vtxChannel);
#endif //IMPULSERC_HELIX

#ifdef KKAUDIOVARIO
  if (millis() > timer.audiolooptimer) {
    timer.audiolooptimer += 20;
    AudioVarioUpdate();
  }
#endif //KKAUDIOVARIO

#ifdef MSP_SPEED_HIGH
  if ((currentMillis - previous_millis_sync) >= sync_speed_cycle) // (Executed > NTSC/PAL hz 33ms)
  {
    previous_millis_sync = previous_millis_sync + sync_speed_cycle;
#ifdef CANVAS_SUPPORT
    if (!fontMode && !canvasMode)
#else
    if (!fontMode)
#endif
    {
#ifdef PROTOCOL_MSP
      if (timer.GUI_active == 0) {
        mspWriteRequest(MSP_ATTITUDE, 0);
      }
#endif
    }
  }
#endif //MSP_SPEED_HIGH

  if ((currentMillis - previous_millis_low) >= lo_speed_cycle) // 10 Hz (Executed every 100ms)
  {
    previous_millis_low = previous_millis_low + lo_speed_cycle;
    timer.tenthSec++;
    timer.halfSec++;
    timer.Blink10hz = !timer.Blink10hz;
#ifdef USEMS5837
    MS5837sensor.read();
#endif //USEMS5837  
    if (GPS_fix && armed) {
      if (Settings[S_UNITSYSTEM])
        tripSum += GPS_speed * 0.0032808;    //  100/(100*1000)*3.2808=0.0016404     cm/sec ---> ft/50msec
      else
        tripSum += GPS_speed * 0.0010;       //  100/(100*1000)=0.0005               cm/sec ---> mt/50msec (trip var is float)
      trip = (uint32_t) tripSum;
    }

#ifndef KISS
    amperagesum += amperage;
#else
    if (!Settings[S_MWAMPERAGE])
      amperagesum += amperage;
#endif   // KISS
#ifndef GPSOSD
#ifdef MSP_SPEED_MED
#ifdef CANVAS_SUPPORT
    if (!fontMode && !canvasMode)
#else
    if (!fontMode)
#endif
    {
#ifdef PROTOCOL_MSP
      if (timer.GUI_active == 0) {
        mspWriteRequest(MSP_ATTITUDE, 0);
      }
#endif // KISS
    }
#endif //MSP_SPEED_MED  
#endif //GPSOSD
  }  // End of slow Timed Service Routine (100ms loop)

  if ((currentMillis - previous_millis_high) >= hi_speed_cycle) // 33 Hz or 100hz in MSP high mode.
  {
    previous_millis_high = previous_millis_high + hi_speed_cycle;
    uint16_t MSPcmdsend = 0;
    if (queuedMSPRequests == 0)
      queuedMSPRequests = modeMSPRequests;
    uint32_t req = queuedMSPRequests & -queuedMSPRequests;
    queuedMSPRequests &= ~req;
    switch (req) {
      case REQ_MSP_STATUS:
        MSPcmdsend = MSP_STATUS;
        break;
#ifdef INTRO_FC
      case REQ_MSP_FC_VERSION:
        MSPcmdsend = MSP_FC_VERSION;
        break;
#endif
      case REQ_MSP_RC:
        MSPcmdsend = MSP_RC;
        break;
      case REQ_MSP_RAW_GPS:
        MSPcmdsend = MSP_RAW_GPS;
        break;
      case REQ_MSP_COMP_GPS:
        MSPcmdsend = MSP_COMP_GPS;
        break;
      case REQ_MSP_ATTITUDE:
        MSPcmdsend = MSP_ATTITUDE;
        break;
      case REQ_MSP_ALTITUDE:
        MSPcmdsend = MSP_ALTITUDE;
        break;
      case REQ_MSP_ANALOG:
        MSPcmdsend = MSP_ANALOG;
        break;
      case REQ_MSP_MISC:
        MSPcmdsend = MSP_MISC;
        break;
      case REQ_MSP_RC_TUNING:
        MSPcmdsend = MSP_RC_TUNING;
        break;
      case REQ_MSP_PID_CONTROLLER:
        MSPcmdsend = MSP_PID_CONTROLLER;
        break;
      case REQ_MSP_PID:
        MSPcmdsend = MSP_PID;
        break;
#ifdef MENU_SERVO
      case REQ_MSP_SERVO_CONF:
        MSPcmdsend = MSP_SERVO_CONF;
        break;
#endif
#ifdef USE_MSP_PIDNAMES
      case REQ_MSP_PIDNAMES:
        MSPcmdsend = MSP_PIDNAMES;
        break;
#endif
#ifdef CORRECTLOOPTIME
      case REQ_MSP_LOOP_TIME:
        MSPcmdsend = MSP_LOOP_TIME;
        break;
#endif
      case REQ_MSP_BOX:
#ifdef BOXNAMES
        MSPcmdsend = MSP_BOXNAMES;
#else
        MSPcmdsend = MSP_BOXIDS;
#endif
        break;
      case REQ_MSP_FONT:
        MSPcmdsend = MSP_OSD;
        break;
#if defined DEBUGMW
      case REQ_MSP_DEBUG:
        MSPcmdsend = MSP_DEBUG;
        break;
#endif
#if defined SPORT
      case REQ_MSP_CELLS:
        MSPcmdsend = MSP_CELLS;
        break;
#endif
#ifdef MULTIWII_V24
      case REQ_MSP_NAV_STATUS:
        if (MwSensorActive & mode.gpsmission)
          MSPcmdsend = MSP_NAV_STATUS;
        break;
#endif
#ifdef CORRECT_MSP_BF1
      case REQ_MSP_CONFIG:
        MSPcmdsend = MSP_CONFIG;
        break;
#endif
#ifdef MENU_FIXEDWING
      case REQ_MSP_FW_CONFIG:
        MSPcmdsend = MSP_FW_CONFIG;
        break;
#endif
#ifdef HAS_ALARMS
      case REQ_MSP_ALARMS:
        MSPcmdsend = MSP_ALARMS;
        break;
#endif
#ifdef MSP_RTC_SUPPORT
      case REQ_MSP_RTC:
        MSPcmdsend = MSP_RTC;
        break;
#endif
      case REQ_MSP_VOLTAGE_METER_CONFIG:
        MSPcmdsend = MSP_VOLTAGE_METER_CONFIG;
        break;
#ifdef MSPV2
      case REQ_MSP2_INAV_AIR_SPEED:
        MSPcmdsend = MSP2_INAV_AIR_SPEED;
        break;
#endif
    }

    if (!fontMode) {
#ifdef KISS
      Serial.write(0x20);
#elif defined SKYTRACK
      DrawSkytrack();
#elif defined PROTOCOL_MSP
#ifdef CANVAS_SUPPORT
      if (!canvasMode)
#endif
      {
        if (MSPcmdsend != 0) {
#ifdef MSPV2
          if (MSPcmdsend > 254) {
            mspV2WriteRequest(MSPcmdsend, 0);
          }
          else
#endif
          {
            mspWriteRequest(MSPcmdsend & 0xff, 0);
          }
        }
      }
#endif // KISS
      MAX7456_DrawScreen();
    }

    ProcessSensors();       // using analogue sensors

    if ( allSec < INTRO_DELAY ) {
      displayIntro();
      timer.lastCallSign = onTime - CALLSIGNINTERVAL;
    }
    else
    {
      if (armed) {
        previousarmedstatus = 1;
        timer.disarmed = OSDSUMMARY;
        if (configMode == 1)
          configExit();
      }
#ifndef HIDESUMMARY
      if (previousarmedstatus && !armed) {
        configPage = 0;
        ROW = 10;
        COL = 1;
        configMode = 1;
        setMspRequests();
      }
#else
      if (previousarmedstatus && !armed) {
        previousarmedstatus = 0;
        configMode = 0;
      }
#endif //HIDESUMMARY      
      if (configMode)
      {
        displayConfigScreen();
      }
#ifdef CANVAS_SUPPORT
      else if (canvasMode)
      {
        // In canvas mode, we don't actively write the screen; just listen to MSP stream.
        if (lastCanvas + CANVAS_TIMO < currentMillis) {
          MAX7456_ClearScreen();
          canvasMode = false;
        }
      }
#endif
      else
      {
        setMspRequests();
#if defined USE_AIRSPEED_SENSOR
        useairspeed();
#endif //USE_AIRSPEED_SENSOR
        displayHorizon(MwAngle[0], MwAngle[1]);
#if defined FORCECROSSHAIR
        displayForcedCrosshair();
#endif //FORCECROSSHAIR
        displayVoltage();
        displayVidVoltage();
        if ((rssi > Settings[S_RSSI_ALARM]) || (timer.Blink2hz))
          displayRSSI();
        if (((amperage / 10) < Settings[S_AMPERAGE_ALARM]) || (timer.Blink2hz))
          displayAmperage();
        if (((!ampAlarming()) || timer.Blink2hz))
          displaypMeterSum();
        displayFlightTime();
#if defined (DISPLAYWATTS)
        displayWatt();
#endif //DISPLAYWATTS
#if defined (DISPLAYEFFICIENCY)
        displayEfficiency();
#endif //DISPLAYEFFICIENCY
#if defined (DISPLAYAVGEFFICIENCY)
        displayAverageEfficiency();
#endif //DISPLAYAVGEFFICIENCY
#ifdef SHOW_TEMPERATURE
        displayTemperature();
#endif
#ifdef VIRTUAL_NOSE
        displayVirtualNose();
#endif
        displayArmed();
        displayCurrentThrottle();
#ifdef FREETEXTLLIGHTS
        if (MwSensorActive & mode.llights) displayCallsign(getPosition(callSignPosition));
#elif  FREETEXTGIMBAL
        if (MwSensorActive & mode.camstab) displayCallsign(getPosition(callSignPosition));
#else
        if (fieldIsVisible(callSignPosition)) {
#ifdef PILOTICON
          if (Settings[S_CALLSIGN_ALWAYS] == 3) {
            displayIcon(getPosition(callSignPosition));
          }
          else if (Settings[S_CALLSIGN_ALWAYS] == 2) {
            if ( (onTime > (timer.lastCallSign + CALLSIGNINTERVAL))) { // Displays 4 sec every 60 secs
              if ( onTime > (timer.lastCallSign + CALLSIGNINTERVAL + CALLSIGNDURATION))
                timer.lastCallSign = onTime;
              displayCallsign(getPosition(callSignPosition));
            }
          }
          else if (Settings[S_CALLSIGN_ALWAYS] == 1) {
            displayCallsign(getPosition(callSignPosition));
          }
#else
          if (Settings[S_CALLSIGN_ALWAYS] == 2) {
            if ( (onTime > (timer.lastCallSign + CALLSIGNINTERVAL))) { // Displays 4 sec every 60 secs
              if ( onTime > (timer.lastCallSign + CALLSIGNINTERVAL + CALLSIGNDURATION))
                timer.lastCallSign = onTime;
              displayCallsign(getPosition(callSignPosition));
            }
          }
          else if (Settings[S_CALLSIGN_ALWAYS] == 1) {
            displayCallsign(getPosition(callSignPosition));
          }
#endif
        }
#endif
        displayHeadingGraph();
        displayHeading();
#if defined SUBMERSIBLE
 #if defined USEMS5837
        MwAltitude = (float)100 * MS5837sensor.depth();
 #endif //USEMS5837
        if (millis() > timer.fwAltitudeTimer) { // To make vario from Submersible altitude
          timer.fwAltitudeTimer += 1000;
          MwVario = MwAltitude - previousfwaltitude;
          previousfwaltitude = MwAltitude;
        }
#endif // SUBMERSIBLE

        displayAltitude(((int32_t)GPS_altitude*10),MwGPSAltPosition,SYM_GPS_ALT);
        displayAltitude(MwAltitude/10,MwAltitudePosition,SYM_ALT);
        displayClimbRate();
        displayVario();
        displayNumberOfSat();
        displayDirectionToHome();
        displayDistanceToHome();
        displayDistanceTotal();
        displayDistanceMax();
        displayAngleToHome();
        displayGPSdop();
        // displayfwglidescope(); //note hook for this is in display horizon function
        if (!armed) 
          GPS_speed = 0;
        display_speed(GPS_speed, GPS_speedPosition, SYM_SPEED_GPS);
        display_speed(AIR_speed, AIR_speedPosition, SYM_SPEED_AIR);
        displayWindSpeed(); // also windspeed if available
        displayItem(MAX_speedPosition, speedMAX, SYM_MAX, speedUnitAdd[Settings[S_UNITSYSTEM]], 0 );
        displayGPSPosition();

#ifdef GPSTIME
        if (fieldIsVisible(GPS_timePosition))
          displayDateTime();
#endif
#ifdef MAPMODE
        mapmode();
#endif
        displayMode();
#ifdef I2CERROR
        displayI2CError();
#endif
#ifdef SPORT
        if (MwSensorPresent)
          displayCells();
#endif
#ifdef DEBUG
        displayDebug();
#endif
#ifdef HAS_ALARMS
        displayAlarms();
#endif
#ifdef MAV_STATUS
        displayMAVstatustext();
#endif
      }
    }
  }  // End of fast Timed Service Routine (50ms loop)

  if (timer.halfSec >= 5) {
    timer.halfSec = 0;
    timer.Blink2hz = ! timer.Blink2hz;

#if 0
    // XXX What is this for? On-Arm power setting?
    // XXX May be "Power up at minimum power, then goto stored power on-arming."for stick based blind operation or something similar
    // XXX Leave commented out until intension is known.
#ifdef VTX_RTC6705
    vtx_set_power(armed ? vtxPower : 0);
#endif // VTX_RTC6705
#endif
  }

  if (millis() > timer.seconds + 1000)  // this execute 1 time a second
  {
#if defined (GPSTIME) && !defined (UBLOX)
    datetime.unixtime++;
    updateDateTime(datetime.unixtime);
#endif //GPSTIME    
    if (timer.armedstatus > 0)
      timer.armedstatus--;
    timer.seconds += 1000;
    timer.tenthSec = 0;
#ifdef MAV_STATUS
    if (timer.MAVstatustext > 0)
      timer.MAVstatustext--;
#endif
#ifdef DEBUGDPOSLOOP
    framerate = timer.loopcount;
    timer.loopcount = 0;
#endif
#ifdef DEBUGDPOSPACKET
    packetrate = timer.packetcount;
    timer.packetcount = 0;
#endif
#ifdef DEBUGDPOSRX
    serialrxrate = timer.serialrxrate;
    timer.serialrxrate = 0;
#endif
    onTime++;
#if defined(AUTOCAM) || defined(MAXSTALLDETECT)
    if (!fontMode)
      MAX7456CheckStatus();
#endif
#ifdef ALARM_GPS
    if (timer.GPS_active == 0) {
      GPS_numSat = 0;
    }
    else {
      timer.GPS_active--;
    }
#endif // ALARM_GPS 
    if (timer.disarmed > 0) {
      timer.disarmed--;
    }
    if (timer.MSP_active > 0) {
      timer.MSP_active--;
    }
    if (timer.GUI_active > 0) {
      timer.GUI_active--;
#if defined GPSOSD
      timer.GPS_initdelay = 2;
#endif
    }
#if defined(GPSOSD) && !defined(NAZA)
    if (timer.GPS_initdelay == 1) {
      GPS_SerialInit();
    }
    if (timer.GPS_initdelay > 0) {
      timer.GPS_initdelay--;
    }
#endif

    if (!armed) {
#ifndef MAPMODENORTH
      armedangle = MwHeading;
#endif
    }
    else {
      flyTime++;
      flyingTime++;
      configMode = 0;
      setMspRequests();
    }
    allSec++;
    /*
        if((timer.accCalibrationTimer==1)&&(configMode)) {
          mspWriteRequest(MSP_ACC_CALIBRATION,0);
          timer.accCalibrationTimer=0;
        }
    */
#ifdef PROTOCOL_MSP
    if ((timer.magCalibrationTimer == 1) && (configMode)) {
      mspWriteRequest(MSP_MAG_CALIBRATION, 0);
      timer.magCalibrationTimer = 0;
    }
    if (timer.magCalibrationTimer > 0) timer.magCalibrationTimer--;
#endif
    if (timer.rssiTimer > 0) timer.rssiTimer--;
  }
  //  setMspRequests();
  serialMSPreceive(1);
#ifdef FIXEDLOOP
  delay(1);
#endif

}  // End of main loop
#endif //main loop


//------------------------------------------------------------------------
//FONT management

uint8_t safeMode() {
  return 1;	// XXX
}


// Font upload queue implementation.
// Implement a window for curr + the previous 6 requests.
// First-chance to retransmit at curr-3 (retransmitQueue & 0x10)
// First-chance retransmit marked as used at retransmitQueue |= 0x01
// 2 to N-chance retransmit marked at curr-6 (retransmitQueue & 0x02)
void initFontMode() {
  if (armed || configMode || fontMode || !safeMode())
    return;
  // queue first char for transmition.
  retransmitQueue = 0x80;
  fontMode = 1;
  //  setMspRequests();
}


void fontCharacterReceived(uint8_t cindex) {
  if (!fontMode)
    return;

  uint8_t bit = (0x80 >> (nextCharToRequest - cindex));

  // Just received a char..
  if (retransmitQueue & bit) {
    // this char war requested and now received for the first time
    retransmitQueue &= ~bit;  // mark as already received
    write_NVM(cindex);       // Write to MVRam
  }
}

int16_t getNextCharToRequest() {
  if (nextCharToRequest != lastCharToRequest) { // Not at last char
    if (retransmitQueue & 0x02) {               // Missed char at curr-6. Need retransmit!
      return nextCharToRequest - 6;
    }

    if ((retransmitQueue & 0x11) == 0x10) {     // Missed char at curr-3. First chance retransmit
      retransmitQueue |= 0x01;                  // Mark first chance as used
      return nextCharToRequest - 3;
    }

    retransmitQueue = (retransmitQueue >> 1) | 0x80; // Add next to queue
    return nextCharToRequest++;                      // Notice post-increment!
  }

  uint8_t temp1 = retransmitQueue & ~0x01;
  uint8_t temp2 = nextCharToRequest - 6;

  if (temp1 == 0) {
    fontMode = 0;                            // Exit font mode
    //  setMspRequests();
    return -1;
  }

  // Already at last char... check for missed characters.
  while (!(temp1 & 0x03)) {
    temp1 >>= 1;
    temp2++;
  }

  return temp2;
}


//------------------------------------------------------------------------
// MISC

void resetFunc(void)
{
#ifdef I2C_UB_SUPPORT
  WireUB.end();
#endif
#if defined HARDRESET
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE) | (1 << WDIE);
  WDTCSR = (1 << WDIE) | (0 << WDE) | (0 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0);
  while (1);
#elif defined BOOTRESET
  asm volatile ("  jmp 0x3800");
#else
  asm volatile ("  jmp 0");
#endif
}


void setMspRequests() {
  if (fontMode) {
    modeMSPRequests = REQ_MSP_FONT;
  }
  else if (configMode) {
    modeMSPRequests =
      REQ_MSP_STATUS |
      REQ_MSP_RAW_GPS |
      REQ_MSP_ATTITUDE |
      REQ_MSP_RAW_IMU |
      REQ_MSP_ALTITUDE |
      REQ_MSP_RC_TUNING |
      REQ_MSP_PID_CONTROLLER |
      REQ_MSP_ANALOG |
#ifdef USE_MSP_PIDNAMES
      REQ_MSP_PIDNAMES |
#endif
      REQ_MSP_PID |
#ifdef CORRECTLOOPTIME
      REQ_MSP_LOOP_TIME |
#endif
#ifdef CORRECT_MSP_BF1
      REQ_MSP_CONFIG |
#endif
#ifdef DEBUGMW
      REQ_MSP_DEBUG |
#endif
#ifdef SPORT
      REQ_MSP_CELLS |
#endif
#ifdef HAS_ALARMS
      REQ_MSP_ALARMS |
#endif
#ifdef MENU_SERVO
      REQ_MSP_SERVO_CONF |
#endif
#ifdef MENU_FIXEDWING
      REQ_MSP_FW_CONFIG |
#endif
#ifdef USE_FC_VOLTS_CONFIG
#if defined(CLEANFLIGHT) || defined(BETAFLIGHT)
      REQ_MSP_VOLTAGE_METER_CONFIG |
#else
      REQ_MSP_MISC |
#endif
#endif
       REQ_MSP_RC;
  }
  else {
    modeMSPRequests =
      REQ_MSP_STATUS |
#ifdef DEBUGMW
      REQ_MSP_DEBUG |
#endif
#ifdef SPORT
      REQ_MSP_CELLS |
#endif
#ifdef MSP_USE_GPS
      REQ_MSP_RAW_GPS |
      REQ_MSP_COMP_GPS |
#endif //MSP_USE_GPS
#ifdef MSP_USE_BARO
      REQ_MSP_ALTITUDE |
#endif //MSP_USE_BARO
#ifdef HAS_ALARMS
      REQ_MSP_ALARMS |
#endif
#ifdef MSP_SPEED_LOW
      REQ_MSP_ATTITUDE |
#endif
#ifdef MSP_USE_ANALOG
      REQ_MSP_ANALOG |
#endif //MSP_USE_ANALOG  
#ifdef MSPV2
      REQ_MSP2_INAV_AIR_SPEED |
#endif
      REQ_MSP_RC;

    if (!armed) {
      modeMSPRequests |= 
        REQ_MSP_BOX |
#if defined INTRO_FC && defined PROTOCOL_MSP
        REQ_MSP_FC_VERSION |
#endif // INTRO_FC && defined PROTOCOL_MSP      
#ifdef USE_FC_VOLTS_CONFIG
#if defined(CLEANFLIGHT) || defined(BETAFLIGHT)
        REQ_MSP_VOLTAGE_METER_CONFIG |
#else
        REQ_MSP_MISC |
#endif // defined(CLEANFLIGHT) || defined(BETAFLIGHT)
#endif // USE_FC_VOLTS_CONFIG
#ifdef MSP_RTC_SUPPORT
        REQ_MSP_RTC |
#endif // MSP_RTC_SUPPORT
        0;
    }
#if defined MULTIWII_V24
    if (MwSensorActive & mode.gpsmission)
      modeMSPRequests |= REQ_MSP_NAV_STATUS;
#endif
  }
  if (timer.GUI_active != 0) {
    modeMSPRequests = 0;
    queuedMSPRequests = 0;
  }
  queuedMSPRequests &= modeMSPRequests;   // so we do not send requests that are not needed.
}


void writeEEPROM(void) // OSD will only change 8 bit values. GUI changes directly
{
  for (uint8_t en = 0; en < EEPROM_SETTINGS; en++) {
    EEPROM.write(en, Settings[en]);
  }
  for (uint8_t en = 0; en < EEPROM16_SETTINGS; en++) {
    uint16_t pos  = EEPROM_SETTINGS + (en * 2);
    uint16_t data = Settings16[en];
    write16EEPROM(pos, data);
  }
  EEPROM.write(0, EEPROMVER);
}


void readEEPROM(void)
{
  for (uint8_t en = 0; en < EEPROM_SETTINGS; en++) {
    Settings[en] = EEPROM.read(en);
  }

  // config dependant - set up interrupts
#if defined INTC3
  if (Settings[S_MWRSSI] == 1) {
    DDRC &= ~(1 << DDC3); //  PORTC |= (1 << PORTC3);
    //DDRC &=B11110111;
  }
#endif
#if defined INTD5
  DDRD &= ~(1 << DDD5); //  PORTD |= (1 << PORTD5);
#endif
  cli();
#if defined INTC3
  if (Settings[S_MWRSSI] == 1) {
    if ((PCMSK1 & (1 << PCINT11)) == 0) {
      PCICR |=  (1 << PCIE1);
      PCMSK1 |= (1 << PCINT11);
    }
  }
#endif
#if defined INTD5
  if ((PCMSK2 & (1 << PCINT21)) == 0) {
    PCICR |=  (1 << PCIE2);
    PCMSK2 |= (1 << PCINT21);
  }
#endif
  sei();


  // config dependant - voltage reference
#ifdef IMPULSERC_HELIX
  //Ignore setting because this is critical to making sure we can detect the
  //VTX power jumper being installed. If we aren't using 5V ref there is
  //the chance we will power up on wrong frequency.
  Settings[S_VREFERENCE] = 1;
#endif //IMPULSERC_HELIX 

  if (Settings[S_VREFERENCE])
    analogReference(DEFAULT);
  else
    analogReference(INTERNAL);


  for (uint8_t en = 0; en < EEPROM16_SETTINGS; en++) {
    uint16_t pos = (en * 2) + EEPROM_SETTINGS;
    Settings16[en] = EEPROM.read(pos);
    uint16_t xx = EEPROM.read(pos + 1);
    Settings16[en] = Settings16[en] + (xx << 8);
  }

  // Read screen layouts
  uint16_t EEPROMscreenoffset = EEPROM_SETTINGS + (EEPROM16_SETTINGS * 2) + (screenlayout * POSITIONS_SETTINGS * 2);
  for (uint8_t en = 0; en < POSITIONS_SETTINGS; en++) {
    uint16_t pos = (en * 2) + EEPROMscreenoffset;
    screenPosition[en] = EEPROM.read(pos);
    uint16_t xx = (uint16_t)EEPROM.read(pos + 1) << 8;
    screenPosition[en] = screenPosition[en] + xx;
    if (flags.signaltype == 1) {
      uint16_t x = screenPosition[en] & 0x1FF;
      if (x > LINE06) screenPosition[en] = screenPosition[en] + LINE;
      if (x > LINE09) screenPosition[en] = screenPosition[en] + LINE;
    }
  }
}


void checkEEPROM(void)
{
  uint8_t EEPROM_Loaded = EEPROM.read(0);
  if (EEPROM_Loaded != EEPROMVER) {
    for (uint8_t en = 0; en < EEPROM_SETTINGS; en++) {
      EEPROM.write(en, pgm_read_byte(&EEPROM_DEFAULT[en]));
    }
    for (uint8_t en = 0; en < EEPROM16_SETTINGS; en++) {
      uint16_t pos  = EEPROM_SETTINGS + (en * 2);
      uint16_t data = pgm_read_word(&EEPROM16_DEFAULT[en]);
      write16EEPROM(pos, data);
    }
    uint16_t base = EEPROM_SETTINGS + (EEPROM16_SETTINGS * 2);
    for (uint8_t ilayout = 0; ilayout < 3; ilayout++) {
      for (uint8_t en = 0; en < POSITIONS_SETTINGS * 2; en++) {
        uint16_t pos  = base + (en * 2);
        uint16_t data = pgm_read_word(&SCREENLAYOUT_DEFAULT[en]);
        write16EEPROM(pos, data);
      }
      base += POSITIONS_SETTINGS * 2;
    }
  }
}


void write16EEPROM(uint16_t pos, uint16_t data)
{
  EEPROM.write(pos  , data & 0xff);
  EEPROM.write(pos + 1, data >> 8  );
}


void gpsdistancefix(void) {
  int8_t speedband;
  static int8_t oldspeedband;
  static int8_t speedcorrection = 0;
  if (GPS_distanceToHome < 10000) speedband = 0;
  else if (GPS_distanceToHome > 50000) speedband = 2;
  else {
    speedband = 1;
    oldspeedband = speedband;
  }
  if (speedband == oldspeedband) {
    if (oldspeedband == 0) speedcorrection--;
    if (oldspeedband == 2) speedcorrection++;
    oldspeedband = speedband;
  }
  GPS_distanceToHome = (speedcorrection * 65535) + GPS_distanceToHome;
}


void ProcessSensors(void) {
  /*
    special note about filter: last row of array = averaged reading
  */
  //-------------- ADC sensor / PWM RSSI / FC data read into filter array
  static uint8_t sensorindex;
  uint16_t sensortemp;
  for (uint8_t sensor = 0; sensor < SENSORTOTAL; sensor++) {
    sensortemp = analogRead(sensorpinarray[sensor]);
    //--- override with FC voltage data if enabled
    if (sensor == 0) {
      if (Settings[S_MAINVOLTAGE_VBAT]) {
        sensortemp = MwVBat;
      }
    }
#ifdef MAV_VBAT2
    // assume vbat2 on FC if vbat1 is
    if (sensor == 1) {
      if (Settings[S_MAINVOLTAGE_VBAT]) {
        sensortemp = MwVBat2;
      }
    }
#endif
    //--- override with PWM, FC RC CH or FC RSSI data if enabled
    if (sensor == 4) {
      if (Settings[S_MWRSSI] == 3) { // RSSI from a TX channel
        sensortemp = MwRcData[Settings[S_RSSI_CH]] >> 1;
      }
      else if (Settings[S_MWRSSI] == 2) { // RSSI from Flight controller
        sensortemp = MwRssi;
      }
      else if (Settings[S_MWRSSI] == 1) { // RSSI from direct OSD - PWM
        sensortemp = pwmRSSI >> 1;
        if (sensortemp == 0) { // timed out - use previous
          sensortemp = sensorfilter[sensor][sensorindex];
        }
      }
      else { // RSSI from direct OSD - Analog
      }
    }
    //--- Apply filtering
#if defined FILTER_HYSTERYSIS  // Hysteris incremental averaged change    
    static uint16_t shfilter[SENSORTOTAL];
    int16_t diff = (sensortemp << FILTER_HYSTERYSIS) - shfilter[sensor];
    if (abs(diff) > (FHBANDWIDTH << FILTER_HYSTERYSIS)) {
      shfilter[sensor] = sensortemp << FILTER_HYSTERYSIS;
    }
    else if (diff > 0) {
      shfilter[sensor]++;
    }
    else if (diff < 0) {
      shfilter[sensor]--;
    }
    sensorfilter[sensor][SENSORFILTERSIZE] = (shfilter[sensor] >> FILTER_HYSTERYSIS << 3);
#elif defined FILTER_AVG   // Use averaged change    
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];
    sensorfilter[sensor][sensorindex] = (sensorfilter[sensor][sensorindex] + sensortemp) >> 1;
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] + sensorfilter[sensor][sensorindex];
#else                      // No filtering
    sensorfilter[sensor][SENSORFILTERSIZE] = sensortemp << 3;
#endif
  }

  //-------------- Voltage
  if (!Settings[S_MAINVOLTAGE_VBAT]) { // not MWII
    uint16_t voltageRaw = sensorfilter[0][SENSORFILTERSIZE];
    if (!Settings[S_VREFERENCE]) {
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (DIVIDER1v1);
    }
    else {
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (DIVIDER5v);
    }
  }
  else {
    voltage = sensorfilter[0][SENSORFILTERSIZE] >> 3;
  }

  vidvoltageWarning = Settings[S_VIDVOLTAGEMIN];
  uint16_t vidvoltageRaw = sensorfilter[1][SENSORFILTERSIZE];
  if (!Settings[S_VREFERENCE]) {
    vidvoltage = float(vidvoltageRaw) * Settings[S_VIDDIVIDERRATIO] * (DIVIDER1v1);
  }
  else {
    vidvoltage = float(vidvoltageRaw) * Settings[S_VIDDIVIDERRATIO] * (DIVIDER5v);
  }

  //-------------- Temperature
#ifdef SHOW_TEMPERATURE
#if defined USEMS5837
  temperature = (float)(10 * MS5837sensor.temperature());
#elif defined PROTOCOL_MAVLINK && !defined USE_TEMPERATURE_SENSOR
#else
  temperature = (sensorfilter[3][SENSORFILTERSIZE] >> 3);
  temperature = map (temperature, Settings16[S16_AUX_ZERO_CAL], Settings16[S16_AUX_CAL], 0 , 100);
#endif
#endif

  //-------------- Current
  if (Settings[S_MWAMPERAGE] == 2) { // Virtual
    uint32_t Vthrottle = map(MwRcData[THROTTLESTICK], LowT, HighT, 0, 100);
    Vthrottle = constrain(Vthrottle, 0, 100);
    amperage = (Vthrottle + (Vthrottle * Vthrottle * 0.02)) * Settings16[S16_AMPDIVIDERRATIO] * 0.01;
    if (armed)
      amperage += Settings16[S16_AMPZERO];
    else
      amperage = Settings16[S16_AMPZERO];
  }
  else if (Settings[S_MWAMPERAGE] == 1) { // from FC
    if (MWAmperage < 0)
      amperage = (MWAmperage - AMPERAGE_DIV / 2) / AMPERAGE_DIV;
    else
      amperage = (MWAmperage + AMPERAGE_DIV / 2) / AMPERAGE_DIV;
  }
  else { // Analog
    amperage = sensorfilter[2][SENSORFILTERSIZE] >> 3;
    amperage = map(amperage, Settings16[S16_AMPZERO], AMPCALHIGH, AMPCALLOW, Settings16[S16_AMPDIVIDERRATIO]);
    if (amperage < 0) amperage = 0;
  }


  //-------------- RSSI

  rssi = sensorfilter[4][SENSORFILTERSIZE] >> 3; // filter and remain 16 bit
  if (configMode) {
    if ((timer.rssiTimer == 15)) {
      Settings16[S16_RSSIMAX] = rssi; // tx on
    }
    if ((timer.rssiTimer == 1)) {
      Settings16[S16_RSSIMIN] = rssi; // tx off
      timer.rssiTimer = 0;
    }
  }
  rssi = map(rssi, Settings16[S16_RSSIMIN], Settings16[S16_RSSIMAX], 0, 100);
  rssi = constrain(rssi, 0, 100);

  //-------------- For filter support
  sensorindex++;
  if (sensorindex >= SENSORFILTERSIZE)
    sensorindex = 0;
}


#if defined INTC3
ISR(PCINT1_vect) { // Default Arduino A3 Atmega C3
  static uint16_t PulseStart;
  static uint8_t  RCchan = 1;
  static uint16_t LastTime = 0;
  uint8_t pinstatus;
  pinstatus = PINC;
#define PWMPIN1 DDC3
  sei();
  uint16_t PulseDuration;
  uint16_t CurrentTime = micros();

  if ((CurrentTime - LastTime) > 3000) RCchan = 1; // assume this is PPM gap
  LastTime = CurrentTime;
  if (!(pinstatus & (1 << PWMPIN1))) { // measures low duration
    PulseDuration = CurrentTime - PulseStart;
    if ((950 < PulseDuration) && (PulseDuration < 2150)) {
      pwmval1 = PulseDuration;
#ifdef INTD5 // Aeromax Hardware so this must be PWMRSSI only
      pwmRSSI = PulseDuration;
#else
#if defined PPM_CONTROL
#define INTC3_PPM_PWM 1
#else
#define INTC3_PPM_PWM 0
#endif
      if (INTC3_PPM_PWM) { //PWM
        if (RCchan <= TX_CHANNELS) // avoid array overflow if > standard ch PPM
          MwRcData[RCchan] = PulseDuration; // Val updated
#if defined (TX_GUI_CONTROL)
        if (RCchan == 4)
          reverseChannels();
#endif // TX_PRYT

      }
      else { //PWM
#ifdef NAZA
        Naza.mode = 0;
        if (PulseDuration > NAZA_PMW_HIGH) {
          Naza.mode = NAZA_MODE_HIGH;
        }
        else if (PulseDuration > NAZA_PMW_MED) {
          Naza.mode = NAZA_MODE_MED;
        }
        else if (PulseDuration > NAZA_PWM_LOW) {
          Naza.mode = NAZA_MODE_LOW;
        }
#endif
#ifdef PWM_OSD_SWITCH
        MwRcData[rcswitch_ch] = PulseDuration;
#elif defined PWM_THROTTLE
        MwRcData[THROTTLESTICK] = PulseDuration;
#else
        pwmRSSI = PulseDuration;
#endif
      }
#endif
    }
    RCchan++;
  }
  else {
    PulseStart = CurrentTime;
  }
}
#endif

#if defined INTD5
ISR(PCINT2_vect) { // // Secondary Arduino D5 Atmega D5
  static uint16_t PulseStart;
  static uint8_t  RCchan = 1;
  static uint16_t LastTime = 0;
  uint8_t pinstatus;
#define PWMPIN2 DDD5
  pinstatus = PIND;
  sei();
  uint16_t PulseDuration;
  uint16_t CurrentTime = micros();

  if ((CurrentTime - LastTime) > 3000) RCchan = 1; // assume this is PPM gap
  LastTime = CurrentTime;

  if (!(pinstatus & (1 << PWMPIN2))) { // measures low duration
    PulseDuration = CurrentTime - PulseStart;
    if ((950 < PulseDuration) && (PulseDuration < 2150)) {
      pwmval2 = PulseDuration;
      if (Settings[S_PWM_PPM]) { //PPM
        if (RCchan <= TX_CHANNELS) // avoid array overflow if > standard ch PPM
          MwRcData[RCchan] = PulseDuration; // Val updated
      }
      else { //PWM
#ifdef NAZA
        Naza.mode = 0;
        if (PulseDuration > NAZA_PMW_HIGH) {
          Naza.mode = NAZA_MODE_HIGH;
        }
        else if (PulseDuration > NAZA_PMW_MED) {
          Naza.mode = NAZA_MODE_MED;
        }
        else if (PulseDuration > NAZA_PWM_LOW) {
          Naza.mode = NAZA_MODE_LOW;
        }
#endif
#ifdef PWM_OSD_SWITCH
        MwRcData[rcswitch_ch] = PulseDuration;
#else // assume throttle connected if not using PPM
        MwRcData[THROTTLESTICK] = PulseDuration;
#endif
      }
    }
    RCchan++;
  }
  else {
    PulseStart = CurrentTime;
  }
}
#endif


void EEPROM_clear() {
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);
}

int16_t filter16( int16_t filtered, int16_t raw, const byte k) {
  filtered = filtered + (raw - filtered) / k; // note extreme values may overrun. 32 it if required.
  return filtered;
}

int32_t filter32( int32_t filtered, int32_t raw, const byte k) {
  filtered = filtered + (raw - filtered) / k; // note extreme values may overrun. 32 it if required.
  return filtered;
}

int32_t filter32F( float filtered, float raw, const byte k) {
  filtered = filtered + (raw - filtered) / k; // note extreme values may overrun. 32 it if required.
  return (int32_t) filtered;
}

#if defined USE_AIRSPEED_SENSOR
void useairspeed() {
#define AIRDENSITY  1.225 // Density of air kg/m3
  int16_t pressuresensor = (int16_t)(sensorfilter[3][SENSORFILTERSIZE] >> 3) - Settings16[S16_AUX_ZERO_CAL];
  if (pressuresensor < 0) {
    pressuresensor = 0;
  }
  pressuresensor = ((int32_t)pressuresensor * Settings16[S16_AUX_CAL]) / 500;
  constrain(pressuresensor, -410, 410);
  int16_t Pa = map(pressuresensor, -410, 410, -2000, 2000); // Pressure - actual pascals
  AIR_speed = (int32_t)100 * sqrt((2 * Pa) / AIRDENSITY); // Speed required in cm/s
}
#endif //USE_AIRSPEED_SENSOR 

void reverseChannels(void) { //ifdef (TX_REVERSE)
  for (uint8_t i = 1; i <= 4; i++) {
    if (Settings[S_TX_CH_REVERSE] & (1 << i))
      MwRcData[i] = 3000 - MwRcData[i];
  }
}


