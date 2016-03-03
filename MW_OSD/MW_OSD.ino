/*
Scarab NG OSD ... 

 Subject to exceptions listed below, this program is free software: you can redistribute it and/or modify
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
 Where there are exceptions, these take precedence over the genereric licencing approach
 Elements of the code provided by Pawelsky (DJI specific) are not for commercial use. See headers in individual files for further details.  
 Libraries used and typically provided by compilers may have licening terms stricter than that of GNU 3
 
*/

//------------------------------------------------------------------------
#define MEMCHECK 3 // to enable memory checking and set debug[x] value
#if 1
  __asm volatile ("nop");
#endif
#ifdef MEMCHECK
  extern uint8_t _end;  //end of program variables 
  extern uint8_t __stack; //start of stack (highest RAM address)
  void PaintStack(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));    //Make sure this is executed at the first time
  void PaintStack(void){
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
                    "    breq .loop"::);} 

  uint16_t UntouchedStack(void){
      const uint8_t *ptr = &_end;
      uint16_t       count = 0;
      while(*ptr == 0xa5 && ptr <= &__stack){ 
          ptr++; count++;}
      return count;} 
#endif

//------------------------------------------------------------------------
//Main defines
char screen[480];      // Main screen ram for MAX7456
char screenBuffer[20];   
#if defined LOADFONT_LARGE
  #include "fontL.h"
#elif defined LOADFONT_DEFAULT 
  #include "fontD.h"
#elif defined LOADFONT_BOLD 
  #include "fontB.h"
#endif
#define MWVERS "MW-OSD - R1.6"  
#define MWOSDVER 12      // for eeprom layout verification    was 9  
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "Config.h"
#include "Def.h"
#include "symbols.h"
#include "GlobalVariables.h"
#include "EEPROMdata.h"
#include "Max7456.h"
#include "calcs.h"
#include "fontdata.h"
#include "Serial.h"
#include "Screen.h"
#include "GPS.h"
#include "math.h"
#if defined NAZA
  #include "Naza.h"
#endif

unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
unsigned long previous_millis_sync =0;
unsigned long previous_millis_rssi =0;


//------------------------------------------------------------------------
void setup(){
  Serial.begin(BAUDRATE);
  //---- override UBRR with MWC settings
  uint8_t h = ((F_CPU  / 4 / (BAUDRATE) -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / (BAUDRATE) -1) / 2);
  UCSR0A  |= (1<<U2X0); UBRR0H = h; UBRR0L = l; 
  //---
  Serial.flush();
  pinMode(PWMRSSIPIN, INPUT);
  pinMode(RSSIPIN, INPUT);
  pinMode(LEDPIN,OUTPUT);
  #if defined (INTPWMRSSI) || defined (PPMOSDCONTROL)
    initRSSIint();
  #endif
  #if defined EEPROM_CLEAR //Read settings from EEPROM, clear settings if told to do so
    EEPROM_clear();
  #endif  
  checkEEPROM();
  readEEPROM();
  #ifndef STARTUPDELAY
    #define STARTUPDELAY 500
  #endif
  delay(STARTUPDELAY);
  if (Settings[S_VREFERENCE]){
    analogReference(DEFAULT);}
  else{
    analogReference(INTERNAL);}
  MAX7456Setup(); //initialize the MAX chip
  #if defined GPSOSD
    GPS_SerialInit();
  #endif
  #if defined FORCESENSORS
    MwSensorPresent |=GPSSENSOR;
    MwSensorPresent |=BAROMETER;
    MwSensorPresent |=MAGNETOMETER;
    MwSensorPresent |=ACCELEROMETER;
  #endif
  setMspRequests();
  #ifdef ALWAYSARMED
    armed=1;
  #endif
} // End setup

//------------------------------------------------------------------------
//If we are loading fonts, don't load the main program, load from fontdata.h
//If we are not loading fonts, begin with the proper main loop
#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
  uint8_t fontStatus=0;
  boolean ledstatus=HIGH;
  void loop(){ 
    switch(fontStatus){
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
    digitalWrite(LEDPIN,LOW);
  }
#else
  // ampAlarming returns true if the total consumed mAh is greater than
  // the configured alarm value (which is stored as 100s of amps)
  bool ampAlarming() {
    int used = pMeterSum > 0 ? pMeterSum : (amperagesum / 360);
    return used > (Settings[S_AMPER_HOUR_ALARM]*100);}


  //------------------------------------------------------------------------
  //Begin main loop
  void loop(){
    if (flags.reset){
      resetFunc();}
    #ifdef MEMCHECK
      debug[MEMCHECK] = UntouchedStack();
    #endif
    #ifdef PWMTHROTTLE
      MwRcData[THROTTLESTICK] = pwmRSSI;
    #endif //THROTTLE_RSSI
    #if defined (OSD_SWITCH_RC)
      uint8_t rcswitch_ch = Settings[S_RCWSWITCH_CH];
      screenlayout=0;
      if (Settings[S_RCWSWITCH]){
        #ifdef OSD_SWITCH_RSSI
          MwRcData[rcswitch_ch]=pwmRSSI;      
        #endif
        if (MwRcData[rcswitch_ch] > 1600){
          screenlayout=1;}
        else if (MwRcData[rcswitch_ch] > 1400){
          screenlayout=2;}}
      else{
        if (MwSensorActive&mode.osd_switch){
          screenlayout=1;}}
    #else
      if (MwSensorActive&mode.osd_switch){
        screenlayout=1;}
      else{  
        screenlayout=0;}
    #endif
    if (screenlayout!=oldscreenlayout){
      readEEPROM_screenlayout();}
    oldscreenlayout=screenlayout;  //Make the old screen layout match current layout
    if(timer.tenthSec>5){ // Blink Basic Sanity Test Led at 0.5hz
      digitalWrite(LEDPIN,HIGH);}
    else{
      digitalWrite(LEDPIN,LOW);}
  
  
    //---------------  Start Timed Service Routines  ---------------------------------------
    unsigned long currentMillis = millis();
  
    #ifdef MSP_SPEED_HIGH
      if((currentMillis - previous_millis_sync) >= sync_speed_cycle){  // (Executed > NTSC/PAL hz 33ms)
        previous_millis_sync = previous_millis_sync+sync_speed_cycle;    
        if(!fontMode){
          mspWriteRequest(MSP_ATTITUDE,0);}}
    #endif //MSP_SPEED_HIGH
    
    #ifdef INTPWMRSSI
    // to prevent issues with high pulse RSSi consuming CPU
      if((currentMillis - previous_millis_rssi) >= (1000/RSSIhz)){
        previous_millis_rssi = currentMillis; 
        initRSSIint();}
    #endif // INTPWMRSSI
  
    // Start of slow Timed Service Routine (100ms loop)
    if((currentMillis - previous_millis_low) >= lo_speed_cycle){  // 10 Hz (Executed every 100ms)
      previous_millis_low = previous_millis_low+lo_speed_cycle;
      timer.tenthSec++;
      timer.halfSec++;
      timer.Blink10hz=!timer.Blink10hz;
      calculateTrip();
      if (Settings[S_AMPER_HOUR]){
        amperagesum += amperage;}
      #ifndef GPSOSD
        #ifdef MSP_SPEED_MED
          if(!fontMode){
            mspWriteRequest(MSP_ATTITUDE,0);}
        #endif //MSP_SPEED_MED  
      #endif //GPSOSD
     } // End of slow Timed Service Routine (100ms loop)
  
    //Start of fast Timed Service Routine (50ms loop)
    if((currentMillis - previous_millis_high) >= hi_speed_cycle){  // 20 Hz or 100hz in MSP high mode
      previous_millis_high = previous_millis_high+hi_speed_cycle;
      uint8_t MSPcmdsend=0;
      if(queuedMSPRequests == 0){
        queuedMSPRequests = modeMSPRequests;}
      uint32_t req = queuedMSPRequests & -queuedMSPRequests;
      queuedMSPRequests &= ~req;
      switch(req){
        case REQ_MSP_IDENT:
         MSPcmdsend = MSP_IDENT;
          break;
        case REQ_MSP_STATUS:
          MSPcmdsend = MSP_STATUS;
          break;
        case REQ_MSP_RC:
          MSPcmdsend = MSP_RC;
          break;
        case REQ_MSP_RAW_GPS:
          MSPcmdsend = MSP_RAW_GPS;
          break;
        case REQ_MSP_COMP_GPS:
          MSPcmdsend = MSP_COMP_GPS;
          break;
      #ifdef MSP_SPEED_LOW
        case REQ_MSP_ATTITUDE:
          MSPcmdsend = MSP_ATTITUDE;
          break;
      #endif //MSP_SPEED_LOW
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
        case REQ_MSP_LOOP_TIME:
          MSPcmdsend = MSP_LOOP_TIME;
          break;        
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
          if(MwSensorActive&mode.gpsmission){
            MSPcmdsend = MSP_NAV_STATUS;}
          break;
      #endif
      #ifdef CORRECT_MSP_BF1
        case REQ_MSP_CONFIG:
          MSPcmdsend = MSP_CONFIG;
          break;
      #endif
      #ifdef HAS_ALARMS
        case REQ_MSP_ALARMS:
          MSPcmdsend = MSP_ALARMS;
          break;
      #endif
      } //end of req switch
      if(!fontMode){
        #ifndef GPSOSD
          mspWriteRequest(MSPcmdsend, 0);      
        #endif
        MAX7456_DrawScreen();}
      ProcessSensors(); // using analogue sensors
      #ifndef INTRO_DELAY 
        #define INTRO_DELAY 8
      #endif
      if( allSec < INTRO_DELAY ){   //display the intro
        displayIntro();
        timer.lastCallSign=onTime-CALLSIGNINTERVAL;}
      else{                        //otherwise, do normal things
        if(armed){
          previousarmedstatus=1;
            if (configMode==1){
              configExit();}}
        #ifndef HIDESUMMARY
          if(previousarmedstatus && !armed){
            armedtimer=20;
            configPage=0;
            ROW=10;
            COL=1;
            configMode=1;
            setMspRequests();}
        #else
          if(previousarmedstatus && !armed){
            previousarmedstatus=0;
            configMode=0;}
        #endif //HIDESUMMARY
  
        //if we are in config mode, show the config screen
        //otherwise, continue
        if(configMode){
          displayConfigScreen();}
        else{
          setMspRequests();
          #if defined USE_AIRSPEED_SENSOR
            useairspeed();
          #endif
            if(MwSensorPresent&ACCELEROMETER){
              displayHorizon(MwAngle[0],MwAngle[1]);}
          #if defined FORCECROSSHAIR
            displayForcedCrosshair();
          #endif
          if(Settings[S_DISPLAYVOLTAGE]){
            displayVoltage();}
          if(Settings[S_VIDVOLTAGE]){
            displayVidVoltage();}
          if(Settings[S_DISPLAYRSSI]&&((rssi>Settings[S_RSSI_ALARM])||(timer.Blink2hz))){
            displayRSSI();}
          if(Settings[S_AMPERAGE]&&(((amperage/10)<Settings[S_AMPERAGE_ALARM])||(timer.Blink2hz))){
            displayAmperage();}
          if(Settings[S_AMPER_HOUR] && ((!ampAlarming()) || timer.Blink2hz)){
            displaypMeterSum();}
          displayTime();
          #if defined DISPLAYWATTS
            displayWatt();
          #endif
          #ifdef TEMPSENSOR
            if(((temperature<Settings[TEMPERATUREMAX])||(timer.Blink2hz))){
              displayTemperature();}
          #endif
          displayArmed();
          if (Settings[S_THROTTLEPOSITION]){
            displayCurrentThrottle();}
          #ifdef CALLSIGNALWAYS
            if(Settings[S_DISPLAY_CS]){
              displayCallsign(getPosition(callSignPosition));}
          #elif  FREETEXTLLIGHTS
            if (MwSensorActive&mode.llights){
              displayCallsign(getPosition(callSignPosition));}
          #elif  FREETEXTGIMBAL
            if (MwSensorActive&mode.camstab){
              displayCallsign(getPosition(callSignPosition));}
          #else 
            if((onTime > (timer.lastCallSign+CALLSIGNINTERVAL))){  // Displays 4 sec every 5min (no blink during flight)
              if (onTime > (timer.lastCallSign+CALLSIGNINTERVAL+CALLSIGNDURATION)){
                timer.lastCallSign = onTime;}
              if(Settings[S_DISPLAY_CS]){
                displayCallsign(getPosition(callSignPosition));}}
          #endif
          if(MwSensorPresent&MAGNETOMETER){
            displayHeadingGraph();
            displayHeading();}
          if(MwSensorPresent&BAROMETER){
            displayAltitude();
            displayClimbRate();}
          if((MwSensorPresent&GPSSENSOR) && (Settings[S_DISPLAYGPS])){
              displayNumberOfSat();
              displayDirectionToHome();
              displayDistanceToHome();
              displayAngleToHome();
              displayGPS_speed();
              displayGPSPosition();
              #ifdef GPSTIME
                displayGPS_time();
              #endif
              #ifdef MAPMODE
                mapmode();
              #endif
              }
            displayMode();
            displayDebug();
          #ifdef I2CERROR
            displayI2CError();
          #endif        
          #ifdef SPORT        
            if(MwSensorPresent){
              displayCells();}
          #endif
          #ifdef HAS_ALARMS
            displayAlarms();
          #endif
        }
      } //End of if else for config screen
    }  // End of fast Timed Service Routine (50ms loop)
  
    if(timer.halfSec >= 5){
      timer.halfSec = 0;
      timer.Blink2hz =! timer.Blink2hz;}
  
    if(millis() > timer.seconds+1000){     // this execute 1 time a second
      timer.seconds+=1000;
      timer.tenthSec=0;
      onTime++;
    #ifdef MAXSTALLDETECT
      if (!fontMode){
        MAX7456Stalldetect();}
    #endif 
    #ifdef GPSACTIVECHECK
      if (timer.GPS_active==0){
        GPS_numSat=0;}
      else{
        timer.GPS_active--;}
    #endif
    if (timer.MSP_active>0){
      timer.MSP_active--;}
    if(!armed){
      // setMspRequests();
      #ifndef MAPMODENORTH
        armedangle=MwHeading;
      #endif
      }
    else{
      flyTime++;
      flyingTime++;
      configMode=0;
      setMspRequests();}
    allSec++;
    if((timer.magCalibrationTimer==1)&&(configMode)){
      mspWriteRequest(MSP_MAG_CALIBRATION,0);
      timer.magCalibrationTimer=0;}
    if(timer.magCalibrationTimer>0){
      timer.magCalibrationTimer--;}
    if(timer.rssiTimer>0){
      timer.rssiTimer--;}
    } // End of 1 per second loop
    serialMSPreceive(1);
  }  // End of main loop
#endif
