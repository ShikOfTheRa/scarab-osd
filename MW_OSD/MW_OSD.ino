/*
MultiWii NG OSD ...

 This program is free software: you can redistribute it and/or modify
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
 
*/
            
uint32_t voltageRawArray[8];


#include <avr/pgmspace.h>
#include <EEPROM.h> //Needed to access eeprom read/write functions
#include "Config.h"
#include "symbols.h"
#include "GlobalVariables.h"

// Screen is the Screen buffer between program an MAX7456 that will be writen to the screen at 10hz
char screen[480];
// ScreenBuffer is an intermietary buffer to created Strings to send to Screen buffer
char screenBuffer[20];

uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;

//-------------- Timed Service Routine vars (No more needed Metro.h library)

// May be moved in GlobalVariables.h
unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
//----------------


void setup()
{
  Serial.begin(115200);
//---- override UBRR with MWC settings
  uint8_t h = ((F_CPU  / 4 / (115200) -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / (115200) -1) / 2);
  UCSR0A  |= (1<<U2X0); UBRR0H = h; UBRR0L = l; 
//---
  Serial.flush();
  
  //RSSI
  pinMode(PWMRSSIPIN, INPUT);
  pinMode(RSSIPIN, INPUT);
  
  //Led output
  pinMode(LEDPIN,OUTPUT);
 
  checkEEPROM();
  readEEPROM();
  MAX7456Setup();
  
  if (Settings[S_VREFERENCE])
    analogReference(DEFAULT);
  else
    analogReference(INTERNAL);

  setMspRequests();

  blankserialRequest(MSP_IDENT);
}
void (* resetFunc)(void)=0;


void setMspRequests() {
  if(fontMode) {
      modeMSPRequests = REQ_MSP_FONT;
  }
  else if(configMode) {
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
      REQ_MSP_ATTITUDE|
      REQ_MSP_RAW_IMU|
      REQ_MSP_ALTITUDE|
      REQ_MSP_RC_TUNING|
      REQ_MSP_PID|
      REQ_MSP_RC;
  }
  else {
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
      REQ_MSP_COMP_GPS|
      REQ_MSP_ATTITUDE|
      REQ_MSP_ALTITUDE;

#ifdef DEBUG
      modeMSPRequests |= REQ_MSP_DEBUG;
#endif

    if(MwVersion == 0)
      modeMSPRequests |= REQ_MSP_IDENT;

    if(!armed || Settings[S_THROTTLEPOSITION] || fieldIsVisible(pMeterSumPosition) || fieldIsVisible(amperagePosition) )
      modeMSPRequests |= REQ_MSP_RC;

    if(mode_armed == 0) {
        modeMSPRequests |= REQ_MSP_BOX;

    }
  }
 
  if(Settings[S_MAINVOLTAGE_VBAT] ||
     Settings[S_VIDVOLTAGE_VBAT] ||
     Settings[S_MWRSSI])
    modeMSPRequests |= REQ_MSP_ANALOG;

  // so we do not send requests that are not needed.
  queuedMSPRequests &= modeMSPRequests;
  lastCallSign = onTime;

}

void loop()
{
 

  // Blink Basic Sanity Test Led at 1hz
  if(tenthSec>10)
    digitalWrite(LEDPIN,HIGH);
  else
    digitalWrite(LEDPIN,LOW);

  //---------------  Start Timed Service Routines  ---------------------------------------
  unsigned long currentMillis = millis();

  if((currentMillis - previous_millis_low) >= lo_speed_cycle)  // 10 Hz (Executed every 100ms)
  {
    previous_millis_low = previous_millis_low+lo_speed_cycle;    
    if(!fontMode)
      blankserialRequest(MSP_ATTITUDE);
    if (Settings[S_DISPLAYRSSI])                 ProcessRSSI();           

   }  // End of slow Timed Service Routine (100ms loop)

  if((currentMillis - previous_millis_high) >= hi_speed_cycle)  // 20 Hz (Executed every 50ms)
  {
    previous_millis_high = previous_millis_high+hi_speed_cycle;   

    tenthSec++;
    halfSec++;
    Blink10hz=!Blink10hz;
    calculateTrip();
    
      uint8_t MSPcmdsend;
      if(queuedMSPRequests == 0)
        queuedMSPRequests = modeMSPRequests;
      uint32_t req = queuedMSPRequests & -queuedMSPRequests;
      queuedMSPRequests &= ~req;
      switch(req) {
      case REQ_MSP_IDENT:
        MSPcmdsend = MSP_IDENT;
        break;
      case REQ_MSP_STATUS:
        MSPcmdsend = MSP_STATUS;
        break;
      case REQ_MSP_RAW_IMU:
        MSPcmdsend = MSP_RAW_IMU;
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
      case REQ_MSP_ATTITUDE:
        MSPcmdsend = MSP_ATTITUDE;
        break;
      case REQ_MSP_ALTITUDE:
        MSPcmdsend = MSP_ALTITUDE;
        break;
      case REQ_MSP_ANALOG:
        MSPcmdsend = MSP_ANALOG;
        break;
      case REQ_MSP_RC_TUNING:
        MSPcmdsend = MSP_RC_TUNING;
        break;
      case REQ_MSP_PID:
        MSPcmdsend = MSP_PID;
        break;
      case REQ_MSP_BOX:
#ifdef USE_BOXNAMES
        MSPcmdsend = MSP_BOXNAMES;
#else
        MSPcmdsend = MSP_BOXIDS;
#endif
         break;
       case REQ_MSP_FONT:
         MSPcmdsend = MSP_OSD;
         break;
       case REQ_MSP_DEBUG:
         MSPcmdsend = MSP_DEBUG;
         break;
    }
      if(!fontMode)
      blankserialRequest(MSPcmdsend);      

  ProcessAnalogue();       // using analogue sensors
  if (Settings[S_AMPERAGE_VIRTUAL]) ProcessVirtualSensors(); // using virtual sensors

    MAX7456_DrawScreen();
    if( allSec < 7 ){
      displayIntro();
    }  
    else
    {
      if(armed){
        previousarmedstatus=1;
      }
      if(previousarmedstatus && !armed){
        armedtimer=20;
        configPage=8;
        ROW=10;
        COL=1;
        configMode=1;
        setMspRequests();
      }
      if(configMode)
      {
        displayConfigScreen();
      }
      else
      {
       // CollectStatistics();      DO NOT DELETE

        if(Settings[S_DISPLAYVOLTAGE]&&((voltage>Settings[S_VOLTAGEMIN])||(Blink2hz))) displayVoltage();
        if(Settings[S_DISPLAYRSSI]&&((rssi>Settings[S_RSSI_ALARM])||(Blink2hz))) displayRSSI();

        displayTime();
        
        if(Settings[S_DISPLAYTEMPERATURE]&&((temperature<Settings[S_TEMPERATUREMAX])||(Blink2hz))) displayTemperature();

        if(Settings[S_AMPERAGE]) displayAmperage();

        if(Settings[S_AMPER_HOUR])  displaypMeterSum();
        displayArmed();
        if (Settings[S_THROTTLEPOSITION])
          displayCurrentThrottle();

#if defined CALLSIGNALWAYS
        displayCallsign();       
#else 
        if ( (onTime > (lastCallSign+300)) || (onTime < (lastCallSign+4)))
       {
           // Displays 4 sec every 5min (no blink during flight)
        if ( onTime > (lastCallSign+300))lastCallSign = onTime; 
        displayCallsign();       
       }
#endif

        if(MwSensorPresent&ACCELEROMETER)
           displayHorizon(MwAngle[0],MwAngle[1]);


        if(MwSensorPresent&MAGNETOMETER) {
          displayHeadingGraph();
          displayHeading();
        }

        if(MwSensorPresent&BAROMETER) {
          displayAltitude();
          displayClimbRate();
        }

        if(MwSensorPresent&GPSSENSOR) 
        if(Settings[S_DISPLAYGPS]){
          displayNumberOfSat();
          displayDirectionToHome();
          displayDistanceToHome();
          displayAngleToHome();
          displayGPS_speed();
          displayGPSPosition();
          displayGPS_time();
          if(Settings[S_ENABLEADC]) mapmode();
        }
         displayMode();       
         displayDebug();
      }
    }
  }  // End of fast Timed Service Routine (50ms loop)

  if(halfSec >= 10) {
    halfSec = 0;
    Blink2hz =! Blink2hz;
  }

  if(tenthSec >= 20)     // this execute 1 time a second
  {
    tenthSec=0;
    onTime++;

  amperagesum += amperage;

    if(!armed) {
#ifndef MAPMODENORTH
        armedangle=MwHeading;
#endif
    }
    else {
      flyTime++;
      flyingTime++;
      configMode=0;
      setMspRequests();
    }
    allSec++;

    if((accCalibrationTimer==1)&&(configMode)) {
      blankserialRequest(MSP_ACC_CALIBRATION);
      accCalibrationTimer=0;
    }

    if((magCalibrationTimer==1)&&(configMode)) {
      blankserialRequest(MSP_MAG_CALIBRATION);
      magCalibrationTimer=0;
    }

//    if(accCalibrationTimer>0) accCalibrationTimer--;
    if(magCalibrationTimer>0) magCalibrationTimer--;

    if(rssiTimer>0) rssiTimer--;
  }

  serialMSPreceive();


}  // End of main loop
//---------------------  End of Timed Service Routine ---------------------------------------

void calculateTrip(void)
{
  if(GPS_fix && armed && (GPS_speed>0)) {
    if(Settings[S_UNITSYSTEM])
      trip += GPS_speed *0.0016404;     //  50/(100*1000)*3.2808=0.0016404     cm/sec ---> ft/50msec
    else
      trip += GPS_speed *0.0005;        //  50/(100*1000)=0.0005               cm/sec ---> mt/50msec (trip var is float)      
  }
}

void writeEEPROM(void)
{
  Settings[S_AMPMAXH] = S16_AMPMAX>>8;
  Settings[S_AMPMAXL] = S16_AMPMAX&0xFF;
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
    EEPROM.write(en,Settings[en]);
  } 
}

void readEEPROM(void)
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
     Settings[en] = EEPROM.read(en);
  }
  S16_AMPMAX=(Settings[S_AMPMAXH]<<8)+Settings[S_AMPMAXL];
//  S16_AMPMAX=Settings[S_AMPMAXL];
}

// for first run to ini
void checkEEPROM(void)
{
  uint8_t EEPROM_Loaded = EEPROM.read(0);
  if (!EEPROM_Loaded){
    for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
      EEPROM.write(en,EEPROM_DEFAULT[en]);
    }
  }
}

uint8_t safeMode() {
  return 1;	// XXX
}

// Font upload queue implementation.
// Implement a window for curr + the previous 6 requests.
// First-chance to retransmit at curr-3 (retransmitQueue & 0x10)
// First-chance retransmit marked as used at retransmitQueue |= 0x01
// 2 to N-chance retransmit marked at curr-6 (retransmitQueue & 0x02)
void initFontMode() {
  if(armed || configMode || fontMode|| !safeMode()) 
    return;
  // queue first char for transmition.
  retransmitQueue = 0x80;

  fontMode = 1;
  setMspRequests();
}

void fontCharacterReceived(uint8_t cindex) {
  if(!fontMode)
    return;

  uint8_t bit = (0x80 >> (nextCharToRequest-cindex));

  // Just received a char..
  if(retransmitQueue & bit) {
    // this char war requested and now received for the first time
    retransmitQueue &= ~bit;  // mark as already received
    write_NVM(cindex);       // Write to MVRam
  }
}

int16_t getNextCharToRequest() {
  if(nextCharToRequest != lastCharToRequest) { // Not at last char
    if(retransmitQueue & 0x02) {                // Missed char at curr-6. Need retransmit!
      return nextCharToRequest-6;
    }

    if((retransmitQueue & 0x11) == 0x10) {      // Missed char at curr-3. First chance retransmit
      retransmitQueue |= 0x01;                  // Mark first chance as used
      return nextCharToRequest-3;
    }

    retransmitQueue = (retransmitQueue >> 1) | 0x80; // Add next to queue
    return nextCharToRequest++;                      // Notice post-increment!
  }

  uint8_t temp1 = retransmitQueue & ~0x01; 
  uint8_t temp2 = nextCharToRequest - 6; 

  if(temp1 == 0) {
    fontMode = 0;                            // Exit font mode
  setMspRequests();
    return -1;
  }

  // Already at last char... check for missed characters.
  while(!(temp1 & 0x03)) {
    temp1 >>= 1;
    temp2++;
  }

  return temp2;
}

void ProcessAnalogue(void) {

  if (Settings[S_DISPLAYTEMPERATURE]){
    temperature=(analogRead(TEMPPIN)-102)/2.048; 
  }

  if (!Settings[S_MAINVOLTAGE_VBAT]){ // not MWII
    static uint16_t ind = 0;
    //static uint32_t voltageRawArray[8];
    voltageRawArray[(ind++)%8] = analogRead(VOLTAGEPIN);                  
    uint16_t voltageRaw = 0;
    for (uint16_t i=0;i<8;i++)
      voltageRaw += voltageRawArray[i];
    if (!Settings[S_VREFERENCE]){
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (0.0001);  
    }
    else {
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (0.0005);     
    }
    }

  if (!Settings[S_VIDVOLTAGE_VBAT]) {
    if (!Settings[S_VREFERENCE]){
      vidvoltage = float(analogRead(VIDVOLTAGEPIN)) * Settings[S_VIDDIVIDERRATIO] * (1.1/102.3/4);
    }
    else {
      vidvoltage = float(analogRead(VIDVOLTAGEPIN)) * Settings[S_VIDDIVIDERRATIO] * (1.1/102.3);
    }
  }

  if (!Settings[S_AMPERAGE_VIRTUAL]) {
//    amperage = (AMPRERAGE_OFFSET - (analogRead(amperagePin)*AMPERAGE_CAL))/10.23;
    processAmperage();
  }  
}

void ProcessVirtualSensors(void){
  uint32_t Vthrottle = constrain(MwRcData[THROTTLESTICK],1000,2000);
  Vthrottle = constrain((Vthrottle-1000)/10,10,100);
//    amperage = (Vthrottle+(Vthrottle*Vthrottle*0.02))*Settings[S_AMPDIVIDERRATIO]*0.01;
    amperage = (Vthrottle+(Vthrottle*Vthrottle*0.02))*S16_AMPMAX*0.01;
if(armed)
  amperage += Settings[S_AMPMIN];
else 
  amperage = Settings[S_AMPMIN];
}

void ProcessRSSI(void){
  if (Settings[S_PWMRSSI]){
    rssi = pulseIn(PWMRSSIPIN, HIGH,21000)>>3;
  }
  else if(Settings[S_MWRSSI]) {
    rssi = MwRssi;
  }
  else { 
    rssi = analogRead(RSSIPIN);
    rssi=(rssi+oldrssi)>>1;
    if (rssi > oldrssi) oldrssi++;
    else if (rssi < oldrssi) oldrssi--;
    rssi = oldrssi>>2;                 // move to 8 bit  
  }

  if((rssiTimer==15)&&(configMode)) {
    Settings[S_RSSIMAX]=rssi; // tx on
  }
  if((rssiTimer==1)&&(configMode)) {
    Settings[S_RSSIMIN]=rssi; // tx off
    rssiTimer=0;
  }
  rssi = map(rssi, Settings[S_RSSIMIN], Settings[S_RSSIMAX], 0, 100);
  if (rssi < 0) rssi=0;
  else if (rssi > 100) rssi=100;
}

void processAmperage(void) {
  amperage = analogRead(AMPERAGEPIN);
  amperage = map(amperage, Settings[S_AMPMIN]+AMPERAGEOFFSET, S16_AMPMAX, 0, AMPERAGEMAX);
  if (amperage < 0) amperage=0;
//  else if (amperage > 999) amperage=999;
}

void gpsdistancefix(void){
  int8_t speedband;
  static int8_t oldspeedband;
  static int8_t speedcorrection=0;
  if (GPS_distanceToHome < 10000) speedband = 0;
  else if (GPS_distanceToHome > 50000) speedband = 2;
  else{
    speedband = 1;
    oldspeedband = speedband;
  }    
  if (speedband==oldspeedband){
    if (oldspeedband==0) speedcorrection--;
    if (oldspeedband==2) speedcorrection++;
    oldspeedband = speedband;
  }
  GPS_distanceToHome=(speedcorrection*65535) + GPS_distanceToHome;
}  
