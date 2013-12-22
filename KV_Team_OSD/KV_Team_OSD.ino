/*
KV Team OSD
http://code.google.com/p/rush-osd-development/
July  2013  r370
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see http://www.gnu.org/licenses/
*/

// This Team work is based on the earlier work developed by Jean Gabriel Maurice known as Rushduino. http://code.google.com/p/rushduino-osd/
// Rushduino OSD <Multiwii forum>  http://www.multiwii.com/forum/viewtopic.php?f=8&t=922
// Minim OSD <Multiwii forum>  http://www.multiwii.com/forum/viewtopic.php?f=8&t=2918
// Thanks to all developers that coded this software before us, and all users that also help us to improve.
// This team wish you great flights.


// This software communicates using MSP via the serial protocol. Therefore Multiwii develop-dependent.
              // Changes the values of pid and rc-tuning, writes in eeprom of Multiwii FC.
              // In config mode, can do acc and mag calibration.
              // In addition, it works by collecting information analogue inputs. Such as voltage, amperage, rssi and temperature.
              // At the end of the flight may be useful to look at the statistics.



              /***********************************************************************************************************************************************/
              /*                                                            KV_OSD_Team                                                                      */
              /*                                                                                                                                             */
              /*                                                                                                                                             */
              /*                                             This software is the result of a team work                                                      */
              /*                                                                                                                                             */
              /*                                     KATAVENTOS               ITAIN                    CARLONB                                               */
              /*                         POWER67                  LIAM2317             NEVERLANDED                                                           */
              /*                                                                                                                                             */
              /*                                                                                                                                             */
              /*                                                                                                                                             */
              /*                                                                                                                                             */
              /***********************************************************************************************************************************************/


            


#include <avr/pgmspace.h>
#include <EEPROM.h> //Needed to access eeprom read/write functions
#include "symbols.h"
#include "Config.h"
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
uint8_t hi_speed_cycle = 50;
uint8_t lo_speed_cycle = 100;
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
  
  //PWM RSSI
  pinMode(PwmRssiPin, INPUT);
  
  //Led output
  pinMode(7,OUTPUT);
 
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

    if(!mode_llights == 0)
      modeMSPRequests |= REQ_MSP_DEBUG;

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
}

void loop()
{
  // Process AI   
  if (Settings[S_ENABLEADC]){
    temperature=(analogRead(temperaturePin)-102)/2.048; 
    if (!Settings[S_MAINVOLTAGE_VBAT]){
      static uint16_t ind = 0;
      static uint32_t voltageRawArray[8];
      voltageRawArray[(ind++)%8] = analogRead(voltagePin);                  
      uint16_t voltageRaw = 0;
      for (uint16_t i=0;i<8;i++)
        voltageRaw += voltageRawArray[i];
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (1.1/102.3/4/8);  
    }
    if (!Settings[S_VIDVOLTAGE_VBAT]) {
      vidvoltage = float(analogRead(vidvoltagePin)) * Settings[S_VIDDIVIDERRATIO] * (1.1/102.3/4);
    }
    if (!Settings[S_MWRSSI]) {
      rssiADC = (analogRead(rssiPin)*1.1*100)/1023;  // RSSI Readings, result in mV/10 (example 1.1V=1100mV=110 mV/10)
    }
    amperage = (AMPRERAGE_OFFSET - (analogRead(amperagePin)*AMPERAGE_CAL))/10.23;
  }
  if (Settings[S_MWRSSI]) {
      rssiADC = MwRssi;
  } 
   if (Settings[S_PWMRSSI]){
   rssiADC = pulseIn(PwmRssiPin, HIGH);     
  }
 
  //Shiki mod - virtual current sensor
if (Settings[S_AMPERAGE_VIRTUAL]){
  uint32_t Vthrottle = constrain(MwRcData[THROTTLESTICK],1000,2000);
  Vthrottle = constrain((Vthrottle-1000)/10,10,100);
    amperage = (Vthrottle+(Vthrottle*Vthrottle*0.02))*Settings[S_AMPDIVIDERRATIO]*0.01;
if(armed)
  amperage += AMPERAGE_VIRTUAL_IDLE;
else 
  amperage = AMPERAGE_VIRTUAL_IDLE;
}

  // Blink Basic Sanity Test Led at 1hz
  if(tenthSec>10)
    digitalWrite(7,HIGH);
  else
    digitalWrite(7,LOW);

  //---------------  Start Timed Service Routines  ---------------------------------------
  unsigned long currentMillis = millis();

  if((currentMillis - previous_millis_low) >= lo_speed_cycle)  // 10 Hz (Executed every 100ms)
  {
    previous_millis_low = currentMillis;    
    if(!fontMode)
      blankserialRequest(MSP_ATTITUDE);
      
    if(Settings[S_DISPLAYRSSI])
      calculateRssi();

      // Scrolling decoration
      if (GPS_speed > old_GPS_speed+10){
      old_GPS_speed = GPS_speed;
        SYM_AH_DECORATION_LEFT--;
        if (SYM_AH_DECORATION_LEFT<0x10)
          SYM_AH_DECORATION_LEFT=0x15;
      }
      else if (GPS_speed +10 < old_GPS_speed){
      old_GPS_speed = GPS_speed;
        SYM_AH_DECORATION_LEFT++;
        if (SYM_AH_DECORATION_LEFT>0x15)
          SYM_AH_DECORATION_LEFT=0x10;
      }
 
      if (MwAltitude > old_MwAltitude+10){
       old_MwAltitude = MwAltitude;
      SYM_AH_DECORATION_RIGHT--;
        if (SYM_AH_DECORATION_RIGHT<0x10)
          SYM_AH_DECORATION_RIGHT=0x15;
      }
      else if (MwAltitude < old_MwAltitude-10){
        old_MwAltitude = MwAltitude;
      SYM_AH_DECORATION_RIGHT++;
        if (SYM_AH_DECORATION_RIGHT>0x15)
          SYM_AH_DECORATION_RIGHT=0x10;
      }  

  }  // End of slow Timed Service Routine (100ms loop)

  if((currentMillis - previous_millis_high) >= hi_speed_cycle)  // 20 Hz (Executed every 50ms)
  {
    previous_millis_high = currentMillis;   

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

    MAX7456_DrawScreen();
    if( allSec < 8 ){
      displayIntro();
      lastCallSign = onTime;
    }  
    else
    {
      if(armed){
        previousarmedstatus=1;
      }
      if(previousarmedstatus && !armed){
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
        displayMode();
        
        if(Settings[S_DISPLAYTEMPERATURE]&&((temperature<Settings[S_TEMPERATUREMAX])||(Blink2hz))) displayTemperature();

        if(Settings[S_AMPERAGE]) displayAmperage();

        if(Settings[S_AMPER_HOUR])  displaypMeterSum();
        displayArmed();
        if (Settings[S_THROTTLEPOSITION])
          displayCurrentThrottle();

        if ( (onTime > (lastCallSign+300)) || (onTime < (lastCallSign+4)))
       {
           // Displays 4 sec every 5min (no blink during flight)
        if ( onTime > (lastCallSign+300))lastCallSign = onTime; 
        displayCallsign(); 
       
       }

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
        }
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

 //Shiki mod - virtual current sensor
if (Settings[S_AMPERAGE_VIRTUAL])
  amperagesum += amperage *100/ AMPDIVISION; //(mAh)
else
  amperagesum += amperage / AMPDIVISION; //(mAh)



    if(!armed) {
      // Shiki mod to prevent reset of flytime when disarming
      //flyTime=0;
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

    if((eepromWriteTimer==1)&&(configMode)) {
      blankserialRequest(MSP_EEPROM_WRITE);
      eepromWriteTimer=0;
    }

    if(accCalibrationTimer>0) accCalibrationTimer--;
    if(magCalibrationTimer>0) magCalibrationTimer--;
    if(eepromWriteTimer>0) eepromWriteTimer--;

    if((rssiTimer==1)&&(configMode)) {
      Settings[S_RSSIMIN]=rssiADC;  // set MIN RSSI signal received (tx off?)
      rssiTimer=0;
    }
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

void calculateRssi(void)
{
  float aa=0;
 
 if (Settings[S_PWMRSSI]){
     //Digital read Pin
   aa = pulseIn(PwmRssiPin, HIGH);
   aa = ((aa-Settings[S_RSSIMIN]) *101)/((Settings[S_RSSIMAX]*4)-Settings[S_RSSIMIN]) ;
 }
  else { 
      if (Settings[S_MWRSSI]) {
        aa =  MwRssi;
      }
      else {
        aa=rssiADC;  // actual RSSI analogic signal received
      }
  aa = ((aa-Settings[S_RSSIMIN]) *101)/(Settings[S_RSSIMAX]-Settings[S_RSSIMIN]) ;  // Percentage of signal strength
  rssi_Int += ( ( (signed int)((aa*rssiSample) - rssi_Int )) / rssiSample );  // Smoothing the readings
  rssi = rssi_Int / rssiSample ;
  if(rssi<0) rssi=0;
  if(rssi>100) rssi=100;
  }
}

void writeEEPROM(void)
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
    if (EEPROM.read(en) != Settings[en]) EEPROM.write(en,Settings[en]);
  } 
}

void readEEPROM(void)
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
     Settings[en] = EEPROM.read(en);
  }
}


// for first run to ini
void checkEEPROM(void)
{
  uint8_t EEPROM_Loaded = EEPROM.read(0);
  if (!EEPROM_Loaded){
    for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
      if (EEPROM.read(en) != EEPROM_DEFAULT[en])
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


