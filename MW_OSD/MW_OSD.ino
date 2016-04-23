

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

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
//#ifdef PROGMEM
//  #undef PROGMEM
//  #define PROGMEM __attribute__((section(".progmem.data")))
//#endif

//------------------------------------------------------------------------

#if 1
__asm volatile ("nop");
#endif

// memory-checking assembly
#include "memory.h"

// everything else
#include "platform.h"

unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
unsigned long previous_millis_sync =0;
unsigned long previous_millis_rssi =0;

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
uint8_t fontStatus=0;
boolean ledstatus=HIGH;
//uint8_t fontData[54];
//uint8_t Settings[1];
#endif


//------------------------------------------------------------------------
void setup()
{

  Serial.begin(BAUDRATE);
//---- override UBRR with MWC settings
  uint8_t h = ((F_CPU  / 4 / (BAUDRATE) -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / (BAUDRATE) -1) / 2);
  UCSR0A  |= (1<<U2X0); UBRR0H = h; UBRR0L = l; 
//---
  Serial.flush();

  // tell the serial lib about our serial port
  serialInit(Serial);

  pinMode(PWMRSSIPIN, INPUT);
  pinMode(RSSIPIN, INPUT);
  pinMode(LEDPIN,OUTPUT);

#if defined (INTPWMRSSI) || defined (PPMOSDCONTROL)
  initRSSIint();
#endif

#if defined EEPROM_CLEAR
  Eeprom.clear();
#endif  
  Eeprom.check();
  Eeprom.read();
  
  #ifndef STARTUPDELAY
    #define STARTUPDELAY 500
  #endif
  delay(STARTUPDELAY);
 
  if (Settings[S_VREFERENCE])
    analogReference(DEFAULT);
  else
    analogReference(INTERNAL);

  MAX7456.Setup();
  #if defined GPSOSD
    GPS_SerialInit();
  #else
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
  #endif //ALWAYSARMED

}

//------------------------------------------------------------------------
#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
void loop()
{
  switch(fontStatus) {
    case 0:
      MAX7456.WriteString_P(messageF0, 32);
      MAX7456.DrawScreen();
      delay(3000);
      MAX7456.DisplayFont();  
      MAX7456.WriteString_P(messageF1, 32);
      MAX7456.DrawScreen();
      fontStatus++;
      delay(3000);      
      break;
    case 1:
      MAX7456.UpdateFont();
      MAX7456.Setup(); 
      MAX7456.WriteString_P(messageF2, 32);
      MAX7456.DisplayFont();  
      MAX7456.DrawScreen();
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
    return used > (Settings[S_AMPER_HOUR_ALARM]*100);
}


//------------------------------------------------------------------------
void loop()
{
  if (flags.reset){
    resetFunc();
  }
  #if defined (MEMCHECK) && defined (DEVELOPMENT)
    debug[MEMCHECK] = UntouchedStack();
  #endif

  #ifdef PWMTHROTTLE
    MwRcData[THROTTLESTICK] = pwmRSSI;
  #endif //THROTTLE_RSSI

  // TODO: $$$ screenlayout global -> local
  #if defined (OSD_SWITCH_RC)                   
    uint8_t rcswitch_ch = Settings[S_RCWSWITCH_CH];
    screenlayout=0;
    if (Settings[S_RCWSWITCH]){
      #ifdef OSD_SWITCH_RSSI
        MwRcData[rcswitch_ch]=pwmRSSI;      
      #endif
      if (MwRcData[rcswitch_ch] > 1600){
        screenlayout=1;
      }
      else if (MwRcData[rcswitch_ch] > 1400){
        screenlayout=2;
      }
    } 
    else{
      if (MwSensorActive&mode.osd_switch){
        screenlayout=1;
      }
    }
  #else 
    if (MwSensorActive&mode.osd_switch)
      screenlayout=1;
    else  
      screenlayout=0;
  #endif
  
#if defined (DEVELOPMENT)
      screenlayout=0;
#endif

  if (screenlayout!=oldscreenlayout){
    Screen.ReadLayout(Eeprom.getEEPROM());
  }
  oldscreenlayout=screenlayout;
    
  // Blink Basic Sanity Test Led at 0.5hz
  if(timer.tenthSec>5)
    digitalWrite(LEDPIN,HIGH);
  else
    digitalWrite(LEDPIN,LOW);

  //---------------  Start Timed Service Routines  ---------------------------------------
  unsigned long currentMillis = millis();

#ifdef MSP_SPEED_HIGH
  if((currentMillis - previous_millis_sync) >= sync_speed_cycle)  // (Executed > NTSC/PAL hz 33ms)
  {
    previous_millis_sync = previous_millis_sync+sync_speed_cycle;    
    if(!Font.inFontMode())
      mspWriteRequest(MSP_ATTITUDE,0);
  }
#endif //MSP_SPEED_HIGH

#ifdef INTPWMRSSI
// to prevent issues with high pulse RSSi consuming CPU
  if((currentMillis - previous_millis_rssi) >= (1000/RSSIhz)){  
    previous_millis_rssi = currentMillis; 
    initRSSIint();
  }   
#endif // INTPWMRSSI

  if((currentMillis - previous_millis_low) >= lo_speed_cycle)  // 10 Hz (Executed every 100ms)
  {
    previous_millis_low = previous_millis_low+lo_speed_cycle;    
    timer.tenthSec++;
    timer.halfSec++;
    timer.Blink10hz=!timer.Blink10hz;
    calculateTrip();
    if (Settings[S_AMPER_HOUR]) 
      amperagesum += amperage;
    #ifndef GPSOSD 
      #ifdef MSP_SPEED_MED
        if(!Font.inFontMode())
          mspWriteRequest(MSP_ATTITUDE,0);
      #endif //MSP_SPEED_MED  
    #endif //GPSOSD
   }  // End of slow Timed Service Routine (100ms loop)

  if((currentMillis - previous_millis_high) >= hi_speed_cycle)  // 20 Hz or 100hz in MSP high mode
  {
    previous_millis_high = previous_millis_high+hi_speed_cycle;       
      uint8_t MSPcmdsend=0;
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
           if(MwSensorActive&mode.gpsmission)
         MSPcmdsend = MSP_NAV_STATUS;
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
    }
    
    if(!Font.inFontMode()){
      #ifndef GPSOSD
      mspWriteRequest(MSPcmdsend, 0);      
      #endif //GPSOSD
      MAX7456.DrawScreen();

    }

    ProcessSensors();       // using analogue sensors


#ifndef INTRO_DELAY 
#define INTRO_DELAY 8
#endif
    if( allSec < INTRO_DELAY ){
      Screen.DisplayIntro();
      timer.lastCallSign=onTime-CALLSIGNINTERVAL;
    }  
    else
    {
      if(armed){
        previousarmedstatus=1;
        if (configMode==1)
          configExit();
      }
#ifndef HIDESUMMARY
      if(previousarmedstatus && !armed){
        armedtimer=20;
        configPage=0;
        ROW=10;
        COL=1;
        configMode=1;
        setMspRequests();
      }
#else
      if(previousarmedstatus && !armed){
        previousarmedstatus=0;
        configMode=0;
      }
#endif //HIDESUMMARY      
      if(configMode)
      {
        Screen.DisplayConfigScreen();
      }
      else
      {
        setMspRequests();
#if defined USE_AIRSPEED_SENSOR
        useairspeed();
#endif //USE_AIRSPEED_SENSOR
        if(MwSensorPresent&ACCELEROMETER)
           Screen.DisplayHorizon(MwAngle[0],MwAngle[1]);
#if defined FORCECROSSHAIR
        Screen.DisplayForcedCrosshair();
#endif //FORCECROSSHAIR
        if(Settings[S_DISPLAYVOLTAGE])
          Screen.DisplayVoltage();
        if (Settings[S_VIDVOLTAGE])
          Screen.DisplayVidVoltage();
        if(Settings[S_DISPLAYRSSI]&&((rssi>Settings[S_RSSI_ALARM])||(timer.Blink2hz)))
          Screen.DisplayRSSI();
        if(Settings[S_AMPERAGE]&&(((amperage/10)<Settings[S_AMPERAGE_ALARM])||(timer.Blink2hz)))
          Screen.DisplayAmperage();
        if(Settings[S_AMPER_HOUR] && ((!ampAlarming()) || timer.Blink2hz))
          Screen.DisplaypMeterSum();
        Screen.DisplayTime();
#if defined DISPLAYWATTS
        Screen.DisplayWatt();
#endif //DISPLAYWATTS

#ifdef TEMPSENSOR
        if(((temperature<Settings[TEMPERATUREMAX])||(timer.Blink2hz))) Screen.DisplayTemperature();
#endif
        Screen.DisplayArmed();
        if (Settings[S_THROTTLEPOSITION])
          Screen.DisplayCurrentThrottle();
#ifdef CALLSIGNALWAYS
        if(Settings[S_DISPLAY_CS]) Screen.DisplayCallsign(getPosition(callSignPosition)); 
#elif  FREETEXTLLIGHTS
        if (MwSensorActive&mode.llights) Screen.DisplayCallsign(getPosition(callSignPosition)); 
#elif  FREETEXTGIMBAL
        if (MwSensorActive&mode.camstab) DisplayCallsign(getPosition(callSignPosition)); 
#else 
        if ( (onTime > (timer.lastCallSign+CALLSIGNINTERVAL)))
       {
           // Displays 4 sec every 5min (no blink during flight)
        if ( onTime > (timer.lastCallSign+CALLSIGNINTERVAL+CALLSIGNDURATION)) timer.lastCallSign = onTime; 
        if(Settings[S_DISPLAY_CS]) Screen.DisplayCallsign();      
       }
#endif
        if(MwSensorPresent&MAGNETOMETER) {
          Screen.DisplayHeadingGraph();
          Screen.DisplayHeading();
        }
        if(MwSensorPresent&BAROMETER) {
          Screen.DisplayAltitude();
          Screen.DisplayClimbRate();
        }
        if(MwSensorPresent&GPSSENSOR) 
        if(Settings[S_DISPLAYGPS]){
          Screen.DisplayNumberOfSat();
          Screen.DisplayDirectionToHome();
          Screen.DisplayDistanceToHome();
          Screen.DisplayAngleToHome();
          #ifdef USEGLIDESCOPE
            // Screen.Displayfwglidescope(); //note hook for this is in display horizon function
          #endif //USEGLIDESCOPE  
          Screen.DisplayGPSSpeed();
          Screen.DisplayGPSPosition();  
      
#ifdef GPSTIME
          Screen.DisplayGPSTime();
#endif
#ifdef MAPMODE
          Screen.MapMode();
#endif
        }
        Screen.DisplayMode();       
        Screen.DisplayDebug();
#ifdef I2CERROR
        Screen.DisplayI2CError();
#endif        
#ifdef SPORT        
        if(MwSensorPresent)
          Screen.DisplayCells();
#endif
#ifdef HAS_ALARMS
        Screen.DisplayAlarms();
#endif
      }
    }
  }  // End of fast Timed Service Routine (50ms loop)

  if(timer.halfSec >= 5) {
    timer.halfSec = 0;
    timer.Blink2hz =! timer.Blink2hz;
  }

  if(millis() > timer.seconds+1000)     // this execute 1 time a second
  {
    timer.seconds+=1000;
    timer.tenthSec=0;
    onTime++;
    #ifdef MAXSTALLDETECT
      if (!Font.inFontMode())
        MAX7456.Stalldetect();
    #endif 
    #ifdef GPSACTIVECHECK
      if (timer.GPS_active==0){
        GPS_numSat=0;
      }
      else {
        timer.GPS_active--;
      }      
    #endif // GPSACTIVECHECK 
    if (timer.MSP_active>0){
      timer.MSP_active--;
    }  
    if(!armed) {
//      setMspRequests();
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
/*
    if((timer.accCalibrationTimer==1)&&(configMode)) {
      mspWriteRequest(MSP_ACC_CALIBRATION,0);
      timer.accCalibrationTimer=0;
    }
*/    
    if((timer.magCalibrationTimer==1)&&(configMode)) {
      mspWriteRequest(MSP_MAG_CALIBRATION,0);
      timer.magCalibrationTimer=0;
    }
    if(timer.magCalibrationTimer>0) timer.magCalibrationTimer--;
    if(timer.rssiTimer>0) timer.rssiTimer--;
  }
//  setMspRequests();
  serialMSPreceive(1);
}  // End of main loop
#endif //main loop

//------------------------------------------------------------------------
// MISC

void resetFunc(void)
{
  asm volatile ("  jmp 0"); 
} 

void calculateTrip(void)
{
  static float tripSum = 0; 
  if(GPS_fix && armed && (GPS_speed>0)) {
    if(Settings[S_UNITSYSTEM])
      tripSum += GPS_speed *0.0032808;     //  100/(100*1000)*3.2808=0.0016404     cm/sec ---> ft/50msec
    else
      tripSum += GPS_speed *0.0010;        //  100/(100*1000)=0.0005               cm/sec ---> mt/50msec (trip var is float)      
  }
  trip = (uint32_t) tripSum;
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

unsigned long FastpulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t stateMask = (state ? bit : 0);
  unsigned long width = 0;
  unsigned long numloops = 0;
  unsigned long maxloops = timeout;
	
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;
	
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;
	
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    width++;
  }
  return width; 
}

#if defined INTPWMRSSI
void initRSSIint() { // enable ONLY RSSI pin A3 for interrupt (bit 3 on port C)
  DDRC &= ~(1 << DDC3);
//  PORTC |= (1 << PORTC3);
  cli();
  PCICR =  (1 << PCIE1);
  PCMSK1 = (1 << PCINT11);
  sei();
}


ISR(PCINT1_vect) { //
  static uint16_t PulseStart;  
  static uint8_t PulseCounter;  
  uint8_t pinstatus;
  pinstatus = PINC;
  sei();
  uint16_t CurrentTime;
  uint16_t PulseDuration;
  CurrentTime = micros();
  if (!(pinstatus & (1<<DDC3))) { // RSSI pin A3 - ! measures low duration
    if (PulseCounter >1){ // why? - to skip any partial pulse due to toggling of int's
      PulseDuration = CurrentTime-PulseStart; 
      PulseCounter=0;
    #if defined FASTPWMRSSI
      pwmRSSI = PulseDuration;
      PCMSK1 =0;
    #else
      if ((750<PulseDuration) && (PulseDuration<2250)) {
        pwmRSSI = PulseDuration;
        PCMSK1 =0;
      }
    #endif 
    }
    PulseCounter++;
  } 
  else {
    PulseStart = CurrentTime;
  }
//  sei();   
}
#endif // INTPWMRSSI


#if defined PPMOSDCONTROL
void initRSSIint() { // enable ONLY RSSI pin A3 for interrupt (bit 3 on port C)
  DDRC &= ~(1 << DDC3);
//  PORTC |= (1 << PORTC3);
  cli();
  PCICR =  (1 << PCIE1);
  PCMSK1 = (1 << PCINT11);
  sei();
}


ISR(PCINT1_vect) { //
  static uint16_t PulseStart;
  static uint8_t RCchan = 0; 
  static uint16_t LastTime = 0; 
  uint8_t pinstatus;
  pinstatus = PINC;
  sei();
  uint16_t CurrentTime;
  uint16_t PulseDuration;
  CurrentTime = micros(); 
  if((CurrentTime-LastTime)>3000) RCchan = 0; // assume this is PPM gap
  LastTime = CurrentTime;
  if (!(pinstatus & (1<<DDC3))) { // RSSI pin A3 - ! measures low duration
    PulseDuration = CurrentTime-PulseStart; 
    if ((750<PulseDuration) && (PulseDuration<2250)) {
      if (RCchan<8)// avoid array overflow if > standard 8 ch PPM
        MwRcData[RCchan] = PulseDuration; // Val updated
    }
    RCchan++;
  } 
  else {
    PulseStart = CurrentTime;
  }
}
#endif //PPMOSDCONTROL

#if defined USE_AIRSPEED_SENSOR
void useairspeed(){
  float airspeed_cal = AIRSPEED_CAL; //AIRSPEED_CAL; // move to GUI or config
  uint16_t airspeedsensor = sensorfilter[3][SENSORFILTERSIZE]>>3;
  if (airspeedsensor>(AIRSPEED_ZERO)){
    airspeedsensor = airspeedsensor-AIRSPEED_ZERO;
  }
  else {
    airspeedsensor = 0;
  }
  GPS_speed = 27.7777 * sqrt(airspeedsensor * airspeed_cal); // Need in cm/s for this
}
#endif //USE_AIRSPEED_SENSOR 
