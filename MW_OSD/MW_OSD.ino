

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

// Singletons go here, declared extern in their headers to ensure we only get 1 copy in SRAM
MAX7456Class MAX7456;
ScreenClass Screen;
EepromClass Eeprom(EEPROM);
SensorsClass Sensors;
MSPClass MSP(Serial);
FontClass Font;
StatsClass Stats;

// Global structs / vars go here

unsigned long previous_millis_low = 0;
unsigned long previous_millis_high =0;
unsigned long previous_millis_sync =0;
unsigned long previous_millis_rssi =0;

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
uint8_t fontStatus=0;
boolean ledstatus=HIGH;
//uint8_t fontData[54];
//uint8_t Eeprom.Settings[1];
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

  pinMode(PWMRSSIPIN, INPUT);
  pinMode(RSSIPIN, INPUT);
  pinMode(LEDPIN,OUTPUT);

#if defined (INTPWMRSSI) || defined (PPMOSDCONTROL)
  initRSSIint();
#endif

#if defined EEPROM_CLEAR
  Eeprom.ClearSettings();
#endif  
  Eeprom.CheckSettings();
  Eeprom.ReadSettings();
  
  #ifndef STARTUPDELAY
    #define STARTUPDELAY 500
  #endif
  delay(STARTUPDELAY);
 
  if (Eeprom.Settings[S_VREFERENCE])
    analogReference(DEFAULT);
  else
    analogReference(INTERNAL);

  MAX7456.Setup();
  #if defined GPSOSD
    GPS_SerialInit();
  #else
  #endif
  #if defined FORCESENSORS
    Sensors.Force()
  #endif
  MSP.SetRequests();
  
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

  Screen.UpdateLayout();

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
      MSP.WriteRequest(MSP_ATTITUDE,0);
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
    Stats.CalculateTrip();
    Stats.CalculateAmps();
    #ifndef GPSOSD 
      #ifdef MSP_SPEED_MED
        if(!Font.inFontMode())
          MSP.WriteRequest(MSP_ATTITUDE,0);
      #endif //MSP_SPEED_MED  
    #endif //GPSOSD
   }  // End of slow Timed Service Routine (100ms loop)

  if((currentMillis - previous_millis_high) >= hi_speed_cycle)  // 20 Hz or 100hz in MSP high mode
  {
    previous_millis_high = previous_millis_high+hi_speed_cycle;       

    MSP.BuildRequests();
    
    if(!Font.inFontMode()){
      #ifndef GPSOSD
      MSP.SendRequests();
      #endif //GPSOSD
      MAX7456.DrawScreen();

    }

    Sensors.Process();       // using analogue sensors


#ifndef INTRO_DELAY 
#define INTRO_DELAY 8
#endif
    if( allSec < INTRO_DELAY ){
      Screen.DisplayIntro();
      timer.lastCallSign=Stats.onTime-CALLSIGNINTERVAL;
    }  
    else
    {
      if(armed){
        previousarmedstatus=1;
        if (configMode==1)
          MSP.ConfigExit();
      }
#ifndef HIDESUMMARY
      if(previousarmedstatus && !armed){
        timer.armed=20;
        configPage=0;
        ROW=10;
        COL=1;
        configMode=1;
        MSP.SetRequests();
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
        MSP.SetRequests();
#if defined USE_AIRSPEED_SENSOR
        useairspeed();
#endif //USE_AIRSPEED_SENSOR
        if(Sensors.IsPresent(ACCELEROMETER))
           Screen.DisplayHorizon(MwAngle[0],MwAngle[1]);
#if defined FORCECROSSHAIR
        Screen.DisplayForcedCrosshair();
#endif //FORCECROSSHAIR
        if(Eeprom.Settings[S_DISPLAYVOLTAGE])
          Screen.DisplayVoltage();
        if (Eeprom.Settings[S_VIDVOLTAGE])
          Screen.DisplayVidVoltage();
        if(Eeprom.Settings[S_DISPLAYRSSI]&&((rssi>Eeprom.Settings[S_RSSI_ALARM])||(timer.Blink2hz)))
          Screen.DisplayRSSI();
        if(Eeprom.Settings[S_AMPERAGE]&&(((amperage/10)<Eeprom.Settings[S_AMPERAGE_ALARM])||(timer.Blink2hz)))
          Screen.DisplayAmperage();
        if(Eeprom.Settings[S_AMPER_HOUR] && ((!Stats.IsAmpAlarming()) || timer.Blink2hz))
          Screen.DisplaypMeterSum();
        Screen.DisplayTime();
#if defined DISPLAYWATTS
        Screen.DisplayWatt();
#endif //DISPLAYWATTS

#ifdef TEMPSENSOR
        if(((temperature<Eeprom.Settings[TEMPERATUREMAX])||(timer.Blink2hz))) Screen.DisplayTemperature();
#endif
        Screen.DisplayArmed();
        if (Eeprom.Settings[S_THROTTLEPOSITION])
          Screen.DisplayCurrentThrottle();
#ifdef CALLSIGNALWAYS
        if(Eeprom.Settings[S_DISPLAY_CS]) Screen.DisplayCallsign(getPosition(callSignPosition)); 
#elif  FREETEXTLLIGHTS
        if (Sensors.IsActive(mode.llights)) Screen.DisplayCallsign(getPosition(callSignPosition)); 
#elif  FREETEXTGIMBAL
        if (Sensors.IsActive(mode.camstab)) DisplayCallsign(getPosition(callSignPosition)); 
#else 
        if ( (Stats.onTime > (timer.lastCallSign+CALLSIGNINTERVAL)))
       {
           // Displays 4 sec every 5min (no blink during flight)
        if ( Stats.onTime > (timer.lastCallSign+CALLSIGNINTERVAL+CALLSIGNDURATION)) timer.lastCallSign = Stats.onTime; 
        if(Eeprom.Settings[S_DISPLAY_CS]) Screen.DisplayCallsign();      
       }
#endif
        if(Sensors.IsPresent(MAGNETOMETER)) {
          Screen.DisplayHeadingGraph();
          Screen.DisplayHeading();
        }
        if(Sensors.IsPresent(BAROMETER)) {
          Screen.DisplayAltitude();
          Screen.DisplayClimbRate();
        }
        if(Sensors.IsPresent(GPSSENSOR)) 
        if(Eeprom.Settings[S_DISPLAYGPS]){
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
// TODO: $$$ bug? what sensor?
//   should be Sensors.Active(some_sensor)
//#ifdef SPORT        
//        if(MwSensorPresent)
//          Screen.DisplayCells();
//#endif
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
    Stats.onTime++;
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
//      MSP.SetRequests();
#ifndef MAPMODENORTH
      armedangle=MwHeading;
#endif
    }
    else {
      Stats.flyTime++;
      Stats._flyingTime++;
      configMode=0;
      MSP.SetRequests();
    }
    timer.allSec++;
/*
    if((timer.accCalibrationTimer==1)&&(configMode)) {
      MSP.WriteRequest(MSP_ACC_CALIBRATION,0);
      timer.accCalibrationTimer=0;
    }
*/    
    if((timer.magCalibrationTimer==1)&&(configMode)) {
      MSP.WriteRequest(MSP_MAG_CALIBRATION,0);
      timer.magCalibrationTimer=0;
    }
    if(timer.magCalibrationTimer>0) timer.magCalibrationTimer--;
    if(timer.rssiTimer>0) timer.rssiTimer--;
  }
//  MSP.SetRequests();
  MSP.Receive(1);
}  // End of main loop
#endif //main loop

//------------------------------------------------------------------------
// MISC

void resetFunc(void)
{
  asm volatile ("  jmp 0"); 
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


// PWM RSSI 
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
