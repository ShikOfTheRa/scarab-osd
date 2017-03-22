

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
 Where there are exceptions, these take precedence over the genereric licencing approach
 Elements of the code provided by Pawelsky (DJI specific) are not for commercial use. See headers in individual files for further details.  
 Libraries used and typically provided by compilers may have licening terms stricter than that of GNU 3
 
*/

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

    while(*ptr == 0xa5 && ptr <= &__stack) 
    { 
        ptr++; count++; 
    } 

    return count; 
} 
#endif

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
//#ifdef PROGMEM
//  #undef PROGMEM
//  #define PROGMEM __attribute__((section(".progmem.data")))
//#endif

// Frequently used expressions
#define PGMSTR(p) (char *)pgm_read_word(p)

//------------------------------------------------------------------------
#define MWVERS "MW-OSD - R1.7"  
#define MWOSDVERSION 1701 // 1660=1.6.6.0 for GUI
#define EEPROMVER 13      // for eeprom layout verification
#include <avr/pgmspace.h>
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))
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

char screen[480];          // Main screen ram for MAX7456
#ifdef INVERTED_CHAR_SUPPORT
uint8_t screenAttr[480/8]; // Attribute (INV) bits for each char in screen[]
#endif
char screenBuffer[20]; 
uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;
uint8_t sensorpinarray[]={VOLTAGEPIN,VIDVOLTAGEPIN,AMPERAGEPIN,TEMPPIN,RSSIPIN};  
unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
unsigned long previous_millis_sync =0;
unsigned long previous_millis_rssi =0;

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
uint8_t fontStatus=0;
//uint16_t MAX_screen_size;
boolean ledstatus=HIGH;
//uint8_t fontData[54];
//uint8_t Settings[1];
#endif



//------------------------------------------------------------------------
void setup()
{

  Serial.begin(BAUDRATE);
  #ifndef PROTOCOL_MAVLINK //use double speed asynch mode (multiwii compatible)
    uint8_t h = ((F_CPU  / 4 / (BAUDRATE) -1) / 2) >> 8;
    uint8_t l = ((F_CPU  / 4 / (BAUDRATE) -1) / 2);
    UCSR0A  |= (1<<U2X0); UBRR0H = h; UBRR0L = l; 
  #endif
  Serial.flush();

#ifdef I2C_UB_SUPPORT
  // I2C initialization
  WireUB.begin(I2C_UB_ADDR, -1);
  TWBR=2; // Probably has no effect.
  // Compute rx queue timeout in microseconds
  //   = 3 character time at current bps (10bits/char)
  WireUB.setWriteTimo((1000000UL) / (I2C_UB_BREQUIV/10) * 3);
#endif

  MAX7456SETHARDWAREPORTS

  pinMode(A6, INPUT);
  pinMode(RSSIPIN, INPUT);
  pinMode(TEMPPIN, INPUT);
  pinMode(LEDPIN,OUTPUT);
  
  initPulseInts();  // initialise PWM / PPM inputs

#if defined EEPROM_CLEAR
  EEPROM_clear();
#endif  
  checkEEPROM();
  readEEPROM();
  
  #ifndef STARTUPDELAY
    #define STARTUPDELAY 500
  #endif
  delay(STARTUPDELAY);
 
  if (Settings[S_VREFERENCE])
    analogReference(DEFAULT);
  else
    analogReference(INTERNAL);

  MAX7456Setup();
  #if defined GPSOSD
    timer.GPS_initdelay=3; 
    //GPS_SerialInit();
  #else
  #endif
#if defined FORECSENSORACC
    MwSensorPresent |=ACCELEROMETER;
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
    return used > (Settings[S_AMPER_HOUR_ALARM]*100);
}


//------------------------------------------------------------------------
void loop()
{
  alarms.active=0;
  timer.loopcount++;
  if (flags.reset){
    resetFunc();
  }
  #if defined (KISS)      
    if (Kvar.mode==1)
      screenlayout=1;
    else
      screenlayout=0; 
  #elif defined (OSD_SWITCH_RC)                   
    rcswitch_ch = Settings[S_RCWSWITCH_CH];
    screenlayout=0;
    if (Settings[S_RCWSWITCH]){
      if (MwRcData[rcswitch_ch] > TX_CHAN_HIGH){
        screenlayout=1;
      }
      else if (MwRcData[rcswitch_ch] > TX_CHAN_MID){
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
  
  if (screenlayout!=oldscreenlayout){
    readEEPROM_screenlayout();
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
#ifdef CANVAS_SUPPORT
    if(!fontMode && !canvasMode)
#else
    if(!fontMode)
#endif
    {
       #ifndef KISS
       if (timer.GUI_active==0){
         mspWriteRequest(MSP_ATTITUDE,0);
       }
       #endif
    }
  }
#endif //MSP_SPEED_HIGH

  if((currentMillis - previous_millis_low) >= lo_speed_cycle)  // 10 Hz (Executed every 100ms)
  {
    previous_millis_low = previous_millis_low+lo_speed_cycle;    
    timer.tenthSec++;
    timer.halfSec++;
    timer.Blink10hz=!timer.Blink10hz;
    calculateTrip();
    if (Settings[S_AMPER_HOUR]) 
    {
      #ifndef KISS
        amperagesum += amperage;
      #else 
        if (!Settings[S_MWAMPERAGE]) 
          amperagesum += amperage;
      #endif   // KISS
    }
    #ifndef GPSOSD 
      #ifdef MSP_SPEED_MED
        #ifdef CANVAS_SUPPORT
        if(!fontMode && !canvasMode)
        #else
        if(!fontMode)
        #endif
        {
          #ifndef KISS
            if (timer.GUI_active==0){
              mspWriteRequest(MSP_ATTITUDE,0);
            }
          #endif // KISS
        }
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
       //if (flags.box!=1)
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
    }
    
    if(!fontMode){
      #ifndef GPSOSD
       #ifdef KISS
       Serial.write(0x20);
       #else     
         #ifdef CANVAS_SUPPORT
         if (!canvasMode)
         #endif
         {
           if (timer.GUI_active==0){
             mspWriteRequest(MSPcmdsend, 0);
           } 
         }
       #endif // KISS
      #endif //GPSOSD
      #ifdef SKYTRACK
//       DrawSkytrack();
      #endif
     MAX7456_DrawScreen();
    }

    ProcessSensors();       // using analogue sensors


#ifndef INTRO_DELAY 
  #define INTRO_DELAY 5
  #ifdef DEBUG 
    #define INTRO_DELAY 0
  #endif
#endif
    if( allSec < INTRO_DELAY ){
      displayIntro();
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

        // Place a small indicator for canvas mode to detect spurious 
        // canvas requests.
        // In a normal situation, It should go away on the very first
        // clear screen request, but may remain until next clear screen
        // if the begin and the first clear request comes in back-to-back
        // before the indicator is drawn.

        if (canvasFirst) {
          MAX7456_WriteString("*", (LINE01+01));
          canvasFirst = false;
        }
      }
#endif
      else
      {
        setMspRequests();
#if defined USE_AIRSPEED_SENSOR
        useairspeed();
#endif //USE_AIRSPEED_SENSOR
        if(MwSensorPresent&ACCELEROMETER)
           displayHorizon(MwAngle[0],MwAngle[1]);
#if defined FORCECROSSHAIR
        displayForcedCrosshair();
#endif //FORCECROSSHAIR
        if(Settings[S_DISPLAYVOLTAGE])
          displayVoltage();
        if (Settings[S_VIDVOLTAGE])
          displayVidVoltage();
        if(Settings[S_DISPLAYRSSI]&&((rssi>Settings[S_RSSI_ALARM])||(timer.Blink2hz)))
          displayRSSI();
        if(Settings[S_AMPERAGE]&&(((amperage/10)<Settings[S_AMPERAGE_ALARM])||(timer.Blink2hz)))
          displayAmperage();
        if(Settings[S_AMPER_HOUR] && ((!ampAlarming()) || timer.Blink2hz))
          displaypMeterSum();
        displayTime();
#if defined (DISPLAYWATTS)
        displayWatt();
#endif //DISPLAYWATTS
#if defined (DISPLAYEFFICIENCY)
        displayEfficiency();
#endif //DISPLAYEFFICIENCY
#if defined (DISPLAYMAHMIN)
        displaymAhmin();
#endif //DISPLAYMAHMIN
#ifdef TEMPSENSOR
        if(((temperature<Settings[TEMPERATUREMAX])||(timer.Blink2hz))) displayTemperature();
#endif
#ifdef VIRTUAL_NOSE
        displayVirtualNose();
#endif
#ifdef DISPLAYDOP
        displayDOP();
#endif
        displayArmed();
        if (Settings[S_THROTTLEPOSITION])
          displayCurrentThrottle();
#ifdef CALLSIGNALWAYS
        if(Settings[S_DISPLAY_CS]) displayCallsign(getPosition(callSignPosition)); 
#elif  FREETEXTLLIGHTS
        if (MwSensorActive&mode.llights) displayCallsign(getPosition(callSignPosition)); 
#elif  FREETEXTGIMBAL
        if (MwSensorActive&mode.camstab) displayCallsign(getPosition(callSignPosition)); 
#else 
        if ( (onTime > (timer.lastCallSign+CALLSIGNINTERVAL)))
       {
           // Displays 4 sec every 5min (no blink during flight)
        if ( onTime > (timer.lastCallSign+CALLSIGNINTERVAL+CALLSIGNDURATION)) timer.lastCallSign = onTime; 
        if(Settings[S_DISPLAY_CS]) displayCallsign(getPosition(callSignPosition));      
       }
#endif
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
          #ifdef USEGLIDESCOPE
            // displayfwglidescope(); //note hook for this is in display horizon function
          #endif //USEGLIDESCOPE  
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
#ifdef I2CERROR
        displayI2CError();
#endif        
#ifdef SPORT        
        if(MwSensorPresent)
          displayCells();
#endif
        displayDebug();
#ifdef HAS_ALARMS
        displayAlarms();
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
    #ifdef DEBUGDPOSLOOP
      framerate=timer.loopcount;
      timer.loopcount=0;
    #endif
    #ifdef DEBUGDPOSPACKET
      packetrate=timer.packetcount;
      timer.packetcount=0;
    #endif
    #ifdef DEBUGDPOSRX    
      serialrxrate=timer.serialrxrate;
      timer.serialrxrate=0;
    #endif
    onTime++;
    #if defined(AUTOCAM) || defined(MAXSTALLDETECT)
      if (!fontMode)
        MAX7456CheckStatus();
    #endif
    #ifdef ALARM_GPS
      if (timer.GPS_active==0){
        GPS_numSat=0;
      }
      else {
        timer.GPS_active--;
      }      
    #endif // ALARM_GPS 
    if (timer.MSP_active>0){
      timer.MSP_active--;
    }  
    if (timer.GUI_active>0){
      timer.GUI_active--;
    }  
    #if defined GPSOSD
    if (timer.GPS_initdelay==1){
      GPS_SerialInit();
    }  
    if (timer.GPS_initdelay>0){
      timer.GPS_initdelay--;
    }  
    #endif

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
//  delay(1);

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
  if(armed || configMode || fontMode|| !safeMode()) 
    return;
  // queue first char for transmition.
  retransmitQueue = 0x80;
  fontMode = 1;
//  setMspRequests();
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
//  setMspRequests();
    return -1;
  }

  // Already at last char... check for missed characters.
  while(!(temp1 & 0x03)) {
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
  asm volatile ("  jmp 0"); 
} 

void setMspRequests() {
  if(fontMode) {
    modeMSPRequests = REQ_MSP_FONT;
  }
  else if(configMode) {
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
#ifdef MSP_SPEED_LOW
      REQ_MSP_ATTITUDE|
#endif
      REQ_MSP_RAW_IMU|
      REQ_MSP_ALTITUDE|
      REQ_MSP_RC_TUNING|
      REQ_MSP_PID_CONTROLLER|
#ifdef USE_MSP_PIDNAMES
      REQ_MSP_PIDNAMES|
#endif
      REQ_MSP_PID|
#ifdef CORRECTLOOPTIME
      REQ_MSP_LOOP_TIME|
#endif
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
#ifdef MENU_SERVO
      REQ_MSP_SERVO_CONF|
#endif
#ifdef MENU_FIXEDWING
      REQ_MSP_FW_CONFIG|
#endif
      REQ_MSP_RC;
  }
  else {
//wtf:?? try deleting next 4 lines and what happens to memory. is it local vs global in some way?
//    MwSensorPresent |=GPSSENSOR;
//    MwSensorPresent |=BAROMETER;
//    MwSensorPresent |=MAGNETOMETER;
//    MwSensorPresent |=ACCELEROMETER;

    modeMSPRequests = 
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
#ifdef MSP_SPEED_LOW
      REQ_MSP_ATTITUDE|
#endif
      0; // Sigh...

    if(MwSensorPresent&BAROMETER){ 
      modeMSPRequests |= REQ_MSP_ALTITUDE;
    }
    if(flags.ident!=1){
      modeMSPRequests |= REQ_MSP_IDENT;
    }
    if(MwSensorPresent&GPSSENSOR) 
      modeMSPRequests |= REQ_MSP_RAW_GPS| REQ_MSP_COMP_GPS;
    if(mode.armed == 0)
      modeMSPRequests |=REQ_MSP_BOX;
#if defined MULTIWII_V24
    if(MwSensorActive&mode.gpsmission)
    modeMSPRequests |= REQ_MSP_NAV_STATUS;
#endif
  }
 
  if(Settings[S_MAINVOLTAGE_VBAT] ||
    Settings[S_MWRSSI]) {
    modeMSPRequests |= REQ_MSP_ANALOG;
    
#ifdef USE_FC_VOLTS_CONFIG
    modeMSPRequests |= REQ_MSP_MISC;
#endif

  }

  queuedMSPRequests &= modeMSPRequests;   // so we do not send requests that are not needed.
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


void writeEEPROM(void) // OSD will only change 8 bit values. GUI changes directly
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
    EEPROM.write(en,Settings[en]);
  } 
  for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
    uint16_t pos=EEPROM_SETTINGS+(en*2);
    EEPROM.write(pos,Settings16[en]&0xFF);
    EEPROM.write(pos+1,Settings16[en]>>8);
  } 
  EEPROM.write(0,EEPROMVER);
}


void readEEPROM(void)
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
     Settings[en] = EEPROM.read(en);
  }
  #ifdef AUTOCELL
  Settings[S_BATCELLS]=1;
  #endif

  for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
     uint16_t pos=(en*2)+EEPROM_SETTINGS;
     Settings16[en] = EEPROM.read(pos);
     uint16_t xx = EEPROM.read(pos+1);
     Settings16[en] = Settings16[en]+(xx<<8);
  }

  readEEPROM_screenlayout();
}


void readEEPROM_screenlayout(void)
{

  uint16_t EEPROMscreenoffset=EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(screenlayout*POSITIONS_SETTINGS*2);
  for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
    uint16_t pos=(en*2)+EEPROMscreenoffset;
    screenPosition[en] = EEPROM.read(pos);
    uint16_t xx=(uint16_t)EEPROM.read(pos+1)<<8;
    screenPosition[en] = screenPosition[en] + xx;

    if(Settings[S_VIDEOSIGNALTYPE]){
      uint16_t x = screenPosition[en]&0x1FF; 
      if (x>LINE06) screenPosition[en] = screenPosition[en] + LINE;
      if (x>LINE09) screenPosition[en] = screenPosition[en] + LINE;
    }
#ifdef SHIFTDOWN
    if ((screenPosition[en]&0x1FF)<LINE04) screenPosition[en] = screenPosition[en] + LINE;
#endif
  }
}


void checkEEPROM(void)
{
  uint8_t EEPROM_Loaded = EEPROM.read(0);
  if (EEPROM_Loaded!=EEPROMVER){
    for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
      EEPROM.write(en,pgm_read_byte(&EEPROM_DEFAULT[en]));
    }
    for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
      uint16_t pos=EEPROM_SETTINGS+(en*2);
      uint16_t w = pgm_read_word(&EEPROM16_DEFAULT[en]);
      EEPROM.write(pos, w & 0xff);
      EEPROM.write(pos+1, w >> 8);
    }

#define L0START EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)
#define L1START EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*2)
#define L2START EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*4)

    for(uint8_t en=0;en<POSITIONS_SETTINGS*2;en++){
      uint16_t w;
      w = pgm_read_word(&SCREENLAYOUT_DEFAULT[en]);
      EEPROM.write(L0START+(en*2), w & 0xff);
      EEPROM.write(L0START+(en*2)+1, w >> 8);

      w = pgm_read_word(&SCREENLAYOUT_DEFAULT_OSDSW[en]);
      EEPROM.write(L1START+(en*2), w & 0xff);
      EEPROM.write(L1START+(en*2)+1, w >> 8);

      w = pgm_read_word(&SCREENLAYOUT_DEFAULT[en]);
      EEPROM.write(L2START+(en*2), w & 0xff);
      EEPROM.write(L2START+(en*2)+1, w >> 8);
    }
/*
    for(uint8_t osd_switch_pos=0;osd_switch_pos<3;osd_switch_pos++){
      for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
        EEPROM.write(EEPROM_SETTINGS+(POSITIONS_SETTINGS*osd_switch_pos)+(en*2),SCREENLAYOUT_DEFAULT_OSDSW[en]&0xFF);
        EEPROM.write(EEPROM_SETTINGS+(POSITIONS_SETTINGS*osd_switch_pos)+1+(en*2),SCREENLAYOUT_DEFAULT_OSDSW[en]>>8);
      }
    }    
*/
  }
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


void ProcessSensors(void) {
  /*
    special note about filter: last row of array = averaged reading
  */ 
//-------------- ADC sensor / PWM RSSI / FC data read into filter array
  static uint8_t sensorindex;
  uint16_t sensortemp;
  for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++) {
    sensortemp = analogRead(sensorpinarray[sensor]);
    //--- override with FC voltage data if enabled
    if (sensor ==0) { 
      if (Settings[S_MAINVOLTAGE_VBAT]){
        sensortemp=MwVBat;
      }
    }
    //--- override with PWM, FC RC CH or FC RSSI data if enabled    
    if (sensor ==4) { 
      if (Settings[S_PWMRSSI]){
        sensortemp = pwmRSSI>>1;
        if (sensortemp==0) { // timed out - use previous
          sensortemp=sensorfilter[sensor][sensorindex];
        }
      }
      if(Settings[S_MWRSSI]) {
        sensortemp = MwRssi;
      }
      #if defined RCRSSI
        sensortemp = MwRcData[RCRSSI]>>1;
      #endif
    }
    //--- Apply filtering    
#if defined FILTER_HYSTERYSIS  // Hysteris incremental averaged change    
      static uint16_t shfilter[SENSORTOTAL];
      int16_t diff = (sensortemp<<FILTER_HYSTERYSIS)-shfilter[sensor];
      if (abs(diff)>(FHBANDWIDTH<<FILTER_HYSTERYSIS)){
        shfilter[sensor]=sensortemp<<FILTER_HYSTERYSIS;        
      }
      else if (diff>0){
        shfilter[sensor]++;
      }
      else if (diff<0){
        shfilter[sensor]--;
      }
      sensorfilter[sensor][SENSORFILTERSIZE]=(shfilter[sensor]>>FILTER_HYSTERYSIS<<3);
#elif defined FILTER_AVG   // Use averaged change    
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];         
    sensorfilter[sensor][sensorindex] = (sensorfilter[sensor][sensorindex] + sensortemp)>>1;
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] + sensorfilter[sensor][sensorindex];
#else                      // No filtering
    sensorfilter[sensor][SENSORFILTERSIZE] = sensortemp<<3;
#endif
  } 

//-------------- Voltage
  if (!Settings[S_MAINVOLTAGE_VBAT]){ // not MWII
    uint16_t voltageRaw = sensorfilter[0][SENSORFILTERSIZE];
    if (!Settings[S_VREFERENCE]){
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (DIVIDER1v1);  
    }
    else {
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (DIVIDER5v);     
    }
  }
  else{
      voltage=sensorfilter[0][SENSORFILTERSIZE]>>3;
  }

  vidvoltageWarning = Settings[S_VIDVOLTAGEMIN];
  uint16_t vidvoltageRaw = sensorfilter[1][SENSORFILTERSIZE];
    if (!Settings[S_VREFERENCE]){
      vidvoltage = float(vidvoltageRaw) * Settings[S_VIDDIVIDERRATIO] * (DIVIDER1v1);
    }
    else {
      vidvoltage = float(vidvoltageRaw) * Settings[S_VIDDIVIDERRATIO] * (DIVIDER5v);
    }

//-------------- Temperature
#ifdef TEMPSENSOR
    temperature=sensorfilter[3][SENSORFILTERSIZE]>>3-TEMPZERO;
    temperature = map (temperature, TEMPZERO, 1024, 0 , TEMPMAX);
#endif

//-------------- Current
  
  if(!Settings[S_MWAMPERAGE]) {
    if (!Settings[S_AMPERAGE_VIRTUAL]) { // Analogue
      amperage = sensorfilter[2][SENSORFILTERSIZE]>>3;
      amperage = map(amperage, Settings16[S16_AMPZERO], 1024, 0, Settings16[S16_AMPDIVIDERRATIO]);
      if (amperage < 0) amperage=0;
    }  
    else {  // Virtual
      uint32_t Vthrottle = constrain(MwRcData[THROTTLESTICK],LowT,HighT);
      Vthrottle = constrain((Vthrottle-1000)/10,0,100);
      amperage = (Vthrottle+(Vthrottle*Vthrottle*0.02))*Settings16[S16_AMPDIVIDERRATIO]*0.01;
      if(armed)
        amperage += Settings16[S16_AMPZERO];
      else 
        amperage = Settings16[S16_AMPZERO];
    }  
  }
  else{
    // Apply rounding math
    if (MWAmperage < 0)
      amperage = (MWAmperage - AMPERAGE_DIV / 2) / AMPERAGE_DIV;
    else
      amperage = (MWAmperage + AMPERAGE_DIV / 2) / AMPERAGE_DIV;
  }

//-------------- RSSI
  if (Settings[S_DISPLAYRSSI]) {           
    rssi = sensorfilter[4][SENSORFILTERSIZE]>>3; // filter and remain 16 bit
    if (configMode){
      if((timer.rssiTimer==15)) {
        Settings16[S16_RSSIMAX]=rssi; // tx on
      }
      if((timer.rssiTimer==1)) {
        Settings16[S16_RSSIMIN]=rssi; // tx off
        timer.rssiTimer=0;
      }
    }
    rssi = map(rssi, Settings16[S16_RSSIMIN], Settings16[S16_RSSIMAX], 0, 100);
    rssi=constrain(rssi,0,100);
  }

//-------------- For filter support
  sensorindex++;                    
  if (sensorindex >= SENSORFILTERSIZE)              
    sensorindex = 0;                           
}


void initPulseInts() { //RSSI/PWM/PPM int initialisation
  #if defined INTC3
  DDRC &= ~(1 << DDC3); //  PORTC |= (1 << PORTC3);
  #endif
  #if defined INTD5
  DDRD &= ~(1 << DDD5); //  PORTD |= (1 << PORTD5);
  #endif
  cli();
  #if defined INTC3
  if ((PCMSK1&(1 << PCINT11))==0){
    PCICR |=  (1 << PCIE1);
    PCMSK1 |= (1 << PCINT11);
  }
  #endif
  #if defined INTD5
  if ((PCMSK2&(1 << PCINT21))==0){
    PCICR |=  (1 << PCIE2);
    PCMSK2 |= (1 << PCINT21);
  }
  #endif
  sei();
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
  uint16_t CurrentTime= micros();

  if((CurrentTime-LastTime)>3000) RCchan = 1; // assume this is PPM gap
  LastTime = CurrentTime;
  if (!(pinstatus & (1<<PWMPIN1))) { // measures low duration
    PulseDuration = CurrentTime-PulseStart; 
    if ((750<PulseDuration) && (PulseDuration<2250)) {    
  #ifdef INTD5
      pwmRSSI = PulseDuration;
  #else
    #ifdef PPM_CONTROL
      PulseType=1;
    #endif
      if (PulseType){ //PPM
        if (RCchan<=TX_CHANNELS)// avoid array overflow if > standard ch PPM
          MwRcData[RCchan] = PulseDuration; // Val updated
      }
      else{ //PWM
      #ifdef NAZA
        Naza.mode=0;
        if (PulseDuration > NAZA_PMW_HIGH){
          Naza.mode=NAZA_MODE_HIGH;
        }
        else if (PulseDuration > NAZA_PMW_MED){
          Naza.mode=NAZA_MODE_MED;
        }
        else if (PulseDuration > NAZA_PWM_LOW){
          Naza.mode=NAZA_MODE_LOW;
        }
      #endif  
      #ifdef PWM_OSD_SWITCH
        MwRcData[rcswitch_ch]=PulseDuration;
      #elif defined PWM_THROTTLE
        MwRcData[THROTTLESTICK] = PulseDuration;
      #else      
        pwmRSSI = PulseDuration;
      #endif
      }
    #endif
    }
    RCchan++;
    pwmval1=PulseDuration;
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
  uint16_t CurrentTime= micros();

  if((CurrentTime-LastTime)>3000) RCchan = 1; // assume this is PPM gap
  LastTime = CurrentTime;
  
  if (!(pinstatus & (1<<PWMPIN2))) { // measures low duration
    PulseDuration = CurrentTime-PulseStart; 
    if ((750<PulseDuration) && (PulseDuration<2250)) {    
    #ifdef PPM_CONTROL
      PulseType=1;
    #endif
      if (PulseType){ //PPM
        if (RCchan<=TX_CHANNELS)// avoid array overflow if > standard ch PPM
          MwRcData[RCchan] = PulseDuration; // Val updated
      }
      else{ //PWM
      #ifdef NAZA
        Naza.mode=0;
        if (PulseDuration > NAZA_PMW_HIGH){
          Naza.mode=NAZA_MODE_HIGH;
        }
        else if (PulseDuration > NAZA_PMW_MED){
          Naza.mode=NAZA_MODE_MED;
        }
        else if (PulseDuration > NAZA_PWM_LOW){
          Naza.mode=NAZA_MODE_LOW;
        }
      #endif  
      #ifdef PWM_THROTTLE
        MwRcData[THROTTLESTICK] = PulseDuration;
      #elif defined OSD_SWITCH_RC     
        MwRcData[rcswitch_ch]=PulseDuration;
      #endif
      }
    }
    RCchan++;
    pwmval2=PulseDuration;
  }  
  else {
    PulseStart = CurrentTime;
  }
}
#endif


void EEPROM_clear(){
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);
}



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
