#include "platform.h"

void ProcessSensors(void) {
  /*
    special note about filter: last row of array = averaged reading
  */ 
//-------------- ADC and PWM RSSI sensor read into filter array
  static uint8_t sensorindex;
  for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++) {
    uint16_t sensortemp;
    sensortemp = analogRead(sensorpinarray[sensor]);
    
    if (sensor ==0) { 
      if (Settings[S_MAINVOLTAGE_VBAT]){
        sensortemp=MwVBat;
      }
    }
    
    if (sensor ==4) { 
      if (Settings[S_PWMRSSI]){
#if defined RCRSSI
//        sensortemp = constrain(MwRcData[RCRSSI],1000,2000)>>1;
        sensortemp = MwRcData[RCRSSI]>>1;
// #elif defined FASTPWMRSSI
//        sensortemp = FastpulseIn(PWMRSSIPIN, HIGH,1024);
#elif defined INTPWMRSSI
        sensortemp = pwmRSSI>>1;
#else
        sensortemp = pulseIn(PWMRSSIPIN, HIGH,18000)>>1;        
#endif
        if (sensortemp==0) { // timed out - use previous
          sensortemp=sensorfilter[sensor][sensorindex];
        }
      }
      if(Settings[S_MWRSSI]) {
        sensortemp = MwRssi;
      }
    }
#if defined STAGE2FILTER // Use averaged change    
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];         
    sensorfilter[sensor][sensorindex] = (sensorfilter[sensor][sensorindex] + sensortemp)>>1;
#elif defined SMOOTHFILTER // Shiki variable constraint probability trend change filter. Smooth filtering of small changes, but react fast to consistent changes
    #define FILTERMAX 128 //maximum change permitted each iteration 
    uint8_t filterdir;
    static uint8_t oldfilterdir[SENSORTOTAL];
    int16_t sensoraverage=sensorfilter[sensor][SENSORFILTERSIZE]>>3;
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];         
    if (sensorfilter[sensor][SENSORFILTERSIZE+1]<1) sensorfilter[sensor][SENSORFILTERSIZE+1]=1;

    if (sensortemp != sensoraverage ){
      // determine direction of change
      if (sensortemp > sensoraverage ) {  //increasing
        filterdir=1;
      }
      else if (sensortemp < sensoraverage ) {  //increasing
        filterdir=0;
      }
      // compare to previous direction of change
      if (filterdir!=oldfilterdir[sensor]){ // direction changed => lost trust in value - reset value truth probability to lowest
        sensorfilter[sensor][SENSORFILTERSIZE+1] = 1; 
      }
      else { // direction same => increase trust that change is valid - increase value truth probability
        sensorfilter[sensor][SENSORFILTERSIZE+1]=sensorfilter[sensor][SENSORFILTERSIZE+1] <<1;
      }
      // set maximum trust permitted per sensor read
      if (sensorfilter[sensor][SENSORFILTERSIZE+1] > FILTERMAX) {
        sensorfilter[sensor][SENSORFILTERSIZE+1] = FILTERMAX;
      }
      // set constrained value or if within limits, start to narrow filter 
      if (sensortemp > sensoraverage+sensorfilter[sensor][SENSORFILTERSIZE+1]) { 
        sensorfilter[sensor][sensorindex] = sensoraverage+sensorfilter[sensor][SENSORFILTERSIZE+1]; 
      }  
      else if (sensortemp < sensoraverage-sensorfilter[sensor][SENSORFILTERSIZE+1]){
        sensorfilter[sensor][sensorindex] = sensoraverage-sensorfilter[sensor][SENSORFILTERSIZE+1]; 
      }
      // as within limits, start to narrow filter 
      else { 
        sensorfilter[sensor][sensorindex] = sensortemp; 
        sensorfilter[sensor][SENSORFILTERSIZE+1]=sensorfilter[sensor][SENSORFILTERSIZE+1] >>2;
      }
      oldfilterdir[sensor]=filterdir;
    }
    // no change, reset filter 
    else {
      sensorfilter[sensor][sensorindex] = sensortemp; 
      sensorfilter[sensor][SENSORFILTERSIZE+1]=1;  
    }    
#else // Use a basic averaging filter
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];         
    sensorfilter[sensor][sensorindex] = sensortemp;
#endif
    sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] + sensorfilter[sensor][sensorindex];
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
    amperage = MWAmperage / AMPERAGE_DIV;
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
