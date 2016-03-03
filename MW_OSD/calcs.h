//declare variables
uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;
uint8_t sensorpinarray[]={VOLTAGEPIN,VIDVOLTAGEPIN,AMPERAGEPIN,TEMPPIN,RSSIPIN};

#if defined INTPWMRSSI
  void initRSSIint() { // enable ONLY RSSI pin A3 for interrupt (bit 3 on port C)
    DDRC &= ~(1 << DDC3);
  //  PORTC |= (1 << PORTC3);
    cli();
    PCICR =  (1 << PCIE1);
    PCMSK1 = (1 << PCINT11);
    sei();}
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

void resetFunc(void){
  asm volatile ("  jmp 0");
}


void setMspRequests(){
  if(fontMode){
    modeMSPRequests = REQ_MSP_FONT;}
  else if(configMode){
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
      REQ_MSP_ATTITUDE|
      REQ_MSP_RAW_IMU|
      REQ_MSP_ALTITUDE|
      REQ_MSP_RC_TUNING|
      REQ_MSP_PID_CONTROLLER|
      REQ_MSP_PID|
      REQ_MSP_LOOP_TIME|
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
      REQ_MSP_RC;}
  else{
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
      REQ_MSP_ATTITUDE;
      if(MwSensorPresent&BAROMETER){ 
        modeMSPRequests |= REQ_MSP_ALTITUDE;}
    if(flags.ident!=1){
      modeMSPRequests |= REQ_MSP_IDENT;}
    if(MwSensorPresent&GPSSENSOR){
      modeMSPRequests |= REQ_MSP_RAW_GPS| REQ_MSP_COMP_GPS;}
    if(mode.armed == 0){
      modeMSPRequests |=REQ_MSP_BOX;}
    #if defined MULTIWII_V24
        if(MwSensorActive&mode.gpsmission){
        modeMSPRequests |= REQ_MSP_NAV_STATUS;}
    #endif
  }
  if(Settings[S_MAINVOLTAGE_VBAT] || Settings[S_MWRSSI]) {
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
  if(GPS_fix && armed && (GPS_speed>0)){
    if(Settings[S_UNITSYSTEM]){
      tripSum += GPS_speed *0.0032808;}     //  100/(100*1000)*3.2808=0.0016404     cm/sec ---> ft/50msec
    else{
      tripSum += GPS_speed *0.0010;}}        //  100/(100*1000)=0.0005               cm/sec ---> mt/50msec (trip var is float)      
  trip = (uint32_t) tripSum;
}

void gpsdistancefix(void){
  int8_t speedband;
  static int8_t oldspeedband;
  static int8_t speedcorrection=0;
  if (GPS_distanceToHome < 10000){
    speedband = 0;}
  else if (GPS_distanceToHome > 50000){
    speedband = 2;}
  else{
    speedband = 1;
    oldspeedband = speedband;}    
  if (speedband==oldspeedband){
    if (oldspeedband==0){
      speedcorrection--;}
    if (oldspeedband==2){
      speedcorrection++;}
    oldspeedband = speedband;}
  GPS_distanceToHome=(speedcorrection*65535) + GPS_distanceToHome;
} 


void ProcessSensors(void) {
  /*
    special note about filter: last row of array = averaged reading
  */ 
//-------------- ADC and PWM RSSI sensor read into filter array
  static uint8_t sensorindex;
  for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++){
    uint16_t sensortemp;
    sensortemp = analogRead(sensorpinarray[sensor]);
    if (sensor ==0){ 
      if (Settings[S_MAINVOLTAGE_VBAT]){
        sensortemp=MwVBat;}}
    if (sensor ==4){ 
      if (Settings[S_PWMRSSI]){
        #if defined RCRSSI
          sensortemp = MwRcData[RCRSSI]>>1;
        #elif defined INTPWMRSSI
          sensortemp = pwmRSSI>>1;
        #else
          sensortemp = pulseIn(PWMRSSIPIN, HIGH,18000)>>1;        
        #endif
        if (sensortemp==0) { // timed out - use previous
          sensortemp=sensorfilter[sensor][sensorindex];}}
      if(Settings[S_MWRSSI]){
        sensortemp = MwRssi;}}
    #if defined STAGE2FILTER // Use averaged change    
      sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];         
      sensorfilter[sensor][sensorindex] = (sensorfilter[sensor][sensorindex] + sensortemp)>>1;
    #elif defined SMOOTHFILTER // Shiki variable constraint probability trend change filter. Smooth filtering of small changes, but react fast to consistent changes
      #define FILTERMAX 128 //maximum change permitted each iteration 
      uint8_t filterdir;
      static uint8_t oldfilterdir[SENSORTOTAL];
      int16_t sensoraverage=sensorfilter[sensor][SENSORFILTERSIZE]>>3;
      sensorfilter[sensor][SENSORFILTERSIZE] = sensorfilter[sensor][SENSORFILTERSIZE] - sensorfilter[sensor][sensorindex];         
      if (sensorfilter[sensor][SENSORFILTERSIZE+1]<1){
        sensorfilter[sensor][SENSORFILTERSIZE+1]=1;}
      if (sensortemp != sensoraverage ){
          // determine direction of change
          if (sensortemp > sensoraverage ) {  //increasing
            filterdir=1;}
          else if (sensortemp < sensoraverage){  //increasing
            filterdir=0;}
          // compare to previous direction of change
          if (filterdir!=oldfilterdir[sensor]){ // direction changed => lost trust in value - reset value truth probability to lowest
            sensorfilter[sensor][SENSORFILTERSIZE+1] = 1;}
          else { // direction same => increase trust that change is valid - increase value truth probability
            sensorfilter[sensor][SENSORFILTERSIZE+1]=sensorfilter[sensor][SENSORFILTERSIZE+1] <<1;}
          // set maximum trust permitted per sensor read
          if (sensorfilter[sensor][SENSORFILTERSIZE+1] > FILTERMAX) {
            sensorfilter[sensor][SENSORFILTERSIZE+1] = FILTERMAX;}
          // set constrained value or if within limits, start to narrow filter 
          if (sensortemp > sensoraverage+sensorfilter[sensor][SENSORFILTERSIZE+1]) {
            sensorfilter[sensor][sensorindex] = sensoraverage+sensorfilter[sensor][SENSORFILTERSIZE+1];}  
          else if (sensortemp < sensoraverage-sensorfilter[sensor][SENSORFILTERSIZE+1]){
            sensorfilter[sensor][sensorindex] = sensoraverage-sensorfilter[sensor][SENSORFILTERSIZE+1];}
          // as within limits, start to narrow filter 
          else {
            sensorfilter[sensor][sensorindex] = sensortemp; 
            sensorfilter[sensor][SENSORFILTERSIZE+1]=sensorfilter[sensor][SENSORFILTERSIZE+1] >>2;}
          oldfilterdir[sensor]=filterdir;}
        // no change, reset filter
        else {
          sensorfilter[sensor][sensorindex] = sensortemp; 
          sensorfilter[sensor][SENSORFILTERSIZE+1]=1;}   
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
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (DIVIDER1v1);}
    else{
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] * (DIVIDER5v);}}
  else{
      voltage=sensorfilter[0][SENSORFILTERSIZE]>>3;}
  vidvoltageWarning = Settings[S_VIDVOLTAGEMIN];
  uint16_t vidvoltageRaw = sensorfilter[1][SENSORFILTERSIZE];
    if (!Settings[S_VREFERENCE]){
      vidvoltage = float(vidvoltageRaw) * Settings[S_VIDDIVIDERRATIO] * (DIVIDER1v1);}
    else {
      vidvoltage = float(vidvoltageRaw) * Settings[S_VIDDIVIDERRATIO] * (DIVIDER5v);}
  //-------------- Temperature
  #ifdef TEMPSENSOR
      temperature=sensorfilter[3][SENSORFILTERSIZE]>>3-TEMPZERO;
      temperature = map (temperature, TEMPZERO, 1024, 0 , TEMPMAX);
  #endif
  //-------------- Current
  if(!Settings[S_MWAMPERAGE]){
    if (!Settings[S_AMPERAGE_VIRTUAL]){ // Analogue
      amperage = sensorfilter[2][SENSORFILTERSIZE]>>3;
      amperage = map(amperage, Settings16[S16_AMPZERO], 1024, 0, Settings16[S16_AMPDIVIDERRATIO]);
      if (amperage < 0){
        amperage=0;}}
  else{  // Virtual
    uint32_t Vthrottle = constrain(MwRcData[THROTTLESTICK],LowT,HighT);
    Vthrottle = constrain((Vthrottle-1000)/10,0,100);
    amperage = (Vthrottle+(Vthrottle*Vthrottle*0.02))*Settings16[S16_AMPDIVIDERRATIO]*0.01;
    if(armed){
      amperage += Settings16[S16_AMPZERO];}
    else{
      amperage = Settings16[S16_AMPZERO];}}}
  else{
    amperage = MWAmperage / AMPERAGE_DIV;}
  //-------------- RSSI
  if (Settings[S_DISPLAYRSSI]){           
    rssi = sensorfilter[4][SENSORFILTERSIZE]>>3; // filter and remain 16 bit
    if (configMode){
      if((timer.rssiTimer==15)) {
        Settings16[S16_RSSIMAX]=rssi;} // tx on
      if((timer.rssiTimer==1)) {
        Settings16[S16_RSSIMIN]=rssi; // tx off
        timer.rssiTimer=0;}}
    rssi = map(rssi, Settings16[S16_RSSIMIN], Settings16[S16_RSSIMAX], 0, 100);
    rssi=constrain(rssi,0,100);}
  //-------------- For filter support
  sensorindex++;
  if (sensorindex >= SENSORFILTERSIZE){
    sensorindex = 0;}
}

unsigned long FastpulseIn(uint8_t pin, uint8_t state, unsigned long timeout){
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t stateMask = (state ? bit : 0);
  unsigned long width = 0;
  unsigned long numloops = 0;
  unsigned long maxloops = timeout;
  while ((*portInputRegister(port) & bit) == stateMask){
    if (numloops++ == maxloops){
      return 0;}}
  while ((*portInputRegister(port) & bit) != stateMask){
    if (numloops++ == maxloops){
      return 0;}}
  while ((*portInputRegister(port) & bit) == stateMask){
    if (numloops++ == maxloops){
      return 0;}
    width++;}
  return width; 
}

//------------------------------------------------------------------------
// MISC
#if defined PPMOSDCONTROL
  void initRSSIint() { // enable ONLY RSSI pin A3 for interrupt (bit 3 on port C)
    DDRC &= ~(1 << DDC3);
    cli();
    PCICR =  (1 << PCIE1);
    PCMSK1 = (1 << PCINT11);
    sei();
  }
  
  ISR(PCINT1_vect) {
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
    if (!(pinstatus & (1<<DDC3))){ // RSSI pin A3 - ! measures low duration
      PulseDuration = CurrentTime-PulseStart;
      if ((750<PulseDuration) && (PulseDuration<2250)) {
        if (RCchan<8){// avoid array overflow if > standard 8 ch PPM
          MwRcData[RCchan] = PulseDuration;}} // Val updated
      RCchan++;}
    else{
      PulseStart = CurrentTime;}
  }
#endif //PPMOSDCONTROL


#if defined USE_AIRSPEED_SENSOR
  void useairspeed(){
    float airspeed_cal = AIRSPEED_CAL; //AIRSPEED_CAL; // move to GUI or config
    uint16_t airspeedsensor = sensorfilter[3][SENSORFILTERSIZE]>>3;
    if (airspeedsensor>(AIRSPEED_ZERO)){
      airspeedsensor = airspeedsensor-AIRSPEED_ZERO;}
    else{
      airspeedsensor = 0;}
    GPS_speed = 27.7777 * sqrt(airspeedsensor * airspeed_cal); // Need in cm/s for this
  }
#endif //USE_AIRSPEED_SENSOR 
