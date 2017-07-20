#ifdef KKAUDIOVARIO

/*
  KK AUDIO VARIO components by 
  By Rolf R Bakke (kapeteinkuk)
  Modified by Shiki for MWOSD
*/

/*
uint8_t _pinMask = 0;         // Pin bitmask.
volatile uint8_t *_pinOutput; // Output port register

void NewTone(uint8_t pin, unsigned long frequency, unsigned long length) {
  uint8_t prescaler = _BV(CS10);                 // Try using prescaler 1 first.
  unsigned long top = F_CPU / frequency / 4 - 1; // Calculate the top.
  if (top > 65535) {                             // If not in the range for prescaler 1, use prescaler 256 (61 Hz and lower @ 16 MHz).
    prescaler = _BV(CS12);                       // Set the 256 prescaler bit.
    top = top / 256 - 1;                         // Calculate the top using prescaler 256.
  }

  if (_pinMask == 0) {
    _pinMask = digitalPinToBitMask(pin);                    // Get the port register bitmask for the pin.
    _pinOutput = portOutputRegister(digitalPinToPort(pin)); // Get the output port register for the pin.
    uint8_t *_pinMode = (uint8_t *) portModeRegister(digitalPinToPort(pin)); // Get the port mode register for the pin.
    *_pinMode |= _pinMask; // Set the pin to Output mode.
  }

  ICR1    = top;                     // Set the top.
  if (TCNT1 > top) TCNT1 = top;      // Counter over the top, put within range.
  TCCR1B  = _BV(WGM13)  | prescaler; // Set PWM, phase and frequency corrected (ICR1) and prescaler.
  TCCR1A  = _BV(COM1B0);
  TIMSK1 |= _BV(OCIE1A);             // Activate the timer interrupt.
}

void noNewTone(uint8_t pin) {
  TIMSK1 &= ~_BV(OCIE1A);   // Remove the timer interrupt.
  TCCR1B  = _BV(CS11);      // Default clock prescaler of 8.
  TCCR1A  = _BV(WGM10);     // Set to defaults so PWM can work like normal (PWM, phase corrected, 8bit).
  *_pinOutput &= ~_pinMask; // Set pin to LOW.
  _pinMask = 0; // Flag so we know note is no longer playing.
}

ISR(TIMER1_COMPA_vect) { // Timer interrupt vector.
  *_pinOutput ^= _pinMask; // Toggle the pin state.
}
*/

void AudioVarioUpdate()
{
#ifdef AUDIOVARIOSWITCH
  if(!fieldIsVisible(MwClimbRatePosition))
    return;
  if(!Settings[S_VARIO])
    return;
#endif //AUDIOVARIOSWITCH
#ifdef AUDIOVARIORC // no audio vario when throttle on
  if (MwRcData[THROTTLESTICK]> AUDIOVARIORC) {
    return; 
  }
#endif //AUDIOVARIORC

  
  pressure = getPressure();
  lowpassFast = lowpassFast + (pressure - lowpassFast) * 0.1;
  lowpassSlow = lowpassSlow + (pressure - lowpassSlow) * 0.05;
  
  toneFreq = (lowpassSlow - lowpassFast) * 50;  
  toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;

  toneFreq = constrain(toneFreqLowpass, -500, 500);
  ddsAcc += toneFreq * 100 + 2000;
  
  if ((toneFreq < KKDEADBANDLOW) ||  ((toneFreq > KKDEADBANDHIGH)  && (ddsAcc > 0))) 
  {
    tone(KKAUDIOVARIO, toneFreq + 510);  
//    NewTone(KKAUDIOVARIO, toneFreq + 510, 0);  
  }
  else
  {
    noTone(KKAUDIOVARIO);
//    noNewTone(KKAUDIOVARIO);
  }
}


long getPressure()
{
  long D1, D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;
 
  D1 = getData(0x48, 10);
  D2 = getData(0x50, 1);

  dT = D2 - ((long)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)) / (float)100;
  OFF = ((unsigned long)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
    
  return P;
}


long getData(byte command, byte del)
{
  long result = 0;
  twiSendCommand(0x77, command);
  delay(del);
  twiSendCommand(0x77, 0x00);
  Wire.requestFrom(0x77, 3);
  if(Wire.available()!=3) ; //Serial.println("Error: raw data not available");
  for (int i = 0; i <= 2; i++)
  {
    result = (result<<8) | Wire.read(); 
  }
  return result;
}


void setupSensor()
{
  twiSendCommand(0x77, 0x1e);
  delay(100);
  
  for (byte i = 1; i <=6; i++)
  {
    unsigned int low, high;

    twiSendCommand(0x77, 0xa0 + i * 2);
    Wire.requestFrom(0x77, 2);
    if(Wire.available()!=2) ; //Serial.println("Error: calibration data not available");
    high = Wire.read();
    low = Wire.read();
    calibrationData[i] = high<<8 | low;
//    Serial.print("calibration data #");
//    Serial.print(i);
//    Serial.print(" = ");
//    Serial.println( calibrationData[i] ); 
  }
}


void twiSendCommand(byte address, byte command)
{
  Wire.beginTransmission(address);
  if (!Wire.write(command)) ; //Serial.println("Error: write()");
  if (Wire.endTransmission()) 
  {
//    Serial.print("Error when sending command: ");
//    Serial.println(command, HEX);
  }
}
#endif KKAUDIOVARIO


