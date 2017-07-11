#ifdef KKAUDIOVARIO

/*
  KK AUDIO VARIO components by 
  By Rolf R Bakke (kapeteinkuk)
  Modified by Shiki for MWOSD
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
  }
  else
  {
    noTone(KKAUDIOVARIO);
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


