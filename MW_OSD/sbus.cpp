/*
  sbus.cpp

  Copyright (c) 2017, Fabrizio Di Vittorio (fdivitto2013@gmail.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Origin: https://github.com/fdivitto/sbus
  Modified to only enable interrupts on PCINT1_vect
*/

#include "sbus.h"


// quick IO functions
#define portOfPin(P)        portOutputRegister(digitalPinToPort(P))
#define ddrOfPin(P)         portModeRegister(digitalPinToPort(P))
#define pinOfPin(P)         portInputRegister(digitalPinToPort(P))
#define pinMask(P)          digitalPinToBitMask(P)
#define pinAsInput(P)       (*(ddrOfPin(P)) &= ~pinMask(P))
#define pinAsInputPullUp(P) (*(ddrOfPin(P)) &= ~pinMask(P)); digitalHigh(P)
#define pinAsOutput(P)      (*(ddrOfPin(P)) |= pinMask(P))
#define digitalLow(P)       (*(portOfPin(P)) &= ~pinMask(P))
#define digitalHigh(P)      (*(portOfPin(P)) |= pinMask(P))
#define isHigh(P)           ((*(pinOfPin(P)) & pinMask(P)) > 0)
#define isLow(P)            ((*(pinOfPin(P)) & pinMask(P)) == 0)
#define pinGet(pin, mask)   (((*(pin)) & (mask)) > 0)

static volatile uint8_t * s_pin;
static uint8_t s_pinMask;
static uint8_t s_PCICRMask;

static volatile mode_t s_mode;

// contains which word is currently receiving (0..24, 24 is the last word)
static volatile uint8_t s_receivingWordIndex = 0;

// received frame
static volatile uint8_t s_frame[23];

static volatile bool s_hasSignal = false;

  
struct chinfo_t {
  uint8_t idx;
  uint8_t shift1;
  uint8_t shift2;
  uint8_t shift3; // 11 = byte 3 ignored
};


static const chinfo_t CHINFO[16] PROGMEM = { {0,  0, 8, 11}, {1,  3, 5, 11}, {2,  6, 2, 10}, {4,  1, 7, 11}, {5,  4, 4, 11},
                                           {6,  7, 1, 9},  {8,  2, 6, 11}, {9,  5, 3, 11}, {11, 0, 8, 11}, {12, 3, 5, 11},               
                                           {13, 6, 2, 10}, {15, 1, 7, 11}, {16, 4, 4, 11}, {17, 7, 1, 9},  {19, 2, 6, 11},  {20, 5, 3, 11} };               

// flag field
#define FLAG_CHANNEL_17  1
#define FLAG_CHANNEL_18  2
#define FLAG_SIGNAL_LOSS 4
#define FLAG_FAILSAFE    8

#define CHANNEL_MIN 173
#define CHANNEL_MAX 1812



inline void enablePinChangeInterrupts()
{
  // clear any outstanding interrupt
  PCIFR |= s_PCICRMask; 
  
  // enable interrupt for the group
  PCICR |= s_PCICRMask;   
}


inline void disablePinChangeInterrupts()
{
  PCICR &= ~s_PCICRMask; 
}


// handle pin change interrupt here
static void handleInterrupt();
#define ISRN(N) ISR(PCINT ## N ## _vect) { if (s_PCICRMask == 1 << (N)) handleInterrupt(); }
//ISRN(0)
#ifdef SBUS_CONTROL
#ifdef SBUS_ISR2
ISRN(2)
#else
ISRN(1)
#endif
#endif // SBUS_CONTROL

static void handleInterrupt()
{
  
  // start bit?
  if (pinGet(s_pin, s_pinMask)) {

    // reset timer 2 counter
    TCNT2 = 0;

    disablePinChangeInterrupts();

    // start bit received    
    uint8_t receivingWord = 0;    

    // reset OCF2A flag by writing "1"
    TIFR2 |= 1 << OCF2A;

    uint8_t parity = 0xFF;    

    // receive other bits (including parity bit, ignore stop bits)
    for (uint8_t receivedBitIndex = 0; receivedBitIndex < 9; ++receivedBitIndex) {

      // wait for TCNT2 == OCR2A      
      // warn: inside this loop interrupts are re-enabled to allow other libraries to work correctly (ie Servo library)
      interrupts();
      while (!(TIFR2 & (1 << OCF2A)))
        ;
      noInterrupts();

      // reset OCF2A flag by writing "1"
      TIFR2 |= 1 << OCF2A;
      
      // sample current bit  
      if (pinGet(s_pin, s_pinMask))
        receivingWord |= 1 << receivedBitIndex; // parity shift here as >7 bit, so it is just discarded
      else
        parity = ~parity;
      
    }

    // check parity
    if (!parity) {
      
      // parity check failed!
      s_receivingWordIndex = 0;
      
    } else {

      // parity ok
      
      receivingWord = ~receivingWord;
  
      if (s_receivingWordIndex == 0) {
  
        //  check start word (must be 0x0F)
        if (receivingWord == 0x0F)          
          ++s_receivingWordIndex; // bypass this word

      } else if (s_receivingWordIndex == 24) {

        if (receivingWord == 0x00) 
          ++s_receivingWordIndex; // bypass this word

      } else {
          
        // save channels and flags and last ending word
        s_frame[s_receivingWordIndex - 1] = receivingWord;
        s_hasSignal = true;

        // next word          
        ++s_receivingWordIndex;
        
      }
  
    }
    
    // reset pin change interrupt flag
    PCIFR |= s_PCICRMask;

    if (s_receivingWordIndex < 25 || s_mode == sbusNonBlocking)
      enablePinChangeInterrupts();  

    if (s_receivingWordIndex == 25)                
        s_receivingWordIndex = 0;    // last word, restart word count
  
  }

    
}


// used only for blocking operations (mode = sbusBlocking)
bool SBUS::waitFrame(uint32_t timeOut)
{
  if (s_mode == sbusBlocking) {

    // will be disabled inside the ISR
    enablePinChangeInterrupts();  
    
    // wait until Pin change bit return back to 0
    uint32_t t0 = millis();
    while (PCICR & s_PCICRMask)
      if (millis() - t0 >= timeOut)  // note: this millis() compare is overflow safe due unsigned diffs
        return false; // timeout!
  }

  return true;
}


// channels start from 1 to 18
// returns value 173..1812 (a received by SBUS)
uint16_t SBUS::getChannelRaw(uint8_t channelIndex)
{
  if (channelIndex >= 1 && channelIndex <= 16) {
    
    chinfo_t chinfo;
    memcpy_P(&chinfo, CHINFO + channelIndex - 1, sizeof(chinfo_t));
  
    uint8_t idx = chinfo.idx;
  
    noInterrupts();
    uint8_t b1 = s_frame[idx++];
    uint8_t b2 = s_frame[idx++];
    uint8_t b3 = s_frame[idx];
    interrupts();
  
    return ((b1 >> chinfo.shift1) | (b2 << chinfo.shift2) | (b3 << chinfo.shift3)) & 0x7FF;
    
  } else if (channelIndex == 17 || channelIndex == 18) {

    if (channelIndex == 17)
      return s_frame[22] & FLAG_CHANNEL_17 ? CHANNEL_MAX : CHANNEL_MIN;
    else
      return s_frame[22] & FLAG_CHANNEL_18 ? CHANNEL_MAX : CHANNEL_MIN;
    
  } else
  
    return 0; // error
}


// channels start from 1 to 18
// returns value 988..2012 (cleanflight friendly)
uint16_t SBUS::getChannel(uint8_t channelIndex)
{
  return 5 * getChannelRaw(channelIndex) / 8 + 880;
}


bool SBUS::hasSignal()
{
  return s_hasSignal;
}


bool SBUS::signalLossActive()
{
  return s_frame[22] & FLAG_SIGNAL_LOSS;
}


bool SBUS::failsafeActive()
{
  return s_frame[22] & FLAG_FAILSAFE;
}


// if mode = sbusNonBlocking, an interrupt is always generated for every frame received. getChannel (or getChannelRaw) is no-blocking
// if mode = sbusBlocking, interrupts are enabled only inside getChannel (or getChannelRaw), which becomes blocking (until arrive of a new frame)
void SBUS::begin(uint8_t pin, mode_t mode)
{
  s_mode = mode;

  s_pin        = pinOfPin(pin);
  s_pinMask    = pinMask(pin);
  s_PCICRMask  = 1 << digitalPinToPCICRbit(pin);
  
  //// setup TIMER 2 CTC

  // select "Clear Timer on Compare (CTC)" mode
  TCCR2A = 1 << WGM21; 
  
  // no prescaling
  TCCR2B = 1 << CS20; 

  // set TOP timer 2 value
  // with no-prescaler 1/16000000=0.00000000625, each bit requires 10us, so reset timer 2 every 10us, so reset timer every 10/0.0625 = 160 ticks
  OCR2A = F_CPU / 1000000 * 10 - 1; // for 16MHz = 159;  

  //// setup pin change interrupt
  
  pinAsInput(pin);
  
  // enable pin
  *digitalPinToPCMSK(pin) |= 1 << digitalPinToPCMSKbit(pin);  

  if (mode == sbusNonBlocking)
    enablePinChangeInterrupts();
}

  









