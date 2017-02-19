#ifdef VTX_RTC6705

#include "wireMacro.h"

#if defined(IMPULSERC_HELIX)

# define RTC_SPILE   15 // A1
# define RTC_SPICLK  14 // A0
# define RTC_SPIDATA 16 // A2

# define VTX_PSW1_PIN 5
# define VTX_PSW2_PIN 6
# define VTX_LED_PIN  8

// Note: A6 is additionally used for power sensing

#elif defined(FFPV_INNOVA)

# define RTC_SPILE    9
# define RTC_SPICLK   8
# define RTC_SPIDATA 10

#else
# error Unknown VTX integrated board
#endif

void vtx_init() {

  _pinModeOut(RTC_SPILE);
  _pinModeOut(RTC_SPICLK);
  _pinModeOut(RTC_SPIDATA);

#ifdef IMPULSERC_HELIX
  _pinModeOut(VTX_PSW1_PIN);
  _pinModeOut(VTX_PSW2_PIN);
#endif

#ifdef VTX_LED
  _pinModeOut(VTX_LED_PIN);
#endif

  // CS high
  _digitalHi(RTC_SPILE);

#ifdef IMPULSERC_HELIX
  // Low output power
  _digitalLo(VTX_PSW1_PIN);
  _digitalLo(VTX_PSW2_PIN);
#endif

#ifdef VTX_LED
  // LED off
  _digitalHi(VTX_LED_PIN);
#endif

  vtxPower = Settings[S_VTX_POWER];
  vtxBand = Settings[S_VTX_BAND];
  vtxChannel = Settings[S_VTX_CHANNEL];

  vtx_set_frequency(vtxBand, vtxChannel);
}

void vtx_transfer(int reg, int rw, uint32_t data)
{
  uint32_t regdata = (reg << 0)|(rw << 4)|(data << 5);

  _digitalLo(RTC_SPICLK);
  _digitalLo(RTC_SPILE);

  for (int i = 0; i <25; i ++)
  {

    if (regdata & ((uint32_t) 1 << i))
      _digitalHi(RTC_SPIDATA);
    else
      _digitalLo(RTC_SPIDATA);
      
    _delay_us (40);

    _digitalHi(RTC_SPICLK);
    _delay_us (40);

    _digitalLo(RTC_SPICLK);
  }

  _delay_us (10);
  _digitalLo(RTC_SPILE);
} 

void vtx_set_power(uint8_t power)
{ 
#if defined(IMPULSERC_HELIX)
  switch (power) {
    case 0:
      _digitalLo(VTX_PSW1_PIN);
      _digitalLo(VTX_PSW2_PIN);
      break;

    case 1:
      _digitalHi(VTX_PSW1_PIN);
      _digitalLo(VTX_PSW2_PIN);
      break;

    case 2:
      _digitalHi(VTX_PSW1_PIN);
      _digitalHi(VTX_PSW2_PIN);
      break;
  }

#elif defined(FFPV_INNOVA)

  switch (power) {
    case 0:
      vtx_transfer(7, 1, 0x00040); // 25mW, per FFPV
      break;

    case 1:
      vtx_transfer(7, 1, 0x04E8D); // 200mW, per FFPV
      break;
  }

#else
# error Unknown VTX integrated board
#endif
}

void vtx_set_frequency(uint8_t band, uint8_t channel)
{
  uint16_t frequency = pgm_read_word(&vtx_frequencies[band][channel]);
  
  uint32_t N = 0;
  uint16_t A = 0;
  uint32_t data;
  uint8_t i = 0;
  if (frequency <5956 && frequency> 5644)
  {
    vtx_transfer(0, 1, 400);
    
    N = ((uint32_t) frequency * 25) / 64;
    A = ((uint32_t) frequency * 25 - N * 64);

    data = (A & 0x7F) | ((N & 0x1FFF) << 7);

    vtx_transfer(1, 1, data);
  }
}

#ifdef IMPULSERC_HELIX
bool powered = 1;
bool wasPowered = 0;
uint32_t debounce = 0;
#endif

#ifdef VTX_LED 
uint16_t ledToggle = 0;
uint32_t lastToggle = 0;

void vtx_flash_led(uint8_t count)
{
  if (ledToggle == 0)
    ledToggle = count * 2;
}

void vtx_toggle_led(uint32_t currentMillis)
{
  if (ledToggle > 0 && (currentMillis - lastToggle) > 150)
  {
    _digitalToggle(VTX_LED_PIN);

    lastToggle = currentMillis;
    ledToggle--;
  }
}
#endif

#ifdef IMPULSERC_HELIX
void vtx_process_state(uint32_t currentMillis, uint8_t band, uint8_t channel)
{
#ifdef VTX_LED
  vtx_toggle_led(currentMillis);
#endif

  bool reading = analogRead(A6) > 900 ? true : false;

  if (reading != wasPowered)
    debounce = currentMillis;

  if ((currentMillis - debounce) > 100 && reading != powered)
  {
    powered = reading;

    if (powered)
      vtx_set_frequency(band, channel);
  }
  
  wasPowered = reading;
}
#endif

#if 0 // XXX Not currently used
void vtx_save(){
  Settings[S_VTX_POWER]=vtxPower;
  if (vtxBand != Settings[S_VTX_BAND] || vtxChannel != Settings[S_VTX_CHANNEL])
  {
    Settings[S_VTX_BAND]=vtxBand;
    Settings[S_VTX_CHANNEL]=vtxChannel;
    vtx_set_frequency(vtxBand, vtxChannel);
  }
}
#endif

#endif
