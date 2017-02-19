#ifdef VTX_RTC6705

#include "wireMacro.h"

#if defined(IMPULSERC_HELIX)

# define RTC_SPILE   15 // A1
# define RTC_SPICLK  14 // A0
# define RTC_SPIDATA 16 // A2

# define VTX_PSW1_PIN 5
# define VTX_PSW2_PIN 6
# define VTX_LED_PIN  8

// A6 is used for power sensing

#if 0
// SPILE = D15 (A1)
#define CS_RBIT   _digital_pin_to_rbit(15)
#define CS_PORT   _digital_pin_to_port(15)
#define CS_DDR    _digital_pin_to_ddr(15)
// SPICLK = D14 (A0)
#define SCK_RBIT  _digital_pin_to_rbit(14)
#define SCK_PORT  _digital_pin_to_port(14)
#define SCK_DDR   _digital_pin_to_ddr(14)
// SPIDATA = D16 (A2)
#define MOSI_RBIT _digital_pin_to_rbit(16)
#define MOSI_PORT _digital_pin_to_port(16)
#define MOSI_DDR  _digital_pin_to_ddr(16)
// PSW1 = D5
# define PSW1_RBIT _digital_pin_to_rbit(5)
# define PSW1_PORT _digital_pin_to_port(5)
# define PSW1_DDR  _digital_pin_to_ddr(5)
// PSW2 = D6
# define PSW2_RBIT _digital_pin_to_rbit(6)
# define PSW2_PORT _digital_pin_to_port(6)
# define PSW2_DDR  _digital_pin_to_ddr(6)
// VTXLED = D8
# define VTXLED_RBIT _digital_pin_to_rbit(8)
# define VTXLED_PORT _digital_pin_to_port(8)
# define VTXLED_DDR  _digital_pin_to_ddr(8)
#endif


#elif defined(FFPV_INNOVA)

# define RTC_SPILE    9
# define RTC_SPICLK   8
# define RTC_SPIDATA 10

#if 0
// SPILE = D9
#define CS_RBIT   _digital_pin_to_rbit(RTC_SPILE)
#define CS_PORT   _digital_pin_to_port(RTC_SPILE)
#define CS_DDR    _digital_pin_to_ddr(RTC_SPILE)
// SPICLK = D8
#define SCK_RBIT  _digital_pin_to_rbit(RTC_SPICLK)
#define SCK_PORT  _digital_pin_to_port(RTC_SPICLK)
#define SCK_DDR   _digital_pin_to_ddr(RTC_SPICLK)
// SPIDATA = D10
#define MOSI_RBIT _digital_pin_to_rbit(RTC_SPIDATA)
#define MOSI_PORT _digital_pin_to_port(RTC_SPIDATA)
#define MOSI_DDR  _digital_pin_to_ddr(RTC_SPIDATA)
#endif

#else
# error Unknown VTX integrated board
#endif

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define TOGGLE(x,y) (x^=(1<<y))

void vtx_init() {

#if 1
  _pinModeOut(RTC_SPILE);
  _pinModeOut(RTC_SPICLK);
  _pinModeOut(RTC_SPIDATA);
#else
  CS_DDR |= (1 << CS_RBIT);
  MOSI_DDR |= (1 << MOSI_RBIT);
  SCK_DDR |= (1 << SCK_RBIT);
#endif

#ifdef IMPULSERC_HELIX

#if 1
  _pinModeOut(VTX_PSW1_PIN);
  _pinModeOut(VTX_PSW2_PIN);
#else
  PSW1_DDR |= (1 << PSW1_RBIT);
  PSW2_DDR |= (1 << PSW2_RBIT);
#endif

#endif

#ifdef VTX_LED

#if 1
  _pinModeOut(VTX_LED_PIN);
#else
  VTXLED_DDR |= (1 << VTXLED_RBIT);
#endif

#endif

  // CS high
#if 1
  _digitalHi(RTC_SPILE);
#else
  SET(CS_PORT, CS_RBIT);
#endif

#ifdef IMPULSERC_HELIX
  // Low output power
#if 1
  _digitalLo(VTX_PSW1_PIN);
  _digitalLo(VTX_PSW2_PIN);
#else
  CLR(PSW1_PORT, PSW1_RBIT);
  CLR(PSW2_PORT, PSW2_RBIT);
#endif

#endif

#ifdef VTX_LED
  // LED off
#if 1
  _digitalHi(VTX_LED_PIN);
#else
  SET(VTXLED_PORT, VTXLED_RBIT);
#endif

#endif

  vtxPower = Settings[S_VTX_POWER];
  vtxBand = Settings[S_VTX_BAND];
  vtxChannel = Settings[S_VTX_CHANNEL];

  vtx_set_frequency(vtxBand, vtxChannel);
}

void vtx_transfer(int reg, int rw, uint32_t data)
{
  uint32_t regdata = (reg << 0)|(rw << 4)|(data << 5);

#if 1
  _digitalLo(RTC_SPICLK);
  _digitalLo(RTC_SPILE);
#else
  CLR(SCK_PORT, SCK_RBIT);
  CLR(CS_PORT, CS_RBIT);
#endif

  for (int i = 0; i <25; i ++)
  {

    if (regdata & ((uint32_t) 1 << i))
#if 1
      _digitalHi(RTC_SPIDATA);
#else
      SET(MOSI_PORT, MOSI_RBIT);
#endif
    else
#if 1
      _digitalLo(RTC_SPIDATA);
#else
      CLR(MOSI_PORT, MOSI_RBIT);
#endif
      
    _delay_us (40);
#if 1
    _digitalHi(RTC_SPICLK);
#else
    SET(SCK_PORT, SCK_RBIT);
#endif
    _delay_us (40);
#if 1
    _digitalLo(RTC_SPICLK);
#else
    CLR(SCK_PORT, SCK_RBIT);
#endif
  }

  _delay_us (10);
#if 1
  _digitalLo(RTC_SPILE);
#else
  SET(CS_PORT, CS_RBIT);
#endif
} 

void vtx_set_power(uint8_t power)
{ 
#if defined(IMPULSERC_HELIX)
  switch (power) {
    case 0:
#if 1
      _digitalLo(VTX_PSW1_PIN);
      _digitalLo(VTX_PSW2_PIN);
#else
      CLR(PSW1_PORT, PSW1_RBIT);
      CLR(PSW2_PORT, PSW2_RBIT);
#endif
      break;
    case 1:
#if 1
      _digitalHi(VTX_PSW1_PIN);
      _digitalLo(VTX_PSW2_PIN);
#else
      SET(PSW1_PORT, PSW1_RBIT);
      CLR(PSW2_PORT, PSW2_RBIT);
#endif
      break;
    case 2:
#if 1
      _digitalHi(VTX_PSW1_PIN);
      _digitalHi(VTX_PSW2_PIN);
#else
      SET(PSW1_PORT, PSW1_RBIT);
      SET(PSW2_PORT, PSW2_RBIT);
#endif
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

//http://fpv-community.de/showthread.php?28337-RTC6705-Sender-auf-FatShark-Frequenz&p=844717&viewfull=1#post844717

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
#if 1
    _digitalToggle(VTX_LED_PIN);
#else
    TOGGLE(VTXLED_PORT, VTXLED_RBIT);
#endif
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
