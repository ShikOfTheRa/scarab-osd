#ifdef VTX_RTC6705

#include "wireMacro.h"

#if defined(IMPULSERC_HELIX)

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

// A6 is used for power sensing

#elif defined(FFPV_INNOVA)

// SPILE = D9
#define CS_RBIT   _digital_pin_to_rbit(9)
#define CS_PORT   _digital_pin_to_port(9)
#define CS_DDR    _digital_pin_to_ddr(9)
// SPICLK = D8
#define SCK_RBIT  _digital_pin_to_rbit(8)
#define SCK_PORT  _digital_pin_to_port(8)
#define SCK_DDR   _digital_pin_to_ddr(8)
// SPIDATA = D10
#define MOSI_RBIT _digital_pin_to_rbit(10)
#define MOSI_PORT _digital_pin_to_port(10)
#define MOSI_DDR  _digital_pin_to_ddr(10)

#else
# error Unknown VTX integrated board
#endif

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define TOGGLE(x,y) (x^=(1<<y))

void vtx_init() {
  //Set pins to output
  CS_DDR |= (1 << CS_RBIT);
  MOSI_DDR |= (1 << MOSI_RBIT);
  SCK_DDR |= (1 << SCK_RBIT);

#ifdef IMPULSERC_HELIX
  PSW1_DDR |= (1 << PSW1_RBIT);
  PSW2_DDR |= (1 << PSW2_RBIT);
#endif

#ifdef VTX_LED
  VTXLED_DDR |= (1 << VTXLED_RBIT);
#endif

  // CS high
  SET(CS_PORT, CS_RBIT);

#ifdef IMPULSERC_HELIX
  // Low output power
  CLR(PSW1_PORT, PSW1_RBIT);
  CLR(PSW2_PORT, PSW2_RBIT);
#endif

#ifdef VTX_LED
  // LED off
  SET(VTXLED_PORT, VTXLED_RBIT);
#endif

  vtxPower = Settings[S_VTX_POWER];
  vtxBand = Settings[S_VTX_BAND];
  vtxChannel = Settings[S_VTX_CHANNEL];

  vtx_set_frequency(vtxBand, vtxChannel);
}

void vtx_transfer(int reg, int rw, uint32_t data)
{
  uint32_t regdata = (reg << 0)|(rw << 4)|data;

  for (int i = 0; i <25; i ++)
  {
    if (regdata & ((uint32_t) 1 << i))
      SET(MOSI_PORT, MOSI_RBIT);
    else
      CLR(MOSI_PORT, MOSI_RBIT);
      
    _delay_us (40);
    SET(SCK_PORT, SCK_RBIT);
    _delay_us (40);
    CLR(SCK_PORT, SCK_RBIT);
  }

  _delay_us (10);
    
  SET(CS_PORT, CS_RBIT);
} 

void vtx_set_power(uint8_t power)
{ 
#if defined(IMPULSERC_HELIX)
  switch (power) {
    case 0:
      CLR(PSW1_PORT, PSW1_RBIT);
      CLR(PSW2_PORT, PSW2_RBIT);
      break;
    case 1:
      SET(PSW1_PORT, PSW1_RBIT);
      CLR(PSW2_PORT, PSW2_RBIT);
      break;
    case 2:
      SET(PSW1_PORT, PSW1_RBIT);
      SET(PSW2_PORT, PSW2_RBIT);
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
  uint32_t data0 = 0;
  uint8_t i = 0;
  if (frequency <5956 && frequency> 5644)
  {
    data0 = (1 << 4) |  (400 << 5);

    CLR(SCK_PORT, SCK_RBIT);
    CLR(CS_PORT, CS_RBIT);
    
    
    for (i = 0; i <25; i ++)
    {
      
      if (data0 & ((uint32_t) 1 << i))
        SET(MOSI_PORT, MOSI_RBIT);
      else
        CLR(MOSI_PORT, MOSI_RBIT);
      
      _delay_us (40);
      SET(SCK_PORT, SCK_RBIT);
      _delay_us (40);
      CLR(SCK_PORT, SCK_RBIT);
    }
    
    _delay_us (10);
  
    N = ((uint32_t) frequency * 25) / 64;
    A = ((uint32_t) frequency * 25 - N * 64);
    data0 = (1 << 0) |  (1 << 4) |  ((A & 0x7F) << 5) |  ((N & 0x1FFF) << 12);

    vtx_transfer(1, 1, data0);
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

void vtx_toggle_led(void)
{
  if (ledToggle > 0 && (currentMillis - lastToggle) > 150)
  {
    TOGGLE(VTXLED_PORT, VTXLED_RBIT);
    lastToggle = currentMillis;
    ledToggle--;
  }
}
#endif

#ifdef IMPULSERC_HELIX
void vtx_process_state(uint32_t currentMillis, uint8_t band, uint8_t channel)
{
#ifdef VTX_LED
  vtx_toggle_led();
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
