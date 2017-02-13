#ifdef VTX_RTC6705

#define CS        PINC1
#define CS_PORT   PORTC
#define CS_DDR    DDRC

#define SCK       PINC0
#define SCK_PORT  PORTC
#define SCK_DDR   DDRC

#define MOSI      PINC2
#define MOSI_PORT PORTC
#define MOSI_DDR  DDRC

#define PSW1      PIND5
#define PSW1_PORT PORTD
#define PSW1_DDR  DDRD

#define PSW2      PIND6
#define PSW2_PORT PORTD
#define PSW2_DDR  DDRD

#define LED      PINB0
#define LED_PORT PORTB
#define LED_DDR  DDRB

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define TOGGLE(x,y) (x^=(1<<y))

void vtx_init() {
  //Set pins to output
  CS_DDR |= (1 << CS);
  MOSI_DDR |= (1 << MOSI);
  SCK_DDR |= (1 << SCK);
  PSW1_DDR |= (1 << PSW1);
  PSW2_DDR |= (1 << PSW2);
  LED_DDR |= (1 << LED);

  //CS high, low output power, LED off
  SET(CS_PORT, CS);
  CLR(PSW1_PORT, PSW1);
  CLR(PSW2_PORT, PSW2);
  SET(LED_PORT, LED);
}

void vtx_set_power(uint8_t power)
{ 
  switch (power)
  {
    case 0:
      CLR(PSW1_PORT, PSW1);
      CLR(PSW2_PORT, PSW2);
      break;
    case 1:
      SET(PSW1_PORT, PSW1);
      CLR(PSW2_PORT, PSW2);
      break;
    case 2:
      SET(PSW1_PORT, PSW1);
      SET(PSW2_PORT, PSW2);
      break;
  }
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

    CLR(SCK_PORT, SCK);
    CLR(CS_PORT, CS);
    
    
    for (i = 0; i <25; i ++)
    {
      
      if (data0 & ((uint32_t) 1 << i))
        SET(MOSI_PORT, MOSI);
      else
        CLR(MOSI_PORT, MOSI);
      
      _delay_us (40);
      SET(SCK_PORT, SCK);
      _delay_us (40);
      CLR(SCK_PORT, SCK);
    }
    
    _delay_us (10);
  
    N = ((uint32_t) frequency * 25) / 64;
    A = ((uint32_t) frequency * 25 - N * 64);
    data0 = (1 << 0) |  (1 << 4) |  ((A & 0x7F) << 5) |  ((N & 0x1FFF) << 12);
  
    for (i = 0; i <25; i ++)
    {
      if (data0 & ((uint32_t) 1 << i))
        SET(MOSI_PORT, MOSI);
      else
        CLR(MOSI_PORT, MOSI);
      
      _delay_us (40);
      SET(SCK_PORT, SCK);
      _delay_us (40);
      CLR(SCK_PORT, SCK);
    }
    
    _delay_us (10);
    
    SET(CS_PORT, CS);
   }
 } 

bool powered = 1;
bool wasPowered = 0;
uint32_t debounce = 0;
uint16_t ledToggle = 0;
uint32_t lastToggle = 0;


void vtx_flash_led(uint8_t count)
{
  if (ledToggle == 0)
    ledToggle = count * 2;
}

void vtx_process_state(uint32_t currentMillis, uint8_t band, uint8_t channel)
{
  if (ledToggle > 0 && (currentMillis - lastToggle) > 150)
  {
    TOGGLE(LED_PORT, LED);
    lastToggle = currentMillis;
    ledToggle--;
  }
  
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

void vtx_save(){
  Settings[S_VTX_POWER]=vtxPower;
  if (vtxBand != Settings[S_VTX_BAND] || vtxChannel != Settings[S_VTX_CHANNEL])
  {
    Settings[S_VTX_BAND]=vtxBand;
    Settings[S_VTX_CHANNEL]=vtxChannel;
    vtx_set_frequency(vtxBand, vtxChannel);
  }
}

void vtx_read(){
  vtxPower=Settings[S_VTX_POWER];
  vtxBand=Settings[S_VTX_BAND];
  vtxChannel=Settings[S_VTX_CHANNEL];
  vtx_set_frequency(vtxBand, vtxChannel);
}
#endif
