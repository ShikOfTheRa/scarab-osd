#include "platform.h"

void MAX7456Class::WriteBytes_P(const void* src, uint16_t address, size_t n)
{
#ifdef ARDUINO
  memcpy_P(&_screen[address], src, n);
#else
  memcpy(&_screen[address], src, n);
#endif
}

// Copy char into the screen buffer
void MAX7456Class::WriteChar(const char c, uint16_t address)
{
  _screen[address] = c;
}

// Copy string from ram into screen buffer
void MAX7456Class::WriteString(const char *string, uint16_t address)
{
  strcpy(&_screen[address], string);
}

// Copy string from progmem into the screen buffer
void MAX7456Class::WriteString_P(const char *string, uint16_t address)
{
  WriteBytes_P(string, address, strlen(string));
}

void MAX7456Class::Setup(void)
{
  uint8_t _max7456_reset=0x02;

  pinMode(MAX7456RESET,OUTPUT);
  digitalWrite(MAX7456RESET,LOW); //force reset
  delay(100);
  digitalWrite(MAX7456RESET,HIGH); //hard enable
  delay(100);

  pinMode(MAX7456SELECT,OUTPUT);
  digitalWrite(MAX7456SELECT,HIGH); //disable device

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(VSYNC, INPUT);

  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (4 meg)

  SPCR = (1<<SPE)|(1<<MSTR);
  SPSR = (1<<SPI2X);
  uint8_t spi_junk;
  spi_junk=SPSR;
  spi_junk=SPDR;
  delay(100);

  // force soft reset on Max7456
  digitalWrite(MAX7456SELECT,LOW);
  Send(VM0_reg, _max7456_reset);
  delay(100);


#ifdef AUTOCAM 
  pinMode(MAX7456SELECT,OUTPUT);
  digitalWrite(MAX7456SELECT,LOW);
  uint8_t srdata = 0;
  #if defined AUTOCAMWAIT 
    while ((B00000011 & srdata) == 0){
      spi_transfer(0xa0);
      srdata = spi_transfer(0xFF); 
      delay(100);
    }
  #else  
    spi_transfer(0xa0);
    srdata = spi_transfer(0xFF); 
  #endif //AUTOCAMWAIT  
  if ((B00000001 & srdata) == B00000001){     //PAL
      Settings[S_VIDEOSIGNALTYPE]=1; 
  }
  else if((B00000010 & srdata) == B00000010){ //NTSC
      Settings[S_VIDEOSIGNALTYPE]=0;
  }
#endif //AUTOCAM
   
#ifdef FASTPIXEL 
  // force fast pixel timing
  Send(MAX7456ADD_OSDM, 0x00);
  // Send(MAX7456ADD_OSDM, 0xEC);
  // uint8_t srdata = spi_transfer(0xFF); //get data byte
  // srdata = srdata & 0xEF;
  // Send(0x6c, srdata);
  delay(100);
#endif

  if(Settings[S_VIDEOSIGNALTYPE]) {   // PAL
    _max7456_reset = 0x42;
    _screen_size = 480;
    _screen_rows = 16;
  }
  else {                              // NTSC
    _max7456_reset = 0x02;
    _screen_size = 390;
    _screen_rows = 13;
  }

  // set all rows to same charactor black/white level
  uint8_t x;
  for(x = 0; x < _screen_rows; x++) {
    Send(MAX7456ADD_RB0+x, BWBRIGHTNESS);
  }

  // make sure the Max7456 is enabled
  spi_transfer(VM0_reg);

  if (Settings[S_VIDEOSIGNALTYPE]){
    spi_transfer(OSD_ENABLE|VIDEO_MODE_PAL);
  }
  else{
    spi_transfer(OSD_ENABLE|VIDEO_MODE_NTSC);
  }
  digitalWrite(MAX7456SELECT,HIGH);
  delay(100);
# ifdef USE_VSYNC
  EIMSK |= (1 << INT0);  // enable interuppt
  EICRA |= (1 << ISC01); // interrupt at the falling edge
  sei();
#endif
}

#ifdef USE_VSYNC
  volatile unsigned char vsync_wait = 0;
  ISR(INT0_vect) {
    vsync_wait = 0;
  }
#endif

void MAX7456Class::DrawScreen()
{
  uint16_t xx;
  #ifdef USE_VSYNC
    digitalWrite(MAX7456SELECT,LOW);
    spi_transfer(DMM_reg);
    spi_transfer(1);
    spi_transfer(DMAH_reg);
    spi_transfer(0);
    spi_transfer(DMAL_reg);
    spi_transfer(0);
    vsync_wait = 1;
    uint32_t vsynctimer=40+millis();

  #endif

  digitalWrite(MAX7456SELECT,LOW);

  for(xx=0;xx<_screen_size;++xx){
    #ifdef USE_VSYNC
      SPDR = MAX7456ADD_DMDI;
      if (xx==240) {
        vsynctimer=40+millis();
        vsync_wait = 1;      // Not enough time to load all characters within VBI period. This splits and waits until next field
      }
      while (!(SPSR & (1<<SPIF)));     // Wait the end of the last SPI transmission is clear
      while (vsync_wait){
        if (millis()>vsynctimer){
          vsync_wait=0;
        }
        else{
          serialMSPreceive(0); // Might as well do something whilst waiting :)
        }
      }
      SPDR = _screen[xx];
      _screen[xx] = ' ';
      while (!(SPSR & (1<<SPIF)));     // Wait the end of the last SPI transmission is clear
    #else   
      Send(MAX7456ADD_DMAH, xx>>8);
      Send(MAX7456ADD_DMAL, xx);
      Send(MAX7456ADD_DMDI, _screen[xx]);
      _screen[xx] = ' ';
    #endif
  }
  #ifdef USE_VSYNC
    spi_transfer(DMDI_reg);
    spi_transfer(END_string);
    spi_transfer(DMM_reg);
    spi_transfer(B00000000);
  #endif
  digitalWrite(MAX7456SELECT,HIGH);
}


void MAX7456Class::Send(uint8_t add, uint8_t data)
{
  spi_transfer(add);
  spi_transfer(data);
}


void MAX7456Class::WriteNvm(uint8_t char_address)
{
  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  spi_transfer(VM0_reg); 
  //spi_transfer(DISABLE_display);

  //digitalWrite(MAX7456SELECT,LOW);
  //spi_transfer(VM0_reg);
  spi_transfer(Settings[S_VIDEOSIGNALTYPE]?0x40:0);

  spi_transfer(MAX7456ADD_CMAH); // set start address high
  spi_transfer(char_address);

  for(uint8_t x = 0; x < NVM_ram_size; x++) // write out 54 bytes of character to shadow ram
  {
    spi_transfer(MAX7456ADD_CMAL); // set start address low
    spi_transfer(x);
    spi_transfer(MAX7456ADD_CMDI);
    spi_transfer(fontData[x]);
  }

  // transfer 54 bytes from shadow ram to NVM
  spi_transfer(MAX7456ADD_CMM);
  spi_transfer(WRITE_nvr);
  
  // wait until bit 5 in the status register returns to 0 (12ms)
  while ((spi_transfer(MAX7456ADD_STAT) & STATUS_reg_nvr_busy) != 0x00);

 spi_transfer(VM0_reg); // turn on screen next vertical
  //spi_transfer(ENABLE_display_vert); 
 spi_transfer(Settings[S_VIDEOSIGNALTYPE]?0x4c:0x0c);
  digitalWrite(MAX7456SELECT,HIGH);  
}

void MAX7456Class::Stalldetect(void)
{
  uint8_t srdata;
  pinMode(MAX7456SELECT,OUTPUT);
  digitalWrite(MAX7456SELECT,LOW);  
  spi_transfer(0x80);
  srdata = spi_transfer(0xFF); 
  digitalWrite(MAX7456SELECT,HIGH);
  if ((B00001000 & srdata) == 0)
    Setup(); 
}

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
void MAX7456Class::DisplayFont()
{
  for(uint8_t x = 0; x < 255; x++) {
    screen[90+x] = x;
  }
}

void MAX7456Class::UpdateFont()
{ 
  for(uint8_t x = 0; x < 255; x++){
    for(uint8_t i = 0; i < 54; i++){
      fontData[i] = (uint8_t)pgm_read_byte(fontdata+(64*x)+i);
    }
    WriteNvm(x);
    ledstatus=!ledstatus;
    if (ledstatus==true){
      digitalWrite(LEDPIN,HIGH);
    }
    else{
      digitalWrite(LEDPIN,LOW);
    }
    delay(20); // Shouldn't be needed due to status reg wait.
  }
}

#endif
