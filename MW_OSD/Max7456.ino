#include "bitarray.h"

#if defined WHITEBRIGHTNESS | defined BLACKBRIGHTNESS
  #ifndef WHITEBRIGHTNESS
    #define WHITEBRIGHTNESS 0x01
  #endif
  #ifndef BLACKBRIGHTNESS
    #define BLACKBRIGHTNESS 0x00
  #endif
  #define BWBRIGHTNESS ((BLACKBRIGHTNESS << 2) | WHITEBRIGHTNESS)
#endif

// video mode register 0 bits
#define VIDEO_BUFFER_DISABLE 0x01
//#define MAX7456_RESET 0x02
#define VERTICAL_SYNC_NEXT_VSYNC 0x04
#define OSD_ENABLE 0x08
#define SYNC_MODE_AUTO 0x00
#define SYNC_MODE_INTERNAL 0x30
#define SYNC_MODE_EXTERNAL 0x20
#define VIDEO_MODE_PAL 0x40
#define VIDEO_MODE_NTSC 0x00

// video mode register 1 bits


// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x40

//MAX7456 commands
#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff


#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0

// Selectable by video mode
//uint8_t ENABLE_display;
//uint8_t ENABLE_display_vert;
//uint8_t DISABLE_display;
uint16_t MAX_screen_size;

// Goods for tidiness
#define VIDEO_MODE (Settings[S_VIDEOSIGNALTYPE] ? VIDEO_MODE_PAL : VIDEO_MODE_NTSC)

uint8_t detectedCamType = 0;

//////////////////////////////////////////////////////////////
uint8_t spi_transfer(uint8_t data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
    ;
  return SPDR;                    // return the received byte
}

// ============================================================   WRITE TO SCREEN

#ifdef MAX_SOFTRESET
/*
 * Try to bring the MAX to known state.
 * Three consequtive 0xFF writes seems to take the MAX out of
 * any intermediate state (reg-data & auto increment mode).
 */
void MAX7456SoftReset(void)
{
  uint8_t rval;

  MAX7456ENABLE;

  spi_transfer(MAX7456ADD_STAT);
  rval = spi_transfer(0xFF);

  if (rval & 7)
    return;

  // The magic triplet
  spi_transfer(0xFF);
  spi_transfer(0xFF);
  spi_transfer(0xFF);

  spi_transfer(MAX7456ADD_STAT);
  rval = spi_transfer(0xFF);

  if ((rval & 7) == 0) {
    // Should alert...
    // Serial.println("\r\nFailed to synchronize with MAX");
    delay(1000);
  }

  // Issue software reset
  MAX7456_Send(MAX7456ADD_VM0, (1 << 1));
  MAX7456DISABLE;
}
#endif

void MAX7456Setup(void)
{
  uint8_t MAX7456_reset=0x0C;
  uint8_t MAX_screen_rows;

  MAX7456DISABLE

  MAX7456HWRESET
  
  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (4 meg)
  //SPI2X will double the rate (8 meg)

  SPCR = (1<<SPE)|(1<<MSTR);
  SPSR = (1<<SPI2X);
  uint8_t spi_junk;
  spi_junk=SPSR;
  spi_junk=SPDR;
  delay(10);

#ifdef MAX_SOFTRESET
  MAX7456SoftReset();
#endif

  MAX7456ENABLE

#ifdef AUTOCAM 
  uint8_t srdata;

  delay(1000/10); // Extra delay for input sync detection. 4 frames

  spi_transfer(MAX7456ADD_STAT);
  srdata = spi_transfer(0xFF); 
  srdata &= B00000011;
  if (srdata == B00000001){      // PAL
    Settings[S_VIDEOSIGNALTYPE]=1; 
    flags.signaltype = 1;
  }
  else if (srdata == B00000010){ // NTSC
    Settings[S_VIDEOSIGNALTYPE]=0;
    flags.signaltype = 0;
  }
  else{
    flags.signaltype = 2 + Settings[S_VIDEOSIGNALTYPE]; // NOT DETECTED    
  }
  detectedCamType = srdata;
#else
  flags.signaltype = Settings[S_VIDEOSIGNALTYPE];
#endif //AUTOCAM
  if(Settings[S_VIDEOSIGNALTYPE]) {   // PAL
    MAX7456_reset = 0x4C;
    MAX_screen_size = 480;
    MAX_screen_rows = 16;
  }
  else {                              // NTSC
    MAX_screen_size = 390;
    MAX_screen_rows=13;
  }

  // Set up the Max chip. Enable display + set standard.
  MAX7456_Send(MAX7456ADD_VM0, MAX7456_reset);
   
#ifdef FASTPIXEL // force fast pixel timing helps with ghosting for some cams
  MAX7456_Send(MAX7456ADD_OSDM, 0x00);
#endif

#ifdef BWBRIGHTNESS // change charactor black/white level brightess from default 
  uint8_t x;
  for(x = 0; x < MAX_screen_rows; x++) {
    MAX7456_Send(MAX7456ADD_RB0+x, BWBRIGHTNESS);
  }
#endif
  MAX7456DISABLE

# ifdef USE_VSYNC
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    EIMSK |= (1 << INT0);  // enable interuppt
    EICRA |= (1 << ISC01); // interrupt at the falling edge
  }
#endif
  readEEPROM_screenlayout();
}

// Copy string from ram into screen buffer

#ifdef INVERTED_CHAR_SUPPORT

// XXX Have to check which derives the smaller code:
// XXX (1) Prepare WriteString and WriteStringWithAttr
// XXX (2) Pass call to WriteString to WriteStringWithAttr

void MAX7456_WriteString(const char *string, int addr)
{
  MAX7456_WriteStringWithAttr(string, addr, 0);
}

void MAX7456_WriteStringWithAttr(const char *string, int addr, int attr)
#else
void MAX7456_WriteString(const char *string, int addr)
#endif
{
  char *screenp = &screen[addr];
  while (*string) {
    *screenp++ = *string++;
#ifdef INVERTED_CHAR_SUPPORT
    if (attr) {
      bitSET(screenAttr, addr);
    } else {
      bitCLR(screenAttr, addr);
    }
    addr++;
#endif
  }
}

// Copy string from progmem into the screen buffer
void MAX7456_WriteString_P(const char *string, int Adresse)
{
  char c;
  char *screenp = &screen[Adresse];
  while((c = (char)pgm_read_byte(string++)) != 0)
    *screenp++ = c;
}

#ifdef CANVAS_SUPPORT
void MAX7456_ClearScreen(void)
{
  for(uint16_t xx = 0; xx < MAX_screen_size; ++xx) {
    screen[xx] = ' ';
#ifdef INVERTED_CHAR_SUPPORT
    bitCLR(screenAttr, xx);
#endif
  }
}
#endif

#ifdef USE_VSYNC
volatile unsigned char vsync_wait = 0;

ISR(INT0_vect) {
    vsync_wait = 0;
}

void MAX7456_WaitVSYNC(void)
{
  uint32_t vsync_timer = 40 + millis();

  vsync_wait = 1;

  while (vsync_wait) {
    if (millis() > vsync_timer){
      vsync_wait=0;
    } else{
      serialMSPreceive(0); // Might as well do something whilst waiting :)
    }
  }
}
#endif

void MAX7456_DrawScreen()
{
  uint16_t xx;

  MAX7456ENABLE;

  MAX7456_Send(MAX7456ADD_DMAH, 0);
  MAX7456_Send(MAX7456ADD_DMAL, 0);
  MAX7456_Send(MAX7456ADD_DMM, 1);

#ifdef USE_VSYNC
  MAX7456_WaitVSYNC();
#endif

  for(xx = 0; xx < MAX_screen_size; ++xx) {
#ifdef USE_VSYNC
    // We don't actually need this?
    if (xx == 240)
        MAX7456_WaitVSYNC(); // Don't need this?
#endif

#ifdef INVERTED_CHAR_SUPPORT
    bool invActive = false;
    if (!invActive && bitISSET(screenAttr, xx)) {
      MAX7456_Send(MAX7456ADD_DMM, 1|(1<<3));
      invActive = true;
    } else if (invActive && bitISCLR(screenAttr, xx)) {
      MAX7456_Send(MAX7456ADD_DMM, 1);
      invActive = false;
    }
#endif

    MAX7456_Send(MAX7456ADD_DMDI, screen[xx]);

#ifdef CANVAS_SUPPORT
    if (!canvasMode) // Don't erase in canvas mode
#endif
    {
      screen[xx] = ' ';
    #ifdef INVERTED_CHAR_SUPPORT
      bitCLR(screenAttr, xx);
    #endif
    }
  }

  MAX7456_Send(MAX7456ADD_DMDI, END_string);
  MAX7456_Send(MAX7456ADD_DMM, 0);

  MAX7456DISABLE;
}

void MAX7456_Send(uint8_t add, uint8_t data)
{
  spi_transfer(add);
  spi_transfer(data);
}

//MAX7456 commands

#define WRITE_TO_MAX7456
#define NVM_ram_size 54

#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff
#define WRITE_nvr 0xa0
// with NTSC
#define ENABLE_display 0x08
#define ENABLE_display_vert 0x0c
//#define MAX7456_reset 0x02
#define DISABLE_display 0x00
#define STATUS_reg_nvr_busy 0x20

void write_NVM(uint8_t char_address)
{
#ifdef WRITE_TO_MAX7456
  // disable display
  MAX7456ENABLE
  MAX7456_Send(MAX7456ADD_VM0, VIDEO_MODE);

  MAX7456_Send(MAX7456ADD_CMAH, char_address); // set start address high

  for(uint8_t x = 0; x < NVM_ram_size; x++) // write out 54 bytes of character to shadow ram
  {
    MAX7456_Send(MAX7456ADD_CMAL, x); // set start address low
    MAX7456_Send(MAX7456ADD_CMDI, fontData[x]);
  }

  // transfer 54 bytes from shadow ram to NVM
  MAX7456_Send(MAX7456ADD_CMM, WRITE_nvr);
  
  // wait until bit 5 in the status register returns to 0 (12ms)
  while ((spi_transfer(MAX7456ADD_STAT) & STATUS_reg_nvr_busy) != 0x00);

  MAX7456_Send(MAX7456ADD_VM0, OSD_ENABLE|VERTICAL_SYNC_NEXT_VSYNC|VIDEO_MODE); // turn on screen next vertical
  MAX7456DISABLE  
  delay(20);
#else
  delay(12);
#endif
}

#if defined(AUTOCAM) || defined(MAXSTALLDETECT)
void MAX7456CheckStatus(void){
  uint8_t srdata;
  MAX7456ENABLE

#ifdef AUTOCAM
  spi_transfer(MAX7456ADD_STAT);
  srdata = spi_transfer(0xFF);
  srdata &= B00000011;
  if (detectedCamType != srdata) {
    MAX7456Setup();
    return;
  }
#endif

#ifdef MAXSTALLDETECT
  spi_transfer(0x80);
  srdata = spi_transfer(0xFF); 
  
  if ((B00001000 & srdata) == 0)
    MAX7456Setup(); 
#endif
}
#endif

#if defined LOADFONT_DEFAULT || defined LOADFONT_LARGE || defined LOADFONT_BOLD
void displayFont()
{
  for(uint8_t x = 0; x < 255; x++) {
    screen[90+x] = x;
  }
}

void updateFont()
{ 
  for(uint8_t x = 0; x < 255; x++){
    for(uint8_t i = 0; i < 54; i++){
      fontData[i] = (uint8_t)pgm_read_byte(fontdata+(64*x)+i);
    }
    write_NVM(x);
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
