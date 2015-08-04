// Font updater utility for MINIM based OSD's
// This Utility will overwrite any existing sketch on the arduino 
// It will not overwrite any EEPROM settings


// Select the font you wish to install
#define STANDARD
//#define LARGE

// Once sketch uploaded:
// Power cycle the OSD. 
// After approx 30-60 seconds the LED will stop flashing.
// The display will dvise update comple and display the font map.
// Re-install the OSD sketch.

//--------------------------------------------------------------
//
#define LEDPIN        7
#define MAX7456SELECT 6         // ss
#define MAX7456RESET  10        // RESET
#define DECIMAL '.'
#define POS_MASK        0x01FF
#define PAL_MASK        0x0003
#define PAL_SHFT             9
#define DISPLAY_MASK    0xC000
#define DISPLAY_ALWAYS  0xC000
#define DISPLAY_NEVER   0x0000
#define DISPLAY_COND    0x4000
#define DISPLAY_MIN_OFF     0x8000
#define POS(pos, pal_off, disp)  (((pos)&POS_MASK)|((pal_off)<<PAL_SHFT)|(disp))
#if defined SHIFTDOWN
#define TOPSHIFT        LINE
#else
#define TOPSHIFT        0
#endif
#define S_VIDEOSIGNALTYPE 0
#define DATAOUT 11              // MOSI
#define DATAIN  12              // MISO
#define SPICLOCK  13            // sck
#define VSYNC 2                 // INT0
#ifndef WHITEBRIGHTNESS
  #define WHITEBRIGHTNESS 0x01
#endif
#ifndef BLACKBRIGHTNESS
  #define BLACKBRIGHTNESS 0x00
#endif
#define BWBRIGHTNESS ((BLACKBRIGHTNESS << 2) | WHITEBRIGHTNESS)
#define DMM_reg   0x04
#define DMAH_reg  0x05
#define DMAL_reg  0x06
#define DMDI_reg  0x07
#define VM0_reg   0x00
#define VM1_reg   0x01
#define VIDEO_BUFFER_DISABLE 0x01
#define VERTICAL_SYNC_NEXT_VSYNC 0x04
#define OSD_ENABLE 0x08
#define SYNC_MODE_AUTO 0x00
#define SYNC_MODE_INTERNAL 0x30
#define SYNC_MODE_EXTERNAL 0x20
#define VIDEO_MODE_PAL 0x40
#define VIDEO_MODE_NTSC 0x00
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07
#define BACKGROUND_MODE_GRAY 0x40
#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff
#define MAX7456ADD_VM0          0x00  
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

uint16_t MAX_screen_size;
boolean ledstatus=HIGH;
uint8_t fontData[54];
uint8_t Settings[1];
const char message0[] PROGMEM = "DO NOT POWER OFF";
const char message1[] PROGMEM = "SCREEN WILL GO BLANK";
const char message2[] PROGMEM = "UPDATE COMPLETE";


//------------------------------------------------------------------------
#include <avr/pgmspace.h>
#if defined LARGE
  #include "fontL.h"
#else
  #include "fontD.h"
#endif
char screen[480];      // Main screen ram for MAX7456
char screenBuffer[20]; 
uint8_t fontStatus=0;


//------------------------------------------------------------------------
void setup()
{
  #define STARTUPDELAY 1000
  delay(STARTUPDELAY);
  MAX7456Setup(); 
  pinMode(LEDPIN,OUTPUT);
  digitalWrite(LEDPIN,HIGH);
//  for(uint8_t x = 0; x < 480; x++) {
//    screen[x] = 0X3F;
//  }
}



//------------------------------------------------------------------------
void loop()
{
  switch(fontStatus) {
    case 0:
      MAX7456_WriteString_P(message0, 32);
      MAX7456_DrawScreen();
      delay(3000);
      displayFont();  
      MAX7456_WriteString_P(message1, 32);
      MAX7456_DrawScreen();
      fontStatus++;
      delay(3000);      
      break;
    case 1:
      updateFont();
      MAX7456Setup(); 
      MAX7456_WriteString_P(message2, 32);
      displayFont();  
      MAX7456_DrawScreen();
      fontStatus++;
      break;
  }
  digitalWrite(LEDPIN,LOW);
}


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


//////////////////////////////////////////////////////////////
uint8_t spi_transfer(uint8_t data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
    ;
  return SPDR;                    // return the received byte
}

// ============================================================   WRITE TO SCREEN

void MAX7456Setup(void)
{
  uint8_t MAX7456_reset=0x02;
  uint8_t MAX_screen_rows;

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
  MAX7456_Send(VM0_reg, MAX7456_reset);
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
  MAX7456_Send(MAX7456ADD_OSDM, 0x00);
  // MAX7456_Send(MAX7456ADD_OSDM, 0xEC);
  // uint8_t srdata = spi_transfer(0xFF); //get data byte
  // srdata = srdata & 0xEF;
  // MAX7456_Send(0x6c, srdata);
  delay(100);
#endif

  if(Settings[S_VIDEOSIGNALTYPE]) {   // PAL
    MAX7456_reset = 0x42;
    MAX_screen_size = 480;
    MAX_screen_rows = 16;
  }
  else {                              // NTSC
    MAX7456_reset = 0x02;
    MAX_screen_size = 390;
    MAX_screen_rows = 13;
  }

  // set all rows to same charactor black/white level
  uint8_t x;


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

// Copy string from ram into screen buffer
void MAX7456_WriteString(const char *string, int Adresse)
{
  uint8_t xx;
  for(xx=0;string[xx]!=0;)
  {
    screen[Adresse++] = string[xx++];
  }
}

// Copy string from progmem into the screen buffer
void MAX7456_WriteString_P(const char *string, int Adresse)
{
  uint8_t xx = 0;
  char c;
  while((c = (char)pgm_read_byte(&string[xx++])) != 0)
  {
    screen[Adresse++] = c;
  }
}

#ifdef USE_VSYNC
  volatile unsigned char vsync_wait = 0;
  ISR(INT0_vect) {
    vsync_wait = 0;
  }
#endif

void MAX7456_DrawScreen()
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

  for(xx=0;xx<MAX_screen_size;++xx){
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
//          serialMSPreceive(0); // Might as well do something whilst waiting :)
        }
      }
      SPDR = screen[xx];
      screen[xx] = ' ';
      while (!(SPSR & (1<<SPIF)));     // Wait the end of the last SPI transmission is clear
    #else   
      MAX7456_Send(MAX7456ADD_DMAH, xx>>8);
      MAX7456_Send(MAX7456ADD_DMAL, xx);
      MAX7456_Send(MAX7456ADD_DMDI, screen[xx]);
      screen[xx] = ' ';
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
#define MAX7456_reset 0x02
#define DISABLE_display 0x00
#define STATUS_reg_nvr_busy 0x20

void write_NVM(uint8_t char_address)
{
#ifdef WRITE_TO_MAX7456
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
#else
  delay(12);
#endif
}


char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  } 

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *FormatGPSCoord(int32_t val, char *str, uint8_t p, char pos, char neg) {
  if(val < 0) {
    pos = neg;
    val = -val;
  }

  uint8_t bytes = p+6;
  val = val / 100;
 
  str[bytes] = 0;
  str[--bytes] = pos;
  for(;;) {
    if(bytes == p) {
      str[--bytes] = DECIMAL;
      continue;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (bytes < 3 && val == 0))
      break;   }

   while(bytes != 0)
     str[--bytes] = ' ';

   return str;
}

// Take time in Seconds and format it as 'MM:SS'
// Alternately Take time in Minutes and format it as 'HH:MM'
// If hhmmss is 1, display as HH:MM:SS
char *formatTime(uint32_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}

uint8_t FindNull(void)
{
  uint8_t xx;
  for(xx=0;screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}

 


