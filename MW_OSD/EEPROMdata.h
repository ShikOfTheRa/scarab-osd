void writeEEPROM(void) // OSD will only change 8 bit values. GUI changes directly
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
    EEPROM.write(en,Settings[en]);
  } 
  for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
    uint16_t pos=EEPROM_SETTINGS+(en*2);
    EEPROM.write(pos,Settings16[en]&0xFF);
    EEPROM.write(pos+1,Settings16[en]>>8);
  } 
  EEPROM.write(0,MWOSDVER);
}


void readEEPROM_screenlayout(void)
{

  uint16_t EEPROMscreenoffset=EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(screenlayout*POSITIONS_SETTINGS*2);
  for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
    uint16_t pos=(en*2)+EEPROMscreenoffset;
    screenPosition[en] = EEPROM.read(pos);
    uint16_t xx=(uint16_t)EEPROM.read(pos+1)<<8;
    screenPosition[en] = screenPosition[en] + xx;

    if(Settings[S_VIDEOSIGNALTYPE]){
      uint16_t x = screenPosition[en]&0x1FF; 
      if (x>LINE06) screenPosition[en] = screenPosition[en] + LINE;
      if (x>LINE09) screenPosition[en] = screenPosition[en] + LINE;
    }
#ifdef SHIFTDOWN
    if ((screenPosition[en]&0x1FF)<LINE04) screenPosition[en] = screenPosition[en] + LINE;
#endif
  }
}

void readEEPROM(void)
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
     Settings[en] = EEPROM.read(en);
  }
  #ifdef AUTOCELL
  Settings[S_BATCELLS]=1;
  #endif

  for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
     uint16_t pos=(en*2)+EEPROM_SETTINGS;
     Settings16[en] = EEPROM.read(pos);
     uint16_t xx = EEPROM.read(pos+1);
     Settings16[en] = Settings16[en]+(xx<<8);
  }

  readEEPROM_screenlayout();
}

void checkEEPROM(void)
{
  uint8_t EEPROM_Loaded = EEPROM.read(0);
  if (EEPROM_Loaded!=MWOSDVER){
    for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
      EEPROM.write(en,EEPROM_DEFAULT[en]);
    }
    for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
      uint16_t pos=EEPROM_SETTINGS+(en*2);
      EEPROM.write(pos,EEPROM16_DEFAULT[en]&0xFF);
      EEPROM.write(pos+1,EEPROM16_DEFAULT[en]>>8);
    }
    for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
      EEPROM.write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(en*2),SCREENLAYOUT_DEFAULT[en]&0xFF);
      EEPROM.write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+1+(en*2),SCREENLAYOUT_DEFAULT[en]>>8);
      EEPROM.write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*2)+(en*2),SCREENLAYOUT_DEFAULT_OSDSW[en]&0xFF);
      EEPROM.write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*2)+1+(en*2),SCREENLAYOUT_DEFAULT_OSDSW[en]>>8);
      EEPROM.write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*4)+(en*2),SCREENLAYOUT_DEFAULT[en]&0xFF);
      EEPROM.write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*4)+1+(en*2),SCREENLAYOUT_DEFAULT[en]>>8);
    }
  }
}


void EEPROM_clear(){
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);
}
