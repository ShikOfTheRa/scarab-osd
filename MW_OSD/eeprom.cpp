#include "platform.h"

EepromClass::EepromClass(EEPROMClass &EEPROM) {
  _EEPROM = &EEPROM;
}

uint8_t EepromClass::Read(int idx) {
  return _EEPROM->read(idx);
}

void EepromClass::Write(int idx, uint8_t val) {
  _EEPROM->write(idx, val);
}

void EepromClass::WriteSettings(void) // OSD will only change 8 bit values. GUI changes directly
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
    _EEPROM->write(en,Settings[en]);
  } 
  for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
    uint16_t pos=EEPROM_SETTINGS+(en*2);
    _EEPROM->write(pos,Settings16[en]&0xFF);
    _EEPROM->write(pos+1,Settings16[en]>>8);
  } 
  _EEPROM->write(0,MWOSDVER);
}


void EepromClass::ReadSettings(void)
{
  for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
     Settings[en] = _EEPROM->read(en);
  }
  #ifdef AUTOCELL
  Settings[S_BATCELLS]=1;
  #endif

  for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
     uint16_t pos=(en*2)+EEPROM_SETTINGS;
     Settings16[en] = _EEPROM->read(pos);
     uint16_t xx = _EEPROM->read(pos+1);
     Settings16[en] = Settings16[en]+(xx<<8);
  }

  Screen.ReadLayout();
}

void EepromClass::CheckSettings(void)
{
  uint8_t EEPROM_Loaded = _EEPROM->read(0);
  if (EEPROM_Loaded!=MWOSDVER){
    for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
      _EEPROM->write(en,pgm_read_byte_near(EEPROM_DEFAULT + en));
    }
    for(uint8_t en=0;en<EEPROM16_SETTINGS;en++){
      uint16_t pos = EEPROM_SETTINGS+(en*2);
      uint16_t val = pgm_read_word_near(EEPROM16_DEFAULT + en);
      _EEPROM->write(pos,val&0xFF);
      _EEPROM->write(pos+1,val>>8);
    }
    for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
      uint16_t screen_layout_default_val = pgm_read_word_near(SCREENLAYOUT_DEFAULT + en);
      uint16_t screen_layout_default_osdsw_val = pgm_read_word_near(SCREENLAYOUT_DEFAULT_OSDSW + en);
      // why is this val assigned 2 places? (place 1)
      _EEPROM->write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(en*2),screen_layout_default_val&0xFF);
      _EEPROM->write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+1+(en*2),screen_layout_default_val>>8);
      _EEPROM->write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*2)+(en*2),screen_layout_default_osdsw_val&0xFF);
      _EEPROM->write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*2)+1+(en*2),screen_layout_default_osdsw_val>>8);
      // why is this val assigned 2 places? (place 2)
      _EEPROM->write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*4)+(en*2),screen_layout_default_val&0xFF);
      _EEPROM->write(EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(POSITIONS_SETTINGS*4)+1+(en*2),screen_layout_default_val>>8);
    }
/*
    for(uint8_t osd_switch_pos=0;osd_switch_pos<3;osd_switch_pos++){
      for(uint8_t en=0;en<POSITIONS_SETTINGS;en++){
        _EEPROM->write(EEPROM_SETTINGS+(POSITIONS_SETTINGS*osd_switch_pos)+(en*2),SCREENLAYOUT_DEFAULT_OSDSW[en]&0xFF);
        _EEPROM->write(EEPROM_SETTINGS+(POSITIONS_SETTINGS*osd_switch_pos)+1+(en*2),SCREENLAYOUT_DEFAULT_OSDSW[en]>>8);
      }
    }    
*/
  }
}

void EepromClass::ClearSettings()
{
  for (int i = 0; i < 512; i++) {
    _EEPROM->write(i, 0);
  }
}
