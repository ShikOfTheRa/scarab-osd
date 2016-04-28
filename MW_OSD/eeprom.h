#ifndef __EEPROM_H
#define __EEPROM_H

#include <EEPROM.h>

class EepromClass
{
  public:
    EepromClass(EEPROMClass &EEPROM);

    uint8_t Read(int idx);
    void Write(int idx, uint8_t val);

    void WriteSettings(void);
    void ReadSettings(void);
    void CheckSettings(void);
    void ClearSettings(void);
    EEPROMClass* getEEPROM();

    // TODO: encapsulation
    uint8_t  Settings[EEPROM_SETTINGS];
    uint16_t Settings16[EEPROM16_SETTINGS];
  private:
    EEPROMClass* _EEPROM;
};

extern EepromClass Eeprom;

#endif
