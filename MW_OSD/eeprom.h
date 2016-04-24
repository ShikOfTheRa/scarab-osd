#ifndef __EEPROM_H
#define __EEPROM_H

#include <EEPROM.h>

class EepromClass
{
  public:
    EepromClass(EEPROMClass &EEPROM);
    void write(void);
    void read(void);
    void check(void);
    void clear(void);
    EEPROMClass* getEEPROM();
  private:
    EEPROMClass* _EEPROM;
};

extern EepromClass Eeprom;

#endif
