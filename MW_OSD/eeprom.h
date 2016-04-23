#ifndef __EEPROM_H
#define __EEPROM_H

#include <EEPROM.h>

class EepromClass
{
  public:
    EepromClass(EEPROMClass &EEPROM);
    void write(void);
    void read(void);
    void read_screenlayout(void);
    void check(void);
    void clear(void);
  private:
    EEPROMClass* _EEPROM;
};

static EepromClass Eeprom(EEPROM);

#endif
