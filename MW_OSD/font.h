#ifndef __FONT_H
#define __FONT_H

class FontClass
{
  public:
    FontClass();
    void enterFontMode();
    void fontCharacterReceived(uint8_t cindex);
    int16_t getNextCharToRequest();
    uint8_t inFontMode();
  private:
    uint8_t _inFontMode;
};

static FontClass Font;

#endif
