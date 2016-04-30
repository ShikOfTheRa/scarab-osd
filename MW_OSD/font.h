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
    uint8_t fontStatus=0;
  private:
    uint8_t _inFontMode;
    uint8_t retransmitQueue;
};

extern FontClass Font;

#endif
