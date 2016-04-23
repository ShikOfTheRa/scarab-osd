#include "platform.h"

FontClass::FontClass() {
  // start not in font mode
  _inFontMode = 0;
}

// Font upload queue implementation.
// Implement a window for curr + the previous 6 requests.
// First-chance to retransmit at curr-3 (retransmitQueue & 0x10)
// First-chance retransmit marked as used at retransmitQueue |= 0x01
// 2 to N-chance retransmit marked at curr-6 (retransmitQueue & 0x02)
void FontClass::enterFontMode() {
  if(armed || configMode || _inFontMode) 
    return;
  // queue first char for transmition.
  retransmitQueue = 0x80;
  _inFontMode = 1;
//  setMspRequests();
}


void FontClass::fontCharacterReceived(uint8_t cindex) {
  if(!_inFontMode)
    return;

  uint8_t bit = (0x80 >> (nextCharToRequest-cindex));

  // Just received a char..
  if(retransmitQueue & bit) {
    // this char war requested and now received for the first time
    retransmitQueue &= ~bit;  // mark as already received
    MAX7456.WriteNvm(cindex);       // Write to MVRam
  }
}

int16_t FontClass::getNextCharToRequest() {
  if(nextCharToRequest != lastCharToRequest) { // Not at last char
    if(retransmitQueue & 0x02) {                // Missed char at curr-6. Need retransmit!
      return nextCharToRequest-6;
    }

    if((retransmitQueue & 0x11) == 0x10) {      // Missed char at curr-3. First chance retransmit
      retransmitQueue |= 0x01;                  // Mark first chance as used
      return nextCharToRequest-3;
    }

    retransmitQueue = (retransmitQueue >> 1) | 0x80; // Add next to queue
    return nextCharToRequest++;                      // Notice post-increment!
  }

  uint8_t temp1 = retransmitQueue & ~0x01; 
  uint8_t temp2 = nextCharToRequest - 6; 

  if(temp1 == 0) {
    _inFontMode = 0;                            // Exit font mode
//  setMspRequests();
    return -1;
  }

  // Already at last char... check for missed characters.
  while(!(temp1 & 0x03)) {
    temp1 >>= 1;
    temp2++;
  }

  return temp2;
}

uint8_t FontClass::inFontMode() {
  return _inFontMode;
}
