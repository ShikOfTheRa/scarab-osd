//------------------------------------------------------------------------
//FONT management 

uint8_t safeMode() {
  return 1;  // XXX
}


// Font upload queue implementation.
// Implement a window for curr + the previous 6 requests.
// First-chance to retransmit at curr-3 (retransmitQueue & 0x10)
// First-chance retransmit marked as used at retransmitQueue |= 0x01
// 2 to N-chance retransmit marked at curr-6 (retransmitQueue & 0x02)
void initFontMode(){
  if(armed || configMode || fontMode|| !safeMode()) {
    return;}
  retransmitQueue = 0x80;  // queue first char for transmition.
  fontMode = 1;
}


void fontCharacterReceived(uint8_t cindex){
  if(!fontMode){
    return;}
  uint8_t bit = (0x80 >> (nextCharToRequest-cindex));
  // Just received a char..
  if(retransmitQueue & bit) { // this char war requested and now received for the first time
    retransmitQueue &= ~bit;  // mark as already received
    write_NVM(cindex);}        // Write to MVRam
}

int16_t getNextCharToRequest(){
  if(nextCharToRequest != lastCharToRequest){ // Not at last char
    if(retransmitQueue & 0x02){                // Missed char at curr-6. Need retransmit!
      return nextCharToRequest-6;}
    if((retransmitQueue & 0x11) == 0x10){      // Missed char at curr-3. First chance retransmit
      retransmitQueue |= 0x01;                  // Mark first chance as used
      return nextCharToRequest-3;}
    retransmitQueue = (retransmitQueue >> 1) | 0x80; // Add next to queue
    return nextCharToRequest++;                      // Notice post-increment!
  }
  uint8_t temp1 = retransmitQueue & ~0x01; 
  uint8_t temp2 = nextCharToRequest - 6; 
  if(temp1 == 0) {
    fontMode = 0;                            // Exit font mode
    return -1;}
  while(!(temp1 & 0x03)) {   // Already at last char... check for missed characters.
    temp1 >>= 1;
    temp2++;}
  return temp2;
}
