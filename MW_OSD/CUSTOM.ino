void displayCustom(void) /// Put your custome code here. Example code below
{
  uint16_t position = getPosition(Customposition);
  if (!fieldIsVisible(Customposition))
    return;

  /* alternative
  if (Settings[S_CUSTOM] > 11)
    return;
  */  

  MAX7456_WriteString("COCK", position); // Display text
  // MAX7456_WriteString(itoa(MwRcData[2], screenBuffer, 10), position);  // Display number
}
