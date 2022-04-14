
// Display information when screen element is turned on 
void displayCustom(void) /// Put your custome code here. Example code below
{
  uint16_t position = getPosition(Customposition);
  if (!fieldIsVisible(Customposition))
    return; 
  MAX7456_WriteString("CUSTOM", position); // Display text example
  // MAX7456_WriteString(itoa(MwRcData[2], screenBuffer, 10), position);  // Display number example
}

/*
///////////////////////////////////////////////////////////

// Other example options:


// Display information when screen element is turned on 
void displayCustom(void) /// Put your custome code here. Example code below
{
  uint16_t position = getPosition(Customposition);
  if (!fieldIsVisible(Customposition))
    return;
  MAX7456_WriteString("CUSTOM", position); // Display text example
  // MAX7456_WriteString(itoa(MwRcData[2], screenBuffer, 10), position);  // Display number example
}



// Display information when screen element is turned on and different info for each screen layout
void displayCustom(void) /// Put your custome code here. Example code below
{
   uint16_t position = getPosition(Customposition);
   if (!fieldIsVisible(Customposition))
     return;

   switch (screenlayout) {
     case 0:
       MAX7456_WriteString("CUST 0", position); // Display text custom screen 0
       // MAX7456_WriteString(itoa(MwRcData[0], screenBuffer, 10),position);  // Alternative display number
       break;
     case 1:
       MAX7456_WriteString("CUSTOM1", position); // Display text custom screen 1
       break;
     case 2:
       MAX7456_WriteString("CUSTOM2", position); // Display text custom screen 2
       break;
   }
}



// Display information when GUI custom settings value is set to a value higher than 11
void displayCustom(void) /// Put your custome code here. Example code below
{
  uint16_t position = getPosition(Customposition);
  if (Settings[S_CUSTOM] > 11)
    return;
  MAX7456_WriteString("CUSTOM", position); // Display text
  // MAX7456_WriteString(itoa(MwRcData[2], screenBuffer, 10), position);  // Display number
}


///////////////////////////////////////////////////////////
*/
