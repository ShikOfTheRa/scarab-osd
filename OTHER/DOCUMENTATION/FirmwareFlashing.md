# How to install MW OSD firmware to your OSD hardware 

## Preparation 
* Download the [MW OSD software pack](https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/Downloads.md) of your choice.
* Download the [Arduino flavor](http://arduino.cc/en/Main/Software) that matches your system.
* Have an FTDI breakout board (which usually connects to a USB port) with the proper headers to connect to your OSD hardware.

## Testing Firmware communication flashing 
* Connect the OSD to the FTDI adapter, and that to the PC.
* Start Arduino IDE.
* Choose the appropriate board. For most MinimOSDs etc go to menu Tools > Board > Arduino Pro or Pro Mini 5V, 16MHz, w/ ATmega328.
* Select the proper serial port in menu Tools > Serial Port.
* Click on the "Upload" button and observe the serial console at the bottom of arduino. Success is noted as white text on black background stating just the binary sketch size. If messages in red letters appear, something went wrong.

## Firmware flashing 
* Connect the OSD to the FTDI adapter, and that to the PC.
* Start Arduino IDE.
* Select File > Open.  Browse to the MW_OSD folder and select 'MW_OSD.ino'
* A new Arduino IDE will appear. Close the other window as it it no longer needed.
* If you have not already done so, make sure to do the following:  From the MW_OSD Arduino window, choose the appropriate board. For most MinimOSDs etc go to menu Tools > Board > Arduino Pro or Pro Mini 5V, 16MHz, w/ ATmega328.  Select the proper serial port in menu Tools > Serial Port.
* Edit the config.h tab to select the correct OSD hardware, Flight controller and aircraft type.
* Click on the "Upload" button and observe the serial console at the bottom of arduino. Success is noted as white text on black background stating just the binary sketch size. If messages in red letters appear, something went wrong.

To test the setup, close Arduino and start MW OSD GUI. It should connect and work normally with the newly flashed OSD.

## Updating fonts 
* Connect to the OSD using the GUI.
* Select the font you require otherwise the default one will be used
* The font chosen will be seen in the simulator window
* Select upload from the GUI
* NOTE - check the FAQ - requires the OSD to be powered correctly or teh fonts will not save (upload looks OK)
