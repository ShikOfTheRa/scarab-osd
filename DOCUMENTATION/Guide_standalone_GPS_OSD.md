# How to setup for standalone GPS OSD

See other guides for installing Arduino and uploading MWOSD:
https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FirmwareFlashing.md

# Step 1: prepare your config.h file

## Select the GPS chip type by removing the comments:
* //#define GPSOSD_UBLOX
* //#define GPSOSD_NMEA
* //#define GPSOSD_MTK
  
## Select the aircaraft type:
* //#define ROTORCRAFT
* //#define FIXEDWING

## Select baud rate to use with GPS:
Note - if changed from default 115200, the GUI baud rate will need to be changed. 
* //#define BAUDRATE 115200
* //#define BAUDRATE 57600
* //#define BAUDRATE 38400
* //#define BAUDRATE 19200
* //#define BAUDRATE 9600

## Optional Considerations:
Screeen layout switching options if RSSI is not used. Requires a spare RC channel 
* //#define OSD_SWITCH_RSSI
Full OSD control:
* //#define PPMOSDCONTROL             // Enables full OSD menu, screen switching, RSSI, Throttle fature, virtual current sensor, etc using a PPM signal into OSD RSSI pin 
Disable Armed status (automatically dissappears at launch in GPS mode)
//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status


# Step 2: Upload 
https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FirmwareFlashing.md

# Step 3: Upload fontfile using GUI

# Step 4: Configure OSD using GUI
