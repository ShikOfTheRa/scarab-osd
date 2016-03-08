# How to setup for NAZA OSD

To connect hardware, it can be carried out like this:
http://www.efly.co.il/forums/attachment.php?attachmentid=113999&d=1412342606

See other guides for installing Arduino and uploading MWOSD:
https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FirmwareFlashing.md

# Step 1: prepare your config.h file

## Select the GPS chip type by removing the comments:
* //#define NAZA

## Select the aircaraft type:
* //#define ROTORCRAFT
* //#define FIXEDWING

## Optional Considerations:

/********************       NAZA Settings         ************************/
//#define NAZAMODECONTROL           // Enables NAZA mode control display using a PWM signal into OSD RSSI pin. Can be used with OSD_SWITCH_RSSI   

//#define HIDEARMEDSTATUS           // Enable to hide ARMED / DISARMED status

# Step 2: Upload 
https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FirmwareFlashing.md

# Step 3: Upload fontfile using GUI

# Step 4: Configure OSD using GUI
