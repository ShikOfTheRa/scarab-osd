---

## Known bugs: ##

HARIKIRI / MW2.1 not working (R1.5)- resolution available from master repository

Center crosshair font partially missing on default_large font - resolution available from master repository

---

## Enhancements planned for next  release: ##

Done - video volts alarm

Done - Display anal / PWM sensors on GUI - via MW OSD protocol

Done - Support for PWMRSSI via RC data (Flight Controller types) or PPM stream (GPS OSD) - PENDING TESTING

Done - GPSOSD - PPM support - PENDING TESTING

Done - enable Filtering and calibration for MSP sensor data (RSSI)

Done - enable Filtering for MSP sensor data (Main battery via FC)

Done - Air speed sensor. - PENDING TESTING

Done - autodetect cell count option 

Done - autoset battery warning based upon cell count (for those who swap batteries) 

Done - long distance display for fixed wing - e.g. 15.3km instaed of 1530m 

Fast AHI update to VSYNC rate of 25/30 hz

Temperature sensor improvements

Improved hardware current sensor handling - carried forward, however bugfix implemented for MW2.4

Power consumtion W/Hr 

Fast start - Autodetect MAX ready or timeout

RSSI calibration from GUI


## Enhancement requests for future releases: ##

Migrate controller/aircraft choice to GUI

Support for APM

Support for OP

Additional AHI layouts - multiple pitch angles

Migrate Current Sensor to GUI / potentially migrate to offset / mv/A calc 

Config.h option to enable / disable dist/al/volt alarms etc.

Improved fastpwm and rssi calibration

Fresnel warning indicator

Display option for climb rate value instead of vario slider

Ability to amend all PID settings from menu

support for cleanflight / baseflight loop time change

pitch / roll angles alarm

Support for 8 GUI layouts selectable from OSD menu at field

Improve Standalone / Groundstation startup. Remove multiwii

Flight path vector

Navigation mode layout

Climb rate alarm

Descriptive chracter font for for heading / angle to home


Other changes

Impelement more descriptive "help" text within confih.h


## GUI: ##

BATTERY calibration button

Migrate GUI to chrome

Improve ARMED/DIARMED message

Improve icon buttons

Improved heading / pitch& roll graphics 

Ability to drag/drop items to layout editor

List to enable/disable items on display instea dof clicking to find item


Code quality:

 - Tidy up float calcs
 - Generally review and improve code quality
 - Memory improvements
 
---
 








