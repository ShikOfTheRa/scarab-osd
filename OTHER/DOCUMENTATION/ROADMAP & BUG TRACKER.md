---

## Known bugs: ##

HARIKIRI / MW2.1 not working (R1.5)- resolution available from master repository

---

## Enhancements planned for next  release: ##

Improved hardware current sensor handling - carried forward, however bugfix implemented for MW2.4


## Enhancement requests for future releases: ##

Support for PWMRSSI via RC data (Flight Controller types) or PPM stream (GPS OSD)

Support for NAZA

Support for APM

Support for OP

GPSOSD - PPM support

Additional AHI layouts - multiple pitch angles

Migrate Current Sensor to GUI / potentially migrate to offset / mv/A calc 

Config.h option to enable / disable dist/al/volt alarms etc.

Improved fastpwm and rssi calibration

video volts alarm

autodetect cell count

autoset warning voltage

Fresnel warning indicator

Display option for climb rate value instead of vario slider

Ability to amend all PID settings from menu

support for cleanflight / baseflight loop time change

pitch / roll angles alarm

Support for 8 GUI layouts selectable from OSD menu at field

Fast start - Autodetect MAX ready or timeout

Improve Standalone / Groundstation startup. Remove multiwii

Filtering for MSP sensor data

Flight path vector

Navigation mode layout

Power consumtion W/Hr 

Air speed sensor

Climb rate alarm

Descriptive chracter font for for heading / angle to home


Other changes

Impelement more descriptive "help" text within confih.h


## GUI: ##

Display anal / PWM sensors on GUI - via MW OSD protocol

RSSI calibration button

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
 








