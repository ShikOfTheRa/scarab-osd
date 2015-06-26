---

## Known bugs: ##

Screen updates are slow when in PWM RSSI mode - resolved in master / for next release

Screen updates are slow for GPS data - work in progress

GUI requires a restart after font upload


---

## Enhancements planned for next  release: ##

Support for either NAZA or APM

TX switch ch in OSD menu

Improved PWM RSSI handling

Improved hardware current sensor handling

Improved speed performance for serial and PWM based data (AHI etc.)


## Enhancement requests for future releases: ##

Support for either NAZA or APM

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

pitch / roll angles with alarm

Support for 8 GUI layouts selectable from OSD menu at field

MAX chip hardware stall / recovery detect

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


Code quality:

 - Tidy up float calcs
 - Generally review and improve code quality
 - Memory improvements
 
---
 








