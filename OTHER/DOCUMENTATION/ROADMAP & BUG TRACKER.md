---

## Known bugs: ##

Screen updates are slow when in PWM RSSI mode - resolved in master / for next release

Screen updates are slow for GPS data - resolved in master / for next release

GUI requires a restart after font upload - resolved in master / for next release


---

## Enhancements planned for next  release: ##

Support for either NAZA or APM - NAZA implemented, but subsequently removed due to lack of interest

TX switch ch in OSD menu - resolved in master / for next release

Improved PWM RSSI handling - resolved in master / for next release

Improved hardware current sensor handling - carried forward, however bugfix implemented for MW2.4

Improved speed performance for serial and PWM based data (AHI etc.) - resolved in master / for next release

Adjustable Horizon pitch/ roll limits - resolved in master / for next release

MAX chip hardware stall / recovery detect - resolved in master / for next release

Support for GPSOSD screen layout switching - resolved in master / for next release


## Enhancement requests for future releases: ##

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


Code quality:

 - Tidy up float calcs
 - Generally review and improve code quality
 - Memory improvements
 
---
 








