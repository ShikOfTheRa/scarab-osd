---

## Outstanding bugs: ##

---

## Bugfixes / Enhancements completed for next  release: ##

 * OSD    - Video volts alarm via config.h
 * OSD    - Display analogue / PWM sensors on GUI - via MW OSD protocol
 * OSD    - Support for PWMRSSI via RC data (Flight Controller types)
 * OSD    - Support for PWMRSSI via PPM stream (GPS OSD) - PENDING TESTING
 * OSD    - Support for PPM menu control via PPM for GSOSD - PENDING TESTING
 * OSD    - Enabled filtering and calibration for RSSI data from Flight Controller
 * OSD    - Enabled filtering and calibration for Main battery voltage from Flight Controller
 * OSD    - Support for Air speed sensor
 * OSD    - Added autodetect cell count option 
 * OSD    - Added autoset battery warning based upon cell count (for those who swap batteries) 
 * OSD    - Added long distance display for fixed wing - e.g. 15.3km instead of 1530m 
 * OSD    - Added Faster support options for VSYNC rate of 25/30 hz
 * OSD    - Added support for forced crosshair for boards without accelerometer
 * OSD    - Added support for Mode 1 TX users
 * GUI    - Support for RSSI Auto calibration from GUI
 * BUGFIX - HARIKIRI / MW2.1 not working since R1.5
 * BUGFIX - Center crosshair font partially missing on default_large font
 * BUGFIX - AHI incroect at large angles
 * BUGFIX - Analogue ports had RSSI pulled high - impacting range and accuracy of values 
 * COMPAT - Compatability support for latest cleanflight - conflict with MultiWii
 * FONT   - improved large font from Schnupperm enhanced by Leo

---

## Bugfixes / Enhancements planned for next  release: ##

WIP - Watts display

WIP - Temperature sensor improvements

Improved hardware current sensor handling - carried forward, however bugfix implemented for MW2.4

Power consumtion W/Hr and W/Km 

Fast start - Autodetect MAX ready or timeout

RSSI - increase resolution

---

## Enhancement requests for future releases: ##

Migrate controller/aircraft choice to GUI

Support for APM

Support for TauLabs

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

Impelement more descriptive "help" text within confih.h

Migrate GUI to chrome

Improve ARMED/DIARMED message

Improve icon buttons

Improved heading / pitch& roll graphics 

Ability to drag/drop items to layout editor

List to enable/disable items on display instead of clicking to find item

Move AHI/crosshair with camera gimbal RC ch


Code quality:

 - Tidy up float calcs
 - Generally review and improve code quality
 - Memory improvements
 
---
 








