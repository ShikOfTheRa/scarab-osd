---



---

## Enhancement requests for future releases: ##

Fresnel warning indicator

Provide optimisation of serial data requests fro slower baud rates

Support for newer Cleanflight releases

Display option for climb rate value instead of vario slider

Ability to amend all PID settings from menu

support for cleanflight / baseflight loop time change

video volts alarm

pitch / roll angles with alarm

glidescope for fixedwing 

autodetect cell count

autoset warning voltage

3 OSD layouts for use with 3 way tx switch

Support for 8 GUI layouts selectable from OSD menu at field

MAX chip hardware stall / recovery detect

Fast start - Autodetect MAX ready or timeout

Autodetect PAL/NTSC or timeout to last known settings (fix blank screen for incorrect labelled cams)

Improve Standalone / Groundstation startup. Remove multiwii

Filtering for MSP sensor data

Flight path vector

Navigation mode layout

Power consumtion W/Hr 

Air speed sensor

Climb rate alarm

Descriptive chracter font for for heading / angle to home

Support for APM


Other changes

Impelement more descriptive "help" test within confih.h


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
 
## Fixed issues since last full release R1.3: ##

BUG! "Use MW" options not saving backup file (single entry key)....

BUG! When set the "Time Zone Offset" to say 2.0 and then do a "WRITE" the value will change (display) to 0.2

BUG! Compass OK, but home arrow is inconsistent in fixedwing mode

BUG! compile errors for GPS in soem Arduino versions

BUG! PID menu options after row 5 are incorrect. e.g. level does not show level settings

## Improvements since last full release R1.3: ##

Fixedwing - Heading / compass support for controllers without MAG. Default assumes no MAG.

Fixedwing - Altitude support for controllers without BARO. Default assumes no BARO.

Fixedwing - Vario support for controllers without BARO. Default assumes no BARO.

Horizon bar set behind other screen items as they have higher priority.

AUTOCAM now default option (Auto sense PAL / NTSC cam at startup) to help with incorrectly marked cams / user error. 

VSYNC now default option as most boards now support this. Clearer display.

Added max Amps to flight summary

Decimalise GUI value for min battery voltage etc.

GUI support for FC passthrough configuration settings (access OSD config via FC). 

Amended 00 font to be blank character - recommended to reduce sparklies



