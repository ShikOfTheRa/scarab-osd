Known BUGS

BUG! "Use MW" options not saving backup file (single entry key)....
BUG! When set the "Time Zone Offset" to say 2.0 and then do a "WRITE" the value will change (display) to 0.2
BUG! Compass OK, but home arrow is inconsistent in fixedwing mode.
BUG! compile errors for GPS in soem Arduino versions

Enhancement requests for future releases:
GUI support for FC passthrough configuration settings (access OSD config via FC) 
add max distance to summary
add max altitude to summary
support for altitude reset at arm
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
Fixedwing - support for naze32 no mag fixedwing - use GPS heading for compass
Fixedwing - use GPS data for ladders / vario
Support for APM

Other changes
Remove references to MultiWii (as no longer exclusively MultiWii)
Impelement more desciptive "help" test within confih.h

GUI:
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
