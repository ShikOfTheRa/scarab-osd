---

## Outstanding bugs: ##

---

## Bugfixes / Enhancements completed for next release (1.6): ##

 * OSD    - Support for TUALABS - thanks to Dustin
 * OSD    - Support for NAZA
 * OSD    - Support for BETAFLIGHT
 * OSD    - Support for BETAFLIGHT / CLEANFLIGHT / BASEFLIGHT FC profiles selection
 * OSD    - Support for BETAFLIGHT / CLEANFLIGHT / BASEFLIGHT FC pid controller selection
 * OSD    - BETAFLIGHT/CLEANFLIGHT AIRMODE display indicator
 * OSD    - BETAFLIGHT ACRO PLUS mode display indicator
 * OSD    - Support for CLEANFLIGHT loop time selection
 * OSD    - Video volts alarm adjustable via GUI
 * OSD    - Video volts independant of main battery
 * OSD    - Battery voltage status icon improved accuracy - thanks to Hwurzburg
 * OSD    - Battery capacity status icon added - thanks to Hwurzburg
 * OSD    - Display analogue / PWM sensors on GUI - via MW OSD protocol
 * OSD    - Support for PWMRSSI via RC data (Flight Controller types)
 * OSD    - Support for PWMRSSI via PPM stream (GPS OSD) - PENDING TESTING
 * OSD    - Support for PPM menu control via PPM for GSOSD - PENDING TESTING
 * OSD    - Support for using RSSI pin as PWM throttle for throttle % / virtual current GPSOSD/standalone/NAZA
 * OSD    - Enabled filtering and calibration for RSSI data from Flight Controller
 * OSD    - Enabled filtering and calibration for Main battery voltage from Flight Controller
 * OSD    - Support for Air speed sensor
 * OSD    - Added autodetect cell count option 
 * OSD    - Added autoset battery warning based upon cell count (for those who swap batteries) 
 * OSD    - Added long distance display for fixed wing - e.g. 15.3km instead of 1530m 
 * OSD    - Added Faster support options for VSYNC rate of 25/30 hz
 * OSD    - Added support for forced crosshair for boards without accelerometer
 * OSD    - Added support for Mode 1 TX users
 * OSD    - Watts display added. Available via layouts editor
 * OSD    - RSSI sensitivity via OSD inputs quadrupled. Inreased resolution for some RX. e.g. Scherrer
 * OSD    - Support for consumed current form flight controller
 * OSD    - Support for temperature sensor re-introduced
 * OSD    - Added option to disable display of summary screen via config.h
 * OSD    - Added option to display some maximum values immediately below live data - for record breaking attempts
 * OSD    - Support for fixed wing throttle actions when disarmed.
 * OSD    - Throttle calibration values can be amended for inreased accuracy of throllte in some scenarios
 * GUI    - Support for RSSI Auto calibration from GUI
 * GUI    - Support for Current sensor calibration from GUI - using hardware parameters (Offset and gain) 
 * GUI    - Displays active screen refresh rate
 * BUGFIX - HARIKIRI / MW2.1 not working since R1.5
 * BUGFIX - Center crosshair font partially missing on default_large font
 * BUGFIX - AHI incroect at large angles
 * BUGFIX - Analogue ports had RSSI pulled high - impacting range and accuracy of values 
 * BUGFIX - intermittant sticking on writing to OSD. Also resolves other intermittant checksum errors
 * BUGFIX - PASTHROUGH icon priority matches FC for FIXEDWING
 * BUGFIX - GPS altitude only enabled/displayed if GPS location enabled
 * BUGFIX - OSD exits menu first time after disarm when throttl eabove minimum threshold. Highered threshold
 * COMPAT - Compatability support for latest cleanflight - conflict with MultiWii
 * FONT   - improved large font from Schnupperm enhanced by Leo
 * FONT   - Support for Betaflight airmode. Displays font icon when active

---

## Bugfixes / Enhancements intended for 1.7: ##

Support for APM

Support for Multiple crosshair types

Improve GUI speed


---

## Enhancement requests for future releases: ##

Vortex style capacity indicator

Display PIDS when flying for in flight tuning

Migrate controller/aircraft choice to GUI

Power consumtion W/Hr and W/Km 

Support for OP

Support for displaying PID RPW values for in flight tuning

Improvements to PAL/NTSC autodetect / config change.

Review throttle autoscaling

Additional AHI layouts - multiple pitch angles

Config.h option to enable / disable dist/al/volt alarms etc.

Fresnel warning indicator

Display option for climb rate value instead of vario slider

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

Display GPS max instead of BARO max at end of flight summary

Use aernotautical units - knots - speed, nautical miles - distance, feet - altitude

Fast start - Autodetect MAX ready or timeout

Servo config page for fixedwing



Code quality:

 - Tidy up float calcs
 - Generally review and improve code quality
 - Memory improvements
 
---
 








