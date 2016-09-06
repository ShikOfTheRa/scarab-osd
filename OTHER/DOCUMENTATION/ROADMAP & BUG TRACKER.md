---

## Outstanding bugs: ##

---

## Bugfixes / Enhancements completed for next release (1.7): ##
 * OSD    - very improved camera auto detection.  
 * OSD    - long distance and negative altitude support .  
 * OSD    - Support for power Efficiency 
 * OSD    - Additional hardware support - OSD's without MAX hardware reset line connected 
 * OSD    - Low voltage hysteresis filter for short high throttle situations  
 * OSD    - Support for MAVLINK/APM/PIXHAWK
 * OSD    - GPS OSD mode home set improvement - higher sat counts and consecutive valid fix required.
 * OSD    - Support for LTM protocol OSD
 * OSD    - Support for KISS OSD
 * OSD    - Support for GPS / NAV PID adjustments - BF FIXEDWING
 * OSD    - Support for servo adjustments - BF FIXEDWING
 * OSD    - Legacy FC version display removed
 * OSD    - Option - display signal type at startup
 * OSD    - Additional hardware support - kylin 250
 * OSD    - Support for long distance - max travelled greater than 32km
 * GUI    - Support for LTM,MAVLINK,KISS protocol
 * GUI    - Option to disable simulator graphics for improved speed
 * GUI    - Video volts shown in decimal format
 * BUG    - Armed timer bugfix
 * BUG    - Warning message display overwriting
 * BUG    - Added simple GPS glitch filter for MAX speed errors from GPS.
 * BUG    - fix for high altitude > 9999.
 * BUG    - fix for negative amperage situations.
 * CODE   - Quality improvements - tidying 
 * CODE   - Memory improvements - significant memory savings 

---

## Bugfixes / Enhancements intended for 1.7: ##

 * GUI    - New Chrome GUI
 * GUI    - Improved speed
 * OSD    - RC switch migrated to 1-8 rather than 0-7 

---

## Enhancement requests for future releases: ##

RC switch channel limited to 8CH (0-7). Also starts at 0

LTM support 

Vortex style capacity indicator

MutiWii Nav PID config

Scaleable pitch - for racers to ensure ahi stays on screen

Display PIDS when flying for in flight tuning

Migrate controller/aircraft choice to GUI

Power consumtion W/Hr and W/Km / Efficiency similar to other OSD

Support for displaying PID RPW values for in flight tuning

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
 








