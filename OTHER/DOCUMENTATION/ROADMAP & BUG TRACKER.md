---

## Outstanding bugs: ##

---

## Bugfixes / Enhancements completed for next release (1.7): ##
 * OSD    - very improved camera auto detection.  
 * OSD    - long distance and negative altitude support .  
 * OSD    - Support for power Efficiency 
 * OSD    - Additional hardware support - OSD's without MAX hardware reset line connected 
 * OSD    - Low voltage hysteresis filter for short high throttle situations  
 * OSD    - GPS OSD mode home set improvement - higher home sat count / consecutive valid fix / PDOP
 * OSD    - Support for LTM protocol OSD
 * OSD    - Support for KISS OSD
 * OSD    - Support for MAVLINK 1.0 - APM/PIXHAWK 
 * OSD    - Support for GPS / NAV PID adjustments - BF FIXEDWING
 * OSD    - Support for servo adjustments - BF FIXEDWING
 * OSD    - Legacy FC version display removed
 * OSD    - Option - display signal type at startup
 * OSD    - Additional hardware support - kylin 250
 * OSD    - Support for long distance - max travelled greater than 32km
 * OSD    - Ability to enable/disable indifividual warning text alarms via config.h
 * OSD    - Ability to disable warning text alarms via OSD menu 
 * OSD    - I2C MSP slave support for FC's that support this method
 * OSD    - Support for up to 16 TX channels
 * OSD    - Support for efficiency display - Watts/KMh or Mph
 * OSD    - Improved support for AB7456 hardware
 * OSD    - Support for boards with 2 PWM/PWM+PPM input. AEROMAX
 * OSD    - Support for jflpyers canvas mode - BETAFLIGHT Configuration menus 
 * OSD    - Support for NAZA control modes mode inicator - GPS/ATTI/MANU/FAIL 
 * OSD    - Support for AHI roll and pitch scaling 
 * OSD    - Debug screen for testers and troubleshooting
 * GUI    - Support for LTM,MAVLINK,KISS protocol
 * GUI    - Option to disable simulator graphics for improved speed
 * GUI    - Video volts shown in decimal format
 * GUI    - Improved speed
 * GUI    - Display OSD info. Hardware,config etc.
 * GUI    - SImulator send rcdata for OSD RC Switch
 * BUG    - Armed timer bugfix
 * BUG    - Warning message display overwriting
 * BUG    - Added simple GPS glitch filter for MAX speed errors from GPS.
 * BUG    - fix for high altitude > 9999.
 * BUG    - fix for negative amperage situations.
 * CODE   - Quality improvements - tidying 
 * CODE   - Memory improvements - significant memory savings 
 * INFO   - GUI / OSD RC channel identifiers are now 1-8 insted of 0-7 to be more human readable
 * INFO   - Clearer AUTO voltage / alarm  cell count 

---

## Bugfixes / Enhancements intended for 1.7: ##

 * GUI    - New Chrome GUI

---

## Enhancement requests for future releases: ##

Wind direction indicator

FRSY implemenation for receiver side OSD

Display minimum voltage

I2C master support for direct compass, baro, acc hardware support.

FRSKY telemetry support 

Vortex style capacity indicator

MutiWii Nav PID config

Scaleable pitch - for racers to ensure ahi stays on screen

Display PIDS when flying for in flight tuning

Migrate controller/aircraft choice to GUI

Support for displaying PID RPW values for in flight tuning

Review throttle autoscaling

Additional AHI layouts - multiple pitch angles

Fresnel warning indicator

Display option for climb rate value instead of vario slider

pitch / roll angles alarm

Support for 8 GUI layouts selectable from OSD menu at field

Flight path vector

Navigation mode layout

Climb rate alarm

Descriptive chracter font for for heading / angle to home

Impelement more descriptive "help" text within confih.h

Migrate GUI to chrome

Improve icon buttons

Improved heading / pitch& roll graphics 

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
 








