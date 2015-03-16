
---

## Summary of features and changes between versions ##

Key Improvements for `R1.3`:- OSD


  * OSD - introduction of GPS based (no FC) standalone OSD support
  * OSD - alarms for distance, altitude, speed, mAh, amps and flytime
  * OSD - large font option
  * OSD - additional MAP mode for advanced pilots
  * OSD - added alarms adjust page for Distance / Altitude / Speed / mah / Current
  * OSD - Increase trip maximum
  * OSD - fast value change in menus
  * OSD - FULLAHI option increased
  * OSD - re-added temperature support (UNTESTED)
  * OSD - ability to fully more PID settings such as MAG etc. previously limited

  * GUI - OSD layouts can be edited from GUI
  * GUI - 256 OSD layouts can be created / stored on GUI
  * GUI - 10 preset and new layouts - ground station and standalone
  * GUI - Alarm adjustment options
  * GUI - added callsign to simulator
  * GUI - call sign position can be edited via GUI
  * GUI - added read & write to OSD verification check.
  * GUI - significantly improved GUI simulator response time for OSD display.
  * GUI - added names to GUI layouts
  * GUI - tidy up layout editor group
  * GUI - added GUI layout name to editor group bar
  * GUI - map modes selectable in GUI instead of config
  * GUI - implementation or URL links for support/ faq / guides / donate etc.
  * GUI - support for different baud rates
  * GUI - relayout to increase screen space
  * GUI - option for OEM name rebranding
  * GUI - GUI / OSD version check
  * GUI - version / name moved to window title
  * GUI - switches turn green when enabled
  * GUI - small improvement to baud rate change handling on GUI
  * GUI - improved simulator toggle handling.

  * CONFIG - option for OSD switch 3 position
  * CONFIG - option for prominent "low voltage" text warning
  * CONFIG - option for displaying low voltage in blank screens
  * CONFIG - option in config to define callsign display frequency and duration
  * CONFIG - option to amend blackbrightness for TX/CAM with compatibility issues
  * CONFIG - definition to support CLEANFLIGHT
  * CONFIG - option to specify minimum sats for a fix
  * CONFIG - predefined baud rate settings
  * CONFIG - definition added to support CLEANFLIGHT
  * CONFIG - option to specify minimum sats for a fix
  * CONFIG - predefined baud rate settings
  * CONFIG - improved OSD SWITCH operation. New section
  * CONFIG - optional Auto PAL/NTSC detect

  * BUGFIX - LOW VOLTS displayed only when ARMED/DISARMED is clear.
  * BUGFIX - OSD menu for cells/vid volts incorrect
  * BUGFIX - Unable to save AMP changes on GUI
  * BUGFIX - OSD menu for sensors display incorrect
  * BUGFIX - GUI - bugfixes to hidden font upload message text. Cosmetic
  * BUGFIX - GUI - bugfixes to remove system warnings on WIN64 or compiler.
  * BUGFIX - RX led not working. Cosmetic
  * BUGFIX - Removed code causing warnings / unnecessary system printing
  * BUGFIX - using incorrect GPSlat/lon position
  * BUGFIX - GPSlat/lon positions were interlinked
  * BUGFIX - bug for unable to exit statistics screen when in in config mode.
  * BUGFIX - OSD menu for sensors display incorrect
  * BUGFIX - Improved write / check stability

  * INFO   - Amended to support newer Arduino compiler


---



Key Improvements for `R1.2`:- OSD

  * OSD - Support for HUD layouts set by GUI
  * GUI - 8 Preset OSD layouts can be selected from GUI

  * OSD - Switch between any 2 Layouts using OSD SWITCH
  * OSD - upport for MultiWii 2.4 waypoint navigation  MISSION. Displays waypoint step
  * OSD - Increased Amperage display to 999A
  * OSD - Increased resolution of MAP mode by approx 900%
  * OSD - MAP mode operation changed - Legacy retained as option
  * OSD - autodisplay FRSKY cell data if connected
  * OSD - Added default Baseflight amperage and heading correction support
  * OSD - vario font amended for sharper display
  * OSD - MAX chip startup delay configurable for boards with low voltage rise time
  * OSD - AUTOPILOT indicator as default. Legacy RTH distance retained as option
  * OSD - Fixedwing Passthrough MODE display support(PatrikE)
  * OSD - Sats warning if <5 sats

  * GUI: Layouts can be assigned to OSD SWITCH. Choice reflected on GUI sim
  * GUI: Simulator - Add FRSKY cells to simulator
  * GUI: Simulator - Display GPS time and support when OSD GPSTIME enabled
  * GUI: Simulator - Added support to show map mode
  * GUI: Many changes to support OSD changes
  * GUI: Added background skin for aesthetics

  * CONFIG - All major options are now supported by default within memory
  * CONFIG - Added support for boards with different resistors / pinout
  * CONFIG - Specified support for HARIKIRI / BASEFLIGHT / MULTIWII
  * CONFIG - Added specified support for witespy v2 boards
  * CONFIG - Amperage support for MW2.4 changes
  * CONFIG - Baud rate option
  * CONFIG - FRSKY SPORT option
  * CONFIG - FAST PWM RX RSSI support
  * CONFIG - Intro display delay option
  * CONFIG - Intro display Time zone option
  * CONFIG - Intro display OSD name option - for OEM suppliers
  * CONFIG - TX OSD menu guide display option
  * CONFIG - Freetext displayed using a switch (e.g. "FLAPS" indicator)
  * CONFIG - Option for a longer AHI line
  * CONFIG - Autodisplay I2C error count is above a user set value


  * bugfix - GPS time ignored when GPSTIME disabled in config
  * bugfix - min\_cell volt correction
  * bugfix - Fix for RSSI when tx switched off (negative value)
  * bugfix - Bugfix for any menus with 8 options


---

Key Improvements for `R1.1`:- OSD


  * All OSD sensors now have 8 point filter - Battery / Video / Current / RSSI / PWM RSSI
  * optional 2nd stage simple filter added
  * optional smooth filter added - very smooth, but with ability to handle large changes in short timelinefor RSSI
  * MWII sensor data remains unfiltered - direct from from MultiWii Flight Controller
  * Vid voltage divider moved to "approx" same as main voltage for easy of configuration
  * Vidvoltage added to OSD menu
  * Zero amps adjust added to OSD menu
  * RSSI adjust max + min added to OSD menu
  * Amps and volts in the OSD menu now in Human readable form - for easier adjustment
  * Heading correction option for to support Baseflight & Harikiri added to config.h file
  * Added support for MultiWii fixedwing branch from - PatrikE
  * I2C GPS distance fix - for OSD with issues at distance > 650m (untested)
  * ability to disable mode icon if required
  * ability to hide ARMED/DISARMED status via config.H if required. For non MultiWii OSD use
  * fastpixel support - **may** improve display for hi-res cameras 650tvl+
  * brightness support into config.h
  * amperage support from multiwii for sensors directly connected to MultiWii - haydent
  * option to hid voltage, video voltage and timer within options for OSD config.h witch control
  * current used during flight added back into flight summary screen
  * GPS timezone and options added into OSD menu - haydent
  * default startup delay - support for MAX chip voltage rise issue with some boards - suggested by bicycle
  * flashing sats warning if <5 sats - suggested by disq
  * freetext that can be enabled / displayed using LLIGHT SWITCH or CAMSTAB. Alternative to CALLSIGN

Key Improvements:- GUI
  * GUI - renamed reset to defaults for extra clarity
  * GUI - task switcher for faster serial - PatrikE
  * GUI - support for changes - FRSKY / MW current sensor. Tip links added

---



Key Improvements for `R1`:- OSD
  * Re layout of OSD - more Pro feel than `r370`
  * Introduction of RADAR and MAP modes
  * Virtual current sensor - no hardware required - surprisingly accurate when calibrated around hover / 50% throttle
  * Implementation of standard / minimal screen using OSD\_SWITCH - sometimes referred to as "smart switch"
  * Optimised icons throughout - less intrusive
  * Support for up to 6s
  * Working PWM RSSI
  * Easy RSSI calibration via OSD
  * Standard and reversed RSSI supported - PWM / Multiwii and OSD ADC
  * Higher resolution for adjustment on voltages and current
  * MODE Indicator button
  * Gimbal active indicator + on / off
  * Support for older I2C GPS - displays correct speed (10% of actual)
  * Support for older I2C GPS - fix for incorrect distance > 650m (untested)
  * Support for GPS time (requires MW 2.3 patched or 2.4) (haydent)
  * Optional Scrolling side bars - indicating altitude + speed change
  * Optional direction indicators at top / bottom of scrolling side bars
  * Optional elevation indicator - additional horizon bars at extremes of angles
  * Vario moved to less intrusive slider
  * Support for VSYNC (Okan/VVK)
  * OSD menu items optimised to minimise need for GUI
  * Main Voltage adjustable in OSD Menu - and can see voltage in menu. Much easier
  * OSD switch items can be selected in config.h
  * Simplified OSD save routines
  * GPS co-ords automatically displayed on RTH (should be set as fail-safe...) - just in case...
  * GPS co-ords shown to 1m accuracy...
  * RTH shows distance to home with Icon
  * New implementation for hardware current sensor
  * Permanent callsign display option
  * All items can now be turned off / on on display via GUI. You have full control.
  * VSYNC can be enabled / disabled in config //define VSYNC
  * DEBUG mode for Multiwii developers in config //define DEBUG
  * Screen squash - for monitors that do not display top line - in config.h #define SHIFTDOWN
  * Many bugfixes

Key Improvements:- GUI
  * Working Simulator
  * Working Simulator display on GUI
  * Full ability to view layout and impact of changes even without OSD
  * Current sensor settings moved to OSD
  * Relayout and separation of key groups
  * Support for GPS time
  * Reference voltage option added
  * Flight mode icons on/off
  * Scrolling side bars for speed/alt change indicators + on / off
  * Baro alt on/off
  * Compass on/off
  * Additional Elevation horizon + on / off
  * Timer on / off
  * Flight sensors on / off
  * Side bar arrows for speed/alt change indicators + on / off
  * Preset defaults to save time and demonstrate GUI/OSD better
  * Support for Map modes
  * Many bugfixes