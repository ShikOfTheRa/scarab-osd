
---

## Summary of features and changes between versions ##

---

Key Improvements for `R1.6`:-

 * OSD    - Support for TUALABS - thanks to Dustin
 * OSD    - Support for NAZA
 * OSD    - Support for BETAFLIGHT
 * OSD    - Support for BETAFLIGHT / CLEANFLIGHT / BASEFLIGHT FC profiles selection
 * OSD    - Support for BETAFLIGHT / CLEANFLIGHT / BASEFLIGHT FC pid controller selection
 * OSD    - BETAFLIGHT AIRMODE display indicator
 * OSD    - Support for CLEANFLIGHT time selection
 * OSD    - Video volts alarm adjustable via GUI
 * OSD    - Video volts independant of main battery
 * OSD    - Battery voltage status icon improved accuracy - thanks to Hwurzburg
 * OSD    - Battery capacity status icon added - thanks to Hwurzburg
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
 * BUGFIX - HARIKIRI / MW2.1 not working since R1.5
 * BUGFIX - Center crosshair font partially missing on default_large font
 * BUGFIX - AHI incroect at large angles
 * BUGFIX - Analogue ports had RSSI pulled high - impacting range and accuracy of values 
 * BUGFIX - intermittant sticking on writing to OSD. Also resolves other intermittant checksum errors  
 * COMPAT - Compatability support for latest cleanflight - conflict with MultiWii
 * FONT   - improved large font from Schnupperm enhanced by Leo
 * FONT   - Support for Betaflight airmode. Displays font icon when active


---

Key Improvements for `R1.5`:-

 * OSD    - support for newer baseflight release - separate pitch/roll config
 * OSD    - support for newer cleanflight release - separate pitch/roll/tpa breakpoint config
 * OSD    - OSD chip automatic fail / recover. Avoids blank screens from supply noise. 
 * OSD    - OSD TX switch channel can be changed in OSD menu
 * OSD    - faster PWM RSSI implementation for standard 50hz PWM
 * OSD    - user adjustable AHI maximum limits
 * OSD    - more accurate mah used / faster display updates. 10 times vs 1 time per second
 * OSD    - choice of MSP speed options for faster display updates
 * OSD    - support for 9600 baud
 * OSD    - general display refresh speed improvements from RSSI / VSNC / MSP
 * OSD    - improved reset function 
 * OSD    - improved eeprom default function (adds equivelent to eeprom_clear) 
 * OSD    - pitch and roll angle display 
 * FONT   - improved new large font from Schnupperm
 * FONT   - improved new large font from Anton Krug
 * BUGFIX - Cleanflight amperage correction
 * BUGFIX - MultiWii 2.4 amperage correction
 * BUGFIX - GUI not loading custom font after saving. 
 * BUGFIX - GUI font patch incorrect. 
 * BUGFIX - Font upload issues when using vsync
 * BUGFIX - When arming in menu, exits without save and enables serial operations
 * BUGFIX - UBLOX/MTK support for GPSOSD users
 * GUI - more realistic serial simulation method
 * GUI - font upload progress display
 * GPSOSD - configurable parameters for GPSOSD arming and flight summary setting
 * GPSOSD - RSSI input can bu used instead for screen switch functionality
 * MISC - option to clear EEPROM
 * MISC - option to upload font without using GUI
 
---
Key Improvements for `R1.4.1`:-

 * BUGFIX - UBLOX/MTK support for GPSOSD users

---

Key Improvements for `R1.4`:-

 * BUGFIX - "Use MW" options not saving backup file (single entry key)
 * BUGFIX - When set the "Time Zone Offset" to say 2.0 and then do a "WRITE" the value will change (display) to 0.2
 * BUGFIX - Compass OK, but home arrow is inconsistent in fixedwing mode
 * BUGFIX - compile errors for GPS in soem Arduino versions
 * BUGFIX - PID menu options after row 5 are incorrect. e.g. level does not show level settings
 * BUGFIX - All versions - timer clock drifts over time
 * BUGFIX - GPS OSD mode MTK - missing option to set 5hz update
 * BUGFIX - Many issues reported with errors saving config
 * BUGFIX - GPSOSD mode - NMEA paser not handling errors in data 
 * BUGFIX - GUI simulator sidebars and voltages match OSD display
 * OSD - Fixedwing - Heading / compass support for controllers without MAG. Default assumes no MAG
 * OSD - Fixedwing - Altitude support for controllers without BARO. Default assumes no BARO
 * OSD - Fixedwing - Vario support for controllers without BARO. Default assumes no BARO
 * OSD - Fixedwing - glidescope ILS
 * OSD - Fixedwing - reset altitude at arm option for height above ground level vs sea level
 * OSD - Maximum total trip distance travelled now > 12.5 miles previous limit
 * OSD - GPSOSD mode now supports a maximum distance from home of > 12.5 miles
 * OSD - Support for newer Cleanflight releases
 * OSD - Horizon bar set behind other screen items as they have higher priority
 * OSD - Sat warning - option for prominent text warning if sats are low (i.e. potential of incorrect alt / direction home)
 * OSD - Sat warning - option to recognise loss of GPS data from FC and warn via low sats. 
 * OSD - Data warning - option to recognise loss of data from FC and warn via "No data" message. 
 * OSD - AUTOCAM option (Auto sense PAL / NTSC cam at startup) to help with incorrectly marked cams / user error  
 * OSD - VSYNC now default option as most boards now support this. Clearer display
 * OSD - Added max Amps to flight summary
 * OSD - Amended 00 font to be blank character - recommended to reduce sparklies
 * OSD - 3 OSD layouts for use with 3 way tx switch
 * OSD - Autodetect PAL/NTSC or timeout to last known settings (fix blank screen for incorrect labelled cams)
 * OSD - support for reverse AHI (DJI/Eastern bloc OSD users)
 * OSD - support for adjusting AHI horizon to match fixed camera angle
 * OSD - support for forcing crosshair to be always on in all modes
 * GUI - GUI support for FC passthrough configuration settings (access OSD config via FC)
 * GUI - GUI support for OSD or 3 way RC switch layout changing
 * GUI - Introduction message 
 * GUI - Decimalise GUI value for min battery voltage etc
 * CONFIG - Provide optimisation and option for serial data requests for slower baud rates

---

Key Improvements for `R1.3`:-

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
 * GUI - significantly improved GUI simulator response time for OSD display
 * GUI - added names to GUI layouts
 * GUI - tidy up layout editor group
 * GUI - added GUI layout name to editor group bar
 * GUI - map modes selectable in GUI instead of config
 * GUI - implementation or URL links for support/ faq / guides / donate etc
 * GUI - support for different baud rates
 * GUI - relayout to increase screen space
 * GUI - option for OEM name rebranding
 * GUI - GUI / OSD version check
 * GUI - version / name moved to window title
 * GUI - switches turn green when enabled
 * GUI - small improvement to baud rate change handling on GUI
 * GUI - improved simulator toggle handling
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
 * BUGFIX -  LOW VOLTS displayed only when ARMED/DISARMED is clear
 * BUGFIX - OSD menu for cells/vid volts incorrect
 * BUGFIX - Unable to save AMP changes on GUI
 * BUGFIX - OSD menu for sensors display incorrect
 * BUGFIX - GUI - bugfixes to hidden font upload message text. Cosmetic
 * BUGFIX - GUI - bugfixes to remove system warnings on WIN64 or compiler
 * BUGFIX - RX led not working. Cosmetic
 * BUGFIX - Removed code causing warnings / unnecessary system printing
 * BUGFIX - using incorrect GPSlat/lon position
 * BUGFIX - GPSlat/lon positions were interlinked
 * BUGFIX - bug for unable to exit statistics screen when in in config mode
 * BUGFIX - OSD menu for sensors display incorrect
 * BUGFIX - Improved write / check stability
 * INFO   - Amended to support newer Arduino compiler

---

Key Improvements for `R1.2`:-

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

Key Improvements for `R1.1`:-

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

Key Improvements for `R1`:-

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

---

Key Improvements in development repository:-

 * GUI - Support for different refresh rates via \data\gui.cfg for slower pcs
 * GUI - more realisitc simulation of operation. Responds like FC - no more active push of data

