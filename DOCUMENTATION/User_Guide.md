
---

# MW OSD quick reference guide #

To access the inbuilt OSD menu, with MultiWii DISARMED:
<li>THROTTLE MIDDLE<br>
<li>YAW RIGHT<br>
<li>PITCH FULL<br>
<br>
To navigate the OSD:<br>
<li>PITCH/ROLL sticks are used to navigate<br>
<li>YAW stick is used to adjust / change values<br>
<br>
<b>How to - video guide to the OSD menu from DEET</b>
<li><a href='http://youtu.be/pd_PXnG_PkI'>http://youtu.be/pd_PXnG_PkI</a>  </li>

<b>How to - connecting to CLEANFLIGHT FC from Tr3TopFlyer</b>
<li><a href='https://drive.google.com/file/d/0B2MInRUrbpWxUjdmdXJIWkRZY0k/view?usp=sharing'>https://drive.google.com/file/d/0B2MInRUrbpWxUjdmdXJIWkRZY0k/view?usp=sharing</a>  </li>

<hr />
<h1>MW OSD GUI quick reference guide</h1>

<h2>Controls and communication</h2>

<b>Comm settings</b>
<li>Toggle between com ports and baud rates</li>

<b>Baud Rate</b>
<li>Hidden option. Minimise "Comm settings" to access and change </li>

<b>Load</b>
<li>Loads previously saved configuration file from disk </li>

<b>Save</b>
<li>Saves current configuration to disk </li>

<h3>Font Tools</h3>

<b>Edit Font</b>
<li>Editor for currently loaded font file </li>

<b>Upload</b>
<li>Uploads fontfile from the GUI into the OSD </li>

<b>Browse</b>
<li>Browse for an alternative font file to load into the GUI </li>

<h3>OSD Controls</h3>

<b>Read</b>
<li>Reads values from OSD into the GUI </li>

<b>Write</b>
<li>Updates the OSD with values set in the GUI </li>

<b>Reset</b>
<li>Resets the board to OSD default settings </li>

<b>Restart</b>
<li>Restarts the OSD - recommended after saving changes </li>

<hr />
<h2>Configuration</h2>

<b>Voltage</b>
<li>Display Voltage - select if you wish to display voltage on OSD<br>
<li>Use MWII - select this if you wish to use the Multiwii controllers voltage. If not selected, it will use the OSD voltage from the hardware pins connection (default).<br>
<li>Voltage adjust - use this to adjust the displayed voltage to match the actual battery voltage.<br>
<li>Battery cells - set this to the correct number of battery cells. It is only used for the battery capacity status icon.<br>
<li>Voltage alarm - set this to the voltage at which you require the battery voltage to start to flash.<br>
<br>
<b>Amperage</b>
<li>Display Amps - select this if you wish to display the instantaneous Amps draw<br>
<li>Display mAh - select this if you wish to display the cumulative mAh consumed during flight<br>
<li>Use virtual sensor - select this instead of a hardware sensor to use a software estimation of calibrated current draw against throttle position.<br>
<li>Zero adjust - use this to adjust the Amps drawn when the copter is idle<br>
<li>Amps adjust - use this to adjust the Amps drawn value. It is recommended to adjust for accuracy against an Ampmeter around 50% throttle with a Multicopter<br>
<li>x100 mAh Alarm - use this to set alarm warning for bat capacity. 10 = 1000mah<br>
<li>x100 Amp Alarm - use this to set alarm warning for Current draw<br>
<li>!! NOTE !! See guide for how to calibrate<br>
<br>
<b>RSSI</b>
<li>Display RSSI - select this if you wish to display RSSI on the OSD<br>
<li>Use MWII - select this if you wish to to use the RSSI value from the RX connected to the Multiwii controller instead of the OSD<br>
<li>Use PWM - select this if you wish to use the RSSI value from a RX providing PWM RSSI connected to the OSD<br>
<li>If neither MWII or PWM are selected, the OSD will use analogue RSSI from a RX directly connected to the OSD<br>
<li>RSSI alarm - set this to the value at which you require the RSSI value to start to flash<br>
<li>!! NOTE !! See guide for how to calibrate<br>
<br>
<b>Reference Voltage</b>
<li>You must select this if you wish to use > 4s<br>
<li>You must select this if you wish to use HW current sensor or Analogue RSSI<br>
<li>!!NOTE!! This requires a smooth power supply to the OSD when enabled<br>
<br>
<b>Display Callsign</b>
<li>You must select this if you wish to use callsign display<br>
<li>Enter the callsign in the Box<br>
<br>
<b>Other</b>
<li>Select Metric or Imperial units for display and measurements<br>
<li>Select NTSC or PAL to match your Camera<br>
<li>Display Throttle position - shows throttle as a percentage value<br>
<li>Display Battery Status - shows battery icon as a fuel guage<br>
<li>Reset stats after arm - enable this only if you want to show measurements over a single flight instead of whilst battery connected<br>
<br>
<b>Display</b>
<li>Display Flight mode - shows ACRO/HORIZON/RTH/HOLD icon<br>
<li> - FM sensors - shows if acc/baro/mag are enabled<br>
<li>Display Gimbal - shows icon when gimbal is activated<br>
<li>Display Vario - shows climb / descend rate<br>
<li>Display BARO ALT - shows the altitude from Barometer<br>
<li>Display Timer - displays flight / on timer<br>
<br>
<b>HUD</b>
<li>HUD layout - choose the default HUD layout you prefer<br>
<li>HUD layout OSDSW - choose the alternative HUD layout that displays when selecting OSD SW<br>
<li>Display Horizon Bar - shows horizon indicator<br>
<li> - HB - shows additional horizon elevation indicators<br>
<li>Display Sidebars - shows vertical HUD sidebars<br>
<li> - SB scrolling - sidebars scroll in direction of change for altitude and speed<br>
<li> - SB direction - indicator arrows for alternative indicator of direction of change for altitude and speed<br>
<br>
<br>
<b>GPS settings</b>
<li>Display GPS - enables sats / direction to home arrow,/ Distance / Speed<br>
<li> - GPS co-ords - displays GPS co-ords<br>
<li> - GPS altitude - display GPS altitude<br>
<li> - MAPMode 0 - disabled<br>
<li> - MAPMode 1 - map center = aircraft, relative home position displayed on map<br>
<li> - MAPMode 2 - map center = home, relative aircraft position displayed on map<br>
<li> - MAPMode 3 - advanced hybrid combination of mapmodes 2 and 3<br>
<br>
<b>Compass</b>
<li>Display Compass - displays compass / heading indicator<br>
<li>Display Heading - displays heading in -180 +180 degrees<br>
<li> - Heading 360 - displays heading in alternative 0-360 degrees<br>
<li>Display angle to home - displays angle to home in degrees relative to direction heading<br>
<br>
<b>Time Settings</b>
<li>Display GPS time - displays the UTC time from satellite<br>
<li>Time zone offset - corrects UTC time to local time in hours<br>
<li>Timezone +/-  determines if offset is forward or back<br>
<li>DST Minutes - corrects for daylight time<br>
<li>!! NOTE !! requires patched Multiwii prior to 2.4<br>
<br>
<b>Alarms</b>
<li>Distance Alarm - use this to set alarm warning for max altitude. Value is <b>100<br></b><li>Altitude Alarm - use this to set alarm warning for max altitude. Value is <b>10<br></b><li>Speed Alarm    - use this to set alarm warning for max speed<br>
<li>Timer  Alarm   - use this to set alarm warning for timer (in minutes)<br>
<br>
<br>
<b>Debug</b>
<li>Displays Debug - for Multiwii developers<br>
<br>
<hr />
<h2>Simulator</h2>

<li>Use this to simulate Multiwii controller connected to your OSD<br>
<li>View real time the impact of changes on your OSD<br>
<li>View a simulated OSD screen on the computer - no cam / rx / screen needed<br>
<br>
<b>Simulator control</b>
<li>Enables the OSD simulator<br>
<br>
<b>Simulate the following</b>
<li>Multiwii sensors - mag / baro / gps<br>
<li>Multiwii flight modes - acro / horizon / angle / RTH / HOLD<br>
<li>Multiwii status - arm /camstab / home / OSD SW<br>
<li>Clear flight screen using OSD SW<br>
<li>Vario<br>
<li>Main battery voltages (simulated from multiwii FC)<br>
<li>RSSI (simulated from multiwii FC)<br>
<li>GPS data - distance / altitude / speed / satellites / heading home<br>
<li>Heading<br>
<li>RC command input<br>
<li>Pitch and Roll<br>
<li>Individual lipo cell status (requires FRSKY sensor / MWC patch)<br>
<br>
<hr />
<h2>Layout Editor</h2>
<b>Understanding the concept of how it works</b>
<li>Up to 256 individual HUD layouts can be created within the GUI<br>
<li>Any 2 can be uploaded to OSD<br>
<li>The layout defines the fields available to view<br>
<li>GUI switches or OSD menu can be used to optionally turn on/off fields available<br>
<br>
<b>Buttons and what they do</b>

+/- HUD:0<br>
<li>Shows the current HUD selected for edit<br>
<li>Click buttons to select the HUD you wish to edit<br>
<br>
+/- "field" e.g. "Sat Indicator"<br>
<li>Shows the current HUD field selected to edit<br>
<li>Click buttons to select the HUD field you wish to edit<br>
<br>
Enabled/Disabled<br>
<li>Shows if the field is available to display<br>
<li>Click buttons to enable or disable<br>
<br>
UP/DOWN/LEFT/RIGHT<br>
<li>Click buttons to move the selected field<br>
<br>
SWITCHES<br>
<li>Click this to set all display switches on - useful when OSD disconnected and to assist in planning layouts<br>
<li>NOTE - when complete, select EXIT to avoid saving switch settings<br>
<br>
ADD<br>
<li>Click this to add an extra hud if you want to leave the existing ones unmodified.<br>
<li>A fresh permanent HUD will be created identical to HUD0<br>
<br>
SAVE<br>
<li>Click this to save any edits made to HUD layouts<br>
<li>NOTE - not needed for ADD<br>
<br>
CANCEL<br>
<li>Click this to reload last saved HUD layouts<br>
<li>Cancel any current edits.<br>
<br>
<br>
<br>
<hr />
<h2>Advanced settings - config.h</h2>

The following settings are available in config.h because rarely used or have not yet been made available via GUI<br>
<br>
FEATURES<br>
<li>SBDIRECTION     - Enable/disable sidebar indicators (changes in speed or altitude)<br>
<li>HORIZON         - Enable/disable HORIZON indicator<br>
<li>MAPMODE         - Enable/disable MAP MODE - map indication of relative positions of home and aircraft<br>
<li>GPSTIME         - Enable/disable GPS Time functions<br>
<li>SPORT           - Enable/disable FRSKY S.PORT cell code<br>
<br>
HARDWARE<br>
<li>MINIMOSD        - Enable to use standard MINIMOSD (default for 95% of boards)<br>
<li>RUSHDUINO       - Enable to use the original RUSHDUINO hardware<br>
<li>WITESPY V1.1    - Enable to use the WITESPY V1.1 and correct for resistor/labelling errors<br>
<br>
CONTROLLER<br>
<li>BASEFLIGHT      - Enable to BASEFLIGHT - uses alternative heading / current divider<br>
<li>CLEANFLIGHT     - Enable to CLEANFLIGHT - uses alternative heading / current divider<br>
<li>HARIKIRI        - Enable to HARIKIRI - uses BOXNAMES identifier<br>
<li>FIXEDWING       - Enable to use MW fixed wing branch from PatrikE<br>
<li>MULTIWII_V21    - Enable to MULTIWII_versions up to 2.1<br>
<li>MULTIWII_V23    - Enable to MULTIWII_versions 2.2 / 2.3<br>
<li>MULTIWII_V24    - Enable to MULTIWII_versions 2.4 (SUBJECT TO CHANGE)<br>
<li>NOCONTROLLER    - Enable if no flight controller - e.g. GPS OSD or groundstation<br>
<br>
FIXEDWING<br>
<li>FIXEDWING       - Enable if you are using MW fixed wing from PatrikE<br>
<br>
FILTER<br>
<li>STAGE2FILTER    - Enable to use a simple filter for analog and RSSI sensors<br>
<li>SMOOOTHFILTER   - Enable to use an advanced smoothing reactive filter for analog and RSSI sensors<br>
<br>
RSSI<br>
<li>FASTPWMRSSI     - Enable if you are using non standard PWM for RSSI ( high frequency )<br>
<br>
CALLSIGN<br>
<li>CALLSIGNINTERVAL- How frequently to display callsign (in seconds)<br>
<li>CALLSIGNDURATION- How long to display callsign (in seconds)<br>
<li>CALLSIGNALWAYS  - Enable to permanently display callsign @ location you specify<br>
<li>FREETEXT x      - Enable to use switchable freetext on OSD<br>
<br>
STARTUP<br>
<li>INTRO_VERSION   - Call the OSD something else on startup.<br>
<li>INTRO_CALLSIGN  - Enable to display callsign at startup<br>
<li>INTRO_TIMEZONE  - Enable to display timezone at startup - if GPS TIME is enabled<br>
<li>INTRO_DELAY     - Define time intro screen should show for. Default is 10<br>
<li>INTRO_MENU      - Enable to display TX stick MENU guide<br>
<li>STARTUPDELAY    - Enable alternative startup delay (in ms) to allow MAX chip voltage to rise fully and initialise before configuring<br>
<br>
GPS TYPE<br>
<li>I2CGPS_SPEED    - Enable if for older I2CGPS to correct speed bug<br>
<li>I2CGPS_DISTANCE - Enable if for older I2CGPS to correct distance bug<br>
<br>
MAP MODE<br>
<li>MAPMODENORTH    - Enable to use North as MAP reference in MODE 1 instead of take off direction<br>
<br>
<li>AMPERAGEMAX     - Define capacity size of current sensor <code>*</code> 10 e.g. 50A sensor = 500<br>
<li>AMPERAGEOFFSET  - Enable Optional extra for bidirectional sensors<br>
<br>
OSD SWITCH<br>
<li>OSD_SWITCH      - Uses 2 way screen switch using OSD Switch via Flight Controller. MUST Ensure enabled on flight controller - e.g. #define OSD_SWITCH on multiwii<br>
<li>OSD_SWITCH_RC 5 - Enables 2 or 3 way screen switch using RC data. Midpoint = blank screen. Specify channel (range 0-7 AUX1=4 AUX4=7)<br>
<br>
<br>
DISPLAY<br>
<li>AUTOCAM         - Enable autodetect Camera type PAL/NTSC. Overrides GUI/OSD settings. Not recommended for > 600TVL cameras<br>
<li>DECIMAL         - Choose decimal point character<br>
<li>USE_VSYNC       - Enable to provide a cleaner display for boards that support VSYNC<br>
<li>MIN_CELL_320    - FRSKY SPORT low cell warning value. 320=3.20v<br>
<li>SHIFTDOWN       - Enable if your monitor cannot display top line fully<br>
<li>ALT_CENTER      - alternative center crosshair<br>
<li>HIDEARMEDSTATUS - Enable to hide ARMED / DISARMED status - for non Multiwii OSD<br>
<li>PASTPIXEL       - Enable for improved character sharpness with some cameras<br>
<li>WHITEBRIGHTNESS - Enable to adjust OSD white brightness<br>
<li>WHITEBRIGHTNESS - Enable to adjust OSD black brightness<br>
<li>FULLAHI         - Enable for a longer AHI line<br>
<li>I2CERROR        - Autodisplay Mutltiwii I2C errors if exceeds specified count. Indicates potential error<br>
<li>APINDICATOR     - Enable to display AUTOPILOT instead of RTH xxxm<br>
<li>SHORTSTATS      - Enables simple flight summary display for flight timer only<br>
<li>DISP_LOW_VOLTS_WARNING - Enable prominent low voltage warning text<br>
<li>FORCE_DISP_LOW_VOLTS   - Enable display low voltage warning override for screen layouts where its disabled<br>
<br>
FRSKY SPORT<br>
<li>MIN_CELL_320    - FRSKY SPORT low cell warning value. 320=3.20v<br>
<br>
BAUD RATE<br>
<li>BAUDRATE XXXXXX - Use to define an alternative baud rate<br>
<br>
SATELLITE FIX<br>
<li>MINSATFIX 5     - define number of sats required for an OSD fix / warning. 5 minimum. More = better<br>
<br>
GPS based OSD<br>
ONLY USE FOR STANDALONE GPS MODE WITH NO FLIGHT CONTROLLER<br>
<li>NMEA            - Enable if using a standard NMEA based GPS<br>
<li>UBLOX           - Enable if using a standard UBLOX based GPS<br>
<li>MTK             - Enable if using a standard MTK based GPS<br>
<li>MTK_BINARY16    - Enable if using MTK3329 chipset based GPS with DIYDrones binary firmware v1.6<br>
<li>MTK_BINARY19    - Enable if using MTK3329 chipset based GPS with DIYDrones binary firmware v1.9<br>
<br>
<br>
---Developer options---<br>
<br>
<li>DEBUG           - Enable to allow display of debug values. requires enabling ong GUI/OSD menu<br>
<li>DEBUGMW         - Enable to use MSP debug values from the MW FC<br>
<br>
<hr />
The following options can be disabled to free memory for development or enable other features:<br>
<li>DEBUG<br>
<li>SBDIRECTION<br>
<li>HORIZON<br>
<li>MAPMODE<br>
<li>GPSTIME<br>
<li>FRSKY SPORT<br>
<li>OSD MENU PAGES<br>
