
---

# MW OSD quick reference guide - MWOSD R1.6 #

To access the inbuilt OSD menu, with the FC DISARMED:
<li>THROTTLE MIDDLE<br>
<li>YAW RIGHT<br>
<li>PITCH FULL<br>
<br>
To navigate the OSD:<br>
<li>PITCH/ROLL sticks are used to navigate<br>
<li>YAW stick is used to adjust / change values<br>
<br>

<b>How to - [video guide to the OSD menu from DEET]
(http://youtu.be/pd_PXnG_PkI)</b>

<b>How to - [connecting to CLEANFLIGHT FC from Tr3TopFlyer]
(https://drive.google.com/file/d/0B2MInRUrbpWxUjdmdXJIWkRZY0k/view?usp=sharing)</b>

<b>How to - [flash the MW OSD firmware to your OSD]
(https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/FirmwareFlashing.md)</b>

<b>How to - [alternative way to access frsky via telemtry from Jindalee10368] 
(http://vk4ghz.com/mwosd-display-rssi-via-spare-radio-channel/)</b>


<hr />
<h1>MW OSD GUI quick reference guide</h1>

<h3>Controls and communication</h3>

<b>Comm settings</b>
<li>Toggle between com ports and baud rates. Selected values Are indicated above.</li>

<b>Baud Rate</b>
<li>Hidden option. Click "Comm settings" to access and change </li>

<h3>DISK</h3>

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
<li>Use FC main voltage - select this if you wish to use the Flight controllers voltage. If not selected, it will use the OSD voltage from the hardware pins connection (default).<br>
<li>Voltage adjust - use this to adjust the displayed voltage to match the actual battery voltage. Note if voltage is taken from the flight controller this adjustment is not applicable.<br>
<li>Battery cells - set this to the correct number of battery cells. It is only used for the battery capacity status icon.<br>
<li>Voltage alarm - set this to the voltage at which you require the battery voltage to start to flash.<br>
<br>
<b>Amperage</b>
<li>Display Amps - select this if you wish to display the instantaneous Amps draw<br>
<li>Display mAh - select this if you wish to display the cumulative mAh consumed during flight<br>
<li>If neither FC or Virtual are selected, the OSD will use analogue input on OSD<br>
<li>Use virtual sensor - select this instead of default hardware sensor to use a software estimation of calibrated current draw against throttle position.<br>
<li>Use FC Amperage - select this instead of deafault hardware sensor to Amerage values from Flight Controller .<br>
<li>Amps adjust - use this to adjust the Amps drawn value. It is recommended to adjust for accuracy against an Ampmeter around 50% <li>Zero adjust - use this to adjust the Amps drawn when the copter is idle<br>
throttle with a Multicopter<br>
<li>x100 mAh Alarm - use this to set alarm warning for bat capacity. 10 = 1000mah<br>
<li>Amp Alarm - use this to set alarm warning for Current draw<br>
<li>CAL Button - use this if you have a HW current sensor to set correct values<br>
<li> !!NOTE !! - remeber to select 5v reference whn using OSD hardware sensor<br>
<br>
<b>RSSI</b>
<li>Display RSSI - select this if you wish to display RSSI on the OSD.<br>
<li>If neither FC or PWM are selected, the OSD will use ANALOGUE RSSI from a RX directly connected to the OSD<br>
<li>Use FC - select this if you wish to to use the RSSI value from the Flight Controller instead of the OSD. The FC must configured to support this<br>
<li>Use PWM - select this if you wish to use the PWM RSSI value from a RX connected directly to the OSD<br> 
<li>Use PWM - (with RCRSSI configured in config.h) select this if you want to use a RC channel 0-7 for RSSI<br>
<li>RSSI alarm - set this to the value at which you require the RSSI value to start to flash<br>
<li>SET (Min) - click this to calibrate RSSI minimum value with TX turned off<br>
<li>SET (Max) - click this to calibrate RSSI maximum value with TX turned on<br>
<li>!! NOTE !! See guide for how to calibrate<br>
<br>
<b>Reference Voltage</b>
<li>You must select this if you wish to use > 4s<br>
<li>You must select this if you wish to use HW current sensor or Analogue RSSI<br>
<li>!!NOTE!! This requires a smooth power supply to the OSD when enabled<br>
<br>
<b>Display Callsign</b>
<li>You must select this if you wish to use callsign display<br>
<li>Enter the callsign in the Box. Use UPPER CASE and NUMERIC only<br>
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
<li>!! NOTE !! Support MultiWii only - and requires specific load<br>
<br>
<b>Alarms</b>
<li>Distance Alarm - use this to set alarm warning for max altitude. Value is <b>100<br></b><li>Altitude Alarm - use this to set alarm warning for max altitude. Value is <b>10<br></b><li>Speed Alarm    - use this to set alarm warning for max speed<br>
<li>Timer  Alarm   - use this to set alarm warning for timer (in minutes)<br>
<br>
<b>LAYOUT</b>
<li>Using OSD - choose to use FC OSD switch - or RC channel to switch screens<br>
<li>OSD ch - in RC channel mode specifies channel used to switch screen<br>
<li>HUD - LOW/MID/HIGH - layouts selected for RC switch option<br>
<li>HUD - Default / OSD SW - layouts selected for FC OSD_SWITCH option<br>
<br>
<b>Debug</b>
<li>Debug - displays debug values for FC/OSD developers<br>
<li>Diagnostic - turns diagnostic values off / on<br>
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
<li>Any 3 can be uploaded to OSD<br>
<li>Use the layout editor to create a layout that is available to the OSD to use.<br>
<li>Fields can be enabled or disabled.<br>
<li>Fields can be moved around.<br>
<li>Note that if a feature is disabled by the switches on the GUI or OSD menu, this overrides all screen layouts on the OSD.<br>
<li>Up to 256 individual HUD layouts can be created within the GUI<br>
<li>The layout defines the fields available to view<br>
<li>GUI switches or OSD menu can be used to optionally turn on/off fields available. Turning off disables field on OSD irrespective of the layout settings.<br>
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
WRITE<br>
<li>Click this to save any edits made to HUD layouts<br>
<li>ALSO updates the OSD<br>
<li>NOTE - not needed for ADD unless changes are made to teh added layout<br>
<br>
CANCEL<br>
<li>Click this to reload last saved HUD layouts<br>
<li>Cancel any current edits.<br>
<br>
<br>
<br>
<hr />
<h2>Further settings</h2>
Further settings are available in config.h due to :
  memory constraints to leave enabled permananetly
  rarely used
  not yet moved to the GUI
<br>
<h2>Advanced settings</h2>
Advanced settings are available in def.h. Primarily for developers


