1 How do I use OSD / smart switch / Screen switching
  * There are 2 methods - using a RC channel (0-7) or by using OSD SWITCH if the FC supports it.
  * Step1 - Select which method on the GUI
  * if using OSD_SWITCH method, enablele #define OSD_SWITCH on multiwii FC or via CLI / GUI for other FC
  * if using OSD_SWITCH method, assign a switch on the FC GUI to the OSD Switch which will now be visible
  * if using RC Channel, just make sure you use teh correct channel. 

---

2 I have nothing on my display?
  * Ensure your cabling is correct
  * Ensure your PAL/NTSC OSD setting matches your camera setting.
  * If using MinimOSD board make sure your output stage is properly powered.
  * Ensure you do not have blank screen layout selected
  * If using 3 way switch try different position
  * If using FRSKY or other high frequency RSSI, unplug until resolved.
  * Some stups require a consistent ground between camera and OSD video output
  * Check camera compatility documentation

---

3 Lines on my display are very feint and flickering at all times
  * This is usually a compatibility issue with the camera. Check camera compatility documentation
  * Use a large font file such as the one from ABL
  * Try increasing brightness and enabling fastpixel in config.h
  * if your camera is hi res (650TVL+) change camera CRT/LCD setting to LCD

---

4 I can't open GUI on my MAC - unidentified developer error / unable to open
  * http://www.imore.com/how-open-apps-unidentified-developer-os-x-mountain-lion
  
* Ensure the application.macosx/MW_OSD_GUI.app/Contents/MacOS/MW_OSD_GUI file is executable
  * If not issue chmod +x on that file

---

5 I can't open GUI on my MAC -  .app is damaged and can’t be opened
  * http://www.tech-recipes.com/rx/45404/mac-downloaded-app-is-damaged-and-cant-be-opened-error-solved/

---

6 How do I setup a current sensor
  * The information is now updated in the calibration guide

---

7 My OSD turns off in flight / after powering up motors?
  * Ensure you have a good ground
  * Use a big cap (e.g. 3300uf) on the 5v supply to the OSD
  * Enable #define MAXSTALLDETECT in config or upgrade if not listed as an option

---

8 I get white glow effect/white streaks behind the osd text with Fatshark PilotHD
  * Set the following in config.h
  * #define FASTPIXEL
  * #define WHITEBRIGHTNESS 0xB

---

9 My voltage doesn't change during flight
 * Check you have selected teh correct board in OSD HARDWARE settings
 * Check you have selected the correct battery option - FC if using FC connection
 * If using a micro board, you probably have to select RTFQMICRO
 * If using a full size witespy board you may have to select RTFQV1 in config.h
 * Ensure you are connected to the correct vbat pin some are reversed bat1 / bat2
 * Enable SWAPVOLTAGEPINS if you don't want to swap physical wires.

---

10 My Voltage/RSSI/Current does not change with adjusters
  * Note that adjusters are only for connections direct to OSD - NOT when connected via the controller

---

11 The OSD always shows NO DATA / DISARMED permanentlty and one or more other items do not work
  * This usually means the OSD is not getting information from the Flight Controller
  * Ensure your connections are correct. Especially tx/rx orientation
  * Makes sure you are using the correct baud rate - 115k is default
  * You cannot share serial with other devices like telemetry
  * Note it can be disabled in config.h
  * Baud rate in config.h MUST match baud rate in GUI (access via comsettings button)

BASEFLIGHT USERS - remove telemetry as follows:
  * feature -TELEMETRY

CLEANFLIGHT USERS - remove telemetry and enable GPS as follows:
  * feature -TELEMETRY
  * set serial\_port\_1\_scenario = 5
  * set serial\_port\_2\_scenario = 2
  * SAVE
setting port 1 to 5 equates to MSP, CLI, GPS-PASSTHROUGH
setting port 2 assigns it for gps , feature GPS will also need to be enabled

HARAKIRI USERS - ensure protocol set as follows:
  * tele\_prot=0

---

12 The OSD always shows random characters on the NESW compass heading
  * This is typically a mismatch between the Flight controller type and the OSD controller selected in config.h
  * Latest version of OSD should correct for this

---

13 Data does not save when press write on the GUI.
  * Pre 1.3, this is typically a mismatch between OSD and GUI versions.
  * If unable to update firmware, also check rx/tx connections are correct.
  * Ensure USB programmer provides correct voltage
  * Note some programmers have 3.3v/5v selection text swapped. Verify with a meter

---

14 I get garbage characters and / or font upload doesn't work.
  * Sounds like fonts need to be re-uploaded - but note:
  * The output stage (max chip) MUST be powered when uploading font. How depends upon your board / configuration
  * A programming voltage of 4.8v or higher is required.
  * Note some programmers have 3.3v/5v selection text swapped. Verify with a meter
  * Some stacked boards with USB connector also require vbat connected which powers the MAX chip and enables font upload

---

15 I don't know if I should enable 5v ADC. When should I?
  * If using analog RSSI connected directly to OSD you must enable 5v ADC
  * If using a hardware Current sensor connected directly to OSD you must enable 5v ADC
  * If using more than 4 cells you must enable 5v ADC

---

16 Direction, distance and speed are not working
  * Normally the FC needs to be armed. Not applicable to GPS/FIXEDWING mode

---

17 I can't enter values into the GUI
  * Click onto the box and drag up / down. Obvious eh ?!

---

18 I can't enter OSD menu
  * FULL RATES are required for the stick menus to be selected currently

---

19 How do I use MAP mode? It doesn't seem to work
  * Quick answer: Select HUD layout 4 and set MAP mode to 1
  * Long answer:
  * Firstly items to be displayed need to be enabled in the hud layout being used. (Map Mode & Map Center)
  * Secondly - on the GUI or OSD menu, map mode buts be set from 1-4.
  *  - mode 1 is where aircraft is center and H displays home location
  *  - mode 2 is where Home is center of the map and X displays relative aircraft position
  *  - mode 3 is a combination of mode 2 and 3
  *  - mode 4 is low resolution, but with ^ indicating location of aircraft and heading

---

20 I get c:\..... \java\javapath\javaw.exe unspecified error or javapath error. How do I fix it.
  * There are many type of Java error. It us usually related to Java not setting up paths correctly. The most common ones have a solution in the video below:
  * https://www.youtube.com/watch?v=66crhKstr70

---

21 I get "Waiting FC” or "Waiting OSD". How do I fix it.
  * Ensure you are connected to the correct com port. 
  * Ensure baud rate in GUI matches baud rate on the OSD
  * Ensure only the OSD is connected to programmer
  * If using OSDOGE ensure switch is in correct position for programming
  * If using cheap OSDOGE/BROSD clone, disconnect board from the Naze32
  * Restart PC – especially if hibernated
  * Ensure physical connections are correct. Test by re-uploading MWOSD if necessary to verify
  * Upload EEPROM_Clear example sketch then re-upload MWOSD sketch. 
  * Set serial port default value on PC to be same as OSD (e.g. device manager>ports>comx>port settings)

---

22 I get A screen full of error messages trying to load the GUI - inside the text is "Can't load AMD 64-bit .dll on a IA 32-bit platform" 
  * You are probably trying to run 64 bit MWOSD GUI on a PC running 32 bit java
  * Try running 32 bit MWOSD GUI instead or change java to 64 bit version instead. 

---
23 How do I configure for standalone GPS OSD (no controller)
  * https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/Guide_standalone_GPS_OSD.md

---
23 GUI will not launch
  * Try both 32 and 64 bit versions
  * Check other GUI FAQ points (therere a re several)
  * Completely uninstall all Java versions and re-install latest. Note you will have to google to find instructions to completely remove. As part of this, make sure path variables are completely erased before re-installing.

---
24 How do I use RSSI with FRSKY (D4R-II)
  * RECOMMENDED - Use an LC filter like CASE 2 here : https://code.google.com/p/minoposd/wiki/AnalogRSSI
  * Connect directly to NAZE like this : https://github.com/cleanflight/cleanflight/blob/master/docs/Rssi.md
  * For Taranis users there is this option: http://blog.oscarliang.net/rssi-ppm-channel-taranis/
  * For 9x/9xr OpenTX users there is this option: http://fpvlab.com/forums/showthread.php?34250-MWOSD-for-MULTIWII-NAZE32-BASEFLIGHT-HARIKIRI&p=741209&viewfull=1#post741209

---
25 Display keeps blinking. May be worse at high throttle
  * This is noise on the power to the OSD. It needs to be sorted
  * Google LC power filter for FPV

---
26 GUI very slow to update / unable to turn simulator off
  * Edit gui.cfg in GUI data folder
  * Set AutoSimulator to 0
  * Note it may also be a sign of an old pc or other application / malware running on PC

---
26 Can I swap between different batteries 3/4s and set alarms automatically
  * Use R1.6 and higher
  * uncomment AUTOCELL
  * Amend Woltage waring section in config.h as you require 
  * Consider use of AUTOCELL_VOLTAGE
  
---

COMPATIBILITY ISSUES
Chack camera compatibility document
 

