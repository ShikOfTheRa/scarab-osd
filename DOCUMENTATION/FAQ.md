1 How do I use OSD / smart switch
  * Enable #define OSD\_SWITCH on multiwii FC
  * Assign a switch on the MultiWii GUI to the OSD Switch which will now be visible

---

2 I have nothing on my display?
  * Ensure your cabling is correct
  * Ensure your PAL/NTSC OSD setting matches your camera setting.
  * If using MinimOSD board make sure your output stage is properly powered.
  * Ensure you do not have blank screen layout selected
  * If using 3 way switch try different position

---

3 Lines on my display are very feint
  * Use a large font file such as the one from ABL
  * Try increasing brightness and enabling fastpixel in config.h
  * if your camera is hi res (650TVL+) change camera CRT/LCD setting to LCD

---

4 I can't open GUI on my MAC - unidentified developer error
  * http://www.imore.com/how-open-apps-unidentified-developer-os-x-mountain-lion

---

5 I can't open GUI on my MAC -  .app is damaged and can’t be opened
  * http://www.tech-recipes.com/rx/45404/mac-downloaded-app-is-damaged-and-cant-be-opened-error-solved/

---

6 I have a bi-directional current sensor - how do I set it up
  * The information is now updated in the calibration guide

---

7 My OSD turns off in flight / after powering up motors?
  * Ensure you have a good ground
  * Use a big cap (e.g. 3300uf) on the 5v supply to the OSD

---

8 I get white glow effect/white streaks behind the osd text with Fatshark PilotHD
  * Set the following in config.h
  * #define FASTPIXEL
  * #define WHITEBRIGHTNESS 0xB

---

9 My voltage doesn't change during flight
  * If using a witespy board you may have to select WITESPY in config.h
  * Ensure you are connected to the correct vbat pin
  * Ensure you have selected the correct battery option - MWII if using FC connection

---

10 My Voltage/RSSI/Current does not change with adjusters
  * Note that adjusters are only for connections direct to OSD - NOT via the controller

---

11 The OSD always shows DISARMED and one or more other items do not work
  * This usually means the OSD is not getting information from the Flight Controller
  * Ensure your connections are correct. Especially tx/rx orientation
  * Makes sure you are using the correct baud rate - 115k is default
  * You cannot share serial with other devices like telemetry
  * Note if using AIRPLANE mode it can be disabled in config.h
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

---

13 Data does not save when press write on the GUI.
  * Pre 1.3, this is typically a mismatch between OSD and GUI versions.
  * If unable to update firmware, also check rx/tx connections are correct.
  * Ensure USB programmer provides correct voltage
  * Note some programmers have 3.3v/5v selection text swapped. Verify with a meter

---

14 I get garbage characters and font upload doesn't work.
  * The output stage (max chip) MUST be powered when uploading font.
  * A programming voltage of 4.8v or higher is required.
  * Note some programmers have 3.3v/5v selection text swapped. Verify with a meter

---

15 I don't know if I should enable 5v ADC. When should I?
  * If using RSSI connected directly to OSD you must enable 5v ADC
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