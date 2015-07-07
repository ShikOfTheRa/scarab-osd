# Displaying Frsky Smart Port (S.Port) Data in OSD #


## NEW FEATURE: SPORT HOST MODE, No receiver needed ! ##
> https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/Frsky_SPort_Host.md

## First things first (you will need) ##

  * Frsky reciever with s.port e.g. x8r
    * S.Port sensor/s e.g. FLVSS (cells), FCS (amps) or VARIO (alt)

  * Serial inverter cable: [MultiwiiCopter](http://www.multiwiicopter.com/products/sbus-inverter-cable-for-paris-v5) , these also double as Futaba S.BUS connectors.
    * And any spare serial port to use it on. [Note on Mega 2560 and Serial 0](#Note_on_Mega_2560_and_Serial_0.md)

  * Multiwii 2.3 modified with the s.port [patch here](http://code.google.com/p/multiwii-osd/source/browse/#hg%2FMWC_Patches%2Ffrsky_sport) ([tools/instructions](http://code.google.com/p/multiwii-osd/source/browse/#hg%2FMWC_Patches)) (or your own hand patched version)
    * As well as adding [new files "SPort.h" & "SPort.cpp"](http://code.google.com/p/multiwii-osd/source/browse/#hg%2FMWC_Patches%2Ffrsky_sport) to MW (unless already included as is case with some MWC products)

  * OSD code with font installed

## What it does ##

  * **The FLVSS module:** provides up to 6 individual cell voltages as well as overall battery voltage.
    * The osd also shows "Min" & "Avg" cell voltages.

  * **The FCS module:** provides amperage reading up to 40 amps as well as overall battery voltage.
    * This also allows calculation of mAh consumed.

  * **The VARIO module:** provides altitude and variometer (speed up/down) readings.
    * Currently vario data is just displayed and not used in flight model.

## Settings ##
> There are 3 groups of settings available:
  * MW config.h - **"frsky s.port telemetry link"** section
    * choose serial port & enabled modules
  * OSD Config.h - **"FrSky S.Port settings"** section
    * enable cell graph (if desired) and cell alarm level
  * OSD GUI - (for available modules)
    * **"Main Voltage"** section (FLVSS & FCS)
      * enable **"Display Voltage"**, **"Use MWii"** & set **"Voltage Alarm"**
    * **"Amperage"** section (FCS only)
      * enable **"Display Amps"**, **"Display mAh"** & **"Use MWii"**
    * **"Display"** section (Vario only)
      * enable **"Display Vario"** & **"Display Baro ALT"**



## Preview ##
**The Cell Graph, Min & Avg readings:**

![http://wiki.scarab-osd.googlecode.com/hg/images/cell_graph.jpg](http://wiki.scarab-osd.googlecode.com/hg/images/cell_graph.jpg)

**Example Setup, Serial 1 on Mega based v5 board** (ignore red traces connected to chip)

![http://wiki.scarab-osd.googlecode.com/hg/images/example_setup.jpg](http://wiki.scarab-osd.googlecode.com/hg/images/example_setup.jpg)

A Servo Plug to Mini Molex adapter, suitable for MWC v5 Boards

![http://imgur.com/ZEefIzt.jpg](http://imgur.com/ZEefIzt.jpg)

## Note on Mega 2560 and Serial 0 ##

Arduino Mega 2560 has a bug that we have fixed. If an actively transmitting serial device is connected to serial 0 during bootup, it never completes startup. We have corrected this bug with a updated boot loader, so you can leave Frsky S.Port device connected to Serial 0.

  * The bootloader is [-->here<--](https://code.google.com/p/multiwii-osd/source/browse/#hg%2FMWC_Patches%2Ffrsky_sport)
  * Im offering support to load this bootloader on your board (>= v5r3 already loaded): [Contact me here](http://www.multiwii.com/forum/viewtopic.php?f=8&t=5164)

## Issues / Support ##
See: http://www.multiwii.com/forum/viewtopic.php?f=8&t=5164
