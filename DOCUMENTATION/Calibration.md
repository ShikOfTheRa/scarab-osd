
---

## Calibration - Voltage ##

NOTE: if use Multiwii FC voltage, then ALL adjustments are made on the FC. The following does not apply.

**Main or video voltage connected directly to OSD**
  1. If > 4s select "Enable, ADC 5v ref"
  1. If using analog RSSI direct to OSD, "Enable ADC 5v ref"
  1. If using current sensor direct to OSD, "Enable ADC 5v ref"
  1. If "Enable ADC 5v ref" is changed, save and restart the OSD
  1. With a voltage meter, change "Voltage adjust" and save until the voltage on the OSD matches the meter
  1. Repeat for video if appropriate


---

## Calibration - Current ##

NOTE: if use MultiWii FC current, then ALL adjustments are made on the FC. The following does not apply.


The easiest way to set up a current sensor is probably to use this link. It contains a number of sensors already entered or you can add a new one, or use custom settings. It will provide the theoretical values you need. From this you can optimise further if required.

> [LINK: Hardware Sensors calibration settings](https://docs.google.com/spreadsheets/d/1-NRiG__0Ym1Sw__0UR1on3XKo4--my3V4uDAUEjcMrk/edit#gid=0)


**Hardware current sensor - with a current meter**
  1. Select "Enable ADC 5v ref"
  1. If "Enable ADC 5v ref" is changed, save and restart the OSD
  1. Either use calibration data from spreadsheet linked above (probably easier) or: 
  1. Starting at zero, increase "Zero adjust" to display correct Amps at motors off
  1. Starting at max, decrease "Amps adjust" to display correct Amps at mid - high throttle setting

**Software current sensor - with a current meter**
  1. Starting at zero, increase "Zero adjust" to display correct Amps at motors off
  1. Starting at max, decrease "Amps adjust" to display correct Amps at approx hover

**Software current sensor alternative method - with a battery charger**
  1. Change "Zero adjust" to display correct Amps at motors off - typically 0.3
  1. Change "Amps adjust" to display "approx" correct Amps at mid throttle ( try bat capacity\*8 where capacity is in Ah )
  1. Fly a complete battery and note the total mah used on the OSD and put back into the battery by your charger
  1. The correct amp adjust = old amp adjust `*` charger mah used / OSD mah used

**BI-DIRECTIONAL current sensors**
  1. If you are using a Bi-Directional current sensor, you will probably have to amend one of the lines in config.h - AMPERAGEOFFSET
  1. As  a rough guide, for a sensor with offset  1.25 use 200
  1. As  a rough guide, for a sensor with offset  2.5v use 450
  1. Actual figure should be 1024\*Voffset/Vcc where Vcc is measured value ~5v

**Maximum Amps**
  1. For optimal resolution we use an AMPERAGEMAX - set this to be 10% higher than your maximum current draw or sensor


---

## Calibration - RSSI ##

  1. If using analogue RSSI on the OSD select "Enable ADC 5v ref"
  1. If "Enable ADC 5v ref" is changed, save and restart the OSD

**Via OSD menu**
  1. With TX approx 2-3 m from the RX, select calibrate RSSI from OSD menu
  1. When timer reaches 10, turn the tx off and wait until the timer finishes counting down before turning back on
  1. Save settings
  1. Minor adjustments can be made on the GUI if necessary

**Via GUI**
  1. Set RSSI min to zero and RSSI max to 255 and write
  1. With TX off, increase RSSI min and write. Repeat until OSD reads 0% consistently
  1. With TX on, decrease RSSI max and write. Repeat until OSD reads 100% consistently



---
