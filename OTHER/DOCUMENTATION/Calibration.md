
---

## Calibration - Voltage ##

NOTE: if use Flight Controller voltage, then ALL adjustments are made on the FC. The following does not apply.

**Main or video voltage connected directly to OSD**
  1. If > 4s select "Enable, ADC 5v ref"
  1. If using analog RSSI direct to OSD, "Enable ADC 5v ref"
  1. If using current sensor direct to OSD, "Enable ADC 5v ref"
  1. If "Enable ADC 5v ref" is changed, save and restart the OSD
  1. With a voltage meter, change "Voltage adjust" and save until the voltage on the OSD matches the meter
  1. Repeat for video if appropriate

---

## Calibration - Current ##

NOTE: if use a hardware sensor connected to teh Flight Controller, then ALL adjustments are made on the FC. The following does not apply.

**Hardware current sensor - from a datasheet**
  1. Select "Enable ADC 5v ref"
  1. If "Enable ADC 5v ref" is changed, save and restart the OSD
  1. Click Amperage "CAL" on the GUI
  1. Enter the OSD onboard 5v voltage measured with a volt meter (recommended for improved accuracy) 
  1. Enter the offset voltage. (Voltage when 0A drawn)
  1. Enter the sensitvity - mV change per 1A change

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

**Software current sensor alternative method - with a battery charger**
 1. Click "WRITE" to save settings to OSD

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
  1. With TX off, click "SET" (Min)
  1. With TX on, click "SET" (Max)
  1. Remeber to "WRITE" to save to OSD

**Software current sensor alternative method - with a battery charger**
 1. Click "WRITE" to save settings to OSD

---
