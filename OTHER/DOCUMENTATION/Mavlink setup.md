# How to setup for PIXHAWK / APM / MAVLINK OSD


See other guides for installing Arduino and uploading MWOSD:

https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FirmwareFlashing.md

# Step 1: prepare your config.h file

## Select the GPS chip type by removing the comments:
* //#define PIXHAWK

## Select the aircraft type:
* //#define ROTORCRAFT
* //#define FIXEDWING

# Step 2: Upload 
https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FirmwareFlashing.md

# Step 3: Change GUI speed to match OSD - usually 57600

# Step 4: Upload fontfile using GUI

# Step 5: Configure OSD using GUI

# Step 6: Configure the MAVLINK settings
  * Connect to Mission Planner
  * Go to config/tuning
  * Locate parameters in parameter list
  * Enter / verify the following parameters and save

If you are connecting OSD to telemetry 1 port
  * SERIAL1_BAUD, 57 (telemetry output at 57600)
  * SR1_EXT_STAT, 2 ( 2hz for waypoints, GPS raw, fence data, current waypoint, etc)
  * SR1_EXTRA1, 5 ( 5hz for attitude and simulation state)
  * SR1_EXTRA2, 2 ( 2hz for VFR_Hud data )
  * SR1_EXTRA3, 3 ( 3hz for AHRS, Hardware Status, Wind )
  * SR1_POSITION, 2 ( 2hz for location data )
  * SR1_RAW_SENS, 2 ( 2hz for raw imu sensor data )
  * SR1_RC_CHAN, 5 ( 5hz for radio input or radio output data )

If you are connecting OSD to telemetry 2 port:
  * SERIAL2_BAUD, 57 (telemetry output at 57600)
  * SR2_EXT_STAT, 2 ( 2hz for waypoints, GPS raw, fence data, current waypoint, etc)
  * SR2_EXTRA1, 5 ( 5hz for attitude and simulation state)
  * SR2_EXTRA2, 2 ( 2hz for VFR_Hud data )
  * SR2_EXTRA3, 3 ( 3hz for AHRS, Hardware Status, Wind )
  * SR2_POSITION, 2 ( 2hz for location data )
  * SR2_RAW_SENS, 2 ( 2hz for raw imu sensor data )
  * SR2_RC_CHAN, 5 ( 5hz for radio input or radio output data )

Notes: 
  * 57600 is default serial buad rate recommended when selecting APM / PIXHAWK in config.h
  * Baud rate in OSD and APM/PIXHAWK must match.
  * Always try 57600 and 115200 if no data
  * Use debug screen to verify: RX > 0 means there is data. PKT > 0 means there is valid data.
  * If no valid data, likely incorrect baud rate
