Step 1: - Configure OSD config.h option and upload
  * #define APM 
  * #define PIXHAWK

Step 2: - Configure the APM/PIXAWK settings to send correct telemetry values
  * Connect to Mission Planner
  * Go to configuration
  * Locate EEPROM parameters
  * Ennter / verify the following parameters

If you are connecting OSD to telmetry 1 port
  * SERIAL1_BAUD, 57 (telemetry output at 57600)
  * SR1_EXT_STAT, 2 ( 2hz for waypoints, GPS raw, fence data, current waypoint, etc)
  * SR1_EXTRA1, 5 ( 5hz for attitude and simulation state)
  * SR1_EXTRA2, 2 ( 2hz for VFR_Hud data )
  * SR1_EXTRA3, 3 ( 3hz for AHRS, Hardware Status, Wind )
  * SR1_POSITION, 2 ( 2hz for location data )
  * SR1_RAW_SENS, 2 ( 2hz for raw imu sensor data )
  * SR1_RC_CHAN, 5 ( 5hz for radio input or radio output data )

If you are connecting OSD to telmetry 2 port:
  * SERIAL2_BAUD, 57 (telemetry output at 57600)
  * SR2_EXT_STAT, 2 ( 2hz for waypoints, GPS raw, fence data, current waypoint, etc)
  * SR2_EXTRA1, 5 ( 5hz for attitude and simulation state)
  * SR2_EXTRA2, 2 ( 2hz for VFR_Hud data )
  * SR2_EXTRA3, 3 ( 3hz for AHRS, Hardware Status, Wind )
  * SR2_POSITION, 2 ( 2hz for location data )
  * SR2_RAW_SENS, 2 ( 2hz for raw imu sensor data )
  * SR2_RC_CHAN, 5 ( 5hz for radio input or radio output data )
