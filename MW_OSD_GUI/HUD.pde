int DISPLAY_ALWAYS=      0xC000;
int DISPLAY_NEVER=       0x0000;
int DISPLAY_COND=        0x4000;
int DISPLAY_MIN_OFF=     0x8000;


// HUD 0: Default Shiki Max 
int[] CONFIG16_0 = {
LINE02+2 |DISPLAY_ALWAYS,  // GPS_numSatPosition
LINE02+22 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE02+24 |DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
LINE07+3 |DISPLAY_ALWAYS,   // speedPosition
LINE05+24 |DISPLAY_ALWAYS,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_ALWAYS,   // MwGPSAltPosition
LINE02+6 |DISPLAY_ALWAYS,   // sensorPosition
LINE04+24 |DISPLAY_ALWAYS,   // MwHeadingPosition
LINE02+10 |DISPLAY_ALWAYS,   // MwHeadingGraphPosition
LINE07+23 |DISPLAY_ALWAYS,   // MwAltitudePosition
LINE07+22 |DISPLAY_ALWAYS,   // MwClimbRatePosition
LINE12+22 |DISPLAY_ALWAYS,   // CurrentThrottlePosition
LINE13+22 |DISPLAY_ALWAYS,   // flyTimePosition
LINE13+22 |DISPLAY_ALWAYS,   // onTimePosition
LINE11+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_ALWAYS,   // MwGPSLatPosition
LINE10+15 |DISPLAY_ALWAYS,   // MwGPSLonPosition
LINE01+2 |DISPLAY_ALWAYS,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_ALWAYS,   // MwGPSLonPositionTop      // On top of screen
LINE12+3 |DISPLAY_ALWAYS,   // rssiPosition
LINE09+3 |DISPLAY_ALWAYS,   // temperaturePosition
LINE13+3 |DISPLAY_ALWAYS,  // voltagePosition
LINE11+3 |DISPLAY_ALWAYS,   // vidvoltagePosition
LINE13+9 |DISPLAY_ALWAYS,   // amperagePosition
LINE13+16 |DISPLAY_ALWAYS,   // pMeterSumPosition
LINE07+7 |DISPLAY_ALWAYS,   // horizonPosition
LINE07+7 |DISPLAY_ALWAYS,   // SideBarPosition
LINE07+7 |DISPLAY_ALWAYS,   // SideBarScrollPosition
LINE10+10 |DISPLAY_ALWAYS,   // CallSign Position
LINE08+10 |DISPLAY_ALWAYS,   // Debug Position
LINE05+2 |DISPLAY_ALWAYS,   // Gimbal Position
LINE12+11 |DISPLAY_ALWAYS,  // GPS_time Position
LINE09+22 |DISPLAY_ALWAYS,   // SportPosition
LINE04+2 |DISPLAY_ALWAYS,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+15 |DISPLAY_NEVER,   // MapCenterPosition
LINE04+10 |DISPLAY_ALWAYS,   // APstatusPosition
};

// HUD 1: Default Shiki Min
int[] CONFIG16_1 = { 
LINE02+2 |DISPLAY_NEVER,  // GPS_numSatPosition
LINE13+19 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE02+12  |DISPLAY_NEVER,   // GPS_distanceToHomePosition
LINE02+3  |DISPLAY_NEVER,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE02+6 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE02+9 |DISPLAY_NEVER,   // MwHeadingGraphPosition
LINE02+23 |DISPLAY_NEVER,   // MwAltitudePosition
LINE07+23 |DISPLAY_NEVER,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE13+22 |DISPLAY_ALWAYS,   // flyTimePosition
LINE13+22 |DISPLAY_ALWAYS,   // onTimePosition
LINE11+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_NEVER,   // MwGPSLatPosition
LINE10+15 |DISPLAY_NEVER,   // MwGPSLonPosition
LINE01+2 |DISPLAY_NEVER,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_NEVER,   // MwGPSLonPositionTop      // On top of screen
LINE12+2 |DISPLAY_NEVER,   // rssiPosition
LINE09+2 |DISPLAY_NEVER,   // temperaturePosition
LINE13+3 |DISPLAY_ALWAYS,  // voltagePosition
LINE11+3 |DISPLAY_NEVER,   // vidvoltagePosition
LINE13+13 |DISPLAY_NEVER,   // amperagePosition
LINE13+23 |DISPLAY_NEVER,   // pMeterSumPosition
LINE05+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarScrollPosition
LINE10+10 |DISPLAY_NEVER,   // CallSign Position
LINE08+10 |DISPLAY_NEVER,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE04+2 |DISPLAY_NEVER,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+15 |DISPLAY_NEVER,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

// HUD 2: EZ
int[] CONFIG16_2 = {
LINE02+19 |DISPLAY_NEVER,  // GPS_numSatPosition
LINE02+18 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE02+10 |DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
LINE02+3 |DISPLAY_ALWAYS,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE03+2 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE02+9 |DISPLAY_NEVER,   // MwHeadingGraphPosition
LINE02+23 |DISPLAY_ALWAYS,   // MwAltitudePosition
LINE03+23 |DISPLAY_NEVER,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE13+22 |DISPLAY_ALWAYS,   // flyTimePosition
LINE13+22 |DISPLAY_ALWAYS,   // onTimePosition
LINE11+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_NEVER,   // MwGPSLatPosition
LINE10+15 |DISPLAY_NEVER,   // MwGPSLonPosition
LINE01+2 |DISPLAY_NEVER,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_NEVER,   // MwGPSLonPositionTop      // On top of screen
LINE12+2 |DISPLAY_NEVER,   // rssiPosition
LINE09+2 |DISPLAY_NEVER,   // temperaturePosition
LINE13+3 |DISPLAY_ALWAYS,  // voltagePosition
LINE11+3 |DISPLAY_NEVER,   // vidvoltagePosition
LINE13+8 |DISPLAY_ALWAYS,   // amperagePosition
LINE13+16 |DISPLAY_ALWAYS,   // pMeterSumPosition
LINE05+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarScrollPosition
LINE10+10 |DISPLAY_ALWAYS,   // CallSign Position
LINE08+10 |DISPLAY_ALWAYS,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE02+8 |DISPLAY_NEVER,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+15 |DISPLAY_NEVER,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

// HUD 3: Blank screen
int[] CONFIG16_3 = {
LINE02+19 |DISPLAY_NEVER,  // GPS_numSatPosition
LINE02+18 |DISPLAY_NEVER,   // GPS_directionToHomePosition
LINE02+10 |DISPLAY_NEVER,   // GPS_distanceToHomePosition
LINE02+3 |DISPLAY_NEVER,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE03+2 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE02+9 |DISPLAY_NEVER,   // MwHeadingGraphPosition
LINE02+23 |DISPLAY_NEVER,   // MwAltitudePosition
LINE03+23 |DISPLAY_NEVER,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE13+22 |DISPLAY_NEVER,   // flyTimePosition
LINE13+22 |DISPLAY_NEVER,   // onTimePosition
LINE11+11 |DISPLAY_NEVER,   // motorArmedPosition
LINE10+2 |DISPLAY_NEVER,   // MwGPSLatPosition
LINE10+15 |DISPLAY_NEVER,   // MwGPSLonPosition
LINE01+2 |DISPLAY_NEVER,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_NEVER,   // MwGPSLonPositionTop      // On top of screen
LINE12+2 |DISPLAY_NEVER,   // rssiPosition
LINE09+2 |DISPLAY_NEVER,   // temperaturePosition
LINE13+3 |DISPLAY_NEVER,  // voltagePosition
LINE11+3 |DISPLAY_NEVER,   // vidvoltagePosition
LINE13+8 |DISPLAY_NEVER,   // amperagePosition
LINE13+16 |DISPLAY_NEVER,   // pMeterSumPosition
LINE07+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarScrollPosition
LINE10+10 |DISPLAY_NEVER,   // CallSign Position
LINE08+10 |DISPLAY_NEVER,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE02+8 |DISPLAY_NEVER,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+15 |DISPLAY_NEVER,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

// HUD 4: MAP
int[] CONFIG16_4 = {
LINE02+19 |DISPLAY_NEVER,  // GPS_numSatPosition
LINE02+19 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE02+11 |DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
LINE02+3 |DISPLAY_ALWAYS,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE03+2 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE02+9 |DISPLAY_NEVER,   // MwHeadingGraphPosition
LINE02+23 |DISPLAY_ALWAYS,   // MwAltitudePosition
LINE03+23 |DISPLAY_NEVER,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE13+22 |DISPLAY_ALWAYS,   // flyTimePosition
LINE13+22 |DISPLAY_ALWAYS,   // onTimePosition
LINE11+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_NEVER,   // MwGPSLatPosition
LINE10+15 |DISPLAY_NEVER,   // MwGPSLonPosition
LINE01+2 |DISPLAY_NEVER,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_NEVER,   // MwGPSLonPositionTop      // On top of screen
LINE12+2 |DISPLAY_NEVER,   // rssiPosition
LINE09+2 |DISPLAY_NEVER,   // temperaturePosition
LINE13+3 |DISPLAY_ALWAYS,  // voltagePosition
LINE11+3 |DISPLAY_NEVER,   // vidvoltagePosition
LINE13+8 |DISPLAY_ALWAYS,   // amperagePosition
LINE13+16 |DISPLAY_ALWAYS,   // pMeterSumPosition
LINE07+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarScrollPosition
LINE10+10 |DISPLAY_ALWAYS,   // CallSign Position
LINE08+10 |DISPLAY_ALWAYS,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE02+7 |DISPLAY_ALWAYS,   // modePosition
LINE02+17 |DISPLAY_ALWAYS,   // MapModePosition
LINE07+14 |DISPLAY_ALWAYS,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

// HUD 5: EMBRAER
int[] CONFIG16_5 = {
LINE02+2 |DISPLAY_NEVER,  // GPS_numSatPosition
LINE13+12 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE13+14 |DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
LINE04+6 |DISPLAY_ALWAYS,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE02+6 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE02+10 |DISPLAY_NEVER,   // MwHeadingGraphPosition
LINE04+20 |DISPLAY_ALWAYS,   // MwAltitudePosition
LINE07+22 |DISPLAY_ALWAYS,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE10+19 |DISPLAY_ALWAYS,   // flyTimePosition
LINE10+19 |DISPLAY_ALWAYS,   // onTimePosition
LINE09+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_ALWAYS,   // MwGPSLatPosition
LINE10+15 |DISPLAY_ALWAYS,   // MwGPSLonPosition
LINE01+2 |DISPLAY_ALWAYS,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_ALWAYS,   // MwGPSLonPositionTop      // On top of screen
LINE12+5 |DISPLAY_ALWAYS,   // rssiPosition
LINE09+3 |DISPLAY_NEVER,   // temperaturePosition
LINE10+5 |DISPLAY_ALWAYS,  // voltagePosition
LINE11+3 |DISPLAY_NEVER,   // vidvoltagePosition
LINE11+4 |DISPLAY_ALWAYS,   // amperagePosition
LINE11+19 |DISPLAY_ALWAYS,   // pMeterSumPosition
LINE07+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_ALWAYS,   // SideBarPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarScrollPosition
LINE10+10 |DISPLAY_ALWAYS,   // CallSign Position
LINE08+10 |DISPLAY_ALWAYS,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE05+3 |DISPLAY_NEVER,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+15 |DISPLAY_NEVER,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

// HUD 6: CYCLOPS
int[] CONFIG16_6 = {
LINE02+23 |DISPLAY_ALWAYS,  // GPS_numSatPosition
LINE12+14 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE13+3 |DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
LINE11+3 |DISPLAY_ALWAYS,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE02+6 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE13+10 |DISPLAY_ALWAYS,   // MwHeadingGraphPosition
LINE12+3 |DISPLAY_ALWAYS,   // MwAltitudePosition
LINE07+22 |DISPLAY_NEVER,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE13+22 |DISPLAY_ALWAYS,   // flyTimePosition
LINE13+22 |DISPLAY_ALWAYS,   // onTimePosition
LINE11+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_ALWAYS,   // MwGPSLatPosition
LINE10+15 |DISPLAY_ALWAYS,   // MwGPSLonPosition
LINE01+2 |DISPLAY_ALWAYS,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_ALWAYS,   // MwGPSLonPositionTop      // On top of screen
LINE03+25 |DISPLAY_ALWAYS,   // rssiPosition
LINE09+3 |DISPLAY_ALWAYS,   // temperaturePosition
LINE12+23 |DISPLAY_ALWAYS,  // voltagePosition
LINE09+23 |DISPLAY_ALWAYS,   // vidvoltagePosition
LINE10+22 |DISPLAY_ALWAYS,   // amperagePosition
LINE11+22 |DISPLAY_ALWAYS,   // pMeterSumPosition
LINE07+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarPosition
LINE07+7 |DISPLAY_NEVER,   // SideBarScrollPosition
LINE10+10 |DISPLAY_ALWAYS,   // CallSign Position
LINE08+10 |DISPLAY_ALWAYS,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE02+3 |DISPLAY_ALWAYS,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+15 |DISPLAY_NEVER,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

// HUD 7: SUPPOSITORY
int[] CONFIG16_7 = {
LINE02+24 |DISPLAY_ALWAYS,  // GPS_numSatPosition
LINE03+14 |DISPLAY_ALWAYS,   // GPS_directionToHomePosition
LINE13+12 |DISPLAY_ALWAYS,   // GPS_distanceToHomePosition
LINE07+3 |DISPLAY_ALWAYS,   // speedPosition
LINE05+24 |DISPLAY_NEVER,   // GPS_angleToHomePosition
LINE03+24 |DISPLAY_NEVER,   // MwGPSAltPosition
LINE02+2 |DISPLAY_NEVER,   // sensorPosition
LINE04+24 |DISPLAY_NEVER,   // MwHeadingPosition
LINE04+10 |DISPLAY_ALWAYS,   // MwHeadingGraphPosition
LINE07+23 |DISPLAY_ALWAYS,   // MwAltitudePosition
LINE07+22 |DISPLAY_ALWAYS,   // MwClimbRatePosition
LINE12+22 |DISPLAY_NEVER,   // CurrentThrottlePosition
LINE02+11 |DISPLAY_ALWAYS,   // flyTimePosition
LINE02+11 |DISPLAY_ALWAYS,   // onTimePosition
LINE11+11 |DISPLAY_ALWAYS,   // motorArmedPosition
LINE10+2 |DISPLAY_ALWAYS,   // MwGPSLatPosition
LINE10+15 |DISPLAY_ALWAYS,   // MwGPSLonPosition
LINE01+2 |DISPLAY_ALWAYS,   // MwGPSLatPositionTop      // On top of screen
LINE01+15 |DISPLAY_ALWAYS,   // MwGPSLonPositionTop      // On top of screen
LINE02+2 |DISPLAY_ALWAYS,   // rssiPosition
LINE09+3 |DISPLAY_ALWAYS,   // temperaturePosition
LINE13+23 |DISPLAY_ALWAYS,  // voltagePosition
LINE12+23 |DISPLAY_ALWAYS,   // vidvoltagePosition
LINE13+1 |DISPLAY_ALWAYS,   // amperagePosition
LINE12+2 |DISPLAY_ALWAYS,   // pMeterSumPosition
LINE07+7 |DISPLAY_NEVER,   // horizonPosition
LINE07+7 |DISPLAY_ALWAYS,   // SideBarPosition
LINE07+7 |DISPLAY_ALWAYS,   // SideBarScrollPosition
LINE10+10 |DISPLAY_ALWAYS,   // CallSign Position
LINE08+10 |DISPLAY_ALWAYS,   // Debug Position
LINE05+2 |DISPLAY_NEVER,   // Gimbal Position
LINE12+11 |DISPLAY_NEVER,  // GPS_time Position
LINE09+22 |DISPLAY_NEVER,   // SportPosition
LINE02+20 |DISPLAY_ALWAYS,   // modePosition
LINE02+16 |DISPLAY_NEVER,   // MapModePosition
LINE07+14 |DISPLAY_NEVER,   // MapCenterPosition
LINE07+15 |DISPLAY_NEVER,   // APstatusPosition
};

int CONFIGITEMS16=CONFIG16_0.length;

int[] CONFIG16 = new int[CONFIGITEMS16];
int[] SimPosn = new int[CONFIGITEMS16];
int[][] ConfigLayout= new int[4][CONFIGITEMS16];


