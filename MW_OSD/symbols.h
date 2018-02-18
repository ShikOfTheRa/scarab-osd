/*
 *  symbols.h
 *  
 *
 *
 */

#define SYM_BLANK 0X20

// Degrees Icon for HEADING/DIRECTION HOME
#define SYM_DEGREES 0XBD


// Direction arrows 
  #define SYM_ARROW_HOME 0X60
  #define SYM_ARROW_DIR 0XE0
#define SYM_ARROW_2 0X61
#define SYM_ARROW_3 0X62
#define SYM_ARROW_4 0X63
#define SYM_ARROW_EAST 0X64
#define SYM_ARROW_6 0X65
#define SYM_ARROW_7 0X66
#define SYM_ARROW_8 0X67
#define SYM_ARROW_NORTH 0X68
#define SYM_ARROW_10 0X69
#define SYM_ARROW_11 0X6A
#define SYM_ARROW_12 0X6B
#define SYM_ARROW_WEST 0X6C
#define SYM_ARROW_14 0X6D
#define SYM_ARROW_15 0X6E
#define SYM_ARROW_16 0X6F

// Heading Graphics
#define SYM_HEADING_N 0X18 
#define SYM_HEADING_S 0X19
#define SYM_HEADING_E 0X1A
#define SYM_HEADING_W 0X1B
#define SYM_HEADING_DIVIDED_LINE 0X1C
#define SYM_HEADING_LINE 0X1D

// FRSKY HUB
#define SYM_CELL0      0xF0
#define SYM_CELL1      0xF1
#define SYM_CELL2      0xF2
#define SYM_CELL3      0xF3
#define SYM_CELL4      0xF4
#define SYM_CELL5      0xF5
#define SYM_CELL6      0xF6
#define SYM_CELL7      0xF7
#define SYM_CELL8      0xF8
#define SYM_CELL9      0xF9
#define SYM_CELLA      0xFA
#define SYM_CELLB      0xFB
#define SYM_CELLC      0xFC
#define SYM_CELLD      0xFD
#define SYM_CELLE      0xFE
#define SYM_CELLF      0xC3

// Map mode
#define SYM_HOME       0x04
#define SYM_AIRCRAFT   0X05
#define SYM_RANGE_100  0x21
#define SYM_RANGE_500  0x22
#define SYM_RANGE_2500 0x23
#define SYM_RANGE_MAX  0x24
#define SYM_DIRECTION  0x72

// GPS Coordinates and Altitude
#define SYM_LAT 0xCA
#define SYM_LON 0XCB
//#define SYM_ALT 0XCC

// GPS Mode and Autopilot
#define SYM_3DFIX 0XDF 
#define SYM_HOLD 0XEF
#define SYM_G_HOME 0XFF
#define SYM_GHOME 0X9D
#define SYM_GHOME1 0X9E
#define SYM_GHOLD 0XCD
#define SYM_GHOLD1 0XCE
#define SYM_GMISSION 0XB5
#define SYM_GMISSION1 0XB6
#define SYM_GLAND 0XB7
#define SYM_GLAND1 0XB8

// Gimbal active Mode 
#define SYM_GIMBAL 0X16 
#define SYM_GIMBAL1 0X17


// Sensor´s Presence
#define SYM_ACC 0XA0
#define SYM_MAG 0XA1
#define SYM_BAR 0XA2
#define SYM_GPS 0XA3
#define SYM_MAN 0XC0
#define SYM_MAN1 0XC1
#define SYM_MAN2 0XC2
#define SYM_BARO10 0XB7
#define SYM_BARO11 0XB8
#define SYM_MAG10 0XB5
#define SYM_MAG11 0XB6

// AH Center screen Graphics
//#define SYM_AH_CENTER 0X01
#ifdef ALT_CENTER
  #define SYM_AH_CENTER_LINE 0XB0
  #define SYM_AH_CENTER 0XB1
  #define SYM_AH_CENTER_LINE_RIGHT 0XB2
#else
  #define SYM_AH_CENTER_LINE 0X26
  #define SYM_AH_CENTER 0X7E
  #define SYM_AH_CENTER_LINE_RIGHT 0XBC
#endif
#define SYM_AH_RIGHT 0X02
#define SYM_AH_LEFT 0X03
#define SYM_AH_DECORATION_UP 0XC9
#define SYM_AH_DECORATION_DOWN 0XCF


// AH Bars
#define SYM_AH_BAR9_0 0x80


// Temperature
#define SYM_TEMP_F 0X0D
#define SYM_TEMP_C 0X0E

// Batt evolution
#define SYM_BATT_FULL 0X90
#define SYM_BATT_5 0X91
#define SYM_BATT_4 0X92
#define SYM_BATT_3 0X93
#define SYM_BATT_2 0X94
#define SYM_BATT_1 0X95
#define SYM_BATT_EMPTY 0X96

// Vario
#define SYM_VARIO 0x8C

// Glidescope
#define SYM_GLIDESCOPE 0xDC

// Batt Icon´s

// Unit Icon´s (Metric)
#define SYM_MS 0X9F
#define SYM_KMH 0XA5
#define SYM_ALTM 0XA7
#define SYM_DISTHOME_M 0XBB
#define SYM_DISTHOME_KM 0XDF
#define SYM_M 0X0C
#define SYM_ALTKM 0X7A
#define SYM_KM            0x7D

// Unit Icon´s (Imperial)
#define SYM_FTS 0X99
#define SYM_MPH 0XA6
#define SYM_ALTFT 0XA8
#define SYM_DISTHOME_FT 0XB9
#define SYM_DISTHOME_MI 0XBB
#define SYM_FT 0X0D

// Voltage and amperage 
#define SYM_VOLT 0XA9
#define SYM_AMP 0X9A
#define SYM_MAH 0XA4
#define SYM_WATT 0X7B

// Flying Mode
#define SYM_ACRO 0XAE
#define SYM_ACROGY 0X98
#define SYM_ACRO1 0XAF
#define SYM_STABLE 0XAC
#define SYM_STABLE1 0XAD
#define SYM_HORIZON 0XC4
#define SYM_HORIZON1 0XC5
#define SYM_PASS 0XAA
#define SYM_PASS1 0XAB
#define SYM_AIR 0XC0
#define SYM_AIR1 0XC1
#define SYM_PLUS 0X01

// Time
#define SYM_ON_M 0X9B
#define SYM_FLY_M 0X9C
#define SYM_ON_H 0X70
#define SYM_FLY_H 0X71

// Throttle Position (%)
#define SYM_THR 0XC8

// Menu cursor
#define SYM_CURSOR SYM_AH_LEFT

//Misc
#define SYM_DASH  0X2D
#define SYM_COLON 0X3A

//sport
#define SYM_MIN 0xB3
#define SYM_AVG 0xB4

//dispaly items lead indicator icons
#ifdef ICON_ANGLE_RTH
#define SYM_ANGLE_RTH     0XF2
#else
#define SYM_ANGLE_RTH     0
#endif

#ifdef ICON_ANGLE_HDG
#define SYM_ANGLE_HDG     0XF3
#else
#define SYM_ANGLE_HDG     0
#endif

#ifdef ICON_DOP
#define SYM_DOP           0XF4
#else
#define SYM_DOP           0
#endif

#ifdef ICON_SPEED_GPS
#define SYM_SPEED_GPS     0XF5 
#else
#define SYM_SPEED_GPS     0 
#endif

#ifdef ICON_SPEED_AIR
#define SYM_SPEED_AIR     0XF6 //GPS +1
#else
#define SYM_SPEED_AIR     0
#endif

#ifdef ICON_MAX
#define SYM_MAX           0XF7 
#else
#define SYM_MAX           0 
#endif

#ifdef ICON_CLIMBRATE
#define SYM_CLIMBRATE     0XF8 
#else
#define SYM_CLIMBRATE     0 
#endif

#ifdef ICON_EFF
#define SYM_EFF           0XF9 
#else
#define SYM_EFF           0 
#endif

#ifdef ICON_PITCH
#define SYM_PITCH         0XFA 
#else
#define SYM_PITCH         0 
#endif

#ifdef ICON_ROLL
#define SYM_ROLL          0XFB 
#else
#define SYM_ROLL          0 
#endif

#ifdef ICON_POWER
#define SYM_POWER         0XFC 
#else
#define SYM_POWER         0 
#endif

#ifdef ICON_AVG_EFF
#define SYM_AVG_EFF       0XFD 
#else
#define SYM_AVG_EFF       0 
#endif

#ifdef ICON_TOTAL
#define SYM_TOTAL         0XFE 
#else
#define SYM_TOTAL         0 
#endif

#ifdef ICON_TMP
#define SYM_TMP           0X06 
#else
#define SYM_TMP           0 
#endif

#ifdef ICON_DTH
#define SYM_DTH           0X7A
#else
#define SYM_DTH           0
#endif

#ifdef ICON_ALT
#define SYM_ALT           0X3F
#else
#define SYM_ALT           0
#endif

#ifdef ICON_GPS_ALT
#define SYM_GPS_ALT       0XBE
#else
#define SYM_GPS_ALT       0
#endif

#ifdef ICON_MAIN_BATT
#define SYM_MAIN_BATT 0X97
#else
#define SYM_MAIN_BATT     0
#endif

#ifdef ICON_VID_BAT
#define SYM_VID_BAT       0XBF
#else
#define SYM_VID_BAT       0
#endif

#ifdef ICON_RSSI
#define SYM_RSSI          0XBA
#else
#define SYM_RSSI          0
#endif

#ifdef ICON_SAT
#define SYM_SAT_L 0X1E
#define SYM_SAT_R 0X1F
#else
#define SYM_SAT           0XF0
#endif



