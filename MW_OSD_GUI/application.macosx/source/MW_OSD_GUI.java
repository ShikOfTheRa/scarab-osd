import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.Serial; 
import controlP5.*; 
import java.io.File; 
import java.lang.*; 
import javax.swing.SwingUtilities; 
import javax.swing.JFileChooser; 
import javax.swing.filechooser.FileFilter; 
import javax.swing.JOptionPane; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.FileOutputStream; 
import java.io.FileInputStream; 
import java.util.*; 
import java.io.FileNotFoundException; 
import java.text.DecimalFormat; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class MW_OSD_GUI extends PApplet {


/*
MultiWii NG OSD ...

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see http://www.gnu.org/licenses/

This work is based on the following open source work :-
 Rush OSD Development GUI  https://code.google.com/p/rush-osd-development/

And to a lesser extent code from the following :-
 Rushduino                 http://code.google.com/p/rushduino-osd/
 Minim OSD                 https://code.google.com/p/arducam-osd/wiki/minimosd

 Its base is taken from "Rush OSD Development" R370

 All credit and full acknowledgement to the incredible work and hours from the many developers, contributors and testers that have helped along the way.
 Jean Gabriel Maurice. He started the revolution. He was the first....
*/

 



 // serial library
 // controlP5 library

 // for efficient String concatemation
 // required for swing and EDT
// Saving dialogue
 // for our configuration file filter "*.mwi"
 // for message dialogue
//import java.io.BufferedReader;


 






//added new imports to support proccessing 2.0b7



String MW_OSD_GUI_Version = "MWOSD R1.4 - NextGeneration";
int MW_OSD_EEPROM_Version = 9;


int  GPS_numSatPosition = 0;
int  GPS_directionToHomePosition = 1;
int  GPS_distanceToHomePosition = 2;
int  speedPosition = 3;
int  GPS_angleToHomePosition = 4;
int  MwGPSAltPosition = 5;
int  sensorPosition = 6;
int  MwHeadingPosition = 7;
int  MwHeadingGraphPosition = 8;
int  MwAltitudePosition = 9;
int  MwClimbRatePosition = 10;
int  CurrentThrottlePosition = 11;
int  flyTimePosition = 12;
int  onTimePosition = 13;
int  motorArmedPosition = 14;
int  MwGPSLatPosition = 15;
int  MwGPSLonPosition = 16;
int  MwGPSLatPositionTop = 17;
int  MwGPSLonPositionTop = 18;
int  rssiPosition = 19;
int  temperaturePosition = 20;
int  voltagePosition = 21;
int  vidvoltagePosition = 22;
int  amperagePosition = 23;
int  pMeterSumPosition = 24;
int  horizonPosition = 25;
int  SideBarPosition =26; 
int  SideBarScrollPosition = 27;
int  callSignPosition = 28;
int  debugPosition = 29;
int  gimbalPosition = 30;
int  GPS_timePosition = 31;
int  SportPosition = 32;
int  ModePosition = 33;
int  MapModePosition = 34;
int  MapCenterPosition = 35;
int  APstatusPosition = 36;

int MSP_sendOrder =0;
PImage img_Clear,GUIBackground,OSDBackground,DONATEimage,RadioPot;

int readcounter=0;
// ScreenType---------- NTSC = 0, PAL = 1 ---------------------------------
int ScreenType = 0;

int TIMEBASE_X1 =  50;
int TIMEBASE = TIMEBASE_X1;
int LINE  =    30;
int LINE01 =   0;
int LINE02  =  30;
int LINE03  =  60;
int LINE04  =  90;
int LINE05  =  120;
int LINE06  =  150;
int LINE07  =  180;
int LINE08  =  210;
int LINE09  =  240;
int LINE10  =  270;
int LINE11  =  300;
int LINE12  =  330;
int LINE13  =  360;
int LINE14  =  390;
int LINE15  =  420;
int LINE16  =  450;
int TestLine = 300;

int TOPSHIFT = 0;

 
  
int DisplayWindowX = 681; //500;
int DisplayWindowY = 5; //5;
int WindowAdjX = -0; //15
int WindowAdjY = -0;
int WindowShrinkX = 8;
int WindowShrinkY = 48;
//image(OSDBackground,DisplayWindowX+WindowAdjX, DisplayWindowY+WindowAdjY, 469-WindowShrinkX, 300-WindowShrinkY); //529-WindowShrinkX, 360-WindowShrinkY);

int currentCol = 0;
int currentRow = 0;  
//Boolean SimulateMW = true;

int S16_AMPMAX = 0; //16bit from 8 EEPROM values

ControlP5 controlP5;
ControlP5 SmallcontrolP5;
ControlP5 ScontrolP5;
ControlP5 FontGroupcontrolP5;
ControlP5 GroupcontrolP5;
Textlabel txtlblWhichcom,txtlblWhichbaud,txtmessage; 
Textlabel txtlblLayoutTxt,txtlblLayoutEnTxt, txtlblLayoutHudTxt; 
Textlabel txtlblLayoutTxt2,txtlblLayoutEnTxt2, txtlblLayoutHudTxt2; 
ListBox commListbox,baudListbox;

//char serialBuffer[] = new char[128]; // this hold the imcoming string from serial O string
//String TestString = "";
///String SendCommand = "";




boolean PortRead = false;
boolean PortWrite = false;
int PortReadtimer = 0;
int ReadConfig = 0;
int ReadMillis = 0;
int WriteConfig = 0;
int WriteMillis = 0;
int linktimer = 0;
ControlGroup messageBox;
Textlabel MessageText;
int LEWvisible=0;
// XML config variables
int DISPLAY_STATE;
int hudsavailable=8;
int hudoptions;
int hudnamelength;
int[][] hudenable;
String[] hudpositiontext;
int xmlloaded=0;
XML xml;
int[][] CONFIGHUD;
int[][] CONFIGHUDEN;
String[] CONFIGHUDTEXT;
String[] CONFIGHUDNAME;
int eeaddressGUI=0;
int eedataGUI=0;
int eeaddressOSD=0;
int eedataOSD=0;
int ReadConfigMSPMillis=0;
int WriteConfigMSPMillis=0;


// XML config editorvariables
int hudeditposition=0;


int[] SimPosn;
int[][] ConfigLayout;
int[] EElookuptable= new int[512];
int WriteLayouts=0;
int OSD_S_HUDSW0=0;
int OSD_S_HUDSW1=0;
int OSD_S_HUDSW2=0;
int EepromWriteSize = 0;

//int[] readcheck;
int readerror=0;
// Int variables
String OSname = System.getProperty("os.name");
String LoadPercent = "";
String CallSign = "";
String Title;
int Passthroughcomm;
int AutoSimulator=0;
int StartupMessage=0;

int init_com = 0;
int commListMax = 0;
int whichKey = -1;  // Variable to hold keystoke values
int inByte = -1;    // Incoming serial data
int[] serialInArray = new int[3];    // Where we'll put what we receive
int[] debug = new int[4];    
String progresstxt="";
int xcolor=20;  


int serialCount = 0;                 // A count of how many bytes we receive
int ConfigEEPROM = -1;
int ConfigVALUE = -1;

// Box locations -------------------------------------------------------------------------
int Col1Width = 180;        int Col2Width = 200;    int Col3Width = 165;

int windowsX    = 1041+Col3Width+5;       int windowsY    =578;        //995; //573;
//int windowsX    = 1041;       int windowsY    =578;        //995; //573;
//int windowsX    = 1200;       int windowsY    =800;        //995; //573;
int xGraph      = 10;         int yGraph      = 35;
int xObj        = 520;        int yObj        = 293; //900,450
int xCompass    = 920;        int yCompass    = 341; //760,336
int xLevelObj   = 920;        int yLevelObj   = 80; //760,80
int xParam      = 120;        int yParam      = 5;
int xRC         = 690;        int yRC         = 10; //850,10
int xMot        = 690;        int yMot        = 155; //850,155
int xButton     = 845;        int yButton     = 231; //685,222
int xBox        = 415;        int yBox        = 10;
//int xGPS        = 853;        int yGPS        = 438; //693,438
int XSim        = DisplayWindowX+WindowAdjX+10;        int YSim        = 305-WindowShrinkY + 95;

//DisplayWindowX+WindowAdjX, DisplayWindowY+WindowAdjY, 360-WindowShrinkX, 288-WindowShrinkY);

int XEEPROM    = 120;        int YEEPROM    = 5;  //hidden do not remove
int XBoard     = 120;        int YBoard   = 5;
int XRSSI      = 120;        int YRSSI    = 317;
int XVREF      = 120;        int YVREF    = 444;
int XVolts     = 120;        int YVolts    = 5;
int XAmps      = 120;        int YAmps    = 190;
int XVVolts    = 120;        int YVVolts  = 114;
int XTemp      = 510;        int YTemp    = 268;
int XCS        = 120;        int YCS    = 486;
int XGPS       = 510;        int YGPS    = 5;
int XCOMPASS   = 510;        int YCOMPASS    = 98;
//int XGPS       = 305;        int YGPS    = 5;
int XTIME      = 510;        int YTIME    = 190;
//int XTIME      = 510;        int YTIME    = 5;
int XHUD       = 305;        int YHUD     = 240;
int XDisplay     = 305;        int YDisplay   = 114; //48;
int XSPORT      = 810;        int YSPORT    = 357;

int XOther     = 305;        int YOther   = 5; //48;
//int XOther     = 305;        int YOther   = 150; //48;
int XPortStat  = 5;          int YPortStat = 20;
int XDebug     = 5;          int YDebug    = 271;
int XFONTTOOLS = 5;          int YFONTTOOLS    = 395;
int XOSD_CONTROLS = 5;       int YOSD_CONTROLS    = 485;
int XSAVE_LOAD = 5;       int YSAVE_LOAD    = 328;
int XMESSAGE = 500;       int YMESSAGE    = 200;
int XLINKS = 690;       int YLINKS    = 275;

int XControlBox= 5;          int YControlBox   = 450;  //389
int XRCSim     = XSim;       int YRCSim = 30;


String FontFileName = "data/default.mcm";
int BaudRate = 115200;

//File FontFile;
int activeTab = 1;
int xx=0;
int YLocation = 0;
int Roll = 0;
int Pitch = 0;
int confmillis = 1000;
int csmillis = 1000;
int confCheck = 0;
int resmillis = 5000;
int resCheck = 1;
int OnTimer = 0;
int FlyTimer = 0;
float SimItem0= 0;
int Armed = 0;
int Showback = 1;
int del = 0;
int armedangle=0;
int oldwpmillis;
int wpno; 

// int variables

// For Heading
char[] headGraph={
  0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d,0x1c,0x1d,0x18,0x1d,0x1c,0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b};

static int MwHeading=0;
char MwHeadingUnitAdd=0xbd;


String[] ConfigNames = {
  "EEPROM Loaded",
  "RSSI Min",
  "RSSI Max",
  "RSSI Alarm",
  "Display RSSI",
  "Use FC RSSI",
  "Use PWM",
  "Display Voltage",
  "Voltage Alarm",
  "Battery Cells",
  "Voltage Adjust",
  "Use FC main voltage",
  "Display Amps",
  "Use FC amperage",
  "Display mAh",
  "Use Virtual Sensor",
  "Amps Adjust",
  "Display Video Voltage",
  "Voltage Adjust",
  "Use FC video voltage",
  "x100 mAh Alarm",
  "Amp Alarm",
  "Display GPS",
  " - GPS Coords",
  " - Coords on Top",
  " - GPS Altitude",
  "Display Angle to Home",
  "Display Heading",
  " - Heading 360",
  "Units",
  "Video Signal",
  "Display Throttle Position",
  "Display Horizon Bar",
  "Display Side Bars",
  "Display Battery Status",
  "Reset Stats After Arm",
  " - Map mode",
  "Enable ADC 5v ref",
  "Use BoxNames",
  "Display Flight Mode",
  "Display CallSign",
  "Display GPS time",
  "Time Zone +/-",
  "Time Zone offset",
  "Debug",
  " - SB Scrolling",
  "Display Gimbal",
  "Display Vario",
  "Display BARO ALT",
  "Display Compass",
  " - HB Elevation",
  "Display Timer",
  " - FM sensors",  
  " - SB direction",  
  "Zero Adjust",
  "Amperage 16L",  
  "Amperage 16H",  
  "RC Switch",  
  "RC Switch ch",  
  "HUD layout Low",  
  "HUD layout High",  
  "HUD layout Mid",  
  "x100 Distance alarm",  
  "x10   Altitude alarm",  
  "Speed alarm",  
  "Timer alarm",  
  "S_CS0",
  "S_CS1",
  "S_CS2",
  "S_CS3",
  "S_CS4",
  "S_CS5",
  "S_CS6",
  "S_CS7",
  "S_CS8",
  "S_CS9",
};

String[] ConfigHelp = {
  "EEPROM Loaded",
  "RSSI Min",
  "RSSI Max",
  "RSSI Alarm",
  "Display RSSI",
  "Use MWii",
  "Use PWM",
  "Display Voltage",
  "Voltage Alarm",
  "Battery Cells",
  "Voltage Adjust",
  "Use MWii",
  "Display Amps",
  "Use MWii",
  "Display mAh",
  "Use Virtual Sensor",
  "Amps Adjust",
  "Display Video Voltage",
  "Voltage Adjust",
  "Use MWii",
  "mAh Alarm",
  "Amp Alarm",
  "Display GPS",
  " - GPS Coords",
  " - Coords on Top",
  " - GPS Altitude",
  "Display Angle to Home",
  "Display Heading",
  " - Heading 360",  
  "Units",
  "Video Signal",
  "Display Throttle Position",
  "Display Horizon Bar",
  "Display Side Bars",
  "Display Battery Status",
  "Reset Stats After Arm",
  "Enable Map mode",
  "Enable ADC 5v ref",
  "Use BoxNames",
  "Display Flight Mode",  
  "Display CallSign",
  "Display GPS time",
  "Time Zone +/-",
  "Time Zone offset",
  "Debug",
  " - SB Scrolling",
  "Display Gimbal",
  "Display Vario",
  "Display BARO ALT",
  "Display Compass",
  " - HB Elevation",
  "Display Timer",
  " - FM sensors",  
  " - SB direction",  
  "Zero Adjust",
  "Amperage 16L",  
  "Amperage 16H",  
  "RC Switch",  
  "RC Switch ch",  
  "HUD layout Low",  
  "HUD layout High",  
  "HUD layout Mid",  
  "Distance alarm",  
  "Altitude alarm",  
  "Speed alarm",  
  "Timer alarm",  
  "S_CS0",
  "S_CS1",
  "S_CS2",
  "S_CS3",
  "S_CS4",
  "S_CS5",
  "S_CS6",
  "S_CS7",
  "S_CS8",
  "S_CS9",
  };




static int CHECKBOXITEMS=0;
int CONFIGITEMS=ConfigNames.length;
static int SIMITEMS=6;

//     System.out.println("MSP: ");

  
int[] ConfigRanges = {
1,   // used for check             0

255,   // S_RSSIMIN                1
255,   // S_RSSIMAX                2
100,   // S_RSSI_ALARM             3
1,     // S_DISPLAYRSSI            4
1,     // S_MWRSSI                 5
1,     // S_PWMRSSI                6

1,     // S_DISPLAYVOLTAGE         7
255,   // S_VOLTAGEMIN             8
6,     // S_BATCELLS               9
255,   // S_DIVIDERRATIO           10
1,     // S_MAINVOLTAGE_VBAT       11

1,     // S_AMPERAGE,              12
1,     // S_MWAMPERAGE,              12a
1,     // S_AMPER_HOUR,            13
1,     // S_AMPERAGE_VIRTUAL,
1023,   // S_AMPDIVIDERRATIO,      // note this is 8>>16 bit EPROM var

1,     // S_VIDVOLTAGE             14
255,   // S_VIDDIVIDERRATIO        15    
1,     // S_VIDVOLTAGE_VBAT        16

10000,     // S_AMPER_HOUR_ALARM       17
255,   // S_AMPERAGE_ALARM         18

//1,     // S_BOARDTYPE              19

1,     // S_DISPLAYGPS             20
1,     // S_COORDINATES            21
1,     // S_GPSCOORDTOP            22
1,     // S_GPSALTITUDE            23
1,     // S_ANGLETOHOME            24
1,     // S_SHOWHEADING            25
1,     // S_HEADING360             26

1,     // S_UNITSYSTEM             27
1,     // S_SCREENTYPE             28
1,     // S_THROTTLEPOSITION       29
1,     // S_DISPLAY_HORIZON_BR     30
1,     // S_WITHDECORATION         31
1,     // S_SHOWBATLEVELEVOLUTION  32
1,     // S_RESETSTATISTICS        33
4,     // S_MAPMODE              34 //map mode
1,     // S_VREFERENCE,
1,     // S_USE_BOXNAMES           35
1,     // S_MODEICON               36

1,     // call sign                37
1,     // GPStime                  37a
1,     // GPSTZ +/-                37b
13,    // GPSTZ                    37c
//60,    // GPSDS                    37d
1,     // Debug                    37e
1,     // S_SCROLLING              37f
1,     // S_GIMBAL                 37g
1,     // S_VARO                   37h
1,     // SHOW BAROALT             38h
1,     // SHOW COMPASS             39h
1,     // SHOW HORIZON ELEVATION   40h
1,     // S_TIMER                  41h
1,     // S_MODESENSOR             42h
1,     //S_SIDEBARTOPS             43h
1023,  // S_AMPMIN,
255,   // S_AMPMAXL,
3,     // S_AMPMAXH,
1,     // S_RCWSWITCH,
7,     // S_RCWSWITCH_CH,
7,     // S_HUDSW0,
7,     // S_HUDSW1,
7,     // S_HUDSW2,
255,   //S_DISTANCE_ALARM,
255,   //S_ALTITUDE_ALARM,
255,   //S_SPEED_ALARM,
255,   //S_TIMER_ALARM,
255,
255,
 255,
 255,
 255,
 255,
 255,
 255,
 255,
 255,

};


String[] SimNames= {
  "Armed:",
  "Acro/Stable:",
  "Bar Mode:",
  "Mag Mode:",
  "GPS Home:",
  "GPS Hold:",
  "Sim 6:",
  "Sim 7:",
  "Sim 8:",
  "Sim 9:",
  "Sim 10:"
};
  
  
  int[] SimRanges = {
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  255,
  1,
  1,
  1};


PFont font8,font9,font10,font11,font12,font15;

//Colors--------------------------------------------------------------------------------------------------------------------
int yellow_ = color(200, 200, 20), 
      green_ = color(30, 120, 30), 
      red_ = color(120, 30, 30), 
      blue_ = color(50, 50, 100),
      grey_ = color(30, 30, 30),
      black_ = color(0, 0, 0),
      switches_ = color(30, 140, 30),
      font_ = color(50, 50, 50),
      clear_ = color(0, 0, 0),
      donateback_ = color(180, 100, 0),
      donatefront_ = color(50, 50, 255),
      osdcontr_ = color(50, 50, 50)
      ;
//Colors--------------------------------------------------------------------------------------------------------------------

// textarea -------------------------------------------------------------------------------------------------------------
//Textarea
Textarea myTextarea;

// textlabels -------------------------------------------------------------------------------------------------------------
Textlabel txtlblconfItem[] = new Textlabel[CONFIGITEMS] ;
Textlabel txtlblSimItem[] = new Textlabel[SIMITEMS] ;
Textlabel FileUploadText, TXText, RXText;
// textlabels -------------------------------------------------------------------------------------------------------------

// Buttons------------------------------------------------------------------------------------------------------------------
Button buttonIMPORT,buttonSAVE,buttonREAD,buttonRESET,buttonWRITE,buttonRESTART, buttonGPSTIMELINK, buttonSPORTLINK;
Button buttonLUP, buttonLDOWN, buttonLLEFT, buttonLRIGHT, buttonLPOSUP, buttonLPOSDOWN;
Button buttonLHUDUP,buttonLPOSHUDDOWN,buttonLPOSEN, buttonLSET, buttonLADD, buttonLSAVE, buttonLCANCEL;
Button buttonLEW;
Button buttonGUIDELINK, buttonFAQLINK, buttonCALIBLINK, buttonSUPPORTLINK, buttonDONATELINK;
// Buttons------------------------------------------------------------------------------------------------------------------

// Toggles------------------------------------------------------------------------------------------------------------------
Toggle toggleConfItem[] = new Toggle[CONFIGITEMS] ;
// Toggles------------------------------------------------------------------------------------------------------------------    

// checkboxes------------------------------------------------------------------------------------------------------------------
CheckBox checkboxConfItem[] = new CheckBox[CONFIGITEMS];

// Toggles------------------------------------------------------------------------------------------------------------------    
RadioButton RadioButtonConfItem[] = new RadioButton[CONFIGITEMS] ;
RadioButton R_PortStat;

//  number boxes--------------------------------------------------------------------------------------------------------------

Numberbox confItem[] = new Numberbox[CONFIGITEMS] ;
int readcheck[] = new int[CONFIGITEMS] ;
//Numberbox SimItem[] = new Numberbox[SIMITEMS] ;
//  number boxes--------------------------------------------------------------------------------------------------------------

Group 
  MGUploadF,
  LEW,
  G_LINKS,
  G_MESSAGE,
  SAVE_LOAD,
  OSD_CONTROLS,
  G_EEPROM,
  G_RSSI,
  G_Voltage,
  G_Amperage,
  G_VVoltage,
  G_Alarms,
  G_Debug,
  G_Board,
  G_GPS,
  G_Other,
  G_CallSign,
  G_PortStatus,
  G_TIME,
  G_VREF,
  G_HUD,
  G_RCSWITCH,
  G_COMPASS,
  G_DISPLAY,
  G_SPORT,
  G_INFO
  ;

// Timers --------------------------------------------------------------------------------------------------------------------
//ControlTimer OnTimer,FlyTimer;

public controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}



public void setup() {
  size(windowsX,windowsY);




//  LoadConfig();
//  xml = loadXML("HUDLAYOUT.xml");
  xml = loadXML("hudlayout.xml");
//  xml = loadXML(dataPath("hudlayout.xml"));
  initxml();
 
//Map<Settings, String> table = new EnumMap<Settings>(Settings.class);
OnTimer = millis();
  frameRate(30); 
OSDBackground = loadImage("OSD_def.jpg");
GUIBackground = loadImage("GUI_def.jpg");
DONATEimage  = loadImage("DON_def.png");
//RadioPot = loadImage("MWImage.jpg");
//PGraphics icon = createGraphics(16, 16, P3D);
//icon.beginDraw();
//icon.beginShape();
//icon.texture(RadioPot);
//icon.endShape();
//icon.endDraw();
//frame.setIconImage(icon.image);
  font8 = createFont("Arial bold",8,false);
  font9 = createFont("Arial bold",10,false);
  font10 = createFont("Arial bold",11,false);
  font11 = createFont("Arial bold",11,false);
  font12 = createFont("Arial bold",12,false);
  font15 = createFont("Arial bold",15,false);

  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(font10);
  //controlP5.setAutoDraw(false);
  

  SmallcontrolP5 = new ControlP5(this); // initialize the GUI controls
  SmallcontrolP5.setControlFont(font9); 
  //SmallcontrolP5.setAutoDraw(false); 
 
  ScontrolP5 = new ControlP5(this); // initialize the GUI controls
  ScontrolP5.setControlFont(font10);  
  //ScontrolP5.setAutoDraw(false);  
 
 
 
 //FontGroupcontrolP5.setAutoDraw(false); 
 
 
  GroupcontrolP5 = new ControlP5(this); // initialize the GUI controls
  GroupcontrolP5.setControlFont(font10);
  //GroupcontrolP5.setColorForeground(color(30,255));
  //GroupcontrolP5.setColorBackground(color(30,255));
  //GroupcontrolP5.setColorLabel(color(0, 110, 220));
  //GroupcontrolP5.setColorValue(0xffff88ff);
  //GroupcontrolP5.setColorActive(color(30,255));
  //GroupcontrolP5.setAutoDraw(false);
  FontGroupcontrolP5 = new ControlP5(this); // initialize the GUI controls
  FontGroupcontrolP5.setControlFont(font10);

  SetupGroups();
  
  //Textarea
  myTextarea = controlP5.addTextarea("txt")
    .setPosition(DisplayWindowX+WindowAdjX+10+10, DisplayWindowY+WindowAdjY+10)
    .setSize(325, 228)
    .setFont(createFont("", 12))
    .setLineHeight(14)
    .setColor(color(255))
    .setColorBackground(black_)
  ;


// BAUD RATE / COM PORT SELECTION ---------------------------------------
  baudListbox = controlP5.addListBox("portBaudList",5,116,110,260); // make a listbox and populate it with the available comm ports
  baudListbox.setItemHeight(15);
  baudListbox.setBarHeight(15);
  baudListbox.captionLabel().set("BAUD");
  baudListbox.setColorBackground(red_);
  baudListbox.addItem("115200",0);
  baudListbox.addItem("57600",1);
  baudListbox.addItem("38400",2);
  baudListbox.addItem("19200",3);
//  baudListbox.setValue(1);
  baudListbox.close();
  txtlblWhichbaud = controlP5.addTextlabel("txtlblWhichbaud","Baud rate: "+str(BaudRate),5,37).setGroup(G_PortStatus); // textlabel(name,text,x,y)


// COM PORT SELECTION ---------------------------------------

  commListbox = controlP5.addListBox("portComList",5,99,110,260); // make a listbox and populate it with the available comm ports
  commListbox.setItemHeight(15);
  commListbox.setBarHeight(15);
  commListbox.captionLabel().set("COM SETTINGS");
  commListbox.setColorBackground(red_);
  for(int i=0;i<Serial.list().length;i++) {
    String pn = shortifyPortName(Serial.list()[i], 13);
    if (pn.length() >0 ) commListbox.addItem(pn,i); // addItem(name,value)
    commListMax = i;
  }
  commListbox.addItem("Close Comm",++commListMax); // addItem(name,value)
  //commListbox.addItem("Pass Thru Comm",commListMax+1); // addItem(name,value)
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",5,22).setGroup(G_PortStatus); // textlabel(name,text,x,y)
  txtmessage = controlP5.addTextlabel("txtmessage","",3,295); // textdebug

// BUTTONS SELECTION ---------------------------------------
  
  buttonSAVE = controlP5.addButton("bSAVE",1,20,5,60,16); buttonSAVE.setLabel("    SAVE").setGroup(SAVE_LOAD).setColorBackground(osdcontr_); 
  buttonIMPORT = controlP5.addButton("bIMPORT",1,20,25,60,16); buttonIMPORT.setLabel("    LOAD").setGroup(SAVE_LOAD).setColorBackground(osdcontr_);   
  buttonREAD = controlP5.addButton("READEEMSP",1,20,5,60,16);buttonREAD.setColorBackground(osdcontr_).setGroup(OSD_CONTROLS).setLabel("    READ");
  buttonWRITE = controlP5.addButton("WRITEEEMSP",1,20,25,60,16);buttonWRITE.setColorBackground(osdcontr_).setGroup(OSD_CONTROLS).setLabel("   WRITE");
  buttonRESET = controlP5.addButton("DEFAULT",1,20,45,60,16);buttonRESET.setColorBackground(osdcontr_).setGroup(OSD_CONTROLS).setLabel(" DEFAULT");
  buttonRESTART = controlP5.addButton("RESTART",1,20,65,60,16);buttonRESTART.setColorBackground(osdcontr_).setGroup(OSD_CONTROLS).setLabel(" RESTART");
  buttonLEW = controlP5.addButton("LEW",1,30,(6*17),92,16);buttonLEW.setColorBackground(osdcontr_).setGroup(G_RCSWITCH).setLabel("Layout Editor");
    

// EEPROM----------------------------------------------------------------

CreateItem(GetSetting("S_CHECK_"), 5, 0, G_EEPROM);
CreateItem(GetSetting("S_AMPMAXL"), 5, 0, G_EEPROM);
CreateItem(GetSetting("S_AMPMAXH"), 5, 0, G_EEPROM);
CreateItem(GetSetting("S_USE_BOXNAMES"),  5,0, G_EEPROM);
CreateItem(GetSetting("S_GPSCOORDTOP"),  5,0, G_EEPROM);


// RSSI  ---------------------------------------------------------------------------

CreateItem(GetSetting("S_DISPLAYRSSI"), 5, 0, G_RSSI);
CreateItem(GetSetting("S_MWRSSI"),  5,1*17, G_RSSI);
CreateItem(GetSetting("S_PWMRSSI"),  5,2*17, G_RSSI);
CreateItem(GetSetting("S_RSSIMIN"), 5, 3*17, G_RSSI);
CreateItem(GetSetting("S_RSSIMAX"), 5,4*17, G_RSSI);
CreateItem(GetSetting("S_RSSI_ALARM"), 5,5*17, G_RSSI);

// Voltage  ------------------------------------------------------------------------

CreateItem(GetSetting("S_DISPLAYVOLTAGE"), 5,0, G_Voltage);
CreateItem(GetSetting("S_MAINVOLTAGE_VBAT"), 5,1*17, G_Voltage);
CreateItem(GetSetting("S_DIVIDERRATIO"), 5,2*17, G_Voltage);
CreateItem(GetSetting("S_BATCELLS"), 5,3*17, G_Voltage);
CreateItem(GetSetting("S_VOLTAGEMIN"), 5,4*17, G_Voltage);
  confItem[GetSetting("S_VOLTAGEMIN")].setDecimalPrecision(1);
  confItem[GetSetting("S_VOLTAGEMIN")].setMultiplier(0.1f);
  
// Amperage  ------------------------------------------------------------------------
CreateItem(GetSetting("S_AMPERAGE"),  5,0, G_Amperage);
CreateItem(GetSetting("S_AMPER_HOUR"),  5,1*17, G_Amperage);
CreateItem(GetSetting("S_AMPERAGE_VIRTUAL"),  5,2*17, G_Amperage);
CreateItem(GetSetting("S_MWAMPERAGE"),  5,3*17, G_Amperage);
CreateItem(GetSetting("S_AMPDIVIDERRATIO"),  5,4*17, G_Amperage);
CreateItem(GetSetting("S_AMPMIN"), 5, 5*17, G_Amperage);
CreateItem(GetSetting("S_AMPER_HOUR_ALARM"),  5,6*17, G_Amperage);
CreateItem(GetSetting("S_AMPERAGE_ALARM"),  5,7*17, G_Amperage);

// Video Voltage  ------------------------------------------------------------------------
CreateItem(GetSetting("S_VIDVOLTAGE"),  5,0, G_VVoltage);
CreateItem(GetSetting("S_VIDVOLTAGE_VBAT"),  5,1*17, G_VVoltage);
CreateItem(GetSetting("S_VIDDIVIDERRATIO"),  5,2*17, G_VVoltage);

//  Temperature  --------------------------------------------------------------------
//CreateItem(GetSetting("S_DISPLAYTEMPERATURE"),  5,0, G_Alarms);
//CreateItem(GetSetting("S_AMPERAGE_ALARM"),  5,1*17, G_Alarms);

//  Debug  --------------------------------------------------------------------
CreateItem(GetSetting("S_DEBUG"),  5,0, G_Debug);

//  Board ---------------------------------------------------------------------------
//CreateItem(GetSetting("S_BOARDTYPE"),  5,0, G_Board);
//BuildRadioButton(GetSetting("S_BOARDTYPE"),  5,0, G_Board, "Rush","Minim");


//  GPS  ----------------------------------------------------------------------------
CreateItem(GetSetting("S_DISPLAYGPS"), 5,0, G_GPS);
CreateItem(GetSetting("S_COORDINATES"),  5,1*17, G_GPS);
CreateItem(GetSetting("S_GPSALTITUDE"),  5,2*17, G_GPS);
CreateItem(GetSetting("S_MAPMODE"),      5,3*17, G_GPS);
//  HUD  ----------------------------------------------------------------------------
//CreateItem(GetSetting("S_HUD"),  5,0*17, G_HUD);
//CreateItem(GetSetting("S_HUDOSDSW"),  5,1*17, G_HUD);
CreateItem(GetSetting("S_DISPLAY_HORIZON_BR"),  5,0*17, G_HUD);
CreateItem(GetSetting("S_HORIZON_ELEVATION"),  5,1*17, G_HUD);
CreateItem(GetSetting("S_WITHDECORATION"),  5,2*17, G_HUD);
CreateItem(GetSetting("S_SCROLLING"),  5,3*17, G_HUD);
CreateItem(GetSetting("S_SIDEBARTOPS"),  5,4*17, G_HUD);
//  RCSWITCH  ----------------------------------------------------------------------------
CreateItem(GetSetting("S_RCWSWITCH"),  5,0*17, G_RCSWITCH);
CreateItem(GetSetting("S_RCWSWITCH_CH"),  5,1*17, G_RCSWITCH);
CreateItem(GetSetting("S_HUDSW0"),  5,2*17, G_RCSWITCH);
CreateItem(GetSetting("S_HUDSW1"),  5,3*17, G_RCSWITCH);
CreateItem(GetSetting("S_HUDSW2"),  5,4*17, G_RCSWITCH);
//  VREF  ----------------------------------------------------------------------------
CreateItem(GetSetting("S_VREFERENCE"),  5,0*17, G_VREF);

//  Compass ---------------------------------------------------------------------------
CreateItem(GetSetting("S_COMPASS"),  5,0*17, G_COMPASS);
CreateItem(GetSetting("S_SHOWHEADING"),  5,1*17, G_COMPASS);
CreateItem(GetSetting("S_HEADING360"),  5,2*17, G_COMPASS);
CreateItem(GetSetting("S_ANGLETOHOME"),  5,3*17, G_COMPASS);


//  Other ---------------------------------------------------------------------------
CreateItem(GetSetting("S_UNITSYSTEM"),  5,0, G_Other);
BuildRadioButton(GetSetting("S_UNITSYSTEM"),  5,0, G_Other, "Metric","Imperial");
CreateItem(GetSetting("S_VIDEOSIGNALTYPE"),  5,1*17, G_Other);
BuildRadioButton(GetSetting("S_VIDEOSIGNALTYPE"),  5,1*17, G_Other, "NTSC","PAL");
CreateItem(GetSetting("S_THROTTLEPOSITION"),  5,2*17, G_Other);
CreateItem(GetSetting("S_SHOWBATLEVELEVOLUTION"),  5,3*17, G_Other);
CreateItem(GetSetting("S_RESETSTATISTICS"),  5,4*17, G_Other);

//  Display ---------------------------------------------------------------------------
CreateItem(GetSetting("S_MODEICON"),  5,0*17, G_DISPLAY);
CreateItem(GetSetting("S_MODESENSOR"),  5,1*17, G_DISPLAY);
CreateItem(GetSetting("S_GIMBAL"),  5,2*17, G_DISPLAY);
CreateItem(GetSetting("S_VARIO"),  5,3*17, G_DISPLAY);
CreateItem(GetSetting("S_BAROALT"),  5,4*17, G_DISPLAY);
CreateItem(GetSetting("S_TIMER"),  5,5*17, G_DISPLAY);

//  TIME  ----------------------------------------------------------------------------
CreateItem(GetSetting("S_GPSTIME"),  5,0*17, G_TIME);
CreateItem(GetSetting("S_GPSTZ"),  5,1*17, G_TIME);
  confItem[GetSetting("S_GPSTZ")].setMultiplier(0.5f);//30min increments, kathmandu would require 15min, it can use DST 
  confItem[GetSetting("S_GPSTZ")].setDecimalPrecision(1);
CreateItem(GetSetting("S_GPSTZAHEAD"),  5,2*17, G_TIME);

//  LINKS  ----------------------------------------------------------------------------
buttonGUIDELINK = controlP5.addButton("GUIDELINK",1,110,10,80,16);
buttonGUIDELINK.setCaptionLabel("User Guide").setGroup(G_LINKS);
buttonFAQLINK = controlP5.addButton("FAQLINK",1,110,30,80,16);
buttonFAQLINK.setCaptionLabel("FAQ").setGroup(G_LINKS);
buttonCALIBLINK = controlP5.addButton("CALIBLINK",1,110,50,80,16);
buttonCALIBLINK.setCaptionLabel("Calibration").setGroup(G_LINKS);
buttonSUPPORTLINK = controlP5.addButton("SUPPORTLINK",1,110,70,80,16);
buttonSUPPORTLINK.setCaptionLabel("Support").setGroup(G_LINKS);
buttonGPSTIMELINK = controlP5.addButton("GPSTIMELINK",1,200,10,125,16);
buttonGPSTIMELINK.setCaptionLabel("GPS Requirements").setGroup(G_LINKS);
buttonSPORTLINK = controlP5.addButton("SPORTLINK",1,200,30,125,16);
buttonSPORTLINK.setCaptionLabel("FRSKY Requirements").setGroup(G_LINKS);
buttonDONATELINK = controlP5.addButton("DONATELINK",1,XLINKS+30,YLINKS+217, 80, 20).setVisible(false);
//   image(DONATEimage,XLINKS+20,YLINKS+207, 100, 40); 



//  ALARMS  ----------------------------------------------------------------------------
CreateItem(GetSetting("S_DISTANCE_ALARM"),  5,0*17, G_Alarms);
CreateItem(GetSetting("S_ALTITUDE_ALARM"),  5,1*17, G_Alarms);
CreateItem(GetSetting("S_SPEED_ALARM"),  5,2*17, G_Alarms);
CreateItem(GetSetting("S_TIMER_ALARM"),  5,3*17, G_Alarms);

//  SPORT  ----------------------------------------------------------------------------


//  Call Sign ---------------------------------------------------------------------------
CreateItem(GetSetting("S_DISPLAY_CS"),  5,0, G_CallSign);

controlP5.addTextfield("CallSign")
     .setPosition(5,1*17)
     .setSize(105,15)
     .setFont(font10)
     .setAutoClear(false)
    .setLabel(" ")
     .setGroup(G_CallSign);
     ;
 controlP5.addTextlabel("TXTCallSign","",120,1*17)
 .setGroup(G_CallSign);
 CreateCS(GetSetting("S_CS0"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS1"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS2"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS3"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS4"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS5"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS6"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS7"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS8"),  0,0, G_CallSign);
 CreateCS(GetSetting("S_CS9"),  0,0, G_CallSign);




       

  for(int i=0;i<CONFIGITEMS;i++) {
    if (ConfigRanges[i] == 0) {
      toggleConfItem[i].hide();
      confItem[i].hide();
    }
    if (ConfigRanges[i] > 1) {
      try{
      toggleConfItem[i].hide();
      }catch(Exception e) {
      }finally {
      }  	
    }
      
    if (ConfigRanges[i] == 1){
      confItem[i].hide();  
    }
  }
  
  //byte[] inBuf = new byte[256];
  for (int txTimes = 0; txTimes<255; txTimes++) {
    inBuf[txTimes] = 0;
  }

  
  

  BuildToolHelp();
  Font_Editor_setup();
   SimSetup();
  img_Clear = LoadFont(FontFileName);
  //toggleMSP_Data = true;
  CloseMode = 0;
  LoadConfig();
  

}


public controlP5.Controller hideCheckbox(controlP5.Controller c) {
  c.hide();
  //c.setLabelVisible(false);
  return c;
}

public controlP5.Controller CheckboxVisable(controlP5.Controller c) {
  c.isVisible(); 

  //c.setLabelVisible(false);
  return c;
}



public void BuildRadioButton(int ItemIndex, int XLoction, int YLocation,Group inGroup, String Cap1, String Cap2){
    
  RadioButtonConfItem[ItemIndex] = controlP5.addRadioButton("RadioButton"+ItemIndex)
         .setPosition(XLoction,YLocation+3)
         .setSize(10,10)
         .setNoneSelectedAllowed(false) 
         //.setColorBackground(color(120))
         //.setColorActive(color(255))
        // .setColorLabel(color(255))
         .setItemsPerRow(2)
         .setSpacingColumn(PApplet.parseInt(textWidth(Cap1))+10)
         .addItem("First"+ItemIndex,0)
         .addItem("Second"+ItemIndex,1)
         .toUpperCase(false)
        //.hideLabels() 
         ;
    RadioButtonConfItem[ItemIndex].setGroup(inGroup);
    RadioButtonConfItem[ItemIndex].getItem(0).setCaptionLabel(Cap1);
    RadioButtonConfItem[ItemIndex].getItem(1).setCaptionLabel(Cap2 + "    " + ConfigNames[ItemIndex]);
    
    toggleConfItem[ItemIndex].hide();
    txtlblconfItem[ItemIndex].hide();
    
  
}

public void CreateCS(int ItemIndex, int XLoction, int YLocation, Group inGroup){
  //numberbox
  confItem[ItemIndex] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("configItem"+ItemIndex,0,XLoction,YLocation,35,14));
  confItem[ItemIndex].setMin(0);
  confItem[ItemIndex].setMax(255);
  confItem[ItemIndex].setDecimalPrecision(0);
  confItem[ItemIndex].setGroup(inGroup);
  confItem[ItemIndex].hide();
  toggleConfItem[ItemIndex] = (controlP5.Toggle) hideLabel(controlP5.addToggle("toggleValue"+ItemIndex));
  toggleConfItem[ItemIndex].hide();
}

public void CreateItem(int ItemIndex, int XLoction, int YLocation, Group inGroup){
  //numberbox
  confItem[ItemIndex] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("configItem"+ItemIndex,0,XLoction,YLocation,35,14));
  confItem[ItemIndex].setColorBackground(red_);
  confItem[ItemIndex].setMin(0);
  confItem[ItemIndex].setDirection(Controller.VERTICAL);
  confItem[ItemIndex].setMax(ConfigRanges[ItemIndex]);
  confItem[ItemIndex].setDecimalPrecision(0);
  confItem[ItemIndex].setGroup(inGroup);
  //confItem[ItemIndex].setMultiplier(10);
  //Toggle
  toggleConfItem[ItemIndex] = (controlP5.Toggle) hideLabel(controlP5.addToggle("toggleValue"+ItemIndex));
  toggleConfItem[ItemIndex].setPosition(XLoction,YLocation+3);
  toggleConfItem[ItemIndex].setSize(35,10);
  toggleConfItem[ItemIndex].setMode(ControlP5.SWITCH);
  toggleConfItem[ItemIndex].setGroup(inGroup);
  //TextLabel
  txtlblconfItem[ItemIndex] = controlP5.addTextlabel("txtlblconfItem"+ItemIndex,ConfigNames[ItemIndex],XLoction+40,YLocation);
  txtlblconfItem[ItemIndex].setGroup(inGroup);
  //controlP5.getTooltip().register("txtlblconfItem"+ItemIndex,ConfigHelp[ItemIndex]);

} 


public void BuildToolHelp(){
  controlP5.getTooltip().setDelay(100);
}





public void MakePorts(){

  if (PortWrite){  
    TXText.setColorValue(color(255,10,0));
  }
  else
  {
    TXText.setColorValue(color(100,10,0));
  }

  if (PortRead){
    PortReadtimer=50+millis();  
    PortRead=false;
  }

  if (PortReadtimer>millis())
  {  
    PortRead=false;
    RXText.setColorValue(color(0,240,0));
  }
   else
  {
    RXText.setColorValue(color(0,100,0));
  }

}

public void draw() {

  
  debug[0]=WriteLayouts;
//  debug[1]=OSD_S_HUDSW0;
//  debug[2]=OSD_S_HUDSW1;
//  debug[3]=OSD_S_HUDSW2;
// Initial setup
  time=millis();
  progresstxt="";

  if (commListbox.isOpen())
    baudListbox.close();
  else
    baudListbox.open();
  

// Process and outstanding Read or Write EEPROM requests
  if((millis()>ReadConfigMSPMillis)&&(millis()>WriteConfigMSPMillis)&&(init_com==1)){
    if (AutoSimulator !=0){
      SimControlToggle.setValue(1);
    }
  }
    
    if (millis()<ReadConfigMSPMillis){
      if (init_com==1){
        READconfigMSP();
        toggleMSP_Data = true;
        int progress=100*eeaddressGUI/CONFIGITEMS;
        if (progress==0){
          progresstxt="Waiting FC...";   
        }
        else{
          progresstxt="Read: "+progress+"%";   
        }    
      }
    }

    if (millis()<WriteConfigMSPMillis){
      if (init_com==1){
        WRITEconfigMSP();
        toggleMSP_Data = true;
        int progress=100*eeaddressGUI/(CONFIGITEMS);
        if (WriteLayouts==1){
          progress=100*eeaddressGUI/(CONFIGITEMS + (hudoptions*3*2));
        }
        if (progress==0){
          progresstxt="Waiting FC...";   
        }
        else{
          if(eeaddressGUI>CONFIGITEMS){
            progresstxt="Layouts: "+progress+"%";          }
          else{
            progresstxt="Write: "+progress+"%";
          }
        }    
      }
    }
    int progressflash=millis()>>8;
    if((progressflash&0x01)==0x01){
      progresstxt="";
    }
    
    txtmessage.setValue(progresstxt);

// Layout editor
  txtlblLayoutTxt.setValue(" : "+ CONFIGHUDTEXT[hudeditposition]);

  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value()); 
  txtlblLayoutHudTxt.setValue(" : "+hudid);
  LEW.setLabel("Layout Editor for profile: "+hudid+" - "+CONFIGHUDNAME[hudid]);
  if (CONFIGHUDEN[hudid][hudeditposition]>0)
    txtlblLayoutEnTxt.setValue(" : Enabled");
  else
    txtlblLayoutEnTxt.setValue(" : Disabled");

// Layout amenedmends based upon choices
  if (PApplet.parseInt(confItem[GetSetting("S_RCWSWITCH")].value())==0){
    txtlblconfItem[GetSetting("S_RCWSWITCH")].setText("Using OSD_SW");
    txtlblconfItem[GetSetting("S_RCWSWITCH_CH")].setText("Not used");

    toggleConfItem[GetSetting("S_HUDSW2")].hide();

    txtlblconfItem[GetSetting("S_HUDSW0")].setText("HUD - Default");
    txtlblconfItem[GetSetting("S_HUDSW1")].setText("HUD - OSD SW");
    txtlblconfItem[GetSetting("S_HUDSW2")].setText("Not used");
  }
  else {
//    toggleConfItem[GetSetting("S_RCWSWITCH_CH")].show();
    txtlblconfItem[GetSetting("S_RCWSWITCH")].setText("Using RC Channel");
    txtlblconfItem[GetSetting("S_RCWSWITCH_CH")].setText("OSD ch (0-7)");
    txtlblconfItem[GetSetting("S_HUDSW0")].setText("HUD - LOW");
    txtlblconfItem[GetSetting("S_HUDSW1")].setText("HUD - HIGH");
    txtlblconfItem[GetSetting("S_HUDSW2")].setText("HUD - MID");
  }
//    confItem[GetSetting("S_RCWSWITCH")].setValue(" : Enabled");
    

// Colour switches when enabled......
  coloriseswitches();
  if ((init_com==1)  && (toggleMSP_Data == true)) {
    MWData_Com();
    if (!FontMode) PortRead = false;
    MakePorts();
  }
  else {
    PortRead = false;
    MakePorts();
  }

  if ((SendSim ==1) && (ClosePort == false)) 
  {
    if (!FontMode&&(ReadConfig==0)&&(WriteConfig==0)) {
      if (init_com==1) {
       
        if (ClosePort) return;
        if (SimControlToggle.getValue()!=0) {

          if (init_com==1)SendCommand(MSP_ATTITUDE);
          if (init_com==1)SendCommand(MSP_RC);
          if (init_com==1)SendCommand(MSP_STATUS);

        MSP_sendOrder++;
        switch(MSP_sendOrder) {
        case 1:
          if (init_com==1)SendCommand(MSP_BOXNAMES);
          if (init_com==1)SendCommand(MSP_BOXIDS);
          if (init_com==1)SendCommand(MSP_IDENT);
          break;
        case 2:
          if (init_com==1)SendCommand(MSP_ANALOG);
          if (init_com==1)SendCommand(MSP_COMP_GPS); 
          break;
        case 3:
          if (init_com==1)SendCommand(MSP_ATTITUDE);
          break;
        case 4:
          if (init_com==1)SendCommand(MSP_RAW_GPS);
          break;
        case 5:
          if (init_com==1)SendCommand(MSP_ALTITUDE);
          //PortWrite = !PortWrite;     
          break;
        case 6: 
          if (init_com==1)SendCommand(MSP_CELLS);
          break;
        case 7: 
          if ((init_com==1)&&(toggleMSP_Data == false)) SendCommand(MSP_BOXNAMES);
          break;
        case 8:
          if ((init_com==1)&&(toggleMSP_Data == false)) SendCommand(MSP_BOXIDS);
          break;
        case 9:
          if (init_com==1)SendCommand(MSP_NAV_STATUS);
          break;
        case 10:
          if (init_com==1)SendCommand(MSP_DEBUG);
          break;
        case 11:
          if (init_com==1)SendCommand(MSP_PID);
          MSP_sendOrder=0;
          break;
        }
        PortWrite = !PortWrite; // toggle TX LED every other    
      } 
      }
    } // End !FontMode
  }

  else
  {
    if (!FontMode) PortWrite = false;
  }

  if ((FontMode) && (time-time2 >100)) {
    SendChar();
  }
    
  MakePorts();  
  
  background(80);
  // ------------------------------------------------------------------------
  // Draw background control boxes
  // ------------------------------------------------------------------------

  image(GUIBackground,0, 0, windowsX, windowsY); 
  if (LEWvisible==0){
   image(DONATEimage,XLINKS+20,YLINKS+207, 100, 40); 
   
  }  
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
    image(OSDBackground,DisplayWindowX+WindowAdjX+10, DisplayWindowY+WindowAdjY, 354-WindowShrinkX, 300-WindowShrinkY); //529-WindowShrinkX, 360-WindowShrinkY);


//################################################################################################################################################################################ 
// Display
//################################################################################################################################################################################ 



 for(int i = 0; i < (hudoptions); i++){
 
   ConfigLayout[0][i]=CONFIGHUD[PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value())][i];
   ConfigLayout[1][i]=CONFIGHUD[PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value())][i];
   ConfigLayout[2][i]=CONFIGHUD[PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value())][i];


   if (toggleModeItems[9].getValue()==2)
      SimPosn[i]=ConfigLayout[2][i];
   else if (toggleModeItems[9].getValue()==1)
      SimPosn[i]=ConfigLayout[1][i];
   else
      SimPosn[i]=ConfigLayout[0][i];


   if (SimPosn[i]<0x4000){
     SimPosn[i]=0x3FF; 
   }
   else{
   SimPosn[i]=SimPosn[i]&0x3FF;
 }
   
  }
  
      
    if (PApplet.parseInt(confItem[GetSetting("S_DISPLAYRSSI")].value()) > 0)    ShowRSSI(); 


  if(confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() > 0) displayHorizon(PApplet.parseInt(MW_Pitch_Roll.arrayValue()[0])*10,PApplet.parseInt(MW_Pitch_Roll.arrayValue()[1])*10*-1);
  SimulateTimer();
  CalcAlt_Vario(); 

  ShowCurrentThrottlePosition();
  if (PApplet.parseInt(confItem[GetSetting("S_DISPLAYRSSI")].value()) > 0)    ShowRSSI(); 
  if (PApplet.parseInt(confItem[GetSetting("S_DISPLAYVOLTAGE")].value()) > 0) ShowVolts(sVBat);

  ShowVideoVolts(sVBat);    
 
  displaySensors();
  displayMode();
  ShowAmps();
  ShowAltitude();
  ShowCallsign();
  ShownAngletohome();
  ShowAmperage();
  ShowVario();
  ShowTemp();
  ShowUTC();
  displayHeadingGraph();
  displayHeading();
  ShowDebug();
  ShowSideBarArrows();
  ShowAPstatus();

  if(confItem[GetSetting("S_DISPLAYGPS")].value() > 0) {
  ShowGPSAltitude();
  ShowDistance();
  ShowLatLon();
  ShowSats();   
  ShowSpeed();
  ShowDirection();
 }
 
  ShowMapMode();
    
  MatchConfigs();
  MakePorts();
  
  ShowSPort();
  
  if ((ClosePort ==true)&& (PortWrite == false)){ //&& (init_com==1)
    ClosePort();
  }

  //Textarea used for introduction
//  if(confItem[GetSetting("S_DEBUG")].value() > 0) StartupMessage=0; // for testing
  
  if (StartupMessage ==0){
    myTextarea.setText("Welcome to ");
    String s = myTextarea.getText();
    s = s+ MW_OSD_GUI_Version;
    s = s+ "\n";
    s = s+ "\n";
    s = s+ "Check out our new website - MWOSD.com";
    s = s+ "\n";
    s = s+ "\n";
    s = s+ "New features in this release:";
    s = s+ "\n";
    s = s+ "FIXEDWING support";
    s = s+ "\n";
    s = s+ "Enhanced CLEANFLIGHT support";
    s = s+ "\n";
    s = s+ "3 way switchable OSD layouts";
    s = s+ "\n";
    s = s+ "Clearer display - less artifacts";
    s = s+ "\n";
    s = s+ "\n";
    s = s+ "Help support continued development - donate just a couple of dollars.";
    s = s+ "\n";
    s = s+ "\n";
    s = s+ "Select a COM port to start using MWOSD! \n";
    s = s+ "\n";
    
    myTextarea.setText(s);
    myTextarea.show();
  }
  else{
    myTextarea.hide();
  }
}


public int GetSetting(String test){
  int TheSetting = 0;
  for (int i=0; i<Settings.values().length; i++) 
  if (Settings.valueOf(test) == Settings.values()[i]){ 
      TheSetting = Settings.values()[i].ordinal();
  }
  return TheSetting;
}


public void ShowSimBackground(float[] a) {
  Showback = PApplet.parseInt(a[0]);
}

//void SimulateMultiWii(float[] a) {
//}

public void BuildCallSign(){
  String CallSText = "";
  for (int i=0; i<10; i++){ 
    //confItem[GetSetting("S_CS0")+i].setValue(0);
    if (PApplet.parseInt(confItem[GetSetting("S_CS0")+i].getValue())>0){
    CallSText+=PApplet.parseChar(PApplet.parseInt(confItem[GetSetting("S_CS0")+i].getValue()));
    }
  }
  controlP5.get(Textfield.class,"CallSign").setText(CallSText);
}  

public void CheckCallSign() {
  // automatically receives results from controller input
  String CallSText = controlP5.get(Textfield.class,"CallSign").getText().toUpperCase();
  controlP5.get(Textfield.class,"CallSign").setText(CallSText);
  //if (CallSText.length()  >0){
    if (CallSText.length()  >10){
      controlP5.get(Textfield.class,"CallSign").setText(CallSText.substring(0, 10));
      CallSText = controlP5.get(Textfield.class,"CallSign").getText();
    } 
    for (int i=0; i<10; i++){ 
    confItem[GetSetting("S_CS0")+i].setValue(0);
    }
    for (int i=0; i<CallSText.length(); i++){ 
      confItem[(GetSetting("S_CS0"))+i].setValue(PApplet.parseInt(CallSText.charAt(i)));
    //println(int(CallSText.charAt(0)));
    //println(controlP5.get(Textfield.class,"CallSign").getText());
    }
  //}
}




public void MatchConfigs(){
 for(int i=0;i<CONFIGITEMS;i++) {
   try{ 
       if (RadioButtonConfItem[i].isVisible()){
          confItem[i].setValue(PApplet.parseInt(RadioButtonConfItem[i].getValue()));
       }
        }catch(Exception e) {}finally {}
    
  
     if  (toggleConfItem[i].isVisible()){
       if (PApplet.parseInt(toggleConfItem[i].getValue())== 1){
       confItem[i].setValue(1);
     }
     else{ 
       confItem[i].setValue(0);
     }
   }
   
   if (ConfigRanges[i] == 0) {
      toggleConfItem[i].hide();
      //RadioButtonConfItem[i].hide();
      confItem[i].hide();
    }
    if (ConfigRanges[i] > 1) {
      toggleConfItem[i].hide();
      
    }  
    if (ConfigRanges[i] == 1){
      confItem[i].hide();  
    }
    
    
  }
  // turn on FlyTimer----
  if ((toggleModeItems[0].getValue() == 0) && (SimItem0 < 1)){
    Armed = 1;
    FlyTimer = millis();
  }
  // turn off FlyTimer----
  if ((toggleModeItems[0].getValue() == 1 ) && (SimItem0 == 1)){
    FlyTimer = 0;
  }



}

// controls comport list click
public void controlEvent(ControlEvent theEvent) {
  
  try{
  if (theEvent.isGroup())
    if (theEvent.name()=="portComList")
      InitSerial(theEvent.group().value()); // initialize the serial port selected
  }catch(Exception e){
    System.out.println("error with Port");
  }

  if (theEvent.name()=="CallSign"){
    CheckCallSign();
  }
    

    if (theEvent.name()=="portBaudList"){
//      BaudRate=200;
      if (init_com==1)
        ClosePort();
      if (PApplet.parseInt(theEvent.group().value()) ==3) BaudRate=19200;
      else if (PApplet.parseInt(theEvent.group().value()) ==2) BaudRate=38400;
      else if (PApplet.parseInt(theEvent.group().value()) ==1) BaudRate=57600;
      else BaudRate=115200;
//      BaudRate=int(theEvent.group().value());
      txtlblWhichbaud.setValue("Baud rate: "+str(BaudRate));
      updateConfig();
      baudListbox.close();
      commListbox.open();

    }
        
  try{
  //for (int i=0;i<col.length;i++) {
    if ((theEvent.getController().getName().substring(0, 7).equals("CharPix")) && (theEvent.getController().isMousePressed())) {
      //println("Got a pixel " + theEvent.controller().id());
        int ColorCheck = PApplet.parseInt(theEvent.getController().value());
        curPixel = theEvent.controller().id();
    }
    if ((theEvent.getController().getName().substring(0, 7).equals("CharMap")) && (theEvent.getController().isMousePressed())) {
      curChar = theEvent.controller().id();    
      //println("Got a Char " + theEvent.controller().id());
    }
   } catch(ClassCastException e){}
     catch(StringIndexOutOfBoundsException se){}
      
}





public void mapchar(int address, int screenAddress){
  int placeX = (screenAddress % 30) * 12;
  int placeY = (screenAddress / 30) * 18;

  blend(img_Clear, 0,address*18, 12, 18, placeX+DisplayWindowX, placeY+DisplayWindowY, 12, 18, BLEND);
}

public void makeText(String inString, int inStartAddress ){
  for (int i = 0; i < inString.length(); i++){
    mapchar(PApplet.parseInt(inString.charAt(i)), inStartAddress +i); 
  }   
}



public void displaySensors()
{
//   mapchar(0xa0,sensorPosition[0]);
//   mapchar(0xa2,sensorPosition[0]+1);
//   mapchar(0xa1,sensorPosition[0]+2);
   /* 
  if(MwSensorPresent&ACCELEROMETER) mapchar("0xa0",sensorPosition[0]);
  else ;
  if(MwSensorPresent&BAROMETER)     screenBuffer[1]=0xa2;
  else screenBuffer[1]=' ';
  if(MwSensorPresent&MAGNETOMETER)  screenBuffer[2]=0xa1;
  else screenBuffer[2]=' ';
  if(MwSensorPresent&GPSSENSOR)     screenBuffer[3]=0xa3;
  else screenBuffer[3]=' ';
  screenBuffer[4]=0;
  MAX7456_WriteString(screenBuffer,sensorPosition[videoSignalType][screenType]);
  */
}







/////////////////////////////////////////////////////////////////////////////////////////////////////////////////// BEGIN FILE OPS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


public void bLUP() {
  int i = 0;
  if (toggleModeItems[9].getValue()==2)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
    
  int ii = CONFIGHUD[i][hudeditposition]&0x1FF;
  ii-=30;
  ii=constrain(ii,0,419);
  println(ii);
  CONFIGHUD[i][hudeditposition]&=0xFE00;
  CONFIGHUD[i][hudeditposition]|=ii;
}

public void bLDOWN() {
  int i = 0;
  if (toggleModeItems[9].getValue()==2)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  int ii = CONFIGHUD[i][hudeditposition]&0x1FF;
  ii+=30;
  ii=constrain(ii,0,419);
  println(ii);
  CONFIGHUD[i][hudeditposition]&=0xFE00;
  CONFIGHUD[i][hudeditposition]|=ii;
//  CONFIGHUD[i][hudeditposition]+=30;
}

public void bLLEFT() {
  int i = 0;
  if (toggleModeItems[9].getValue()==2)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  int ii = CONFIGHUD[i][hudeditposition]&0x1FF;
  ii-=1;
  ii=constrain(ii,0,419);
  println(ii);
  CONFIGHUD[i][hudeditposition]&=0xFE00;
  CONFIGHUD[i][hudeditposition]|=ii;
}

public void bLRIGHT() {
  int i = 0;
  if (toggleModeItems[9].getValue()==2)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  int ii = CONFIGHUD[i][hudeditposition]&0x1FF;
  ii+=1;
  ii=constrain(ii,0,419);
  println(ii);
  CONFIGHUD[i][hudeditposition]&=0xFE00;
  CONFIGHUD[i][hudeditposition]|=ii;
}
  
public void bPOSLUP() {
  hudeditposition--;
  hudeditposition= constrain(hudeditposition,0,hudoptions-1);
}
public void bPOSLDOWN() {
  hudeditposition++;
  hudeditposition= constrain(hudeditposition,0,hudoptions-1);
}
public void bPOSLEN() {
  int i = 0;
  if (toggleModeItems[9].getValue()==2)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  if (CONFIGHUDEN[i][hudeditposition]>0){
    CONFIGHUDEN[i][hudeditposition]=0;
    CONFIGHUD[i][hudeditposition]&=0x3FF;
  }
  else{
    CONFIGHUDEN[i][hudeditposition]=1;
    CONFIGHUD[i][hudeditposition]|=0xC000;
  }
}
public void bLSAVE() {
  OSD_S_HUDSW0=99;
  OSD_S_HUDSW1=99;
  OSD_S_HUDSW2=99;
  if (init_com==1){
    WriteLayouts=1;
    WRITEconfigMSP_init();
  }
  xmlsavelayout();
}

public void bLCANCEL() {
  Lock_All_Controls(false);
  LEW.hide();
  LEWvisible=0;
  G_LINKS.show(); 
  initxml();
  if (init_com==1)
    READconfigMSP_init();
}

public void bLSET() {
  setset();
}

public void bLADD() {
  addchild();
}

public void bHUDLUP() {
  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  hudid--;
  hudid= constrain(hudid,0,hudsavailable-1);
  confItem[GetSetting("S_HUDSW2")].setValue(hudid);
  confItem[GetSetting("S_HUDSW1")].setValue(hudid);
  confItem[GetSetting("S_HUDSW0")].setValue(hudid);
}
public void bHUDLDOWN() {
  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  hudid++;
  hudid= constrain(hudid,0,hudsavailable-1);
  confItem[GetSetting("S_HUDSW2")].setValue(hudid);
  confItem[GetSetting("S_HUDSW1")].setValue(hudid);
  confItem[GetSetting("S_HUDSW0")].setValue(hudid);
}


//save the content of the model to a file
public void bSAVE() {
  updateModel();
  SwingUtilities.invokeLater(new Runnable(){
    public void run() {
     final JFileChooser fc = new JFileChooser(dataPath("")) {

        private static final long serialVersionUID = 7919427933588163126L;

        public void approveSelection() {
            File f = getSelectedFile();
            if (f.exists() && getDialogType() == SAVE_DIALOG) {
                int result = JOptionPane.showConfirmDialog(this,
                        "The file exists, overwrite?", "Existing file",
                        JOptionPane.YES_NO_CANCEL_OPTION);
                switch (result) {
                case JOptionPane.YES_OPTION:
                    super.approveSelection();
                    return;
                case JOptionPane.CANCEL_OPTION:
                    cancelSelection();
                    return;
                default:
                    return;
                }
            }
            super.approveSelection();
        }
    };

      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setFileFilter(new MwiFileFilter());
      int returnVal = fc.showSaveDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File file = fc.getSelectedFile();
        String filePath = file.getPath();
        if(!filePath.toLowerCase().endsWith(".osd")){
          file = new File(filePath + ".osd");
        }

        
        FileOutputStream out =null;
        String error = null;
        try{
          out = new FileOutputStream(file) ;
          MWI.conf.storeToXML(out, "MWOSD Configuration File  " + new  Date().toString());
          JOptionPane.showMessageDialog(null,new StringBuffer().append("configuration saved : ").append(file.toURI()) );
        }catch(FileNotFoundException e){
         
          error = e.getCause().toString();
        }catch( IOException ioe){
                /*failed to write the file*/
                ioe.printStackTrace();
                error = ioe.getCause().toString();
        }finally{
                
          if (out!=null){
            try{
              out.close();
            }catch( IOException ioe){/*failed to close the file*/error = ioe.getCause().toString();}
          }
          if (error !=null){
                  JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
          }
        }
    }
    }
  }
  );
}

public void updateModel(){
  for(int j=0;j<ConfigNames.length;j++) {
         MWI.setProperty(ConfigNames[j],String.valueOf(confItem[j].value()));
  }
}

public void updateView(){
  for(int j=0; j<ConfigNames.length; j++) {
    
    //confItem[j].setValue(int(MWI.getProperty(ConfigNames[j])));
    if(j >= CONFIGITEMS)
    return;
  int value = PApplet.parseInt(MWI.getProperty(ConfigNames[j]));
  float f = new Float(MWI.getProperty(ConfigNames[j]));
  if (j==GetSetting("S_VOLTAGEMIN")){
    confItem[j].setValue(f);
  }
  else{
    confItem[j].setValue(value);
  }

  if (j == CONFIGITEMS-1){
    //buttonWRITE.setColorBackground(green_);
  }  
  if (value >0){
    toggleConfItem[j].setValue(1);
    }
    else {
    toggleConfItem[j].setValue(0);
  }

  try{
    switch(value) {
    case(0):
      RadioButtonConfItem[j].activate(0);
      break;
    case(1):
      RadioButtonConfItem[j].activate(1);
      break;
    }
  }
  catch(Exception e) {}finally {}
  }
  BuildCallSign();
}

public class MwiFileFilter extends FileFilter {
  public boolean accept(File f) {
    if(f != null) {
      if(f.isDirectory()) {
        return true;
      }
      String extension = getExtension(f);
      if("osd".equals(extension)) {
        return true;
      }
    }
    return false;
  }


  public String getExtension(File f) {
    if(f != null) {
      String filename = f.getName();
      int i = filename.lastIndexOf('.');
      if(i>0 && i<filename.length()-1) {
        return filename.substring(i+1).toLowerCase();
      }
    }
    return null;
  } 

  public String getDescription() {
    return "*.osd MWOSD configuration file";
  }   
}




// import the content of a file into the model
public void bIMPORT(){
  SwingUtilities.invokeLater(new Runnable(){
    public void run(){
      final JFileChooser fc = new JFileChooser(dataPath(""));
      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setFileFilter(new MwiFileFilter());
      int returnVal = fc.showOpenDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File file = fc.getSelectedFile();
        FileInputStream in = null;
        boolean completed = false;
        String error = null;
        try{
          in = new FileInputStream(file) ;
          MWI.conf.loadFromXML(in); 
          JOptionPane.showMessageDialog(null,new StringBuffer().append("configuration loaded : ").append(file.toURI()) );
          completed  = true;
          
        }catch(FileNotFoundException e){
                error = e.getCause().toString();

        }catch( IOException ioe){/*failed to read the file*/
                ioe.printStackTrace();
                error = ioe.getCause().toString();
        }finally{
          if (!completed){
                 // MWI.conf.clear();
                 // or we can set the properties with view values, sort of 'nothing happens'
                 updateModel();
          }
          updateView();
          if (in!=null){
            try{
              in.close();
            }catch( IOException ioe){/*failed to close the file*/}
          }
          
          if (error !=null){
                  JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
          }
        }
      }
    }
  }
  );
}


//  our model 
static class MWI {
  private static Properties conf = new Properties();
  public static void setProperty(String key ,String value ){
    conf.setProperty( key,value );
  }
  public static String getProperty(String key ){
    return conf.getProperty( key,"0");
  }
  public static void clear( ){
    conf= null; // help gc
    conf = new Properties();
  }
}



public void updateConfig(){
  String error = null;
  FileOutputStream out =null;
  
//  ConfigClass.setProperty("StartFontFile",FontFileName);
  ConfigClass.setProperty("BaudRate",str(BaudRate));
  ConfigClass.setProperty("Title",Title);
  ConfigClass.setProperty("Passthroughcomm",str(Passthroughcomm));
  ConfigClass.setProperty("AutoSimulator",str(AutoSimulator));
  ConfigClass.setProperty("StartupMessage",str(StartupMessage));
  
  File file = new File(dataPath("gui.cfg"));
  try{
    out = new FileOutputStream(file) ;
    ConfigClass.conf.storeToXML(out, "MW_OSD GUI Configuration File  " + new  Date().toString());
    }catch(FileNotFoundException e){
      error = e.getCause().toString();
      }catch( IOException ioe){
        /*failed to write the file*/
        ioe.printStackTrace();
        error = ioe.getCause().toString();
      }finally{
        if (out!=null){
          try{
            out.close();
            }catch( IOException ioe){/*failed to close the file*/error = ioe.getCause().toString();}
            }
            if (error !=null){
              JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
            }
      }
}


public void LoadConfig(){
  String error = null;
  FileInputStream in =null;  
  BaudRate=0;
  try{
    in = new FileInputStream(dataPath("gui.cfg"));
  }
  catch(FileNotFoundException e){
    //System.out.println("Configuration Failed- Creating Default");
    BaudRate = 115200;
    Title = MW_OSD_GUI_Version;
    Passthroughcomm = 0;
    AutoSimulator = 0;
    StartupMessage = 0;
    updateConfig();
  }
  catch( IOException ioe){
      /*failed to write the file*/
      ioe.printStackTrace();
      error = ioe.getCause().toString();
  }//finally{
  if (in!=null){
    try{
      ConfigClass.conf.loadFromXML(in); 
      BaudRate =PApplet.parseInt(ConfigClass.getProperty("BaudRate"));
      Title =ConfigClass.getProperty("Title");
      Passthroughcomm = PApplet.parseInt(ConfigClass.getProperty("Passthroughcomm"));
      AutoSimulator = PApplet.parseInt(ConfigClass.getProperty("AutoSimulator"));
      StartupMessage = PApplet.parseInt(ConfigClass.getProperty("StartupMessage"));

      img_Clear = LoadFont(FontFileName);
      in.close();
    }
    catch( IOException ioe){
      /*failed to close the file*/error = ioe.getCause().toString();}
  }
  if (error !=null){
    JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
  }
  txtlblWhichbaud.setValue("Baud rate: "+str(BaudRate));
  frame.setTitle(Title);
  if (Passthroughcomm !=0){
    commListbox.addItem("Pass Thru Comm",commListMax+1); 
  }
  if (AutoSimulator !=0){
    SimControlToggle.setValue(1);
  }

}

//  our configuration 
static class ConfigClass {
  private static Properties conf = new Properties();
  public static void setProperty(String key ,String value ){
    conf.setProperty( key,value );
  }
  public static String getProperty(String key ){
    return conf.getProperty( key,"0");
  }
  public static void clear( ){
    conf= null; // help gc
    conf = new Properties();
  }
}


public void mouseReleased() {
  mouseDown = false;
  mouseUp = true;
  if (curPixel>-1)changePixel(curPixel);
  if (curChar>-1)GetChar(curChar);
  ControlLock();
  
} 

        
public void mousePressed() {
                mouseDown = true;
                mouseUp = false;
        }



        public boolean mouseDown() {
                return mouseDown;
        }

        public boolean mouseUp() {
                return mouseUp;
        }
        
        
public void SketchUploader(){
  String ArduioLocal = ConfigClass.getProperty("ArduinoLocation");
  if (ArduioLocal == "0"){
   try {  
    SwingUtilities.invokeAndWait(new Runnable(){
    public void run(){
      JFileChooser fc = new JFileChooser();
      fc.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setDialogTitle("Select Arduino Folder");
      //fc.setFileFilter(new FontFileFilter());
      //fc.setCurrentDirectory();
      int returnVal = fc.showOpenDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File ArduioFile = fc.getSelectedFile();
        String ArduioLocal = ArduioFile.getPath();
        ConfigClass.setProperty("ArduinoLocation",ArduioLocal);
        updateConfig();
        
        String error = null;
        
      }
    }
  }
  );  
  } catch (Exception e) { }
  }

  toggleMSP_Data = false;
  //delay(1000);
  InitSerial(200.00f);
  updateConfig(); 
  //if (init_com==1){
    //init_com=0;
  //g_serial.stop();
  //}
  
 
  
  super.exit();
}


public void GPSTIMELINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/GPSTime.md"); 
}
public void SPORTLINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/Frsky_SPort.md"); 
}
public void CODELINK(){
 link("https://www.mwosd.com/"); 
}
public void FAQLINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/FAQ.md"); 
}
public void GUIDELINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/User_Guide.md"); 
}
public void SUPPORTLINK(){
  link("http://fpvlab.com/forums/showthread.php?34250-MWOSD-for-MULTIWII-NAZE32-BASEFLIGHT-HARIKIRI"); 
}
public void DONATELINK(){
  link("https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=EBS76N8F426G2&lc=GB&item_name=MW%2dOSD&item_number=R1%2e4&currency_code=GBP&bn=PP%2dDonationsBF%3abtn_donate_SM%2egif%3aNonHosted"); 
}
public void CALIBLINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/DOCUMENTATION/Calibration.md"); 
}


public void initxml(){
  int value;
  int enabled;
  int hud;
  int hudindex;
  int xx;
  String text;
  String hudname;

  XML[] xmlhudconfig = xml.getChildren("CONFIG");
  hudsavailable = xmlhudconfig[0].getInt("value");
  XML[] xmlhuddescription = xml.getChildren("DESCRIPTION");
  hudoptions = xmlhuddescription.length;
  XML[] xmlhudname = xml.getChildren("HUDNAME");
  hudnamelength = xmlhudname.length;
  XML[] allhudoptions = xml.getChildren("LAYOUT");
//  hudoptions = allhudoptions.length/hudsavailable;
  CONFIGHUD= new int[hudsavailable][hudoptions];
  CONFIGHUDEN= new int[hudsavailable][hudoptions];
  CONFIGHUDTEXT= new String[hudoptions];
  CONFIGHUDNAME= new String[hudnamelength];
  for (int i = 0; i < hudoptions; i++) {
    text = xmlhuddescription[i].getString("desc");
    CONFIGHUDTEXT[i] = text;
  }
    for (int i = 0; i < hudnamelength; i++) {
    text = xmlhudname[i].getString("hudname");
    CONFIGHUDNAME[i] = text;
  }

    for (int i = 0; i < allhudoptions.length; i++) {
    value = allhudoptions[i].getInt("value");
    enabled = allhudoptions[i].getInt("enabled");
    hud = allhudoptions[i].getInt("hud");
    hudindex = i%hudoptions;
    if (enabled == 1)
      DISPLAY_STATE=0xC000;
    else
      DISPLAY_STATE=0x0000;    
    CONFIGHUD[hud][hudindex] = value | DISPLAY_STATE;  
    CONFIGHUDEN[hud][hudindex] = enabled;      
  }
  SimPosn = new int[hudoptions];
  ConfigLayout= new int[4][hudoptions];
  ConfigRanges[GetSetting("S_HUDSW0")] = hudsavailable-1;
  ConfigRanges[GetSetting("S_HUDSW1")] = hudsavailable-1;
  ConfigRanges[GetSetting("S_HUDSW2")] = hudsavailable-1;

}


public void xmlsavelayout(){
  XML[] allhudoptions = xml.getChildren("LAYOUT");
  int i=0;

  for (int hud = 0; hud < hudsavailable; hud++) {
    for (int hudindex = 0; hudindex < hudoptions; hudindex++) {
      allhudoptions[i].setInt("enabled",CONFIGHUDEN[hud][hudindex]);
      allhudoptions[i].setInt("value",CONFIGHUD[hud][hudindex]&0x3FF);
      i++;  
    }    
  }
  saveXML(xml, dataPath("hudlayout.xml"));
  initxml();
  confItem[GetSetting("S_HUDSW0")].setMax(ConfigRanges[GetSetting("S_HUDSW0")]);
  confItem[GetSetting("S_HUDSW1")].setMax(ConfigRanges[GetSetting("S_HUDSW1")]);
  confItem[GetSetting("S_HUDSW2")].setMax(ConfigRanges[GetSetting("S_HUDSW2")]);
  if (init_com==1){
   // READconfigMSP_init();
  }
}


public void setset(){
toggleConfItem[GetSetting("S_DISPLAYRSSI")].setValue(1);
toggleConfItem[GetSetting("S_DISPLAYVOLTAGE")].setValue(1);
toggleConfItem[GetSetting("S_AMPERAGE")].setValue(1);
toggleConfItem[GetSetting("S_AMPER_HOUR")].setValue(1);
toggleConfItem[GetSetting("S_VIDVOLTAGE")].setValue(1);
toggleConfItem[GetSetting("S_DISPLAYGPS")].setValue(1);
toggleConfItem[GetSetting("S_COORDINATES")].setValue(1);
toggleConfItem[GetSetting("S_GPSCOORDTOP")].setValue(1);
toggleConfItem[GetSetting("S_GPSALTITUDE")].setValue(1);
toggleConfItem[GetSetting("S_DISPLAY_HORIZON_BR")].setValue(1);
toggleConfItem[GetSetting("S_HORIZON_ELEVATION")].setValue(1);
toggleConfItem[GetSetting("S_WITHDECORATION")].setValue(1);
toggleConfItem[GetSetting("S_SCROLLING")].setValue(1);
toggleConfItem[GetSetting("S_SIDEBARTOPS")].setValue(1);
toggleConfItem[GetSetting("S_COMPASS")].setValue(1);
toggleConfItem[GetSetting("S_SHOWHEADING")].setValue(1);
toggleConfItem[GetSetting("S_ANGLETOHOME")].setValue(1);
toggleConfItem[GetSetting("S_COORDINATES")].setValue(1);
toggleConfItem[GetSetting("S_THROTTLEPOSITION")].setValue(1);
toggleConfItem[GetSetting("S_MODEICON")].setValue(1);
toggleConfItem[GetSetting("S_MODESENSOR")].setValue(1);
toggleConfItem[GetSetting("S_GIMBAL")].setValue(1);
toggleConfItem[GetSetting("S_VARIO")].setValue(1);
toggleConfItem[GetSetting("S_BAROALT")].setValue(1);
toggleConfItem[GetSetting("S_TIMER")].setValue(1);
toggleConfItem[GetSetting("S_GPSTIME")].setValue(1);
toggleConfItem[GetSetting("S_DISPLAY_CS")].setValue(1);
confItem[GetSetting("S_MAPMODE")].setValue(1);
}


public void addchild(){

  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
  
  for (int hudindex = 0; hudindex < hudoptions; hudindex++) {
  XML newChild = xml.addChild("LAYOUT");
      newChild.setString("desc",CONFIGHUDTEXT[hudindex]);
      newChild.setInt("enabled",CONFIGHUDEN[hudid][hudindex]);
      newChild.setInt("hud",hudsavailable);
      newChild.setInt("value",CONFIGHUD[hudid][hudindex]&0x3FF);
    }    
  XML[] xmlhudconfig = xml.getChildren("CONFIG");
  xmlhudconfig[0].setInt("value",hudsavailable+1);
  XML newChild = xml.addChild("HUDNAME");    
      newChild.setString("hudname","Custom HUD "+hudsavailable);


  saveXML(xml, dataPath("hudlayout.xml"));
  initxml();

  confItem[GetSetting("S_HUDSW2")].setMax(ConfigRanges[GetSetting("S_HUDSW2")]);
  confItem[GetSetting("S_HUDSW1")].setMax(ConfigRanges[GetSetting("S_HUDSW1")]);
  confItem[GetSetting("S_HUDSW0")].setMax(ConfigRanges[GetSetting("S_HUDSW0")]);
  confItem[GetSetting("S_HUDSW2")].setValue(hudsavailable-1);
  confItem[GetSetting("S_HUDSW1")].setValue(hudsavailable-1);
  confItem[GetSetting("S_HUDSW0")].setValue(hudsavailable-1);
}


public void coloriseswitches(){
  for(int i=0;i<CONFIGITEMS;i++) {
    if (toggleConfItem[i].getValue()==1)
      toggleConfItem[i].setColorActive(switches_);
    else
      toggleConfItem[i].setColorActive(red_);
  }
  if (SimControlToggle.getValue()==1)
    SimControlToggle.setColorActive(switches_);
  else
    SimControlToggle.setColorActive(red_);
}

public void LEW(){
//  Lock_All_Controls(true);
  G_LINKS.hide();
  LEW.show();
  LEWvisible=1;
//  Lock_All_Controls(true);
}

public void READEEMSP(){
  READconfigMSP_init();
}

public void WRITEEEMSP(){
  WRITEconfigMSP_init();
}

public void mouseClicked(){
  if ((mouseX>=(XLINKS+20)) && (mouseX<=(XLINKS+20+100)) && (mouseY>=(YLINKS+207)) && (mouseY<=(YLINKS+207+30)))
    DONATELINK();
}
PImage FullFont;

public boolean FontChanged = false;
public boolean mouseDown = false;
public boolean mouseUp = true;
PImage PreviewChar;
PImage[] CharImages = new PImage[256];
int row = 0;
int gap = 5;
int gapE = 1;
int curPixel = -1;
int curChar = -1;
int editChar = -1;
boolean mouseSet = false;
int gray = color(120);
int white = color(255);
int black = color(0);


// screen locations
int XFullFont = 25;    int YFullFont = 25;
int XcharEdit = 5;    int YcharEdit = 5;
int PreviewX = 60;     int PreviewY = 275;



Bang CharBang[] = new Bang[256] ;
Bang CharPixelsBang[] = new Bang[216] ;
Bang PreviewCharBang;
Textlabel CharTopLabel[] = new Textlabel[16] ;
Textlabel CharSideLabel[] = new Textlabel[16] ;
Textlabel LabelCurChar;
Group FG,FGFull,FGCharEdit, FGPreview;
Button buttonSendFile,buttonBrowseFile,buttonEditFont,buttonFClose,buttonFSave;


public void Font_Editor_setup() {
  for (int i=0; i<256; i++) {
    CharImages[i] = createImage(12, 18, ARGB);
  }
  
 
  
  FG = FontGroupcontrolP5.addGroup("FG")
    .setPosition(150,50)
    .setWidth(680)
    .setBarHeight(12)
    .activateEvent(true)
    .setBackgroundColor(color(200,255))
    .setBackgroundHeight(450)
    .setLabel("Font Editor")
    .setMoveable(true)
    .disableCollapse()
    .hide();
    
    ;
    FGFull = FontGroupcontrolP5.addGroup("FGFull")
    .setPosition(20,20)
    .setWidth( 16*(12+gap)+ 25)
    .setBarHeight(12)
    .activateEvent(true)
    .hideBar() 
    .setBackgroundColor(color(0,255))
    .setBackgroundHeight(16*(18+gap)+35)
    .setLabel("Character List")
    .setMoveable(true)
    .setGroup(FG);
    ;
    FGCharEdit = FontGroupcontrolP5.addGroup("FGCharEdit")
    .setPosition(350,20)
    .setWidth( 12*(10+gapE)+ 33)
    .setBarHeight(12)
    .activateEvent(true)
    .hideBar() 
    .setBackgroundColor(color(180,255))
    .setBackgroundHeight(18*(10+gapE)+120)
    .setMoveable(true)
    .setGroup(FG);
    ;
    
    
    
// RawFont = LoadFont("MW_OSD.mcm");
for (int i=0; i<256; i++) {
    int boxX = XFullFont+(i % 16) * (12+gap);
    int boxY = YFullFont + (i / 16) * (18+gap);
    CharBang[i] = FontGroupcontrolP5.addBang("CharMap"+i)
      .setPosition(boxX, boxY)
      .setSize(12, 18)
      .setLabel("")
      .setId(i)
      .setImages(CharImages[i], CharImages[i], CharImages[i], CharImages[i]) 
      .setGroup(FGFull);
    ;
   
  } 
String[] CharRows = {"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F"};
  
  for (int i=0; i<16; i++) {
    int boxX = XFullFont-15;
    int boxY = YFullFont + i * (18+gap);
    CharSideLabel[i] = FontGroupcontrolP5.addTextlabel("charlabside" +i,CharRows[i],boxX ,boxY )
    .setGroup(FGFull)
    .setColor(white);
    ;
  } 
  for (int i=0; i<16; i++) {
    int boxX = XFullFont + i * (12+gap);
    int boxY = YFullFont-15;
    CharTopLabel[i] =FontGroupcontrolP5.addTextlabel("charlabtop" +i,CharRows[i],boxX ,boxY )
    .setGroup(FGFull)
    .setColor(white);
    ;
  } 
  

 
 for (int i=0; i<216; i++) {
    int boxX = XcharEdit+(i % 12) * (12+gapE);
    int boxY = YcharEdit + (i / 12) * (12+gapE);
    CharPixelsBang[i] = FontGroupcontrolP5.addBang("CharPix"+i)
      .setPosition(boxX, boxY)
      .setSize(12, 12)
      .setLabel("")
      .setId(i)
      .setColorBackground(gray)
      .setColorForeground(gray)
      .setValue(0)
      .setGroup(FGCharEdit)
      ;
    
  }

  LabelCurChar = FontGroupcontrolP5.addTextlabel("LabelCurChar","No Index Set" ,XcharEdit+ 10,YcharEdit + 18*(12+gapE)+5)
    .setColor(white)
    .setGroup(FGCharEdit);
 
  PreviewCharBang = FontGroupcontrolP5.addBang("PreviewCharBang")
    .setPosition(FGCharEdit.getWidth() / 2 -6, PreviewY)
    .setSize(12, 18)
    .setLabel("")
    //.setId(i)
    .setImages(PreviewChar, PreviewChar, PreviewChar, PreviewChar) 
    .setGroup(FGCharEdit);
  ;
  buttonFSave = FontGroupcontrolP5.addButton("FSave",1,FGCharEdit.getWidth()-55, FGCharEdit.getBackgroundHeight()-25,45,18);//buttonFClose.setColorBackground(red_);
  buttonFSave.getCaptionLabel()
    .setFont(font12)
    .toUpperCase(false)
    .setText("SAVE");
  buttonFSave.setGroup(FGCharEdit); 
 
 
 
  buttonFClose = FontGroupcontrolP5.addButton("FCLOSE",1,680- 55 ,10,45,18);buttonFClose.setColorBackground(red_);
  buttonFClose.getCaptionLabel()
    .setFont(font12)
    .toUpperCase(false)
    .setText("CLOSE");
  buttonFClose.setGroup(FG);
 
  MGUploadF = controlP5.addGroup("MGUploadF")
                .setPosition(XFONTTOOLS,YFONTTOOLS)
                .setWidth(110)
                .setBarHeight(15)
                .activateEvent(true)
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(70)
                .setLabel("     Font Tools")
                .disableCollapse();
                //.close() 
               ; 

FileUploadText = controlP5.addTextlabel("FileUploadText","",10,5)
.setGroup(MGUploadF)
.setColorBackground(font_);

buttonEditFont = controlP5.addButton("EditFont",1,20,45,60,16)
.setColorBackground(font_)
.setGroup(MGUploadF);
buttonEditFont.getCaptionLabel()
.setText("Edit Font");

buttonSendFile = controlP5.addButton("FONT_UPLOAD",1,20,25,60,16)
.setColorBackground(font_)
.setGroup(MGUploadF);
buttonSendFile.getCaptionLabel()
.setText("UPLOAD");

buttonBrowseFile = controlP5.addButton("Browse",1,20,5,60,16)
.setColorBackground(font_)
.setGroup(MGUploadF);
buttonBrowseFile.getCaptionLabel()
.setText("  SELECT")
;

buttonSendFile.getCaptionLabel()
    .toUpperCase(false)
    .setText(" UPLOAD");
 
}

public void Send(){
  //sendFontFile();
  //g_serial.clear();
  //CreateFontFile();
}

public void MakePreviewChar(){
  PImage PreviewChar = createImage(12, 18, ARGB);
   PreviewChar.loadPixels();
  for(int byteNo = 0; byteNo < 216; byteNo++) {
    switch(PApplet.parseInt(CharPixelsBang[byteNo].value())) {
      case 0:
        PreviewChar.pixels[byteNo] = gray;
        //CharImages[charNo].pixels[CharIndex] = gray;
      break;     
      case 1:
        PreviewChar.pixels[byteNo] = black;
        //CharImages[charNo].pixels[CharIndex] = black;
      break; 
      case 2:
        PreviewChar.pixels[byteNo] = white;
        //CharImages[charNo].pixels[CharIndex] = white;
      break; 
    }
  }
  PreviewChar.updatePixels();
  PreviewCharBang.setImages(PreviewChar, PreviewChar, PreviewChar, PreviewChar); 
}

public void FSave(){
  UpdateChar();
  CreateFontFile();
}

public void FCLOSE(){
  Lock_All_Controls(false);
  FG.hide();
}

public void EditFont(){
  Lock_All_Controls(true);
 //setLock(ScontrolP5.getController("MwHeading"),true);
  FG.show();
  Lock_All_Controls(true);
}

public void changePixel(int id){
 
  if ((mouseUp) && (id > -1)){
    int curColor = PApplet.parseInt(CharPixelsBang[id].value());
    switch(curColor) {
    
    case 0: 
     CharPixelsBang[id].setColorForeground(black);
     CharPixelsBang[id].setValue(1);
     FontChanged = true;
     //println("0");
    
    break; 
    
    case 1: 
     CharPixelsBang[id].setColorForeground(white);
     CharPixelsBang[id].setValue(2);
     FontChanged = true;
     //println("1");
    
    break; 
    
    case 2: 
     CharPixelsBang[id].setColorForeground(gray);
     CharPixelsBang[id].setValue(0);
     FontChanged = true;
     //println("2");
    
    break; 
   }
  }
  curPixel = -1;
  MakePreviewChar();
 
}  

public void GetChar(int id){
 
  if ((mouseUp) && (id > -1)){
    for(int byteNo = 0; byteNo < 216; byteNo++) {
      CharPixelsBang[byteNo].setColorForeground(gray);
      CharPixelsBang[byteNo].setValue(0);
    }
    for(int byteNo = 0; byteNo < 216; byteNo++) {
      switch(CharImages[id].pixels[byteNo]) {
          case 0xFF000000:
            CharPixelsBang[byteNo].setColorForeground(black);
            CharPixelsBang[byteNo].setValue(1);
          break; 
          case 0xFFFFFFFF:
            CharPixelsBang[byteNo].setColorForeground(white);
            CharPixelsBang[byteNo].setValue(2);
          break;
          default:
            CharPixelsBang[byteNo].setColorForeground(gray);
            CharPixelsBang[byteNo].setValue(0);
          break;  
      }
      
    }
   
    LabelCurChar.setValue(str(id));
    LabelCurChar.setColorBackground(0);
    MakePreviewChar();
    editChar = id;
    //LabelCurChar.update(); 
    curChar = -1;
  }
 
}


public void UpdateChar(){
 int changechar = Integer.parseInt(LabelCurChar.getStringValue());

 
  CharImages[changechar].loadPixels();
  for(int byteNo = 0; byteNo < 216; byteNo++) {
    switch(PApplet.parseInt(CharPixelsBang[byteNo].value())) {
      case 0:
       CharImages[changechar].pixels[byteNo] = gray;
        //CharImages[charNo].pixels[CharIndex] = gray;
      break;     
      case 1:
       CharImages[changechar].pixels[byteNo] = black;
        //CharImages[charNo].pixels[CharIndex] = black;
      break; 
      case 2:
       CharImages[changechar].pixels[byteNo] = white;
        //CharImages[charNo].pixels[CharIndex] = white;
      break; 
    }
  }
  CharImages[changechar].updatePixels();
  CharBang[changechar].setImages(CharImages[changechar], CharImages[changechar], CharImages[changechar], CharImages[changechar]); 
}    
  
  
  




public void setLock(Controller theController, boolean theValue) {
  theController.setLock(theValue);
}

public void Lock_All_Controls(boolean theLock){

 //System.out.println(controlP5.getControllerList());
ControllerInterface[] sctrl = ScontrolP5.getControllerList();
   for(int i=0; i<sctrl.length; ++i)
   {
    try{
      setLock(ScontrolP5.getController(sctrl[i].getName()),theLock);
    }catch (NullPointerException e){}
   }
   
   ControllerInterface[] ctrl5 = controlP5.getControllerList();
   for(int i=0; i<ctrl5.length; ++i)
   {
    try{
      setLock(controlP5.getController(ctrl5[i].getName()),theLock);
    }catch (NullPointerException e){}
   }
}

public void Browse(){
  SwingUtilities.invokeLater(new Runnable(){
    public void run(){
      final JFileChooser fc = new JFileChooser(dataPath(""));
      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setFileFilter(new FontFileFilter());
      //fc.setCurrentDirectory();
      int returnVal = fc.showOpenDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File FontFile = fc.getSelectedFile();
        FileInputStream in = null;
        boolean completed = false;
        String error = null;
        try{
          in = new FileInputStream(FontFile) ;
          FontFileName = FontFile.getPath();
          img_Clear = LoadFont(FontFileName);
          updateConfig(); 
          JOptionPane.showMessageDialog(null,new StringBuffer().append("Font File loaded : ").append(FontFile.toURI()) );
          completed  = true;
          
        }catch(FileNotFoundException e){
                error = e.getCause().toString();

        }catch( IOException ioe){/*failed to read the file*/
                ioe.printStackTrace();
                error = ioe.getCause().toString();
        }finally{
          if (!completed){
                 // MWI.conf.clear();
                 // or we can set the properties with view values, sort of 'nothing happens'
                 //updateModel();
          }
          //updateView();
          if (in!=null){
            try{
              in.close();
            }catch( IOException ioe){/*failed to close the file*/}
          }
          
          if (error !=null){
                  JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
          }
        }
      }
    }
  }
  );  
}


public class FontFileFilter extends FileFilter {
  public boolean accept(File f) {
    if(f != null) {
      if(f.isDirectory()) {
        return true;
      }
      String extension = getExtension(f);
      if("mcm".equals(extension)) {
        return true;
      }
    }
    return false;
  }

  public String getExtension(File f) {
    if(f != null) {
      String filename = f.getName();
      int i = filename.lastIndexOf('.');
      if(i>0 && i<filename.length()-1) {
        return filename.substring(i+1).toLowerCase();
      }
    }
    return null;
  } 

  public String getDescription() {
    return "*.mcm Font File";
  }   
}
public void SetupGroups(){


  //G_EEPROM,
  //G_RSSI,
 // G_Voltage,
  //G_Amperage,
 // G_VVoltage,
 // G_Temperature,
 // G_GPS,
//  G_Other
 
  //.setColorForeground(color(30,255))
  //.setColorBackground(color(30,255))
  //.setColorLabel(color(0, 110, 220))
  //.setColorValue(0xffff88ff)
  //.setColorActive(color(30,255))
  G_PortStatus = FontGroupcontrolP5.addGroup("G_PortStatus")
    .setPosition(XPortStat,YPortStat)
    .setWidth(110)
    .setColorForeground(color(30,255))
    .setColorBackground(color(30,255))
    .setColorLabel(color(0, 110, 220))
    .setBarHeight(15)
    .setBackgroundColor(color(30,255))
    .setColorActive(red_)
    .setBackgroundHeight(60)
    .setLabel("   Port Status")
    .disableCollapse()
    ;
    
    TXText = FontGroupcontrolP5.addTextlabel("TXText","TX",65,5)
    .setColorValue(red_)
    .setGroup("G_PortStatus");
    
    RXText = FontGroupcontrolP5.addTextlabel("RXText","RX",15,5)
    .setColorValue(green_)
    .setGroup("G_PortStatus");
    

  OSD_CONTROLS = controlP5.addGroup("OSD_CONTROLS")
                .setPosition(XOSD_CONTROLS,YOSD_CONTROLS)
                .setWidth(110)
                .setBarHeight(15)
                .activateEvent(true)
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(90)
                .setLabel("   OSD CONTROLS")
                .disableCollapse();
                //.close() 
               ; 
               
  SAVE_LOAD = controlP5.addGroup("SAVE_LOAD")
                .setPosition(XSAVE_LOAD,YSAVE_LOAD)
                .setWidth(110)
                .setBarHeight(15)
                .activateEvent(true)
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(47)
                .setLabel("     SETTINGS")
                .disableCollapse();
                //.close() 
               ; 

 
  G_LINKS = controlP5.addGroup("G_LINKS")
                .setPosition(XLINKS,YLINKS)
                .setWidth(345)
                .setBarHeight(15)
                .activateEvent(true)
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(104)
                .setLabel("LINKS")
                //.hide()
                .disableCollapse()
               ;




 
   G_EEPROM = GroupcontrolP5.addGroup("G_EEPROM")
                .setPosition(XEEPROM,YEEPROM+15)
                .setWidth(Col1Width)
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorForeground(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((1*17) +5)
                .setLabel("EEPROM")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_EEPROM.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;
                G_EEPROM.hide();
G_RSSI = GroupcontrolP5.addGroup("G_RSSI")
                
                .setPosition(XRSSI,YRSSI+49)
                .setWidth(Col1Width)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((6*17) +5)
                .setLabel("RSSI")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_RSSI.captionLabel()
                
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 
              // G_RSSI.setColorBackground(color(30,255));
G_Voltage = GroupcontrolP5.addGroup("G_Voltage")
                .setPosition(XVolts,YVolts+15)
                .setWidth(Col1Width)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((5*17) +5)
                .setLabel("Main Voltage")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_Voltage.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 
 
 G_Amperage = GroupcontrolP5.addGroup("G_Amperage")
                .setPosition(XAmps,YAmps+15)
                .setWidth(Col1Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((8*17) +5)
                .setLabel("Amperage")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_Amperage.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 
 G_VVoltage = GroupcontrolP5.addGroup("G_VVoltage")
                .setPosition(XVVolts,YVVolts+15)
                .setWidth(Col1Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((3*17) +5)
                .setLabel("Video Voltage")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_VVoltage.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 
 G_Alarms = GroupcontrolP5.addGroup("G_Alarms")
                .setPosition(XTemp,YTemp+13)
                .setWidth(Col3Width+10)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((4*17) +5)
                .setLabel("Alarms")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_Alarms.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 

G_GPS = GroupcontrolP5.addGroup("G_GPS")
                .setPosition(XGPS,YGPS+16)
                .setWidth(Col3Width+10)
                .setBarHeight(16)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(16)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((4*17) +5)
                .setLabel("GPS Settings")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_GPS.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;        

G_Board= GroupcontrolP5.addGroup("G_Board")
                .hide()
                .setPosition(XBoard,YBoard+15)
                .setWidth(Col1Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((1*17) +5)
                .setLabel("Board Type")
                //.setGroup(SG)
                .disableCollapse(); 
                G_Board.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;  


               
G_CallSign = GroupcontrolP5.addGroup("G_CallSign")
                .setPosition(XCS,YCS+49)
                .setWidth(Col1Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((2*17) +5)
                .setLabel("Call Sign")
                //.setGroup(SG)
                .disableCollapse(); 
                G_CallSign.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                   

G_Other = GroupcontrolP5.addGroup("G_Other")
                .setPosition(XOther,YOther+15)
                .setWidth(Col2Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,3255))
                .setColorActive(red_)
                .setBackgroundHeight((5*17) +5)
                .setLabel("Other")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_Other.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                                          



G_DISPLAY = GroupcontrolP5.addGroup("G_DISPLAY")
                .setPosition(XDisplay,YDisplay+15)
                .setWidth(Col2Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,3255))
                .setColorActive(red_)
                .setBackgroundHeight((6*17) +5)
                .setLabel("Display")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_DISPLAY.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                                          


G_HUD = GroupcontrolP5.addGroup("G_HUD")
                .setPosition(XHUD,YHUD+15)
                .setWidth(Col2Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,3255))
                .setColorActive(red_)
                .setBackgroundHeight((5*17) +5)
                .setLabel("HUD")
                //.setGroup(G_LINKS)
                .disableCollapse() 
                ; 
                G_HUD.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                                          

G_RCSWITCH = GroupcontrolP5.addGroup("G_RCSWITCH")
                .setPosition(XHUD+735,22)
                .setWidth(Col3Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,3255))
                .setColorActive(red_)
                .setBackgroundHeight((7*17) +5)
                .setLabel("LAYOUT")
                //.setGroup(G_LINKS)
                .disableCollapse() 
                ; 
                G_RCSWITCH.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                                          

G_COMPASS = GroupcontrolP5.addGroup("G_COMPASS")
                .setPosition(XCOMPASS,YCOMPASS+15)
                .setWidth(Col3Width+10)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,3255))
                .setColorActive(red_)
                .setBackgroundHeight((4*17) +5)
                .setLabel("Compass")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_COMPASS.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                                          


G_VREF = GroupcontrolP5.addGroup("G_VREF")
                .setPosition(XVREF,YVREF+49)
                .setWidth(Col1Width)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,3255))
                .setColorActive(red_)
                .setBackgroundHeight((1*17) +5)
                .setLabel("Reference Voltage")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_VREF.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;                                          

G_TIME = GroupcontrolP5.addGroup("G_TIME")
                .setPosition(XTIME,YTIME+16)
                .setWidth(Col3Width+10)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(16)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((3*17) +5)
                .setLabel("Time Settings")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_TIME.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;   
           
G_INFO = GroupcontrolP5.addGroup("G_INFO")
                .setPosition(windowsX/2,windowsY/2)
                .setWidth(Col3Width+10)
                .setColorForeground(yellow_)
//                .setColorBackground(blue_)
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(0)
                .setBackgroundColor(blue_)
                .setColorActive(red_)
                .setBackgroundHeight((3*17) +5)
                .setLabel("")
                .hide()

                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_TIME.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;        
     
/*
G_SPORT = GroupcontrolP5.addGroup("G_SPORT")
                .setPosition(XSPORT,YSPORT)
                .setWidth(Col3Width+10)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(16)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((1*17) +5)
                .setLabel("FrSky S.Port Data in OSD")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_SPORT.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ;        
*/  

 G_Debug = GroupcontrolP5.addGroup("G_Debug")
//                .setPosition(5,250)
                .setPosition(693,541)
                .setWidth(130)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse() 
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight(33)
                .setLabel("Debug")
         //       .setGroup(SG)
                ; 
                G_Debug.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 


}

public boolean toggleRead = false,
        toggleMSP_Data = true,
        toggleReset = false,
        toggleCalibAcc = false,
        toggleCalibMag = false,
        toggleWrite = false,
        toggleSpekBind = false,
        toggleSetSetting = false;
Serial g_serial;      // The serial port
float LastPort = 0;
int time,time1,time2,time3,time4,time5;

boolean ClosePort = false;
boolean PortIsWriting = false;
boolean FontMode = false;
int FontCounter = 0;
//int cindex = 0;
int CloseMode = 0;

/******************************* Multiwii Serial Protocol **********************/
//String boxnames[] = { // names for dynamic generation of config GUI
    //"ANGLE;",
    //"HORIZON;",
    //"BARO;",
    //"MAG;",
    //"ARM;",
    //"LLIGHTS;",
    //"GPS HOME;",
   // "GPS HOLD;",
   // "OSD SW;",
    
  //};


String boxnames[] = { // names for dynamic generation of config GUI
    "ARM;",
    "ANGLE;",
    "HORIZON;",
    "BARO;",
    "MAG;",
    "CAMSTAB;",
    "GPS HOME;",
    "GPS HOLD;",
    "MISSION;",
    "OSD SW;"    
  };
String strBoxNames = join(boxnames,""); 
//int modebits = 0;

private static final String MSP_HEADER = "$M<";

private static final int
  MSP_IDENT                =100,
  MSP_STATUS               =101,
  MSP_RAW_IMU              =102,
  MSP_SERVO                =103,
  MSP_MOTOR                =104,
  MSP_RC                   =105,
  MSP_RAW_GPS              =106,
  MSP_COMP_GPS             =107,
  MSP_ATTITUDE             =108,
  MSP_ALTITUDE             =109,
  MSP_ANALOG               =110,
  MSP_RC_TUNING            =111,
  MSP_PID                  =112,
  MSP_BOX                  =113,
  MSP_MISC                 =114,
  MSP_MOTOR_PINS           =115,
  MSP_BOXNAMES             =116,
  MSP_PIDNAMES             =117,
  MSP_BOXIDS               =119,
  MSP_RSSI                 =120,
  MSP_NAV_STATUS           =121,
  MSP_CELLS                =130,
  MSP_SET_RAW_RC           =200,
  MSP_SET_RAW_GPS          =201,
  MSP_SET_PID              =202,
  MSP_SET_BOX              =203,
  MSP_SET_RC_TUNING        =204,
  MSP_ACC_CALIBRATION      =205,
  MSP_MAG_CALIBRATION      =206,
  MSP_SET_MISC             =207,
  MSP_RESET_CONF           =208,
  MSP_SELECT_SETTING       =210,
  MSP_SPEK_BIND            =240,

  MSP_PASSTHRU_SERIAL      =244,

  MSP_EEPROM_WRITE         =250,
  
  MSP_DEBUGMSG             =253,
  MSP_DEBUG                =254;

private static final int
  MSP_OSD                  =220;

// Subcommands
private static final int
  OSD_NULL                 =0,
  OSD_READ_CMD             =1,
  OSD_WRITE_CMD            =2,
  OSD_GET_FONT             =3,
  OSD_SERIAL_SPEED         =4,
  OSD_RESET                =5,
  OSD_DEFAULT              =6,
  OSD_WRITE_CMD_EE         =8,
  OSD_READ_CMD_EE          =9;


// initialize the serial port selected in the listBox
public void InitSerial(float portValue) {
  if (portValue < commListMax) {
    if(init_com == 0){ 
      try{
      if (StartupMessage ==0){
        StartupMessage=1;
        updateConfig();
      }
      String portPos = Serial.list()[PApplet.parseInt(portValue)];
      txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
      g_serial = new Serial(this, portPos, BaudRate);
      LastPort = portValue;
      init_com=1;
      toggleMSP_Data = true;
      ClosePort = false;
      buttonREAD.setColorBackground(green_);
      buttonWRITE.setColorBackground(green_);
      buttonRESET.setColorBackground(green_);
      commListbox.setColorBackground(green_);
      buttonRESTART.setColorBackground(green_);
      
      g_serial.buffer(512);
            txtmessage.setText("");
//      delay(1500);
      SendCommand(MSP_IDENT);
     

      SendCommand(MSP_STATUS);
       eeaddressGUI=0;   
       ReadConfigMSPMillis=20000+millis();  
       READconfigMSP_init();
       } catch (Exception e) { // null pointer or serial port dead
         noLoop();
        JOptionPane.showConfirmDialog(null,"Error Opening Port It may be in Use", "Port Error", JOptionPane.PLAIN_MESSAGE,JOptionPane.WARNING_MESSAGE);
        loop();
        System.out.println("OpenPort error " + e);
     }
      //+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF));
    }
  }
  else if(portValue > commListMax && init_com == 1){
    if (Passthroughcomm==1) {
      System.out.println("Serial Port Pass Through Starting" );
      SendCommand(MSP_PASSTHRU_SERIAL);
      delay(1000);
      SendCommand(MSP_IDENT);
      SendCommand(MSP_STATUS);
      READconfigMSP_init();
    }
  }
  else {
    if(init_com == 1){
//     System.out.println("Begin Port Down " ); 
      txtlblWhichcom.setValue("Comm Closed");
      //g_serial.clear();
      toggleMSP_Data = false;
      ClosePort = true;
      init_com=0;
    }
  }
  SimControlToggle.setValue(0);
}

public void ClosePort(){
  init_com=0;
  g_serial.clear();
  g_serial.stop();
  
  //System.out.println("Port Turned Off " );
  init_com=0;
  commListbox.setColorBackground(red_);
  buttonREAD.setColorBackground(osdcontr_);
  buttonRESET.setColorBackground(osdcontr_);
  buttonWRITE.setColorBackground(osdcontr_);
  buttonRESTART.setColorBackground(osdcontr_);
  if (CloseMode > 0){
    InitSerial(LastPort);
    CloseMode = 0;
  }
    
  
}




public void SetConfigItem(int index, int value) {
  if(index >= CONFIGITEMS)
    return;

  if(index == GetSetting("S_VOLTAGEMIN"))confItem[index].setValue(PApplet.parseFloat(value)/10);//preserve decimal
  else if(index == GetSetting("S_GPSTZ"))confItem[index].setValue(PApplet.parseFloat(value)/10);//preserve decimal, maybe can go elsewhere - haydent
  else confItem[index].setValue(value);
//  if (index == CONFIGITEMS-1)
//    buttonWRITE.setColorBackground(green_);
    
  if (value >0){
    toggleConfItem[index].setValue(1);
  }
  else{
    toggleConfItem[index].setValue(0);
  }

  try{
    switch(value) {
    case(0):
      RadioButtonConfItem[index].activate(0);
      break;
    case(1):
      RadioButtonConfItem[index].activate(1);
      break;
    }
  }
  catch(Exception e) {
  }finally {
  }
BuildCallSign();  	
}

public void PORTCLOSE(){
  toggleMSP_Data = false;
  CloseMode = 0;
  InitSerial(200.00f);
}

public void BounceSerial(){
  toggleMSP_Data = false;
  CloseMode = 1;
  InitSerial(200.00f);
  
}  

public void RESTART(){
  toggleMSP_Data = true;
  for (int txTimes = 0; txTimes<3; txTimes++) {
    headSerialReply(MSP_OSD, 1);
    serialize8(OSD_RESET);
    tailSerialReply();
    delay(100);
  }
  toggleMSP_Data = false;
  READconfigMSP_init();
}  






public void FONT_UPLOAD(){
  if (init_com==0){
    noLoop();
    JOptionPane.showConfirmDialog(null,"Please Select a Port", "Not Connected", JOptionPane.PLAIN_MESSAGE,JOptionPane.WARNING_MESSAGE);
    loop();
  }else
  {
  SimControlToggle.setValue(0);
//  System.out.println("FONT_UPLOAD");
  //toggleMSP_Data = true;
  FontMode = true;
  PortWrite = true;
  MakePorts();
  FontCounter = 0;
  txtmessage.setText("  Please Wait");
  p = 0;
  inBuf[0] = OSD_GET_FONT;
  //for (int txTimes = 0; txTimes<2; txTimes++) {
    headSerialReply(MSP_OSD, 5);
    serialize8(OSD_GET_FONT);
    serialize16(7456);  // safety code
    serialize8(0);    // first char
    serialize8(255);  // last char
    tailSerialReply();
 //}
  }

}

public void SendChar(){
    time2=time;
    PortWrite = !PortWrite;  // toggle PortWrite to flash TX
    if (PortWrite) 
      txtmessage.setText("Please wait....");
    else
      txtmessage.setText("");
    MakePorts();
//    System.out.println("Sent Char "+FontCounter);
    buttonSendFile.getCaptionLabel().setText("  " +nf(FontCounter, 3)+"/256");
    headSerialReply(MSP_OSD, 56);
    serialize8(OSD_GET_FONT);
    for(int i = 0; i < 54; i++){
      serialize8(PApplet.parseInt(raw_font[FontCounter][i]));
    }
    serialize8(FontCounter);
    tailSerialReply();
    if (FontCounter <255){
      FontCounter++;
    }else{ 
      g_serial.clear();
      PortWrite = false;
      FontMode = false;      
//      System.out.println("Finished Uploading Font");
      buttonSendFile.getCaptionLabel().setText("  Upload");
      txtmessage.setText("");
      READconfigMSP_init();
//      READinit();
      ReadConfig=100;
      RESTART();
    } 
  
}


public void DEFAULT(){
 if (init_com==1){
//    toggleConfItem[0].setValue(0);
    noLoop();
    int Reset_result = JOptionPane.showConfirmDialog(this,"Are you sure you wish to set OSD to DEFAULT values?", "RESET OSD MEMORY",JOptionPane.WARNING_MESSAGE,JOptionPane.YES_NO_CANCEL_OPTION);
    loop();
    switch (Reset_result) {
      case JOptionPane.YES_OPTION:
        toggleMSP_Data = true;
        for (int txTimes = 0; txTimes<3; txTimes++) {
          headSerialReply(MSP_OSD, 1);
          serialize8(OSD_DEFAULT);
          tailSerialReply();
          delay(100);
        }
        toggleMSP_Data = false;
//        READinit();
        READconfigMSP_init();
//        delay(2000);     
//        ReadConfig=100;
        return;
      case JOptionPane.CANCEL_OPTION:
//        SimControlToggle.setValue(1);
        return;
      default:
//        SimControlToggle.setValue(1);
        return;
    }
 }else
 {
   noLoop();
   JOptionPane.showConfirmDialog(null,"Please Select a Port", "Not Connected", JOptionPane.PLAIN_MESSAGE,JOptionPane.WARNING_MESSAGE);
   loop();
 }

}




public void SendCommand(int cmd){
  //int icmd = (int)(cmd&0xFF);
  switch(cmd) {
  
  case MSP_STATUS:
        PortIsWriting = true;
        Send_timer+=1;
        headSerialReply(MSP_STATUS, 11);
        serialize16(Send_timer);
        serialize16(0);
        serialize16(1|1<<1|1<<2|1<<3|0<<4);
        int modebits = 0;
        int BitCounter = 1;
        if(toggleModeItems[0].getValue()> 0) modebits |=1<<0;
        if(toggleModeItems[1].getValue()> 0) modebits |=1<<1;
        if(toggleModeItems[2].getValue()> 0) modebits |=1<<2;
        if(toggleModeItems[3].getValue()> 0) modebits |=1<<3;
        if(toggleModeItems[4].getValue()> 0) modebits |=1<<5;
        if(toggleModeItems[5].getValue()> 0) modebits |=1<<8;
        if(toggleModeItems[6].getValue()> 0) modebits |=1<<10;
        if(toggleModeItems[7].getValue()> 0) modebits |=1<<11;
        if(toggleModeItems[8].getValue()> 0) modebits |=1<<20;
        if(toggleModeItems[9].getValue()> 0) modebits |=1<<19;
//        if(toggleModeItems[8].getValue()> 0) modebits |=1<<16; //Also send LLIGHTS when OSD enabled - for testing
//        if(toggleModeItems[5].getValue()> 0) modebits |=1<<12; //Also send PASS when CAMSTAB enabled - for testing
        serialize32(modebits);
        serialize8(0);   // current setting
        tailSerialReply();
        PortIsWriting = false;
      break;
      
      case MSP_RC:
        PortIsWriting = true;
        headSerialReply(MSP_RC, 14);
        serialize16(PApplet.parseInt(Pitch_Roll.arrayValue()[0]));
        serialize16(PApplet.parseInt(Pitch_Roll.arrayValue()[1]));
        serialize16(PApplet.parseInt(Throttle_Yaw.arrayValue()[0]));
        serialize16(PApplet.parseInt(Throttle_Yaw.arrayValue()[1]));
        for (int i=5; i<8; i++) {
          serialize16(1500);
        }
        tailSerialReply();
        PortIsWriting = false;
      break;
      
      case MSP_IDENT:
        PortIsWriting = true;
        headSerialReply(MSP_IDENT, 7);
        serialize8(101);   // multiwii version
        serialize8(0); // type of multicopter
        serialize8(0);         // MultiWii Serial Protocol Version
        serialize32(0);        // "capability"
        tailSerialReply();
        PortIsWriting = false;
      break;
 
      case MSP_PASSTHRU_SERIAL:
        PortIsWriting = true;
        serialize8('$');
        serialize8('M');
        serialize8('<');//this is the important bit
        outChecksum = 0; // start calculating a new checksum
        serialize8(1);
        serialize8(MSP_PASSTHRU_SERIAL);
        serialize8(0);
        tailSerialReply();
        PortIsWriting = false;
      break;

      case MSP_CELLS:
        PortIsWriting = true;
        headSerialReply(MSP_CELLS, 12);
        if (SFRSKY.arrayValue()[0]<1) break;
       if (PApplet.parseInt(confItem[GetSetting("S_BATCELLS")].value()) <1) break;
        int cellvolt= (PApplet.parseInt(sVBat * 100))/PApplet.parseInt(confItem[GetSetting("S_BATCELLS")].value());
        for (int i=0; i<6; i++) {
          if (i < PApplet.parseInt(confItem[GetSetting("S_BATCELLS")].value())) {
            serialize16(cellvolt);
          }
          else {
            serialize16(0);
          }
        }
        tailSerialReply();
        PortIsWriting = false;
      break;


      case MSP_BOXNAMES:
        PortIsWriting = true;
        headSerialReply(MSP_BOXNAMES,strBoxNames.length());
        serializeNames(strBoxNames.length());
        tailSerialReply();
        PortIsWriting = false;
      break;
      
      case MSP_BOXIDS:
        PortIsWriting = true;
        headSerialReply(MSP_BOXIDS,23);
        for (int i=0; i<23; i++) {
        serialize8(i);
        }
        tailSerialReply();
        PortIsWriting = false;
    break;

     
      case MSP_ATTITUDE:
                PortIsWriting = true;
        headSerialReply(MSP_ATTITUDE, 8);
        serialize16(PApplet.parseInt(MW_Pitch_Roll.arrayValue()[0])*10);
        serialize16(PApplet.parseInt(MW_Pitch_Roll.arrayValue()[1])*10);
        serialize16(MwHeading);
        serialize16(0);
        tailSerialReply();
        PortIsWriting = false;
      break;
     


      case MSP_DEBUG:
        PortIsWriting = true;
        headSerialReply(MSP_DEBUG, 8);

        //for (int i = 0; i < 4; i++) {
        //  debug[i]++;
        //}

        serialize16(debug[0]);
        serialize16(debug[1]);
        serialize16(debug[2]);
        serialize16(debug[3]);
        tailSerialReply();
        PortIsWriting = false;
      break;
     
     
      case MSP_PID:
        int PIDITEMS = 10;
        PortIsWriting = true;
        headSerialReply(MSP_PID, 10*3);
        for(int i=0; i<PIDITEMS; i++) {
          for(int ii=0; ii<3; ii++) {
           int pidval=PApplet.parseInt(i*10)+ii;
            serialize8(pidval);
          }
        }

        tailSerialReply();
        PortIsWriting = false;
      break;

      case MSP_ANALOG:
        PortIsWriting = true;
        headSerialReply(MSP_ANALOG, 7);
        serialize8(PApplet.parseInt(sVBat * 10));
        serialize16(0);
        serialize16(PApplet.parseInt(sMRSSI));
        serialize16(PApplet.parseInt(map(Throttle_Yaw.arrayValue()[1],1000,2000,0,50000)));
        tailSerialReply();
        PortIsWriting = false;
      break;
      

      case MSP_NAV_STATUS:
        PortIsWriting = true;
        headSerialReply(MSP_NAV_STATUS, 7);
        if (millis()>oldwpmillis+1500){
          oldwpmillis=millis();
          wpno++;
          if (wpno>15) wpno=0;
        }
        serialize8(0);
        serialize8(0);
        serialize8(0);
        serialize8(wpno);
        serialize8(0);
        serialize8(0);
        serialize8(0);
        tailSerialReply();
        PortIsWriting = false;
      break;
 
     
      case MSP_RAW_GPS:
        // We have: GPS_fix(0-2), GPS_numSat(0-15), GPS_coord[LAT & LON](signed, in 1/10 000 000 degres), GPS_altitude(signed, in meters) and GPS_speed(in cm/s)  
        headSerialReply(MSP_RAW_GPS,16);
        serialize8(PApplet.parseInt(SGPS_FIX.arrayValue()[0]));
        serialize8(PApplet.parseInt(SGPS_numSat.value()));
        serialize32(430948610);
        serialize32(-718897060);
        serialize16(PApplet.parseInt(SGPS_altitude.value()/100));
        serialize16(PApplet.parseInt(SGPS_speed.value()));
        serialize16(MwHeading*10);     
    break;
    
  
    case MSP_COMP_GPS:
      if(confItem[GetSetting("S_GPSTIME")].value()>0)
        headSerialReply(MSP_COMP_GPS,9);
      else
        headSerialReply(MSP_COMP_GPS,5);
      serialize16(PApplet.parseInt(SGPS_distanceToHome.value()));
      int GPSheading = PApplet.parseInt(SGPSHeadHome.value());
      if(GPSheading < 0) GPSheading += 360;
      serialize16(GPSheading);
      serialize8(0);
      if(confItem[GetSetting("S_GPSTIME")].value()>0) {
        int osdtime=hour()*3600+minute()*60+second();
        osdtime = osdtime*1000;
        serialize32(osdtime);
      }
    break;
    
    
    case MSP_ALTITUDE:
      headSerialReply(MSP_ALTITUDE, 6);
      serialize32(PApplet.parseInt(sAltitude) *100);
      serialize16(PApplet.parseInt(sVario) *10);     
    break;
      
    }
    tailSerialReply();   
  
}

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
public String shortifyPortName(String portName, int maxlen)  {
  String shortName = portName;
  if(shortName.startsWith("/dev/")) shortName = shortName.substring(5);  
  if(shortName.startsWith("tty.")) shortName = shortName.substring(4); // get rid of leading tty. part of device name
  if(portName.length()>maxlen) shortName = shortName.substring(0,(maxlen-1)/2) + "~" +shortName.substring(shortName.length()-(maxlen-(maxlen-1)/2));
  if(shortName.startsWith("cu.")) shortName = "";// only collect the corresponding tty. devices
  return shortName;
}

public static final int
  IDLE = 0,
  HEADER_START = 1,
  HEADER_M = 2,
  HEADER_ARROW = 3,
  HEADER_SIZE = 4,
  HEADER_CMD = 5,
  HEADER_ERR = 6;

private static final String MSP_SIM_HEADER = "$M>";
int c_state = IDLE;
boolean err_rcvd = false;
byte checksum=0;
byte cmd = 0;
int offset=0, dataSize=0;
byte[] inBuf = new byte[256];
int Send_timer = 1;
int p=0;
public int read32() {return (inBuf[p++]&0xff) + ((inBuf[p++]&0xff)<<8) + ((inBuf[p++]&0xff)<<16) + ((inBuf[p++]&0xff)<<24); }
public int read16() {return (inBuf[p++]&0xff) + ((inBuf[p++])<<8); }
public int read8()  {return inBuf[p++]&0xff;}



int outChecksum;


public void serialize8(int val) {
 if (init_com==1)  {
     //if(str(val)!=null){
       try{
       g_serial.write(val);
       outChecksum ^= val;
       } catch(java.lang.Throwable t) {
         System.out.println( t.getClass().getName() ); //this'll tell you what class has been thrown
         t.printStackTrace(); //get a stack trace
       }
     //}
 }

}

public void serialize16(int a) {
  if (str(a)!=null ){
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
  }
}

public void serialize32(int a) {
  if (str(a)!=null ){
    serialize8((a    ) & 0xFF);
    serialize8((a>> 8) & 0xFF);
    serialize8((a>>16) & 0xFF);
    serialize8((a>>24) & 0xFF);
  }
 
}

public void serializeNames(int s) {
  //for (PGM_P c = s; pgm_read_byte(c); c++) {
   // serialize8(pgm_read_byte(c));
  //}
  for (int c = 0; c < strBoxNames.length(); c++) {
    serialize8(strBoxNames.charAt(c));
  }
}

public void headSerialResponse(int requestMSP, Boolean err, int s) {
  //if (FontMode)DelayTimer(1);
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  outChecksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(requestMSP);
}

public void headSerialReply(int requestMSP, int s) {
  if ((str(requestMSP) !=null) && (str(s)!=null)){
    headSerialResponse(requestMSP, false, s);
  }
}

//void headSerialError(int requestMSP, int s) {
// headSerialResponse(requestMSP, true, s);
//}

public void tailSerialReply() {
  if (outChecksum > 0) serialize8(outChecksum);
}

public void DelayTimer(int ms){
  int time = millis();
  while(millis()-time < ms);
}

public void evaluateCommand(byte cmd, int size) {
  if ((init_com==0)  || (toggleMSP_Data == false)){
    return;
  }
  PortRead = true;
  MakePorts(); 
  int icmd = PApplet.parseInt(cmd&0xFF);
  if (icmd !=MSP_OSD)return;  //System.out.println("Not Valid Command");
  time2=time;

  switch(icmd) {
    
    case MSP_OSD:
      int cmd_internal = read8();
      PortRead = true;
      MakePorts();

      if(cmd_internal == OSD_NULL) {
      }

      if(cmd_internal == OSD_READ_CMD_EE) { // response to a read / write request
        if(size == 3) { // confirmed write request received
          int eeaddressOSDL=read8();
          int eeaddressOSDH=read8();
          eeaddressOSD=eeaddressOSDL+(eeaddressOSDH<<8);
          if (eeaddressOSD>=eeaddressGUI){ // update base address
            eeaddressGUI=eeaddressOSD;
          }
          if (WriteLayouts==1){
            if (eeaddressGUI>=(CONFIGITEMS + (hudoptions*2*3))){ // hit end address
              WriteConfigMSPMillis=0;
            }
          }              
          else{
            if (eeaddressGUI>=(CONFIGITEMS)){ // hit end address
              WriteConfigMSPMillis=0;
            }
          }
        }
        else{ // confirmed read request received
          for(int i=0; i<10; i++) {
            eeaddressOSD=read8();
            eeaddressOSD=eeaddressOSD+read8();
            eedataOSD=read8();
            if (eeaddressOSD==0){
              confCheck=eedataOSD;
            }
            if (eeaddressOSD==GetSetting("S_HUDSW0")){
              OSD_S_HUDSW0=eedataOSD;
            }
            if (eeaddressOSD==GetSetting("S_HUDSW1")){
              OSD_S_HUDSW1=eedataOSD;
            }
            if (eeaddressOSD==GetSetting("S_HUDSW2")){
              OSD_S_HUDSW2=eedataOSD;
            }
            if (eeaddressOSD<CONFIGITEMS){
              SetConfigItem(eeaddressOSD, eedataOSD);
            }
            if (eeaddressOSD==GetSetting("S_AMPMAXH")){ // 16 bit value amps manipulation now received all data
              S16_AMPMAX=(PApplet.parseInt(confItem[GetSetting("S_AMPMAXH")].value())<<8)+ PApplet.parseInt(confItem[GetSetting("S_AMPMAXL")].value()); 
              SetConfigItem(GetSetting("S_AMPDIVIDERRATIO"), (int) S16_AMPMAX);
            }
            if (eeaddressOSD==eeaddressGUI){ // update base address
              eeaddressGUI++;
            }
          }
          if (eeaddressGUI>=(CONFIGITEMS)){ // hit end address config only
            ReadConfigMSPMillis=0;
          }
        }
        if (MW_OSD_EEPROM_Version!=confCheck){
          noLoop();
          JOptionPane.showConfirmDialog(null,"GUI version does not match OSD version - use correct GUI version or update OSD.", "Version Mismatch Warning", JOptionPane.PLAIN_MESSAGE,JOptionPane.WARNING_MESSAGE);
          loop();   
          ReadConfigMSPMillis=0;
          WriteConfigMSPMillis=0;   
        }        
      }


        if(cmd_internal == OSD_GET_FONT) {
          if( size == 1) {
            //headSerialReply(MSP_OSD, 5);
            //serialize8(OSD_GET_FONT);
            //serialize16(7456);  // safety code
            //serialize8(0);    // first char
            //serialize8(255);  // last char
           // tailSerialReply();
          }
          if(size == 3) {
           
            //txtmessage.setText("  Please Wait");
           // PortRead = true;
            //PortWrite = true; 
            //int cindex = read16();
            //if((cindex&0xffff) == 0xffff) { // End!
              //FontCounter = 255;
             // headSerialReply(MSP_OSD, 1);
             // serialize8(OSD_NULL);
             // tailSerialReply();
             // toggleMSP_Data = false;
             // g_serial.clear();
             // PortRead = false;
             // PortWrite = false;
              //FontMode = false;      
             // System.out.println("End marker "+cindex);
             // buttonSendFile.getCaptionLabel().setText("  Upload");
             // txtmessage.setText("");
              //InitSerial(200.00);
             // RESTART();
             // g_serial.clear();
             // g_serial.stop();
             // g_serial = new Serial(this, Serial.list()[int(LastPort)], 115200);
             // READ();
              
            //}
            //else {
              //PortWrite = true;
              //MakePorts();
              //headSerialReply(MSP_OSD, 56);
              //serialize8(OSD_GET_FONT);
             // for(int i = 0; i < 54; i++){
                //serialize8(int(raw_font[cindex][i]));
              //}
              //serialize8(cindex);
              //tailSerialReply(); 
              
//     // XXX Fake errors to force retransmission
//        if(int(random(3)) == 0) {
//          System.out.println("Messed char "+cindex);
//          outChecksum ^= int(random(1,256));
//        }
//        else
//     // End fake errors code
          //System.out.println("Sent Char "+cindex);
          //buttonSendFile.getCaptionLabel().setText("  " +nf(cindex, 3)+"/256");
        //}
      }
    }
    break;
  }
}

public void MWData_Com() {
  if ((toggleMSP_Data == false) ||(init_com==0)) return;
  int i,aa;
  float val,inter,a,b,h;
  int c = 0;
  
  //System.out.println("MWData_Com");  
    
    while (g_serial.available()>0 && (toggleMSP_Data == true)) {
    try{
      c = (g_serial.read());
      
     if (str(c) == null)return;
      
      PortRead = true;
      MakePorts();
      if (c_state == IDLE) {
        c_state = (c=='$') ? HEADER_START : IDLE;
      }
      else if (c_state == HEADER_START) {
        c_state = (c=='M') ? HEADER_M : IDLE;
      }
      else if (c_state == HEADER_M) {
        if (c == '<') {
          c_state = HEADER_ARROW;
        } else if (c == '!') {
          c_state = HEADER_ERR;
        } else {
          c_state = IDLE;
        }
      }
      else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
        /* is this an error message? */
        err_rcvd = (c_state == HEADER_ERR);        /* now we are expecting the payload size */
        dataSize = (c&0xFF);
        /* reset index variables */
        p = 0;
        offset = 0;
        checksum = 0;
        checksum ^= (c&0xFF);
        /* the command is to follow */
        c_state = HEADER_SIZE;
      }
      else if (c_state == HEADER_SIZE) {
        cmd = (byte)(c&0xFF);
        checksum ^= (c&0xFF);
        c_state = HEADER_CMD;
      }
      else if (c_state == HEADER_CMD && offset < dataSize) {
          checksum ^= (c&0xFF);
          inBuf[offset++] = (byte)(c&0xFF);
      } 
      else if (c_state == HEADER_CMD && offset >= dataSize) {
        /* compare calculated and transferred checksum */
        if ((checksum&0xFF) == (c&0xFF)) {
          if (err_rcvd) {
            //System.err.println("Copter did not understand request type "+c);
          } else {
            /* we got a valid response packet, evaluate it */
            try{
              if ((init_com==1)  && (toggleMSP_Data == true)) {
                  evaluateCommand(cmd, (int)dataSize);
                  //System.out.println("CMD: "+cmd);
                  PortRead = false;
              }
              else{
//                System.out.println("port is off ");
              }
              
              
              } catch (Exception e) { // null pointer or serial port dead
              System.out.println("write error " + e);
              }
           
          }
        }
        else {
          System.out.println("invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF));
          System.out.print("<"+(cmd&0xFF)+" "+(dataSize&0xFF)+"> {");
          for (i=0; i<dataSize; i++) {
            if (i!=0) { System.err.print(' '); }
            System.out.print((inBuf[i] & 0xFF));
          }
          System.out.println("} ["+c+"]");
          System.out.println(new String(inBuf, 0, dataSize));
        }
        c_state = IDLE;
        
      }
      
      } catch(java.lang.Throwable t) {
         System.out.println( t.getClass().getName() ); //this'll tell you what class has been thrown
         t.printStackTrace(); //get a stack trace
      }
    }
}

  public void READconfigMSP_init(){
//    println("Console test print");
    SimControlToggle.setValue(0);
    ReadConfigMSPMillis=1000+millis(); 
    WriteConfigMSPMillis=millis(); 
    eeaddressGUI=0;   
  }

  public void READconfigMSP(){
  toggleMSP_Data = true;
  inBuf[0] = OSD_WRITE_CMD;
  for (int txTimes = 0; txTimes<1; txTimes++) {
    headSerialReply(MSP_OSD, 4);
    serialize8(OSD_READ_CMD_EE);
    int tmpeeadd = eeaddressGUI&0xFF;
    serialize8(tmpeeadd);
    tmpeeadd = eeaddressGUI>>8;
    serialize8(tmpeeadd);
    serialize8(99);
    tailSerialReply();
//    SimControlToggle.setValue(1);    
    SendSim=1;
    ClosePort = false; 

  }
//  toggleMSP_Data = false; //???????????????????
  ReadConfigMSPMillis=1000+millis(); 
}

  public void WRITEconfigMSP_init(){
    eeaddressGUI=0;  
    CheckCallSign(); 
    EElookuptableReSet();

    WriteLayouts=0;
    if (OSD_S_HUDSW0!=PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value())){
      OSD_S_HUDSW0=PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value());
      WriteLayouts=1;
    }
    if (OSD_S_HUDSW1!=PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value())){
      OSD_S_HUDSW1=PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
      WriteLayouts=1;
    }
    if (OSD_S_HUDSW2!=PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value())){
      OSD_S_HUDSW2=PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
      WriteLayouts=1;
    }

    EepromWriteSize = CONFIGITEMS +1;
    if (WriteLayouts==1){
      EepromWriteSize = CONFIGITEMS + (hudoptions*2*3) +1;
    }

    WriteConfigMSPMillis=1000+millis(); 
  }

  public void WRITEconfigMSP(){
  SimControlToggle.setValue(0);
  toggleMSP_Data = true;
  inBuf[0] = OSD_WRITE_CMD;
  for (int txTimes = 0; txTimes<1; txTimes++) {
    headSerialReply(MSP_OSD, 1 + (3*10));
    serialize8(OSD_WRITE_CMD_EE);
    for(int i=0; i<10; i++) {
      int tmpeeadd = (i+eeaddressGUI)&0xFF;
      serialize8(tmpeeadd);
      tmpeeadd = (i+eeaddressGUI)>>8;
      serialize8(tmpeeadd);
      serialize8(EElookuptable[i+eeaddressGUI]);
//System.out.println(eeaddressGUI+i+":"+EElookuptable[i+eeaddressGUI]);
    }
    tailSerialReply();
  }
  debug[1]=eeaddressGUI;
  debug[2]++;
//  toggleMSP_Data = false; //???????????????????
  WriteConfigMSPMillis=3000+millis(); 
}

  public void EElookuptableReSet(){ // preparing for a write
    confItem[GetSetting("S_AMPMAXL")].setValue(PApplet.parseInt(confItem[GetSetting("S_AMPDIVIDERRATIO")].value())&0xFF); // for 8>>16 bit EEPROM
    confItem[GetSetting("S_AMPMAXH")].setValue(PApplet.parseInt(confItem[GetSetting("S_AMPDIVIDERRATIO")].value())>>8);
    for(int i = 0; i < CONFIGITEMS; i++){
      if(i == GetSetting("S_VOLTAGEMIN")){
        EElookuptable[i]=PApplet.parseInt(confItem[i].value()*10);
      }
      else if(i == GetSetting("S_GPSTZ")){
        EElookuptable[i]=PApplet.parseInt(confItem[i].value()*10);
      }
      else if(i == GetSetting("S_AMPDIVIDERRATIO")){
        EElookuptable[i]=0;
      }
      else{
        EElookuptable[i]=PApplet.parseInt(confItem[i].value());
      }

    }
//     for(int i = 0; i < (hudoptions); i++){
//       ConfigLayout[0][i]=CONFIGHUD[int(confItem[GetSetting("S_HUD")].value())][i];
//       ConfigLayout[1][i]=CONFIGHUD[int(confItem[GetSetting("S_HUDOSDSW")].value())][i];
//     }

     int EElookuptableaddress=CONFIGITEMS;
     for(int i = 0; i < (hudoptions); i++){
       EElookuptable[EElookuptableaddress]=PApplet.parseInt(ConfigLayout[0][i]&0xFF);
       EElookuptableaddress++;
       EElookuptable[EElookuptableaddress]=PApplet.parseInt(ConfigLayout[0][i]>>8);
       EElookuptableaddress++;
     }
     for(int i = 0; i < (hudoptions); i++){
       EElookuptable[EElookuptableaddress]=PApplet.parseInt(ConfigLayout[1][i]&0xFF);
       EElookuptableaddress++;
       EElookuptable[EElookuptableaddress]=PApplet.parseInt(ConfigLayout[1][i]>>8);
       EElookuptableaddress++;
     }
     for(int i = 0; i < (hudoptions); i++){
       EElookuptable[EElookuptableaddress]=PApplet.parseInt(ConfigLayout[2][i]&0xFF);
       EElookuptableaddress++;
       EElookuptable[EElookuptableaddress]=PApplet.parseInt(ConfigLayout[2][i]>>8);
       EElookuptableaddress++;
     }
  }
  
  public void EElookuptableSync(){ // Sync settings with EE table
  }
  




float sAltitude = 0;
float sVario = 0;
float sVBat = 0;
float sMRSSI = 0;
int mode_armed = 0;
int mode_stable = 0;
int mode_horizon = 0;
int mode_baro = 0;
int mode_mag = 0;
int mode_gpshome = 0;
int mode_gpshold = 0;
int mode_gpsmission = 0;
int mode_gpsland = 0;
int mode_llights = 0;
int mode_camstab = 0;
int mode_osd_switch = 0;

int SendSim = 0;

boolean[] keys = new boolean[526];
boolean armed = false;

Group SG,SGControlBox,SGModes,SGAtitude,SGRadio,SGSensors1,SGGPS,SGFRSKY; 

// Checkboxs
CheckBox checkboxSimItem[] = new CheckBox[SIMITEMS] ;
CheckBox ShowSimBackground, UnlockControls, SGPS_FIX,SFRSKY;
//Toggles
Toggle toggleModeItems[] = new Toggle[boxnames.length] ;
Toggle SimControlToggle;
// Toggle HudOptionEnabled;

// Slider2d-
Slider2D Pitch_Roll, Throttle_Yaw,MW_Pitch_Roll;
//Sliders ---
Slider s_Altitude,s_Vario,s_VBat,s_MRSSI;

Textlabel txtlblModeItems[] = new Textlabel[boxnames.length] ;
Textlabel SimControlText;

// Knobs----
Knob HeadingKnob,SGPSHeadHome;

Numberbox SGPS_numSat, SGPS_altitude, SGPS_speed, SGPS_ground_course,SGPS_distanceToHome,SGPS_directionToHome,SGPS_update;
//GPS_distanceToHome=read16();
    //GPS_directionToHome=read16();
    //GPS_update=read8();

    

//CheckBox checkboxModeItems[] = new CheckBox[boxnames.length] ;
DecimalFormat OnePlaceDecimal = new DecimalFormat("0.0");
DecimalFormat TwoPlaceDecimal = new DecimalFormat("0.00");


public void LayoutEditorSetup(){
// Group LEW;

  LEW = FontGroupcontrolP5.addGroup("LEW")
    .setPosition(XLINKS,YLINKS)
    .setWidth(345)
    .setBarHeight(15)
    .activateEvent(true)
    .setBackgroundColor(color(30,255))
    .setBackgroundHeight(103)
    .setLabel("Layout Editor")
    .setMoveable(true)
    .disableCollapse()
    .hide()
    ;
 LEW.captionLabel()
    .toUpperCase(false);

//  buttonLPOSUP = controlP5.addButton("bPOSLUP",1,10,35,12,19)
  buttonLPOSUP = controlP5.addButton("bPOSLUP",1,10,10,12,19)
  .setLabel("-")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
//  buttonLPOSDOWN = controlP5.addButton("bPOSLDOWN",1,23,35,12,19)
  buttonLPOSDOWN = controlP5.addButton("bPOSLDOWN",1,23,10,12,19)
  .setLabel("+")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
//  txtlblLayoutTxt = controlP5.addTextlabel("txtlblLayoutTxt","blah",37,35) 
  txtlblLayoutTxt = controlP5.addTextlabel("txtlblLayoutTxt","blah",70,10) 
  .setGroup(LEW);
  txtlblLayoutTxt2 = controlP5.addTextlabel("txtlblLayoutTxt2","Text",37,10) 
  .setGroup(LEW);



//  buttonLPOSEN = controlP5.addButton("bPOSLEN",1,23,60,12,19)
  buttonLPOSEN = controlP5.addButton("bPOSLEN",1,23,35,12,19)
  .setLabel("*")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
//  txtlblLayoutEnTxt = controlP5.addTextlabel("txtlblLayoutEnTxt","-",37,60)
  txtlblLayoutEnTxt = controlP5.addTextlabel("txtlblLayoutEnTxt","-",70,35)
  .setGroup(LEW);
  txtlblLayoutEnTxt2 = controlP5.addTextlabel("txtlblLayoutEnTxt2","Status",37,35)
  .setGroup(LEW);

//  buttonLPOSUP = controlP5.addButton("bHUDLUP",1,10,10,12,19)
  buttonLPOSUP = controlP5.addButton("bHUDLUP",1,10,60,12,19)
  .setLabel("-")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
//  buttonLPOSDOWN = controlP5.addButton("bHUDLDOWN",1,23,10,12,19)
  buttonLPOSDOWN = controlP5.addButton("bHUDLDOWN",1,23,60,12,19)
  .setLabel("+")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);


//  txtlblLayoutHudTxt = controlP5.addTextlabel("txtlblLayoutHudTxt","-",37,10) 
  txtlblLayoutHudTxt = controlP5.addTextlabel("txtlblLayoutHudTxt","-",70,60) 
  .setGroup(LEW);
  txtlblLayoutHudTxt2 = controlP5.addTextlabel("txtlblLayoutHudTxt2","HUD",37,60) 
  .setGroup(LEW);
 

  buttonLUP = controlP5.addButton("bLUP",1,195,10,40,19)
  .setLabel("   UP")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
  buttonLDOWN = controlP5.addButton("bLDOWN",1,195,60,40,19)
  .setLabel("DOWN")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
  buttonLLEFT = controlP5.addButton("bLLEFT",1,170,35,40,19)
  .setLabel(" LEFT")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
  buttonLRIGHT = controlP5.addButton("bLRIGHT",1,220,35,40,19)
  .setLabel("RIGHT")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);

  buttonLSET = controlP5.addButton("bLSET",1,270,9,65,16)
  .setLabel("Switches")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
  buttonLADD = controlP5.addButton("bLADD",1,270,28,65,16)
  .setLabel("      ADD")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
  buttonLSAVE = controlP5.addButton("bLSAVE",1,270,47,65,16)
  .setLabel("    WRITE")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
  buttonLCANCEL = controlP5.addButton("bLCANCEL",1,270,66,65,16)
  .setLabel("      EXIT")
  .setColorBackground(blue_)
  .setColorCaptionLabel(yellow_)
  .setGroup(LEW);
}

 
public void SimSetup(){

LayoutEditorSetup();
LinksSetup() ;  
 

  SG = ScontrolP5.addGroup("SG")
    .setPosition(305,YSim + 44)
    .setWidth(733)
    .setBarHeight(13)
    .activateEvent(true)
    .disableCollapse()
//    .setBackgroundColor(color(0,255))
    .setBackgroundHeight(192)
   .setLabel("Simulator")
   .setMoveable(true);
    ;
                
 
  SGModes = ScontrolP5.addGroup("SGModes")
                .setPosition(632,18)
                .setWidth(100)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight((boxnames.length*17) + 9)
                .setLabel("Modes")
                .setGroup(SG)
                .disableCollapse() 
                //.close() 
               ; 
               
  SGAtitude = ScontrolP5.addGroup("SGAtitude")
                .setPosition(525,18)
                .setWidth(100)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(162)
                .setLabel("Angle/Heading")
                .setGroup(SG)
                //.close() 
               ;
               
 SGRadio = ScontrolP5.addGroup("SGRadio")
                .setPosition(388,18)
                .setWidth(130)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(62)
                .setLabel("Radio")
                .setGroup(SG)
                //.close() 
               ; 
             
SGSensors1 = ScontrolP5.addGroup("SGSensors1")
                .setPosition(0,18)
                .setWidth(175)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(110)
                .setLabel("FC Sensors")
                .setGroup(SG)
                //.close() 
               ;                                  
SGGPS = ScontrolP5.addGroup("SGGPS")
                .setPosition(182,18)
                .setWidth(200)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(110)
                .setLabel("GPS")
                .setGroup(SG)
                //.close() 
               ;

SGFRSKY = ScontrolP5.addGroup("SGFRSKY")
                .setPosition(182,145)
                .setWidth(200)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(33)
                .setLabel("FRSKY")
                .setGroup(SG)
                //.close() 
               ;   

SGControlBox = ScontrolP5.addGroup("SGControlBox")
                .setPosition(0,145)
                .setWidth(175)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(33)
                .setLabel("Simulator Control")
                .setGroup(SG)
                //.close() 
               ;   



SimControlToggle = (controlP5.Toggle) hideLabel(controlP5.addToggle("SendSim"));
SimControlToggle.setPosition(5,5);
SimControlToggle.setSize(35,10);
SimControlToggle.setMode(ControlP5.SWITCH);
SimControlToggle.setGroup(SGControlBox);
SimControlToggle.setValue(0);

SimControlText = controlP5.addTextlabel("SimControlText","Simulate on OSD",45,3);
SimControlText.setGroup(SGControlBox);

               
SFRSKY =  ScontrolP5.addCheckBox("SFRSKY",5,5);
    SFRSKY.setColorBackground(color(120));
    SFRSKY.setColorActive(color(255));
    SFRSKY.addItem("Simulate FRSKY cells",1);
    SFRSKY.setGroup(SGFRSKY);
//    SFRSKY.activate(0);

SGPS_FIX =  ScontrolP5.addCheckBox("GPS_FIX",5,5);
    SGPS_FIX.setColorBackground(color(120));
    SGPS_FIX.setColorActive(color(255));
    SGPS_FIX.addItem("GPS Fix",1);
    //GPSLock.hideLabels();
    SGPS_FIX.setGroup(SGGPS);
    SGPS_FIX.activate(0);

SGPS_numSat = ScontrolP5.addNumberbox("SGPS_numSat",0,5,20,40,14);
    SGPS_numSat.setLabel("Sats");
    //SGPS_numSat.setColorBackground(red_);
    SGPS_numSat.setMin(0);
    SGPS_numSat.setDirection(Controller.HORIZONTAL);
    SGPS_numSat.setMax(15);
    SGPS_numSat.setDecimalPrecision(0);
    SGPS_numSat.setGroup(SGGPS); 
    SGPS_numSat.setValue(10);
 ScontrolP5.getController("SGPS_numSat").getCaptionLabel()
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);

SGPS_altitude = ScontrolP5.addNumberbox("SGPS_altitude",0,5,40,40,14);
    SGPS_altitude.setLabel("Alt-cm");
    //SGPS_numSat.setColorBackground(red_);
    SGPS_altitude.setMin(0);
    SGPS_altitude.setDirection(Controller.HORIZONTAL);
    SGPS_altitude.setMax(100000);
    SGPS_altitude.setDecimalPrecision(0);
    SGPS_altitude.setMultiplier(100);
    SGPS_altitude.setGroup(SGGPS); 
    SGPS_altitude.setValue(500);

 ScontrolP5.getController("SGPS_altitude").getCaptionLabel()
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);     
 
 SGPS_speed = ScontrolP5.addNumberbox("SGPS_speed",0,5,60,40,14);
    SGPS_speed.setLabel("Speed-cm/s");
    //SGPS_numSat.setColorBackground(red_);
    SGPS_speed.setMin(0);
    SGPS_speed.setMultiplier(10);
    SGPS_speed.setDirection(Controller.HORIZONTAL);
    SGPS_speed.setMax(10000);
    SGPS_speed.setDecimalPrecision(0);
    SGPS_speed.setGroup(SGGPS); 
    SGPS_speed.setValue(1000);
 ScontrolP5.getController("SGPS_speed").getCaptionLabel()
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);   
 
 SGPS_distanceToHome = ScontrolP5.addNumberbox("SGPS_distanceToHome",0,5,80,40,14);
    SGPS_distanceToHome.setLabel("Dist Home-M");
    //SGPS_numSat.setColorBackground(red_);
    SGPS_distanceToHome.setMin(0);
    SGPS_distanceToHome.setDirection(Controller.HORIZONTAL);
    SGPS_distanceToHome.setMax(1000);
    SGPS_distanceToHome.setDecimalPrecision(0);
    SGPS_distanceToHome.setGroup(SGGPS); 
    SGPS_distanceToHome.setValue(350);
 ScontrolP5.getController("SGPS_distanceToHome").getCaptionLabel()
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);   
                 
  SGPSHeadHome = ScontrolP5.addKnob("SGPSHeadHome")
   .setRange(-180,+180)
   .setValue(0)
   .setPosition(140,5)
   .setRadius(25)
   .setLabel("Head Home")
   .setColorBackground(color(0, 160, 100))
   .setColorActive(color(255,255,0))
   .setDragDirection(Knob.HORIZONTAL)
   .setGroup(SGGPS)
   ; 
                 
               
               
  MW_Pitch_Roll = ScontrolP5.addSlider2D("MWPitch/Roll")
    .setPosition(25,5)
    .setSize(50,50)
    .setArrayValue(new float[] {50, 50})
    .setMaxX(45) 
    .setMaxY(-25) 
    .setMinX(-45) 
    .setMinY(25)
    .setValueLabel("") 
    .setLabel("Roll/Pitch")
    .setGroup(SGAtitude)
    ;
 ScontrolP5.getController("MWPitch/Roll").getCaptionLabel()
   .align(ControlP5.CENTER, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);   
 ScontrolP5.getController("MWPitch/Roll").getValueLabel().hide();
 
 HeadingKnob = ScontrolP5.addKnob("MwHeading")
   .setRange(-180,+180)
   .setValue(0)
   .setPosition(25,80)
   .setRadius(25)
   .setLabel("Heading")
   .setColorBackground(color(0, 160, 100))
   .setColorActive(color(255,255,0))
   .setDragDirection(Knob.HORIZONTAL)
   .setGroup(SGAtitude)
   ;
   


 Throttle_Yaw = ScontrolP5.addSlider2D("Throttle/Yaw")
         .setPosition(5,5)
         .setSize(50,50)
         .setArrayValue(new float[] {50, 100})
         .setMaxX(2000) 
         .setMaxY(1000) 
         .setMinX(1000) 
         .setMinY(2000)
         .setValueLabel("") 
        .setLabel("")
         .setGroup(SGRadio)
        ;
 ScontrolP5.getController("Throttle/Yaw").getValueLabel().hide();
 ScontrolP5.getTooltip().register("Throttle/Yaw","Ctrl Key to hold position");


UnlockControls =  ScontrolP5.addCheckBox("UnlockControls",60,25);
    UnlockControls.setColorBackground(color(120));
    UnlockControls.setColorActive(color(255));
    UnlockControls.addItem("UnlockControls1",1);
    UnlockControls.hideLabels();
    UnlockControls.setGroup(SGRadio);
    UnlockControls.activate(0);

  Pitch_Roll = ScontrolP5.addSlider2D("Pitch/Roll")
         .setPosition(75,5)
         .setSize(50,50)
         .setArrayValue(new float[] {50, 50})
         .setMaxX(2000) 
         .setMaxY(1000) 
         .setMinX(1000) 
         .setMinY(2000)
         .setLabel("")
         .setGroup(SGRadio)
         ;
  ScontrolP5.getController("Pitch/Roll").getValueLabel().hide();


 
 

 

               
s_Altitude = ScontrolP5.addSlider("sAltitude")
  .setPosition(5,10)
  .setSize(8,75)
  .setRange(-500,1000)
  .setValue(0)
  .setLabel("Alt")
  .setDecimalPrecision(1)
  .setGroup(SGSensors1)
  .setValue(500);
  ScontrolP5.getController("sAltitude").getValueLabel()
    .setFont(font9);
  ScontrolP5.getController("sAltitude").getCaptionLabel()
    .setFont(font9)
    .align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);

s_Vario = ScontrolP5.addSlider("sVario")
  .setPosition(47,10)
  .setSize(8,75)
  .setRange(-20,20)
  .setNumberOfTickMarks(41)
  .showTickMarks(false)
  .setValue(0)
  .setLabel("Vario")
  .setDecimalPrecision(1)
  .setValue(7)
  .setGroup(SGSensors1);
  ScontrolP5.getController("sVario").getValueLabel()
    .setFont(font9);
  ScontrolP5.getController("sVario").getCaptionLabel()
    .setFont(font9)
    .align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);

s_VBat = ScontrolP5.addSlider("sVBat")
  .setPosition(90,10)
  .setSize(8,75)
  .setRange(9,26)
  .setValue(0)
  .setLabel("VBat")
  .setDecimalPrecision(1)
  .setValue(15.0f)
  .setGroup(SGSensors1);
  ScontrolP5.getController("sVBat").getValueLabel()
     .setFont(font9);
  ScontrolP5.getController("sVBat")
    .getCaptionLabel()
    .setFont(font9)
    .align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);               

s_MRSSI = ScontrolP5.addSlider("sMRSSI")
  .setPosition(130,10)
  .setSize(8,75)
  .setRange(0,1023)
  .setValue(0)
  .setLabel("RSSI")
  .setDecimalPrecision(0)
  .setValue(70)
  .setGroup(SGSensors1);
  ScontrolP5.getController("sMRSSI").getValueLabel()
     .setFont(font9);
  ScontrolP5.getController("sMRSSI")
    .getCaptionLabel()
    .setFont(font9)
    .align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);          



  for(int i=0;i<boxnames.length ;i++) {
    toggleModeItems[i] = (controlP5.Toggle) hideLabel(ScontrolP5.addToggle("toggleModeItems"+i,false));
    toggleModeItems[i].setPosition(5,3+i*16);
    toggleModeItems[i].setSize(10,10);
    //toggleConfItem[i].setMode(ControlP5.SWITCH);
    toggleModeItems[i].setGroup(SGModes);
    txtlblModeItems[i] = controlP5.addTextlabel("ModeItems"+i,boxnames[i].substring(0, boxnames[i].length()-1) ,20,i*16);
    txtlblModeItems[i].setGroup(SGModes);
  }
 
  for(int i=2;i<6 ;i++) {
     toggleModeItems[i].setValue(1);
  }
      
GetModes();  
} 

public boolean checkKey(int k)
{
  if (keys.length >= k) {
    return keys[k];  
  }
  return false;
}



public void keyPressed()
{ 
  keys[keyCode] = true;
  //println(keyCode);
  if((checkKey(CONTROL) == true) && (checkKey(85) == true)){
    SketchUploader();
  }
}


public void keyReleased()
{ 
  keys[keyCode] = false; 
  ControlLock();
  
}



public void CalcAlt_Vario(){
  if (time2 < time - 1000){
    sAltitude += sVario /10;
    time2 = time;
  }
}

public String RightPadd(int inInt,int Places){
  String OutString = nf(inInt,Places).replaceFirst("^0+(?!$)",  " ");
  for(int X=0; X<=3; X++) {
    if (OutString.length() < Places){ OutString = " " + OutString;}
  }
  return OutString;  
  
}






public void ShowVolts(float voltage){  
  if(confItem[GetSetting("S_DISPLAYVOLTAGE")].value() > 0) {
    String output = OnePlaceDecimal.format(voltage);
    int SYM_VOLT = 0XA9;
    output =output+PApplet.parseChar(0XA9);
    mapchar(0x97, SimPosn[voltagePosition]-1);
    makeText(output, SimPosn[voltagePosition]);
  }
}

public void ShowVideoVolts(float voltage){  
  if(confItem[GetSetting("S_VIDVOLTAGE")].value() > 0) {
    String output = OnePlaceDecimal.format(voltage);
    int SYM_VOLT = 0XA9;
    output =output+PApplet.parseChar(0XA9);
    mapchar(0xBF, SimPosn[vidvoltagePosition]-1);
    makeText(output, SimPosn[vidvoltagePosition]);
  }
}


public void ShowFlyTime(String FMinutes_Seconds){

  if (PApplet.parseInt(confItem[GetSetting("S_TIMER")].value()) > 0){
  mapchar(0x9c, SimPosn[flyTimePosition]);
  makeText(FMinutes_Seconds, SimPosn[flyTimePosition]+1);
}}

public void ShowOnTime(String Minutes_Seconds){
  if (PApplet.parseInt(confItem[GetSetting("S_TIMER")].value()) > 0){
  mapchar(0x9b, SimPosn[onTimePosition]);
  makeText(Minutes_Seconds, SimPosn[onTimePosition]+1);
  }}

public void ShowCurrentThrottlePosition(){
  if(confItem[GetSetting("S_THROTTLEPOSITION")].value() > 0) {
  mapchar(0xc8, SimPosn[CurrentThrottlePosition]);
  
  if(armed){
    int CurThrottle = PApplet.parseInt(map(Throttle_Yaw.arrayValue()[1],1000,2000,0,100));
    makeText(RightPadd(CurThrottle,3) + "%", SimPosn[CurrentThrottlePosition]+1);   
  }
  else
  {
    makeText("  --", SimPosn[CurrentThrottlePosition]+1);
  }

  
  
  
  //makeText(" 40%", CurrentThrottlePosition[ScreenType]+1);
}}



public void ShowLatLon(){
  if(confItem[GetSetting("S_COORDINATES")].value() > 0) {
//  if(confItem[GetSetting("S_GPSCOORDTOP")].value() > 0) {
  mapchar(0xca, SimPosn[MwGPSLatPositionTop]);
  makeText(" 43.09486N", SimPosn[MwGPSLatPositionTop]+1);
  mapchar(0xcb, SimPosn[MwGPSLonPositionTop]);
  makeText(" 71.88970W", SimPosn[MwGPSLonPositionTop]+1);
//  }
//  else {
//  mapchar(0xca, SimPosn[MwGPSLatPosition]);
//  makeText(" 43.09486N", SimPosn[MwGPSLatPosition]+1);
//  mapchar(0xcb, SimPosn[MwGPSLonPosition]);
//  makeText(" 71.88970W", SimPosn[MwGPSLonPosition]+1);
//  }
}
}

public void ShowDebug(){
  if(confItem[GetSetting("S_DEBUG")].value() > 0) {
    makeText("0: "+debug[0], SimPosn[debugPosition]);
    makeText("1: "+debug[1], SimPosn[debugPosition]+LINE);
    makeText("2: "+debug[2], SimPosn[debugPosition]+LINE+LINE);
    makeText("3: "+debug[3], SimPosn[debugPosition]+LINE+LINE+LINE+LINE);
  }
}


public void ShowSats(){
  String output = str(PApplet.parseInt(SGPS_numSat.getValue()));
  mapchar(0x1e, SimPosn[GPS_numSatPosition]);
  mapchar(0x1f, SimPosn[GPS_numSatPosition]+1);
  makeText(output, SimPosn[GPS_numSatPosition]+2);
}

public void ShowSpeed(){
  int SimModebits = 0;
  int SimBitCounter = 1;
  for (int i=0; i<boxnames.length; i++) {
      if(toggleModeItems[i].getValue() > 0) SimModebits |= SimBitCounter;
      SimBitCounter += SimBitCounter;
  }
  String output = str(0);
  if((SimModebits&mode_armed) >0){
    output = str(PApplet.parseInt(SGPS_speed.getValue()/27.7778f));
  }
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa6, SimPosn[speedPosition]);}
  else {mapchar(0xa5, SimPosn[speedPosition]);}
  makeText(output, SimPosn[speedPosition]+1);
}

public void ShowDirection(){
  int dirhome=0x60 + ((180+360+22+MwHeading-PApplet.parseInt(SGPSHeadHome.value()))%360)*2/45;
 
  mapchar(dirhome, SimPosn[GPS_directionToHomePosition]);
}

public void ShowGPSAltitude(){
  String output = str(PApplet.parseInt(SGPS_altitude.getValue())/100);
  if(confItem[GetSetting("S_GPSALTITUDE")].value() > 0) {
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa8, SimPosn[MwGPSAltPosition]);}
  else {mapchar(0xa7, SimPosn[MwGPSAltPosition]);}
  makeText(output, SimPosn[MwGPSAltPosition]+1);
}}


public void ShownAngletohome(){
  String output = str((360+PApplet.parseInt(SGPSHeadHome.getValue()))%360/1);
  if(confItem[GetSetting("S_ANGLETOHOME")].value() > 0) {
  

  switch (output.length())
  {
  case 1:
makeText(output, SimPosn[GPS_angleToHomePosition]+2);
break;
  case 2:
makeText(output, SimPosn[GPS_angleToHomePosition]+1);
break;
  case 3:
makeText(output, SimPosn[GPS_angleToHomePosition]);
break;
case 4:
makeText(output, SimPosn[GPS_angleToHomePosition]-1);
break;
}
 mapchar(0xbd, SimPosn[GPS_angleToHomePosition]+3); 

  }}

public void ShowAltitude(){
  String output = str(PApplet.parseInt(s_Altitude.getValue()));
  if(confItem[GetSetting("S_BAROALT")].value() > 0) {
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa8, SimPosn[MwAltitudePosition]);}
  else {mapchar(0xa7, SimPosn[MwAltitudePosition]);}
  makeText(output, SimPosn[MwAltitudePosition]+1);
  }}

public void ShowDistance(){
  String output = str(PApplet.parseInt(SGPS_distanceToHome.getValue()));
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xb9, SimPosn[GPS_distanceToHomePosition]);}
  else {mapchar(0xbb, SimPosn[GPS_distanceToHomePosition]);}
  makeText(output, SimPosn[GPS_distanceToHomePosition]+1);
  }

public void ShowVario(){
  if(confItem[GetSetting("S_VARIO")].value() > 0) {
  mapchar(0x7f, SimPosn[MwClimbRatePosition]-LINE);
  mapchar(0x8c, SimPosn[MwClimbRatePosition]);
  mapchar(0x7f, SimPosn[MwClimbRatePosition]+LINE);
}}

public void ShowRSSI(){
  String output = str(PApplet.parseInt(s_MRSSI.getValue()));
  if(confItem[GetSetting("S_DISPLAYRSSI")].value() > 0) {
  mapchar(0xba, SimPosn[rssiPosition]);
  makeText(output + "%", SimPosn[rssiPosition]+1);
}}

public void ShowAPstatus(){
   if (SimPosn[APstatusPosition]==0x3FF)
      return;

  String output="";
  int SimModebits = 0;
  int SimBitCounter = 1;
    for (int i=0; i<boxnames.length; i++) {
      if(toggleModeItems[i].getValue() > 0) SimModebits |= SimBitCounter;
      SimBitCounter += SimBitCounter;
}
  if((SimModebits&mode_gpshome) >0){
  output = "AUTO RTH";
    }
  else if((SimModebits&mode_gpshold) >0){
  output = "AUTO HOLD";
  }
  else if((SimModebits&mode_gpsmission) >0){
  output = " MISSION";
  }
  else if((SimModebits&mode_gpsland) >0){
  output = "AUTO LAND";
  }
  makeText(output, SimPosn[APstatusPosition]);
}


public void ShowCallsign(){
  if(confItem[GetSetting("S_DISPLAY_CS")].value() > 0) {
    if (LEWvisible==1){
      if (millis() < (csmillis)+4000){
        csmillis = (millis()-4000);
      }
    }


    if (millis() > (csmillis)+5000){
   
      String CallSText = controlP5.get(Textfield.class,"CallSign").getText().toUpperCase();
      makeText(CallSText, SimPosn[callSignPosition]);
      if (millis() > (csmillis + 6000)){
        csmillis = millis();
      }
      
    }
}}

public void ShowAmperage(){
  if(confItem[GetSetting("S_AMPER_HOUR")].value() > 0) {
  mapchar(0xa4, SimPosn[pMeterSumPosition]);
  makeText("1221", SimPosn[pMeterSumPosition]+1);
}}

public void ShowTemp(){
  makeText("30", SimPosn[temperaturePosition]);
  mapchar(0x0e, SimPosn[temperaturePosition]+2);
}

public void ShowAmps(){
  if(confItem[GetSetting("S_AMPERAGE")].value() > 0) {
  makeText("15.2A", SimPosn[amperagePosition]+1);
  }}

public void ShowUTC(){
  if(confItem[GetSetting("S_GPSTIME")].value() > 0) {
    int seconds = second();
    int minutes = minute();
    int hours = hour();
    String PCTimerString ="";

    if (hours < 10){
      PCTimerString = "0" + str(hours) ;
    }
    else
    {
      PCTimerString =  str(hours);
    }
    if (minutes < 10){
      PCTimerString = PCTimerString +":0" + str(minutes)  ;
    }
    else
    {
      PCTimerString =  PCTimerString +":" + str(minutes);
    }
    if (seconds < 10){
      PCTimerString = PCTimerString +":0" + str(seconds);
    }
    else
    {
      PCTimerString =  PCTimerString +":" + str(seconds);
    }
  
    makeText(PCTimerString, SimPosn[GPS_timePosition]);
  }
}

public void displayHeading()
{
  if(confItem[GetSetting("S_SHOWHEADING")].value() > 0) {
  int heading = MwHeading;
  if(confItem[GetSetting("S_HEADING360")].value() > 0) {
  if(heading < 0)
    heading += 360;
  }  
    
  switch (str(heading).length())
  {
  case 1:
    makeText(str(heading), SimPosn[MwHeadingPosition]+2);
    break;
  case 2:
    makeText(str(heading), SimPosn[MwHeadingPosition]+1);
    break;
  case 3:
    makeText(str(heading), SimPosn[MwHeadingPosition]);
    break;
  case 4:
    makeText(str(heading), SimPosn[MwHeadingPosition]-1);
    break;
  }
  mapchar(MwHeadingUnitAdd,SimPosn[MwHeadingPosition]+3);  
  }}


public void SimulateTimer(){
  if (SimPosn[flyTimePosition]==0x3FF){
     return;
  }

  String OnTimerString ="";
  String FlyTimerString ="";
  int seconds = (millis() - OnTimer) / 1000;
  int minutes = seconds / 60;
  int hours = minutes / 60;
  seconds -= minutes * 60;
  minutes -= hours * 60;
  if (seconds < 10){
    OnTimerString = str(minutes) + ":0" + str(seconds);
  }
  else
  {
    OnTimerString = str(minutes) + ":" + str(seconds);
  }
//  ShowOnTime(OnTimerString);


  if (FlyTimer >0) {
    seconds = (millis() - FlyTimer) / 1000;
    minutes = seconds / 60;
    hours = minutes / 60;
    seconds -= minutes * 60;
    minutes -= hours * 60;
    if (seconds < 10){
      FlyTimerString = str(minutes) + ":0" + str(seconds);
    }
    else
    {
      FlyTimerString = str(minutes) + ":" + str(seconds);
    }
  }
   if ((toggleModeItems[0].getValue() == 0) && (SimItem0 < 1)){
     ShowOnTime(OnTimerString);
   }
    else
   {
     ShowFlyTime(FlyTimerString);
   } 
}


public void displayMode()
{
  int SimModebits = 0;
  int SimBitCounter = 1;
    for (int i=0; i<boxnames.length; i++) {
      if(toggleModeItems[i].getValue() > 0) SimModebits |= SimBitCounter;
      SimBitCounter += SimBitCounter;
}
  if(confItem[GetSetting("S_CHECK_")].value() > 0) {
    if((SimModebits&mode_armed) >0){
    makeText(" ARMED", SimPosn[motorArmedPosition]);
    armed = true;
  }
    else{
    makeText("DISARMED", SimPosn[motorArmedPosition]);
    armed = false;
  }
  }
  else{
    makeText("DISCONNECTED", SimPosn[motorArmedPosition]-2);
    armed = false;
  }
  if(confItem[GetSetting("S_MODEICON")].value() > 0) {

    if(confItem[GetSetting("S_MODESENSOR")].value() > 0) {
    
    if((SimModebits&mode_stable) >0)
      mapchar(0xa0,SimPosn[sensorPosition]);

    if((SimModebits&mode_horizon) >0)
      mapchar(0xa0,SimPosn[sensorPosition]);

    if((SimModebits&mode_baro) >0)
      mapchar(0xa2,SimPosn[sensorPosition]+1);

    if((SimModebits&mode_mag) >0)
      mapchar(0xa1,SimPosn[sensorPosition]+2);
  }

  if(confItem[GetSetting("S_GIMBAL")].value() > 0) {
    if((SimModebits&mode_camstab) >0){
      mapchar(0x16,SimPosn[gimbalPosition]);
      mapchar(0x17,SimPosn[gimbalPosition]+1);
    }}

     if (SimPosn[ModePosition]!=0x3FF){ 

    if((SimModebits&mode_gpshome) >0){
      mapchar(0x9d,SimPosn[ModePosition]);
      mapchar(0x9e,SimPosn[ModePosition]+1);
//      mapchar(0x2d,SimPosn[ModePosition]+2);
//      mapchar(0x9e,SimPosn[statusPosition]+3);
//      String output = str(int(SGPS_distanceToHome.getValue()));
//      makeText(output,SimPosn[ModePosition]+3);
//      mapchar(0x0c,SimPosn[ModePosition]+3+output.length());
    }
    else if((SimModebits&mode_gpshold) >0){
      mapchar(0xcd,SimPosn[ModePosition]);
      mapchar(0xce,SimPosn[ModePosition]+1);
    }
    else if((SimModebits&mode_gpsland) >0){
      mapchar(0xb7,SimPosn[ModePosition]);
      mapchar(0xb8,SimPosn[ModePosition]+1);
    }
    else if((SimModebits&mode_gpsmission) >0){
      mapchar(0xb5,SimPosn[ModePosition]);
      mapchar(0xb6,SimPosn[ModePosition]+1);
      mapchar(0x30,SimPosn[ModePosition]+2);
    }
    else if((SimModebits&mode_stable) >0){
      mapchar(0xac,SimPosn[ModePosition]);
      mapchar(0xad,SimPosn[ModePosition]+1);
    }
    else if((SimModebits&mode_horizon) >0){
      mapchar(0xc4,SimPosn[ModePosition]);
      mapchar(0xc5,SimPosn[ModePosition]+1);
    }
    else{
      mapchar(0xae,SimPosn[ModePosition]);
      mapchar(0xaf,SimPosn[ModePosition]+1);
    }
  }

}
}



public void displayHorizon(int rollAngle, int pitchAngle)
{

  int minimalscreen=0 ; 
  if (toggleModeItems[9].getValue()>0) minimalscreen=1 ;

   if (SimPosn[horizonPosition]<0x3FF){

  
  if(pitchAngle>250) pitchAngle=250;                //250
  if(pitchAngle<-200) pitchAngle=-200;
  if(rollAngle>400) rollAngle=400;
  if(rollAngle<-400) rollAngle=-400;

  for(int X=0; X<=8; X++) {
    int Y = (rollAngle * (4-X)) / 64;
    Y += pitchAngle / 8;
    Y += 41;
    if(Y >= 0 && Y <= 81) {
      int pos = SimPosn[horizonPosition] - 2*LINE + LINE*(Y/9) + 3 - 2*LINE + X;
      if(X < 3 || X >5 || (Y/9) != 4 || confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() == 0)
      	mapchar(0x80+(Y%9), pos);
      if(Y>=9 && (Y%9) == 0)
        mapchar(0x89, pos-30);
    }
  }
  if(confItem[GetSetting("S_HORIZON_ELEVATION")].value() > 0) {

  for(int X=3; X<=5; X++) {
    int Y = (rollAngle * (4-X)) / 64;
    Y += pitchAngle / 8;
    Y += 31;
    if(Y >= 0 && Y <= 81) {
      int pos = SimPosn[horizonPosition] - 2*LINE + LINE*(Y/9) + 3 - 2*LINE + X;
//      int pos = 30*(2+Y/9) + 10 + X;
      if(X < 3 || X >5 || (Y/9) != 4 || confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() == 0)
        mapchar(0x80+(Y%9), pos);
      if(Y>=9 && (Y%9) == 0)
        mapchar(0x89, pos-30);
    }
    Y += 20;
    if(Y >= 0 && Y <= 81) {
      int pos = SimPosn[horizonPosition] - 2*LINE + LINE*(Y/9) + 3 - 2*LINE + X;
//      int pos = 30*(2+Y/9) + 10 + X;
      if(X < 3 || X >5 || (Y/9) != 4 || confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() == 0)
        mapchar(0x80+(Y%9), pos);
      if(Y>=9 && (Y%9) == 0)
        mapchar(0x89, pos-30);
    }
  }}


  if(confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() > 0) {
    //Draw center screen
    mapchar(0x7e, 224-30);
    mapchar(0x26, 224-30-1);
    mapchar(0xbc, 224-30+1);
  }
  }
  
  if (SimPosn[SideBarPosition]<0x3FF){
    if(confItem[GetSetting("S_WITHDECORATION")].value() > 0) {
      int centerpos = SimPosn[horizonPosition]+7;
      for(int X=-2; X<=2; X++) {
        mapchar(0x12,centerpos+7+(X*LINE));
        mapchar(0x12,centerpos-7+(X*LINE));
      }
      mapchar(0x02, centerpos+6);
      mapchar(0x03, centerpos-6);
    }
  }
  
}

public void ShowSideBarArrows(){
  int centerpos = SimPosn[horizonPosition]+7;
  if (SimPosn[horizonPosition]==0x3FF)
    return;
  if (SimPosn[SideBarScrollPosition]==0x3FF)
    return;
  if(confItem[GetSetting("S_SIDEBARTOPS")].value() > 0) {
    mapchar(0xCf,centerpos+7+(3*LINE));
    mapchar(0xCf,centerpos-7+(3*LINE));
  }
}


public void displayHeadingGraph()
{
  if(confItem[GetSetting("S_COMPASS")].value() > 0) {
  
  int xx;
  xx = MwHeading * 4;
  xx = xx + 720 + 45;
  xx = xx / 90;
 //for (int i = 0; i < 9; i++){
 
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+1);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+2);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+3);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+4);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+5);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+6);
  mapchar(headGraph[xx++],SimPosn[MwHeadingGraphPosition]+7);
  mapchar(headGraph[xx],SimPosn[MwHeadingGraphPosition]+8);  

}}


public void ControlLock(){
  Pitch_Roll.setArrayValue(new float[] {500, -500});
  if(checkKey(CONTROL) == false) {
    if(UnlockControls.arrayValue()[0] < 1){
      float A = (2000-Throttle_Yaw.getArrayValue()[1])*-1;
      Throttle_Yaw.setArrayValue(new float[] {500, A});
      s_Vario.setValue(0);
      sVario = 0;
    }
  }    
}

public void GetModes(){
  int bit = 1;
  int remaining = strBoxNames.length();
  int len = 0;
 
  mode_armed = 0;
  mode_stable = 0;
  mode_horizon = 0;
  mode_baro = 0;
  mode_mag = 0;
  mode_gpshome = 0;
  mode_gpshold = 0;
  mode_gpsmission = 0;
  mode_gpsland = 0;
  mode_llights = 0;
  mode_camstab = 0;
  mode_osd_switch = 0;
  for (int c = 0; c < boxnames.length; c++) {
    if (boxnames[c] == "ARM;") mode_armed |= bit;
    if (boxnames[c] == "ANGLE;") mode_stable |= bit;
    if (boxnames[c] == "HORIZON;") mode_horizon |= bit;
    if (boxnames[c] == "MAG;") mode_mag |= bit;
    if (boxnames[c] == "BARO;") mode_baro |= bit;
    if (boxnames[c] == "CAMSTAB;") mode_camstab |= bit;
    if (boxnames[c] == "GPS HOME;") mode_gpshome |= bit;
    if (boxnames[c] == "GPS HOLD;") mode_gpshold |= bit;
    if (boxnames[c] == "OSD SW;") mode_osd_switch |= bit;
    if (boxnames[c] == "MISSION;") mode_gpsmission |= bit;
   
    bit <<= 1L;
  }
}

public void ShowSPort(){

  if (SFRSKY.arrayValue()[0]<1) return;

  int SYM_MIN = 0xB3;
  int SYM_AVG = 0xB4;
  int SYM_CELL0 = 0xF0;
  
//  String output = OnePlaceDecimal.format(voltage)
   float cells=confItem[GetSetting("S_BATCELLS")].value();
    if (cells<1) return;
   float fcellvoltage=(sVBat/cells);
   fcellvoltage=constrain(fcellvoltage,3.4f,4.2f); 
   int cellpos=PApplet.parseInt (map(fcellvoltage,3.4f,4.2f,0,14));
//   cellpos=0;
   String cellvoltage=TwoPlaceDecimal.format(fcellvoltage);  

   for(int i=0; i<6; i++) {
     if(i>(cells-1))continue;//empty cell
     mapchar(SYM_CELL0+cellpos, SimPosn[SportPosition]+(5-i));
   }
      
   String output = TwoPlaceDecimal.format(sVBat);
   mapchar(SYM_MIN, SimPosn[SportPosition]+LINE);
   makeText(cellvoltage+"v", SimPosn[SportPosition]+LINE+1);

   output = TwoPlaceDecimal.format(sVBat);
   mapchar(SYM_AVG, SimPosn[SportPosition]+LINE+LINE);
   makeText(cellvoltage+"v", SimPosn[SportPosition]+LINE+LINE+1);

}

public void ShowMapMode(){
  int mi;

  if (toggleModeItems[9].getValue()==2)
    mi = PApplet.parseInt(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    mi = PApplet.parseInt(confItem[GetSetting("S_HUDSW1")].value());
  else
    mi = PApplet.parseInt(confItem[GetSetting("S_HUDSW0")].value()); 

  if (CONFIGHUDEN[mi][MapModePosition]==0)
    return;
int SYM_HOME      = 0x04;
int SYM_AIRCRAFT  = 0X05;
int SYM_RANGE_100 = 0x21;
int SYM_RANGE_500 = 0x22;
int SYM_RANGE_2500= 0x23;
int SYM_RANGE_MAX = 0x24;
int SYM_DIRECTION = 0x72;  int xdir=0;
  int mapstart=0;
  int mapend=0;
  int ydir=0;
  int targetx=0;
  int targety=0;
  int range=200;
  int angle=0;
  int targetpos=0;
  int centerpos=0;
  int maxdistance=0;
  int mapsymbolcenter=0;
  int mapsymboltarget=0;
  int mapsymbolrange=0;
  int tmp=0;
  int GPS_directionToHome=PApplet.parseInt(SGPSHeadHome.value());

  if(GPS_directionToHome < 0) GPS_directionToHome += 360;

  if ((toggleModeItems[0].getValue() == 0 )){
    armedangle=MwHeading;
  }

//  int MwHeading=Mwheading;


  switch(PApplet.parseInt(confItem[GetSetting("S_MAPMODE")].value())) {
    case 1:
      mapstart=0;mapend=1;
      break;
    case 2:
      mapstart=1;mapend=2;
      break;
    case 3:
      mapstart=0;mapend=2;
      break;
    case 4:
      mapstart=1;mapend=2;
      break;
    default:
      return;
  }

 
for(int maptype=mapstart; maptype<mapend; maptype++) {

  if (maptype==1) {
    angle=(180+360+GPS_directionToHome-armedangle)
    %360;
//    angle=(180+360+GPS_directionToHome-armedangle+MwHeading)%360;
  }
  else {
    angle=(360+GPS_directionToHome-MwHeading)%360;  
  }
  tmp = angle/90;
  switch (tmp) {
    case 0:
      xdir=+1;
      ydir=-1;
      break;
    case 1:    
      xdir=+1;
      ydir=+1;
      angle=180-angle;
      break;
    case 2:    
      xdir=-1;
      ydir=+1;
      angle=angle-180;
      break;
    case 3: 
      xdir=-1;
      ydir=-1;
      angle=360-angle;
      break;   
    }  

  float rad  = angle * PI / 180;    // convert to radians  
  int x = PApplet.parseInt(SGPS_distanceToHome.value() * sin(rad));
  int y = PApplet.parseInt(SGPS_distanceToHome.value() * cos(rad));

  if (y > x) maxdistance=y;
  else maxdistance=x;
  if (maxdistance < 100) {
    range = 100;
    mapsymbolrange=SYM_RANGE_100;
  }
  else if (maxdistance < 500) {
    range = 500;
    mapsymbolrange=SYM_RANGE_500;
  }
  else if (maxdistance < 2500) {
    range = 2500;
    mapsymbolrange=SYM_RANGE_2500;
  }
  else {
    range = maxdistance;
    mapsymbolrange=SYM_RANGE_MAX;
  }

  targetx = PApplet.parseInt(xdir*map(x, 0, range, 0, 16));
  targety = PApplet.parseInt(ydir*map(y, 0, range, 0, 15));
 
  if (maxdistance<20) {
    targetx = 0;
    targety = 0;  
  }

  centerpos=SimPosn[MapCenterPosition];
  targetpos= centerpos + (targetx/2) + (LINE*(targety/3)); 


  if (maptype==1) {
    mapsymbolcenter = SYM_HOME;
    mapsymboltarget = SYM_AIRCRAFT;
  }
  else {
    mapsymbolcenter = SYM_AIRCRAFT;
    mapsymboltarget = SYM_HOME;
  }
  
  mapchar(mapsymbolcenter,centerpos);

/*
  if (maptype==0) {
    tmp=(360+382+MwHeading-armedangle)%360/45;
    tmp = SYM_DIRECTION + tmp;
  }
  else {
    tmp = mapsymboltarget;
  }
*/

/*  if (confItem[GetSetting("S_MAPMODE")].value()==4) {
    tmp=(360+382+MwHeading-armedangle)%360/45;
    mapsymboltarget = SYM_DIRECTION + tmp;
  }
*/
 
    int symx = (int)abs(targetx)%2;
    int symy = (int)abs(targety)%3;
    if (ydir==1)
      symy=2-symy;
    if (xdir==-1)
      symx=1-symx;
    if (abs(targety)<3)
      symy = 1 - ydir;
    if (abs(targetx)<2){
      if (targetx<0)
        symx=0;
      else
        symx=1;
    }

  if (maptype==0) 
    tmp = 0xD6;
  else {
    tmp = 0xD0;
/*  if (maptype==1) {
    tmp=(360+382+MwHeading-armedangle)%360/45;
    mapsymboltarget = SYM_DIRECTION + tmp;
    0x72
  }
*/

}
  mapsymboltarget = mapsymboltarget + symy + (symx*3);


    tmp = tmp+ symy + (symx*3);
//System.out.println(xdir+" "+ydir+" "+symx+" "+symy+" "+targetx+" "+targety+" "+tmp);
  mapchar(mapsymbolrange,SimPosn[MapModePosition]);

  if (confItem[GetSetting("S_MAPMODE")].value()==4) {
    tmp=(360+382+MwHeading-armedangle)%360/45;
    tmp = SYM_DIRECTION + tmp;
  }

  if (maxdistance>20) {
    mapchar(tmp,targetpos);
  }
}

 
}

public void LinksSetup(){

}


byte[][] raw_font;
PrintWriter  Output;

public PImage LoadFont(String filename) {
  //System.out.println("LoadFont "+filename);
  raw_font = LoadRawFont(filename);
  return RawFontToImage(raw_font);
}


public byte[][] LoadRawFont(String filename) {
  byte[][] raw = new byte[256][54];

  InputStream in = null;
  
  byte[] header = { 'M','A','X','7','4','5','6' };
  boolean inHeader = true;
  int hIndex = 0;
  int bitNo = 0;
  int byteNo = 0;
  int charNo = 0;
  int curByte = 0;
 
  try {
    in = createInput(filename);
    
    while(in.available() > 0) {
      int inB = in.read();
     
      if(inHeader) {
        if(hIndex < header.length && header[hIndex] == inB) {
          hIndex++;
          continue;
        }
        if(hIndex == header.length && (inB == '\r' || inB == '\n')) {
          inHeader = false;
          //System.out.println("done header");
          continue;
        }
        hIndex = 0;
        continue;
      }
      else {
        switch(inB) {
        case '\r':
        case '\n':
          if (bitNo == 0)
            continue; 
          if (bitNo == 8) {
            if (byteNo < 54) {
              //System.out.println("*charNo="+charNo+" byteNo="+byteNo);//+" value="+(int)raw[charNo][byteNo]);
              raw[charNo][byteNo] = (byte)curByte;
            }
            bitNo = 0;
          
            curByte = 0;
            ++byteNo;
            if(byteNo == 64) {
              //System.out.println("Loaded char "+charNo);
              byteNo = 0;
              ++charNo;
            }
          }
          break;
        case '0':
        case '1':
          if(bitNo >= 8) {
            throw new Exception("File format error");
          }
          curByte = (curByte << 1) | (inB & 1);
          ++bitNo;
          break;
        }
      }
    }
  }
  catch (FileNotFoundException e) {
      System.out.println("File Not Found "+filename);
  }
  catch (IOException e) {
      System.out.println("IOException");
  }
  catch(Exception e) {
      System.out.println("Exception");
  }
  finally {
    if(in != null)
      try {
        in.close();
      }
      catch (IOException ioe) {
      }
  }
  
  return raw;
}

public PImage RawFontToImage(byte[][] raw) {
  PImage img = createImage(12, 18*256, ARGB);
  img.loadPixels();

  // Pixel values
  int white = 0xFFFFFFFF;
  int black = 0xFF000000;
  int transparent = 0x00000000;
  int gray = color(120);

  for(int charNo = 0; charNo < 256; charNo++) {
    CharImages[charNo].loadPixels();
   // CharImages[charNo] = createImage(12, 18, ARGB);
    for(int byteNo = 0; byteNo < 54; byteNo++) {
      for(int i = 0; i < 4; i++) {
	int index = (charNo*12*18) + (byteNo*4) + (3-i);
        int CharIndex = (byteNo*4) + (3-i);
        //System.out.println("charNo="+charNo+" byteNo="+byteNo+" index="+index+" CharIndex="+CharIndex+" Charlimit="+CharImages[charNo].pixels.length + " limit="+img.pixels.length); 
        int curByte = (int)raw[charNo][byteNo];
	switch((curByte >> (2*i)) & 0x03) {
	case 0x00:
	   img.pixels[index] = black;
            CharImages[charNo].pixels[CharIndex] = black;
	   break; 
	case 0x01:
	   img.pixels[index] = transparent;
            CharImages[charNo].pixels[CharIndex] = gray;
	   break; 
	case 0x02:
	   img.pixels[index] = white;
            CharImages[charNo].pixels[CharIndex] = white;
	   break; 
	case 0x03:
	   img.pixels[index] = transparent;
            //CharImages[charNo].pixels[CharIndex] = gray;

	   break; 
	}
      }
    }
    CharImages[charNo].updatePixels();
  }

  img.updatePixels();
  return img;
}

public void CreateFontFile(){
  String gray = "01";  // gray in Gui transparrent in OSD
  String black = "00"; //black
  String white = "10";
  int PixelCounter = 0;
  int fullpixels = 0;
  String OutputLine = "";
  
  Output = createWriter("data/custom.mcm");
  
  Output.println("MAX7456"); // write header
  for(int id = 0; id < 256; id++) {
    for(int byteNo = 0; byteNo < 216; byteNo++) {
      switch(CharImages[id].pixels[byteNo]) {
        case 0xFF000000:
          OutputLine+=black;
          PixelCounter+=1;
        break; 
        case 0xFFFFFFFF:
          OutputLine+=white;
          PixelCounter+=1;
        break;
       default:
         OutputLine+=gray;
         PixelCounter+=1;
        break;  
      }
      
    if(PixelCounter == 4){
      Output.println(OutputLine);
      OutputLine = "";
      PixelCounter = 0;
    }
     
    }
    for(int spacer = 0; spacer < 10; spacer++) {
      Output.println("01010101");
    }
  }
  
 //Output.println("done");
 Output.flush(); // Writes the remaining data to the file
  Output.close();  
  img_Clear = LoadFont("/data/default.mcm");  
}


  
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "MW_OSD_GUI" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
