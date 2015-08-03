
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

 



import processing.serial.Serial; // serial library
import controlP5.*; // controlP5 library
import java.io.File;
import java.lang.*; // for efficient String concatemation
import javax.swing.SwingUtilities; // required for swing and EDT
import javax.swing.JFileChooser;// Saving dialogue
import javax.swing.filechooser.FileFilter; // for our configuration file filter "*.mwi"
import javax.swing.JOptionPane; // for message dialogue
//import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.InputStream;
import java.io.OutputStream; 
import java.io.FileOutputStream;
import java.io.FileInputStream;
import java.util.*;
import java.io.FileNotFoundException;
import java.text.DecimalFormat;

//added new imports to support proccessing 2.0b7



String MW_OSD_GUI_Version = "MWOSD R1.5 - NextGeneration";
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
int  pitchAnglePosition = 15;
int  rollAnglePosition = 16;
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
int GPSstartlat = 430948610;
int GPSstartlon = -718897060;

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
Textlabel txtlblWhichcom,txtlblWhichbaud,txtmessage,mspmessage; 
Textlabel txtlblLayoutTxt,txtlblLayoutEnTxt, txtlblLayoutHudTxt; 
Textlabel txtlblLayoutTxt2,txtlblLayoutEnTxt2, txtlblLayoutHudTxt2; 
ListBox commListbox,baudListbox;

//char serialBuffer[] = new char[128]; // this hold the imcoming string from serial O string
//String TestString = "";
///String SendCommand = "";




boolean PortRead = false;
boolean PortWrite = false;
int PortReadtimer = 0;
//int ReadConfig = 0;
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
int FontMSPMillis=0;


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
int Simtype=0;
int StartupMessage=0;
int FrameRate=30;

int init_com = 0;
int commListMax = 0;
int whichKey = -1;  // Variable to hold keystoke values
int inByte = -1;    // Incoming serial data
int[] serialInArray = new int[3];    // Where we'll put what we receive
int[] debug = new int[4];    
String progresstxt="";
String msptxt="";
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
color yellow_ = color(200, 200, 20), 
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

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}



void setup() {
  size(windowsX,windowsY);
  Locale.setDefault(Locale.US);


//  LoadConfig();
//  xml = loadXML("HUDLAYOUT.xml");
  xml = loadXML("hudlayout.xml");
//  xml = loadXML(dataPath("hudlayout.xml"));
  initxml();
 
//Map<Settings, String> table = new EnumMap<Settings>(Settings.class);
OnTimer = millis();
  frameRate(FrameRate); 
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
  baudListbox.addItem("9600",4);
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
  mspmessage = controlP5.addTextlabel("mspmessage","",XHUD+735,155); // textdebug
//  XHUD+735,22

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
  confItem[GetSetting("S_VOLTAGEMIN")].setMultiplier(0.1);
  
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
  confItem[GetSetting("S_GPSTZ")].setMultiplier(0.5);//30min increments, kathmandu would require 15min, it can use DST 
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


controlP5.Controller hideCheckbox(controlP5.Controller c) {
  c.hide();
  //c.setLabelVisible(false);
  return c;
}

controlP5.Controller CheckboxVisable(controlP5.Controller c) {
  c.isVisible(); 

  //c.setLabelVisible(false);
  return c;
}



void BuildRadioButton(int ItemIndex, int XLoction, int YLocation,Group inGroup, String Cap1, String Cap2){
    
  RadioButtonConfItem[ItemIndex] = controlP5.addRadioButton("RadioButton"+ItemIndex)
         .setPosition(XLoction,YLocation+3)
         .setSize(10,10)
         .setNoneSelectedAllowed(false) 
         //.setColorBackground(color(120))
         //.setColorActive(color(255))
        // .setColorLabel(color(255))
         .setItemsPerRow(2)
         .setSpacingColumn(int(textWidth(Cap1))+10)
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

void CreateCS(int ItemIndex, int XLoction, int YLocation, Group inGroup){
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

void CreateItem(int ItemIndex, int XLoction, int YLocation, Group inGroup){
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


void BuildToolHelp(){
  controlP5.getTooltip().setDelay(100);
}





void MakePorts(){

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

void draw() {

     MWData_Com(); 
//  debug[0]=WriteLayouts;
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

// Check fontmode has finished
   if (millis() > FontMSPMillis){
     FontMode=false;
   }

//    debug[3]=int(SimControlToggle.getValue());


// Process and outstanding Read or Write EEPROM requests
  if((millis()>ReadConfigMSPMillis)&&(millis()>WriteConfigMSPMillis)&&(init_com==1)){
    if (AutoSimulator !=0){
      SimControlToggle.setValue(1);
    }
  }
  if (int(SimControlToggle.getValue())==0){
    toggleModeItems[0].setValue(0);
  }
    
    if (millis()<ReadConfigMSPMillis){
      if (init_com==1){
        READconfigMSP();
//        toggleMSP_Data = true;
        int progress=100*eeaddressGUI/CONFIGITEMS;
        if (progress==0){
          progresstxt="Waiting OSD...";   
        }
        else{
          progresstxt="Read: "+progress+"%";   
        }    
      }
    }

    if (millis()<WriteConfigMSPMillis){
      if (init_com==1){
        WRITEconfigMSP();
//        toggleMSP_Data = true;
        int progress=100*eeaddressGUI/(CONFIGITEMS);
        if (WriteLayouts==1){
          progress=100*eeaddressGUI/(CONFIGITEMS + (hudoptions*3*2));
        }
        if (progress==0){
          progresstxt="Waiting OSD...";   
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
    mspmessage.setValue(msptxt);

// Layout editor
  txtlblLayoutTxt.setValue(" : "+ CONFIGHUDTEXT[hudeditposition]);

  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = int(confItem[GetSetting("S_HUDSW0")].value()); 
  txtlblLayoutHudTxt.setValue(" : "+hudid);
  LEW.setLabel("Layout Editor for profile: "+hudid+" - "+CONFIGHUDNAME[hudid]);
  if (CONFIGHUDEN[hudid][hudeditposition]>0)
    txtlblLayoutEnTxt.setValue(" : Enabled");
  else
    txtlblLayoutEnTxt.setValue(" : Disabled");

// Layout amendments based upon choices
  if (int(confItem[GetSetting("S_RCWSWITCH")].value())==0){
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
  if ((init_com==1)) {
    MWData_Com();
    if (!FontMode) PortRead = false;
    MakePorts();
  }
  else {
    PortRead = false;
    MakePorts();
    msptxt="";
  }

  if ((SendSim ==1) && (ClosePort == false)) 
  {
    if (!FontMode&&(WriteConfig==0)) {
      if (init_com==1) {
        if (ClosePort) return;
 ///*
        if ((int(SimControlToggle.getValue())!=0)&&(Simtype==0)) {

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
          if ((init_com==1)) SendCommand(MSP_BOXNAMES);
          break;
        case 8:
          if ((init_com==1)) SendCommand(MSP_BOXIDS);
          break;
        case 9:
          if (init_com==1)SendCommand(MSP_NAV_STATUS);
          break;
        case 10:
          if (init_com==1)SendCommand(MSP_DEBUG);
          break;
        case 11:
          if (init_com==1)SendCommand(MSP_PID);
          MSP_sendOrder=1;
          break;
        default:  
          MSP_sendOrder=1;
        }
        PortWrite = !PortWrite; // toggle TX LED every other    
      } 
//*/
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
 
   ConfigLayout[0][i]=CONFIGHUD[int(confItem[GetSetting("S_HUDSW0")].value())][i];
   ConfigLayout[1][i]=CONFIGHUD[int(confItem[GetSetting("S_HUDSW1")].value())][i];
   ConfigLayout[2][i]=CONFIGHUD[int(confItem[GetSetting("S_HUDSW2")].value())][i];


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
  
      
    if (int(confItem[GetSetting("S_DISPLAYRSSI")].value()) > 0)    ShowRSSI(); 


  if(confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() > 0) displayHorizon(int(MW_Pitch_Roll.arrayValue()[0])*10,int(MW_Pitch_Roll.arrayValue()[1])*10*-1);
  SimulateTimer();
  CalcAlt_Vario(); 

  ShowCurrentThrottlePosition();
  if (int(confItem[GetSetting("S_DISPLAYRSSI")].value()) > 0)    ShowRSSI(); 
  if (int(confItem[GetSetting("S_DISPLAYVOLTAGE")].value()) > 0) ShowVolts(sVBat);

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


int GetSetting(String test){
  int TheSetting = 0;
  for (int i=0; i<Settings.values().length; i++) 
  if (Settings.valueOf(test) == Settings.values()[i]){ 
      TheSetting = Settings.values()[i].ordinal();
  }
  return TheSetting;
}


void ShowSimBackground(float[] a) {
  Showback = int(a[0]);
}

//void SimulateMultiWii(float[] a) {
//}

public void BuildCallSign(){
  String CallSText = "";
  for (int i=0; i<10; i++){ 
    //confItem[GetSetting("S_CS0")+i].setValue(0);
    if (int(confItem[GetSetting("S_CS0")+i].getValue())>0){
    CallSText+=char(int(confItem[GetSetting("S_CS0")+i].getValue()));
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
      confItem[(GetSetting("S_CS0"))+i].setValue(int(CallSText.charAt(i)));
    //println(int(CallSText.charAt(0)));
    //println(controlP5.get(Textfield.class,"CallSign").getText());
    }
  //}
}




void MatchConfigs(){
 for(int i=0;i<CONFIGITEMS;i++) {
   try{ 
       if (RadioButtonConfItem[i].isVisible()){
          confItem[i].setValue(int(RadioButtonConfItem[i].getValue()));
       }
        }catch(Exception e) {}finally {}
    
  
     if  (toggleConfItem[i].isVisible()){
       if (int(toggleConfItem[i].getValue())== 1){
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
      if (int(theEvent.group().value()) ==4) BaudRate=9600;
      else if (int(theEvent.group().value()) ==3) BaudRate=19200;
      else if (int(theEvent.group().value()) ==2) BaudRate=38400;
      else if (int(theEvent.group().value()) ==1) BaudRate=57600;
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
        int ColorCheck = int(theEvent.getController().value());
        curPixel = theEvent.controller().id();
    }
    if ((theEvent.getController().getName().substring(0, 7).equals("CharMap")) && (theEvent.getController().isMousePressed())) {
      curChar = theEvent.controller().id();    
      //println("Got a Char " + theEvent.controller().id());
    }
   } catch(ClassCastException e){}
     catch(StringIndexOutOfBoundsException se){}
      
}





void mapchar(int address, int screenAddress){
  int placeX = (screenAddress % 30) * 12;
  int placeY = (screenAddress / 30) * 18;

  blend(img_Clear, 0,address*18, 12, 18, placeX+DisplayWindowX, placeY+DisplayWindowY, 12, 18, BLEND);
}

void makeText(String inString, int inStartAddress ){
  for (int i = 0; i < inString.length(); i++){
    mapchar(int(inString.charAt(i)), inStartAddress +i); 
  }   
}



void displaySensors()
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
    i = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = int(confItem[GetSetting("S_HUDSW0")].value());
    
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
    i = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = int(confItem[GetSetting("S_HUDSW0")].value());
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
    i = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = int(confItem[GetSetting("S_HUDSW0")].value());
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
    i = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = int(confItem[GetSetting("S_HUDSW0")].value());
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
    i = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    i = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    i = int(confItem[GetSetting("S_HUDSW0")].value());
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
    hudid = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = int(confItem[GetSetting("S_HUDSW0")].value());
  hudid--;
  hudid= constrain(hudid,0,hudsavailable-1);
  confItem[GetSetting("S_HUDSW2")].setValue(hudid);
  confItem[GetSetting("S_HUDSW1")].setValue(hudid);
  confItem[GetSetting("S_HUDSW0")].setValue(hudid);
}
public void bHUDLDOWN() {
  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = int(confItem[GetSetting("S_HUDSW0")].value());
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
  int value = int(MWI.getProperty(ConfigNames[j]));
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
  ConfigClass.setProperty("Simtype",str(Simtype));
  ConfigClass.setProperty("StartupMessage",str(StartupMessage));
  ConfigClass.setProperty("FrameRate",str(FrameRate));
  
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
    Simtype=1;
    FrameRate = 15;
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
      BaudRate =int(ConfigClass.getProperty("BaudRate"));
      Title =ConfigClass.getProperty("Title");
      Passthroughcomm = int(ConfigClass.getProperty("Passthroughcomm"));
      AutoSimulator = int(ConfigClass.getProperty("AutoSimulator"));
      Simtype = int(ConfigClass.getProperty("Simtype"));
      FrameRate = int(ConfigClass.getProperty("FrameRate"));
      frameRate(FrameRate); 

      StartupMessage = int(ConfigClass.getProperty("StartupMessage"));

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


void mouseReleased() {
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
        
        
void SketchUploader(){
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

//  toggleMSP_Data = false;
  //delay(1000);
  InitSerial(200.00);
  updateConfig(); 
  //if (init_com==1){
    //init_com=0;
  //g_serial.stop();
  //}
  
 
  
  super.exit();
}


public void GPSTIMELINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/GPSTime.md"); 
}
public void SPORTLINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/Frsky_SPort.md"); 
}
public void CODELINK(){
 link("https://www.mwosd.com/"); 
}
public void FAQLINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/FAQ.md"); 
}
public void GUIDELINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/User_Guide.md"); 
}
public void SUPPORTLINK(){
  link("http://fpvlab.com/forums/showthread.php?34250-MWOSD-for-MULTIWII-NAZE32-BASEFLIGHT-HARIKIRI"); 
}
public void DONATELINK(){
  link("https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=EBS76N8F426G2&lc=GB&item_name=MW%2dOSD&item_number=R1%2e4&currency_code=GBP&bn=PP%2dDonationsBF%3abtn_donate_SM%2egif%3aNonHosted"); 
}
public void CALIBLINK(){
 link("https://github.com/ShikOfTheRa/scarab-osd/blob/master/OTHER/DOCUMENTATION/Calibration.md"); 
}


void initxml(){
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


void xmlsavelayout(){
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


void setset(){
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


void addchild(){

  int hudid=0;
  if (toggleModeItems[9].getValue()==2)
    hudid = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    hudid = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    hudid = int(confItem[GetSetting("S_HUDSW0")].value());
  
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


void coloriseswitches(){
  for(int i=0;i<CONFIGITEMS;i++) {
    if (toggleConfItem[i].getValue()==1)
      toggleConfItem[i].setColorActive(switches_);
    else
      toggleConfItem[i].setColorActive(red_);
  }
  if (int(SimControlToggle.getValue())==1)
    SimControlToggle.setColorActive(switches_);
  else
    SimControlToggle.setColorActive(red_);
}

void LEW(){
//  Lock_All_Controls(true);
  G_LINKS.hide();
  LEW.show();
  LEWvisible=1;
//  Lock_All_Controls(true);
}

void READEEMSP(){
  READconfigMSP_init();
}

void WRITEEEMSP(){
  WRITEconfigMSP_init();
}

void mouseClicked(){
  if ((mouseX>=(XLINKS+20)) && (mouseX<=(XLINKS+20+100)) && (mouseY>=(YLINKS+207)) && (mouseY<=(YLINKS+207+30)))
    DONATELINK();
}
