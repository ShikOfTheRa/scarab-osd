


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


void LayoutEditorSetup(){
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

 
void SimSetup(){

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
    .setMaxX(90) 
    .setMaxY(-90) 
    .setMinX(-90) 
    .setMinY(90)
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
  .setValue(15.0)
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

boolean checkKey(int k)
{
  if (keys.length >= k) {
    return keys[k];  
  }
  return false;
}



void keyPressed()
{ 
  keys[keyCode] = true;
  //println(keyCode);
  if((checkKey(CONTROL) == true) && (checkKey(85) == true)){
    SketchUploader();
  }
}


void keyReleased()
{ 
  keys[keyCode] = false; 
  ControlLock();
  
}



void CalcAlt_Vario(){
  if (time2 < time - 1000){
    sAltitude += sVario /10;
    time2 = time;
  }
}

String RightPadd(int inInt,int Places){
  String OutString = nf(inInt,Places).replaceFirst("^0+(?!$)",  " ");
  for(int X=0; X<=3; X++) {
    if (OutString.length() < Places){ OutString = " " + OutString;}
  }
  return OutString;  
  
}






void ShowVolts(float voltage){  
  if(confItem[GetSetting("S_DISPLAYVOLTAGE")].value() > 0) {
    String output = OnePlaceDecimal.format(voltage);
    int SYM_VOLT = 0XA9;
    output =output+char(0XA9);
    mapchar(0x97, SimPosn[voltagePosition]-1);
    makeText(output, SimPosn[voltagePosition]);
  }
}

void ShowVideoVolts(float voltage){  
  if(confItem[GetSetting("S_VIDVOLTAGE")].value() > 0) {
    String output = OnePlaceDecimal.format(voltage);
    int SYM_VOLT = 0XA9;
    output =output+char(0XA9);
    mapchar(0xBF, SimPosn[vidvoltagePosition]-1);
    makeText(output, SimPosn[vidvoltagePosition]);
  }
}


void ShowFlyTime(String FMinutes_Seconds){

  if (int(confItem[GetSetting("S_TIMER")].value()) > 0){
  mapchar(0x9c, SimPosn[flyTimePosition]);
  makeText(FMinutes_Seconds, SimPosn[flyTimePosition]+1);
}}

void ShowOnTime(String Minutes_Seconds){
  if (int(confItem[GetSetting("S_TIMER")].value()) > 0){
  mapchar(0x9b, SimPosn[onTimePosition]);
  makeText(Minutes_Seconds, SimPosn[onTimePosition]+1);
  }}

void ShowCurrentThrottlePosition(){
  if(confItem[GetSetting("S_THROTTLEPOSITION")].value() > 0) {
  mapchar(0xc8, SimPosn[CurrentThrottlePosition]);
  
  if(armed){
    int CurThrottle = int(map(Throttle_Yaw.arrayValue()[1],1000,2000,0,100));
    makeText(RightPadd(CurThrottle,3) + "%", SimPosn[CurrentThrottlePosition]+1);   
  }
  else
  {
    makeText("  --", SimPosn[CurrentThrottlePosition]+1);
  }

  
  
  
  //makeText(" 40%", CurrentThrottlePosition[ScreenType]+1);
}}



void ShowLatLon(){
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

void ShowDebug(){
  if(confItem[GetSetting("S_DEBUG")].value() > 0) {
    makeText("0: "+debug[0], SimPosn[debugPosition]);
    makeText("1: "+debug[1], SimPosn[debugPosition]+LINE);
    makeText("2: "+debug[2], SimPosn[debugPosition]+LINE+LINE);
    makeText("3: "+debug[3], SimPosn[debugPosition]+LINE+LINE+LINE+LINE);
  }
}


void ShowSats(){
  String output = str(int(SGPS_numSat.getValue()));
  mapchar(0x1e, SimPosn[GPS_numSatPosition]);
  mapchar(0x1f, SimPosn[GPS_numSatPosition]+1);
  makeText(output, SimPosn[GPS_numSatPosition]+2);
}

void ShowSpeed(){
  int SimModebits = 0;
  int SimBitCounter = 1;
  for (int i=0; i<boxnames.length; i++) {
      if(toggleModeItems[i].getValue() > 0) SimModebits |= SimBitCounter;
      SimBitCounter += SimBitCounter;
  }
  String output = str(0);
  if((SimModebits&mode_armed) >0){
    output = str(int(SGPS_speed.getValue()/27.7778));
  }
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa6, SimPosn[speedPosition]);}
  else {mapchar(0xa5, SimPosn[speedPosition]);}
  makeText(output, SimPosn[speedPosition]+1);
}

void ShowDirection(){
  int dirhome=0x60 + ((180+360+22+MwHeading-int(SGPSHeadHome.value()))%360)*2/45;
 
  mapchar(dirhome, SimPosn[GPS_directionToHomePosition]);
}

void ShowGPSAltitude(){
  String output = str(int(SGPS_altitude.getValue())/100);
  if(confItem[GetSetting("S_GPSALTITUDE")].value() > 0) {
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa8, SimPosn[MwGPSAltPosition]);}
  else {mapchar(0xa7, SimPosn[MwGPSAltPosition]);}
  makeText(output, SimPosn[MwGPSAltPosition]+1);
}}


void ShownAngletohome(){
  String output = str((360+int(SGPSHeadHome.getValue()))%360/1);
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

void ShowAltitude(){
  String output = str(int(s_Altitude.getValue()));
  if(confItem[GetSetting("S_BAROALT")].value() > 0) {
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa8, SimPosn[MwAltitudePosition]);}
  else {mapchar(0xa7, SimPosn[MwAltitudePosition]);}
  makeText(output, SimPosn[MwAltitudePosition]+1);
  }}

void ShowDistance(){
  String output = str(int(SGPS_distanceToHome.getValue()));
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xb9, SimPosn[GPS_distanceToHomePosition]);}
  else {mapchar(0xbb, SimPosn[GPS_distanceToHomePosition]);}
  makeText(output, SimPosn[GPS_distanceToHomePosition]+1);
  }

void ShowVario(){
  if(confItem[GetSetting("S_VARIO")].value() > 0) {
  mapchar(0x7f, SimPosn[MwClimbRatePosition]-LINE);
  mapchar(0x8c, SimPosn[MwClimbRatePosition]);
  mapchar(0x7f, SimPosn[MwClimbRatePosition]+LINE);
}}

void ShowRSSI(){
  String output = str(int(s_MRSSI.getValue()));
  if(confItem[GetSetting("S_DISPLAYRSSI")].value() > 0) {
  mapchar(0xba, SimPosn[rssiPosition]);
  makeText(output + "%", SimPosn[rssiPosition]+1);
}}

void ShowAPstatus(){
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


void ShowCallsign(){
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

void ShowAmperage(){
  if(confItem[GetSetting("S_AMPER_HOUR")].value() > 0) {
  mapchar(0xa4, SimPosn[pMeterSumPosition]);
  makeText("1221", SimPosn[pMeterSumPosition]+1);
}}

void ShowTemp(){
  makeText("30", SimPosn[temperaturePosition]);
  mapchar(0x0e, SimPosn[temperaturePosition]+2);
}

void ShowAmps(){
  if(confItem[GetSetting("S_AMPERAGE")].value() > 0) {
  makeText("15.2A", SimPosn[amperagePosition]+1);
  }}

void ShowUTC(){
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

void displayHeading()
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


void SimulateTimer(){
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


void displayMode()
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



void displayHorizon(int rollAngle, int pitchAngle)
{

  int minimalscreen=0 ; 
  if (toggleModeItems[9].getValue()>0) minimalscreen=1 ;

  int SYM_COLON = 0xB3;
  String xx=str(pitchAngle/10);
  if (SimPosn[pitchAnglePosition]<0x3FF){
    mapchar(0x50, SimPosn[pitchAnglePosition]);
    makeText(xx, SimPosn[pitchAnglePosition]+1);
  }
  if (SimPosn[rollAnglePosition]<0x3FF){
    xx=str(rollAngle/10);
    mapchar(0x52, SimPosn[rollAnglePosition]);
    makeText(xx, SimPosn[rollAnglePosition]+1);
  }

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

void ShowSideBarArrows(){
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


void displayHeadingGraph()
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


void ControlLock(){
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

void GetModes(){
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

void ShowSPort(){

  if (SFRSKY.arrayValue()[0]<1) return;

  int SYM_MIN = 0xB3;
  int SYM_AVG = 0xB4;
  int SYM_CELL0 = 0xF0;
  
//  String output = OnePlaceDecimal.format(voltage)
   float cells=confItem[GetSetting("S_BATCELLS")].value();
    if (cells<1) return;
   float fcellvoltage=(sVBat/cells);
   fcellvoltage=constrain(fcellvoltage,3.4,4.2); 
   int cellpos=int (map(fcellvoltage,3.4,4.2,0,14));
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

void ShowMapMode(){
  int mi;

  if (toggleModeItems[9].getValue()==2)
    mi = int(confItem[GetSetting("S_HUDSW2")].value());
  else if (toggleModeItems[9].getValue()==1)
    mi = int(confItem[GetSetting("S_HUDSW1")].value());
  else
    mi = int(confItem[GetSetting("S_HUDSW0")].value()); 

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
  int GPS_directionToHome=int(SGPSHeadHome.value());

  if(GPS_directionToHome < 0) GPS_directionToHome += 360;

  if ((toggleModeItems[0].getValue() == 0 )){
    armedangle=MwHeading;
  }

//  int MwHeading=Mwheading;


  switch(int(confItem[GetSetting("S_MAPMODE")].value())) {
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
  int x = int(SGPS_distanceToHome.value() * sin(rad));
  int y = int(SGPS_distanceToHome.value() * cos(rad));

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

  targetx = int(xdir*map(x, 0, range, 0, 16));
  targety = int(ydir*map(y, 0, range, 0, 15));
 
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

void LinksSetup(){

}


