


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
int mode_llights = 0;
int mode_camstab = 0;
int mode_osd_switch = 0;

int SendSim = 0;

boolean[] keys = new boolean[526];
boolean armed = false;

Group SG,SGControlBox,SGModes,SGAtitude,SGRadio,SGSensors1,SGGPS; 

// Checkboxs
CheckBox checkboxSimItem[] = new CheckBox[SIMITEMS] ;
CheckBox ShowSimBackground, UnlockControls, SGPS_FIX;
//Toggles
Toggle toggleModeItems[] = new Toggle[boxnames.length] ;
Toggle SimControlToggle;
// Slider2d ---
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

    
//ControlWindow  Throttle_YawWindow;    

//CheckBox checkboxModeItems[] = new CheckBox[boxnames.length] ;
DecimalFormat OnePlaceDecimal = new DecimalFormat("0.0");



 
void SimSetup(){

  
 

  SG = ScontrolP5.addGroup("SG")
    .setPosition(305,YSim + 38)
    .setWidth(733)
    .setBarHeight(13)
    .activateEvent(true)
    .disableCollapse()
    .setBackgroundColor(color(0,255))
    .setBackgroundHeight(192)
   .setLabel("Simulator")
   .setMoveable(true);
    ;
                
 
  SGModes = ScontrolP5.addGroup("SGModes")
                .setPosition(629,25)
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
                .setPosition(525,25)
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
                .setPosition(391,25)
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
                .setPosition(5,25)
                .setWidth(175)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(110)
                .setLabel("Sensors 1")
                .setGroup(SG)
                //.close() 
               ;                                  
SGGPS = ScontrolP5.addGroup("SGGPS")
                .setPosition(186,25)
                .setWidth(200)
                .setBarHeight(15)
                .activateEvent(true)
                .disableCollapse()
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(111)
                .setLabel("GPS")
                .setGroup(SG)
                //.close() 
               ;

SGControlBox = ScontrolP5.addGroup("SGControlBox")
                .setPosition(5,150)
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
SimControlToggle.setValue(1);

SimControlText = controlP5.addTextlabel("SimControlText","Simulate on OSD",45,3);
SimControlText.setGroup(SGControlBox);
               

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
    SGPS_altitude.setGroup(SGGPS); 
    SGPS_altitude.setValue(10000);

 ScontrolP5.getController("SGPS_altitude").getCaptionLabel()
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);     
 
 SGPS_speed = ScontrolP5.addNumberbox("SGPS_speed",0,5,60,40,14);
    SGPS_speed.setLabel("Speed-cm/s");
    //SGPS_numSat.setColorBackground(red_);
    SGPS_speed.setMin(0);
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
    SGPS_distanceToHome.setValue(500);
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
   .setValue(-90)
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
  .setRange(9,17)
  .setValue(0)
  .setLabel("VBat")
  .setDecimalPrecision(1)
  .setValue(16.2)
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
  .setRange(0,255)
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
    toggleModeItems[i].setPosition(5,3+i*17);
    toggleModeItems[i].setSize(10,10);
    //toggleConfItem[i].setMode(ControlP5.SWITCH);
    toggleModeItems[i].setGroup(SGModes);
    txtlblModeItems[i] = controlP5.addTextlabel("ModeItems"+i,boxnames[i].substring(0, boxnames[i].length()-1) ,20,i*17);
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
  mapchar(0x97, voltagePosition[ScreenType]);
  makeText(output, voltagePosition[ScreenType]+2);
  }}

void ShowVideoVolts(float voltage){  
  if(confItem[GetSetting("S_VIDVOLTAGE")].value() > 0) {
String output = OnePlaceDecimal.format(voltage);
  mapchar(0xBF, voltagePosition[ScreenType]-LINE-LINE);
  makeText(output, voltagePosition[ScreenType]+2-LINE-LINE);
  }}


void ShowFlyTime(String FMinutes_Seconds){

  if (int(confItem[GetSetting("S_TIMER")].value()) > 0){
  mapchar(0x9c, flyTimePosition[ScreenType]);
  makeText(FMinutes_Seconds, flyTimePosition[ScreenType]+1);
}}

//void ShowOnTime(String Minutes_Seconds){
  //mapchar(0x9b, onTimePosition[ScreenType]);
  //makeText(Minutes_Seconds, onTimePosition[ScreenType]+1);
//}

void ShowCurrentThrottlePosition(){
  if(confItem[GetSetting("S_THROTTLEPOSITION")].value() > 0) {
  mapchar(0xc8, CurrentThrottlePosition[ScreenType]);
  
  if(armed){
    int CurThrottle = int(map(Throttle_Yaw.arrayValue()[1],1000,2000,0,100));
    makeText(RightPadd(CurThrottle,3) + "%", CurrentThrottlePosition[ScreenType]+1);   
  }
  else
  {
    makeText(" --", CurrentThrottlePosition[ScreenType]+1);
  }

  
  
  
  //makeText(" 40%", CurrentThrottlePosition[ScreenType]+1);
}}



void ShowLatLon(){
  if(confItem[GetSetting("S_COORDINATES")].value() > 0) {
  if(confItem[GetSetting("S_GPSCOORDTOP")].value() > 0) {
  mapchar(0xca, MwGPSLatPosition[ScreenType]);
  makeText(" 43.09486N", MwGPSLatPosition[ScreenType]+1);
  mapchar(0xcb, MwGPSLonPosition[ScreenType]);
  makeText(" 71.88970W", MwGPSLonPosition[ScreenType]+1);
  }
  else {
  mapchar(0xca, MwGPSMidLatPosition[ScreenType]);
  makeText(" 43.09486N", MwGPSMidLatPosition[ScreenType]+1);
  mapchar(0xcb, MwGPSMidLonPosition[ScreenType]);
  makeText(" 71.88970W", MwGPSMidLonPosition[ScreenType]+1);

  }
}
}

void ShowDebug(){
  if(confItem[GetSetting("S_DEBUG")].value() > 0) {
  makeText("0:    000", debugPosition[ScreenType]);
  makeText("1:    001", debugPosition[ScreenType]+LINE);
  makeText("2:    010", debugPosition[ScreenType]+LINE+LINE);
//  makeText("3:    011", debugPosition[ScreenType]+LINE+LINE+LINE);
}}


void ShowSats(){
  String output = str(int(SGPS_numSat.getValue()));
  mapchar(0x1e, GPS_numSatPosition[ScreenType]);
  mapchar(0x1f, GPS_numSatPosition[ScreenType]+1);
  makeText(output, GPS_numSatPosition[ScreenType]+2);
}

void ShowSpeed(){
  String output = str(int(SGPS_speed.getValue()/27.7778));
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa6, GPS_speedPosition[ScreenType]);}
  else {mapchar(0xa5, GPS_speedPosition[ScreenType]);}
  makeText(output, GPS_speedPosition[ScreenType]+1);
}

void ShowDirection(){
  mapchar(0x68, GPS_directionToHomePosition[ScreenType]);
}

void ShowGPSAltitude(){
  String output = str(int(SGPS_altitude.getValue())/100);
  if(confItem[GetSetting("S_GPSALTITUDE")].value() > 0) {
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa8, MwGPSAltPosition[ScreenType]);}
  else {mapchar(0xa7, MwGPSAltPosition[ScreenType]);}
  makeText(output, MwGPSAltPosition[ScreenType]+1);
}}


void ShownAngletohome(){
  String output = str(int(SGPSHeadHome.getValue())/1);
  if(confItem[GetSetting("S_ANGLETOHOME")].value() > 0) {
  

  switch (output.length())
  {
  case 1:
makeText(output, GPS_angleToHomePosition[ScreenType]+2);
break;
  case 2:
makeText(output, GPS_angleToHomePosition[ScreenType]+1);
break;
  case 3:
makeText(output, GPS_angleToHomePosition[ScreenType]);
break;
case 4:
makeText(output, GPS_angleToHomePosition[ScreenType]-1);
break;
}
 mapchar(0xbd, GPS_angleToHomePosition[ScreenType]+3); 

  }}

void ShowAltitude(){
  String output = str(int(s_Altitude.getValue()));
  if(confItem[GetSetting("S_BAROALT")].value() > 0) {
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xa8, MwAltitudePosition[ScreenType]);}
  else {mapchar(0xa7, MwAltitudePosition[ScreenType]);}
  makeText(output, MwAltitudePosition[ScreenType]+1);
  }}

void ShowDistance(){
  String output = str(int(SGPS_distanceToHome.getValue()));
  if(confItem[GetSetting("S_UNITSYSTEM")].value() > 0) {mapchar(0xb9, GPS_distanceToHomePosition[ScreenType]);}
  else {mapchar(0xbb, GPS_distanceToHomePosition[ScreenType]);}
  makeText(output, GPS_distanceToHomePosition[ScreenType]+1);
  }

void ShowVario(){
  if(confItem[GetSetting("S_VARIO")].value() > 0) {
  mapchar(0x7f, MwAltitudePosition[ScreenType]-1-LINE);
  mapchar(0x8c, MwAltitudePosition[ScreenType]-1);
  mapchar(0x7f, MwAltitudePosition[ScreenType]-1+LINE);
}}

void ShowRSSI(){
  String output = str(int(s_MRSSI.getValue()));
  if(confItem[GetSetting("S_DISPLAYRSSI")].value() > 0) {
  mapchar(0xba, rssiPosition[ScreenType]);
  makeText(output, rssiPosition[ScreenType]+1);
}}

void ShowAmperage(){
  if(confItem[GetSetting("S_AMPER_HOUR")].value() > 0) {
  mapchar(0xa4, amperagePosition[ScreenType]-1);
  makeText("1221", amperagePosition[ScreenType]);
}}

void ShowTemp(){
  if(confItem[GetSetting("S_DISPLAYTEMPERATURE")].value() > 0) {
  makeText("30", temperaturePosition[ScreenType]);
  mapchar(0x0e, temperaturePosition[ScreenType]+2);
}}

void ShowAmps(){
  if(confItem[GetSetting("S_AMPERAGE")].value() > 0) {
  makeText("15.2A", amperagePosition[ScreenType]-7);
  }}

void ShowUTC(){
  if(confItem[GetSetting("S_GPSTIME")].value() > 0) {
  makeText("10:27:07", UTCPosition[ScreenType]);
  }}

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
    makeText(str(heading), MwHeadingPosition[0]+2);
    break;
  case 2:
    makeText(str(heading), MwHeadingPosition[0]+1);
    break;
  case 3:
    makeText(str(heading), MwHeadingPosition[0]);
    break;
  case 4:
    makeText(str(heading), MwHeadingPosition[0]-1);
    break;
  }
  mapchar(MwHeadingUnitAdd,MwHeadingPosition[0]+3);  
  }}


void SimulateTimer(){
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
  
  //ShowOnTime(OnTimerString);

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
  else
  {
    FlyTimerString = "0:00";
  } 
   ShowFlyTime(FlyTimerString);
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
    makeText(" ARMED", motorArmedPosition[0]);
    armed = true;
  }
    else{
    makeText("DISARMED", motorArmedPosition[0]);
    armed = false;
  }
  }
  else{
    makeText("DISCONNECTED", motorArmedPosition[0]-2);
    armed = false;
  }
  if(confItem[GetSetting("S_MODEICON")].value() > 0) {
  if(confItem[GetSetting("S_MODESENSOR")].value() > 0) {
    
    if((SimModebits&mode_stable) >0)
      mapchar(0xa0,sensorPosition[0]);

    if((SimModebits&mode_horizon) >0)
      mapchar(0xa0,sensorPosition[0]);

    if((SimModebits&mode_baro) >0)
      mapchar(0xa2,sensorPosition[0]+1);

    if((SimModebits&mode_mag) >0)
      mapchar(0xa1,sensorPosition[0]+2);
  }

  if(confItem[GetSetting("S_GIMBAL")].value() > 0) {
    if((SimModebits&mode_camstab) >0){
      mapchar(0x16,gimbalPosition[0]);
      mapchar(0x17,gimbalPosition[0]+1);
    }}

    if((SimModebits&mode_gpshome) >0){
      mapchar(0x9d,statusPosition[0]);
      mapchar(0x9e,statusPosition[0]+1);
      mapchar(0x2d,statusPosition[0]+2);
//      mapchar(0x9e,statusPosition[0]+3);
      String output = str(int(SGPS_distanceToHome.getValue()));
      makeText(output,statusPosition[0]+3);
      mapchar(0x0c,statusPosition[0]+3+output.length());
    }
    else if((SimModebits&mode_gpshold) >0){
      mapchar(0xcd,statusPosition[0]);
      mapchar(0xce,statusPosition[0]+1);
    }
    else if((SimModebits&mode_stable) >0){
      mapchar(0xac,statusPosition[0]);
      mapchar(0xad,statusPosition[0]+1);
    }
    else if((SimModebits&mode_horizon) >0){
      mapchar(0xc4,statusPosition[0]);
      mapchar(0xc5,statusPosition[0]+1);
    }
    else{
      mapchar(0xae,statusPosition[0]);
      mapchar(0xaf,statusPosition[0]+1);
    }

}
}



void displayHorizon(int rollAngle, int pitchAngle)
{
  if(pitchAngle>250) pitchAngle=250;                //250
  if(pitchAngle<-200) pitchAngle=-200;
  if(rollAngle>400) rollAngle=400;
  if(rollAngle<-400) rollAngle=-400;

  for(int X=0; X<=8; X++) {
    int Y = (rollAngle * (4-X)) / 64;
    Y += pitchAngle / 8;
    Y += 41;
    if(Y >= 0 && Y <= 81) {
      int pos = 30*(2+Y/9) + 10 + X;
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
      int pos = 30*(2+Y/9) + 10 + X;
      if(X < 3 || X >5 || (Y/9) != 4 || confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() == 0)
        mapchar(0x80+(Y%9), pos);
      if(Y>=9 && (Y%9) == 0)
        mapchar(0x89, pos-30);
    }
    Y += 20;
    if(Y >= 0 && Y <= 81) {
      int pos = 30*(2+Y/9) + 10 + X;
      if(X < 3 || X >5 || (Y/9) != 4 || confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() == 0)
        mapchar(0x80+(Y%9), pos);
      if(Y>=9 && (Y%9) == 0)
        mapchar(0x89, pos-30);
    }
  }}


  if(confItem[GetSetting("S_DISPLAY_HORIZON_BR")].value() > 0) {
    //Draw center screen
    mapchar(0x7e, 224-30);
    mapchar(0x00, 224-30-1);
    mapchar(0xbc, 224-30+1);
  }
  
  //if (WITHDECORATION){
  if(confItem[GetSetting("S_WITHDECORATION")].value() > 0) {
    mapchar(0xC7,128);
    mapchar(0xC7,128+30);
    mapchar(0xC7,128+60);
    mapchar(0xC7,128+90);
    mapchar(0xC7,128+120);
    mapchar(0xC6,128+12);
    mapchar(0xC6,128+12+30);
    mapchar(0xC6,128+12+60);
    mapchar(0xC6,128+12+90);
    mapchar(0xC6,128+12+120);
    mapchar(0x02, 229-30);
    mapchar(0x03, 219-30);
  }
}

void ShowSideBArArrows(){
  if(confItem[GetSetting("S_SIDEBARTOPS")].value() > 0) {
    mapchar(0xCf,128+120+30);
    mapchar(0xCf,128+12+120+30);
  }}


void displayHeadingGraph()
{
  if(confItem[GetSetting("S_COMPASS")].value() > 0) {
  
  int xx;
  xx = MwHeading * 4;
  xx = xx + 720 + 45;
  xx = xx / 90;
 //for (int i = 0; i < 9; i++){
 
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+1);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+2);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+3);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+4);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+5);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+6);
  mapchar(headGraph[xx++],MwHeadingGraphPosition[0]+7);
  mapchar(headGraph[xx],MwHeadingGraphPosition[0]+8);  

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
    
    bit <<= 1L;
  }
  
   
 
}
