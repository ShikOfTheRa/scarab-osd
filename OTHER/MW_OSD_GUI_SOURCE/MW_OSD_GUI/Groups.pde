  Numberbox SHWCS_voltage;
  Numberbox SHWCS_sensitivity;
  Numberbox SHWCS_offset;

void SetupGroups(){


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
                .setLabel("       SETTINGS")
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
                .setLabel("            DISK")
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
                .hide()
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
                .setBackgroundHeight((6*17) +5)
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

 G_CurSensor = GroupcontrolP5.addGroup("G_CurSensor")
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
                .setLabel("HW Current sensor")
                .hide()
                .disableCollapse() 
                ; 
                G_CurSensor.captionLabel()
                .toUpperCase(false)
                .align(controlP5.CENTER,controlP5.CENTER)
                ; 

  SHWCS_voltage = ScontrolP5.addNumberbox("SHWCS_voltage",0,5,0*17,40,14);
    SHWCS_voltage.setLabel("Supply Voltage");
    SHWCS_voltage.setMultiplier(0.1);
    SHWCS_voltage.setMin(4.5);
//    SHWCS_voltage.setDirection(Controller.HORIZONTAL);
    SHWCS_voltage.setMax(5.5);
    SHWCS_voltage.setDecimalPrecision(1);
    SHWCS_voltage.setGroup(G_CurSensor); 
    SHWCS_voltage.setValue(5.0);
  ScontrolP5.getController("SHWCS_voltage").getCaptionLabel()
   .toUpperCase(false)
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);

  SHWCS_offset = ScontrolP5.addNumberbox("SHWCS_offset",0,5,1*17,40,14);
    SHWCS_offset.setLabel("0A offset V");
    SHWCS_offset.setMin(0);
    SHWCS_offset.setMultiplier(0.01);
//    SHWCS_offset.setDirection(Controller.HORIZONTAL);
    SHWCS_offset.setMax(5.00);
    SHWCS_offset.setDecimalPrecision(2);
    SHWCS_offset.setGroup(G_CurSensor); 
    SHWCS_offset.setValue(2.50);
  ScontrolP5.getController("SHWCS_offset").getCaptionLabel()
   .toUpperCase(false)
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);

  SHWCS_sensitivity = ScontrolP5.addNumberbox("SHWCS_sensitivity",0,5,2*17,40,14);
    SHWCS_sensitivity.setLabel("Sensitivity mV/A");
    SHWCS_sensitivity.setMin(0);
    SHWCS_sensitivity.setMultiplier(0.1);
//    SHWCS_sensitivity.setDirection(Controller.HORIZONTAL);
    SHWCS_sensitivity.setMax(250);
    SHWCS_sensitivity.setDecimalPrecision(1);
    SHWCS_sensitivity.setGroup(G_CurSensor); 
    SHWCS_sensitivity.setValue(36.6);
  ScontrolP5.getController("SHWCS_sensitivity").getCaptionLabel()
   .toUpperCase(false)
   .align(ControlP5.LEFT, ControlP5.RIGHT_OUTSIDE).setPaddingX(45);

}
