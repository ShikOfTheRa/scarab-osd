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
 G_Debug = GroupcontrolP5.addGroup("G_Debug")
                .setPosition(XDebug,YDebug+15)
                .setWidth(110)
                .setBarHeight(15)
                .setColorForeground(color(30,255))
                .setColorBackground(color(30,255))
                .setColorLabel(color(0, 110, 220))
                .setBarHeight(15)
                .setBackgroundColor(color(30,255))
                .setColorActive(red_)
                .setBackgroundHeight((1*17) +5)
                .setLabel("Debug")
                //.setGroup(SG)
                .disableCollapse() 
                ; 
                G_Debug.captionLabel()
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
                .setBackgroundHeight((7*17) +5)
                .setLabel("HUD")
                //.setGroup(G_LINKS)
                .disableCollapse() 
                ; 
                G_HUD.captionLabel()
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

}
