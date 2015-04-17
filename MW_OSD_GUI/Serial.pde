
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
void InitSerial(float portValue) {
  if (portValue < commListMax) {
    if(init_com == 0){ 
      try{
      String portPos = Serial.list()[int(portValue)];
      txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
      g_serial = new Serial(this, portPos, BaudRate);
      LastPort = portValue;
      init_com=1;
      toggleMSP_Data = true;
      ClosePort = false;
      buttonREAD.setColorBackground(green_);
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

void ClosePort(){
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




void SetConfigItem(int index, int value) {
  if(index >= CONFIGITEMS)
    return;

  if(index == GetSetting("S_VOLTAGEMIN"))confItem[index].setValue(float(value)/10);//preserve decimal
  else if(index == GetSetting("S_GPSTZ"))confItem[index].setValue(float(value)/10);//preserve decimal, maybe can go elsewhere - haydent
  else confItem[index].setValue(value);
  if (index == CONFIGITEMS-1)
    buttonWRITE.setColorBackground(green_);
    
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

void PORTCLOSE(){
  toggleMSP_Data = false;
  CloseMode = 0;
  InitSerial(200.00);
}

void BounceSerial(){
  toggleMSP_Data = false;
  CloseMode = 1;
  InitSerial(200.00);
  
}  

void RESTART(){
  toggleMSP_Data = true;
  for (int txTimes = 0; txTimes<3; txTimes++) {
    headSerialReply(MSP_OSD, 1);
    serialize8(OSD_RESET);
    tailSerialReply();
    delay(100);
  }
  toggleMSP_Data = false;
  READconfigMSP_init();
//  READinit();
//  eeaddressGUI=0;   
//  ReadConfigMSPMillis=20000+millis();  
}  



public void WRITEinit(){
  WriteConfig=20;
  ReadConfig=1;
  SimControlToggle.setValue(0);
  readerror=1;  
  CheckCallSign();
  S16_AMPMAX = int(confItem[GetSetting("S_AMPDIVIDERRATIO")].value());
  confItem[GetSetting("S_AMPMAXL")].setValue(int(confItem[GetSetting("S_AMPDIVIDERRATIO")].value())&0xFF); // for 8>>16 bit EEPROM
  confItem[GetSetting("S_AMPMAXH")].setValue(int(confItem[GetSetting("S_AMPDIVIDERRATIO")].value())>>8);
  for(int i = 0; i < CONFIGITEMS; i++){
    readcheck[i]=int(confItem[i].value());
  }
  readcheck[GetSetting("S_AMPDIVIDERRATIO")]=0;
}


  public void WRITEconfig(){
  readerror=1;  
  SimControlToggle.setValue(0);

  CheckCallSign();
  PortWrite = true;
  MakePorts();
  toggleMSP_Data = true;
  p = 0;
  inBuf[0] = OSD_WRITE_CMD;
  g_serial.clear();
  for(int ii = 1; ii < CONFIGITEMS; ii++){
    if (ii != GetSetting("S_AMPDIVIDERRATIO"));
      SetConfigItem(ii,readcheck[ii]);
  }
  for (int txTimes = 0; txTimes<1; txTimes++) {
    headSerialReply(MSP_OSD, CONFIGITEMS + (hudoptions*2*2) +1);
    serialize8(OSD_WRITE_CMD);
    for(int i = 0; i < CONFIGITEMS; i++){
      if(i == GetSetting("S_VOLTAGEMIN")){
        serialize8(int(confItem[i].value()*10));//preserve decimal
      }
      else if(i == GetSetting("S_GPSTZ")){
        serialize8(int(confItem[i].value()*10));//preserve decimal, maybe can go elsewhere - haydent
      }
      else if(i == GetSetting("S_AMPDIVIDERRATIO")){
        serialize8(0);
      }
      else{
        serialize8(int(confItem[i].value()));
      }
    }

     int clayout=int(confItem[GetSetting("S_HUD")].value()); 
     for(int i = 0; i < (hudoptions); i++){
       serialize8(int(ConfigLayout[0][i]&0xFF));
       serialize8(int(ConfigLayout[0][i]>>8));
     }
     for(int i = 0; i < (hudoptions); i++){
       serialize8(int(ConfigLayout[1][i]&0xFF));
       serialize8(int(ConfigLayout[1][i]>>8));
     }

    tailSerialReply();
  }
  
  toggleMSP_Data = false;
  g_serial.clear();
  PortWrite = false;
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
      serialize8(int(raw_font[FontCounter][i]));
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
        ReadConfig=100;
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




void SendCommand(int cmd){
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
        serialize16(int(Pitch_Roll.arrayValue()[0]));
        serialize16(int(Pitch_Roll.arrayValue()[1]));
        serialize16(int(Throttle_Yaw.arrayValue()[0]));
        serialize16(int(Throttle_Yaw.arrayValue()[1]));
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
       if (int(confItem[GetSetting("S_BATCELLS")].value()) <1) break;
        int cellvolt= (int(sVBat * 100))/int(confItem[GetSetting("S_BATCELLS")].value());
        for (int i=0; i<6; i++) {
          if (i < int(confItem[GetSetting("S_BATCELLS")].value())) {
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
        serialize16(int(MW_Pitch_Roll.arrayValue()[0])*10);
        serialize16(int(MW_Pitch_Roll.arrayValue()[1])*10);
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
           int pidval=int(i*10)+ii;
            serialize8(pidval);
          }
        }

        tailSerialReply();
        PortIsWriting = false;
      break;

      case MSP_ANALOG:
        PortIsWriting = true;
        headSerialReply(MSP_ANALOG, 5);
        serialize8(int(sVBat * 10));
        serialize16(0);
        serialize16(int(sMRSSI));
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
        serialize8(int(SGPS_FIX.arrayValue()[0]));
        serialize8(int(SGPS_numSat.value()));
        serialize32(430948610);
        serialize32(-718897060);
        serialize16(int(SGPS_altitude.value()/100));
        serialize16(int(SGPS_speed.value()));
        serialize16(MwHeading*10);     
    break;
    
  
    case MSP_COMP_GPS:
      if(confItem[GetSetting("S_GPSTIME")].value()>0)
        headSerialReply(MSP_COMP_GPS,9);
      else
        headSerialReply(MSP_COMP_GPS,5);
      serialize16(int(SGPS_distanceToHome.value()));
      int GPSheading = int(SGPSHeadHome.value());
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
      serialize32(int(sAltitude) *100);
      serialize16(int(sVario) *10);     
    break;
      
    }
    tailSerialReply();   
  
}

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
String shortifyPortName(String portName, int maxlen)  {
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
int read32() {return (inBuf[p++]&0xff) + ((inBuf[p++]&0xff)<<8) + ((inBuf[p++]&0xff)<<16) + ((inBuf[p++]&0xff)<<24); }
int read16() {return (inBuf[p++]&0xff) + ((inBuf[p++])<<8); }
int read8()  {return inBuf[p++]&0xff;}



int outChecksum;


void serialize8(int val) {
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

void serialize16(int a) {
  if (str(a)!=null ){
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
  }
}

void serialize32(int a) {
  if (str(a)!=null ){
    serialize8((a    ) & 0xFF);
    serialize8((a>> 8) & 0xFF);
    serialize8((a>>16) & 0xFF);
    serialize8((a>>24) & 0xFF);
  }
 
}

void serializeNames(int s) {
  //for (PGM_P c = s; pgm_read_byte(c); c++) {
   // serialize8(pgm_read_byte(c));
  //}
  for (int c = 0; c < strBoxNames.length(); c++) {
    serialize8(strBoxNames.charAt(c));
  }
}

void headSerialResponse(int requestMSP, Boolean err, int s) {
  //if (FontMode)DelayTimer(1);
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  outChecksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(requestMSP);
}

void headSerialReply(int requestMSP, int s) {
  if ((str(requestMSP) !=null) && (str(s)!=null)){
    headSerialResponse(requestMSP, false, s);
  }
}

//void headSerialError(int requestMSP, int s) {
// headSerialResponse(requestMSP, true, s);
//}

void tailSerialReply() {
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
  int icmd = int(cmd&0xFF);
  if (icmd !=MSP_OSD)return;  //System.out.println("Not Valid Command");
 
  //System.out.println("evaluateCommand");

    time2=time;
    //int[] requests = {MSP_STATUS, MSP_RAW_IMU, MSP_SERVO, MSP_MOTOR, MSP_RC, MSP_RAW_GPS, MSP_COMP_GPS, MSP_ALTITUDE, MSP_BAT, MSP_DEBUGMSG, MSP_DEBUG};
    switch(icmd) {
    
      case MSP_OSD:
        int cmd_internal = read8();
        PortRead = true;
        MakePorts();

        if(cmd_internal == OSD_NULL) {
        }

        if(cmd_internal == OSD_READ_CMD_EE) { // response to a read / write request
//            System.out.print(" "+size+" ");
          if(size == 2) { // confirmed write request received
            eeaddressOSD=read8();
            eeaddressOSD=eeaddressOSD+read8();
            if (eeaddressOSD>=eeaddressGUI){ // update base address
              eeaddressGUI=eeaddressOSD;
            }
            if (eeaddressGUI>(CONFIGITEMS + (hudoptions*2*2))){ // hit end address
//            if (eeaddressGUI>=(CONFIGITEMS)){ // hit end address config only
              WriteConfigMSPMillis=0;
            }
          }
          else{ // confirmed write request received
          for(int i=0; i<10; i++) {
            eeaddressOSD=read8();
            eeaddressOSD=eeaddressOSD+read8();
            eedataOSD=read8();
            if (eeaddressOSD<CONFIGITEMS){
              SetConfigItem(eeaddressOSD, eedataOSD);
            }
            if (eeaddressOSD==GetSetting("S_AMPMAXH")){ // 16 bit value amps manipulation now received all data
              S16_AMPMAX=(int(confItem[GetSetting("S_AMPMAXH")].value())<<8)+ int(confItem[GetSetting("S_AMPMAXL")].value()); 
              SetConfigItem(GetSetting("S_AMPDIVIDERRATIO"), (int) S16_AMPMAX);
            }
            if (eeaddressOSD==eeaddressGUI){ // update base address
              eeaddressGUI++;
            }
          }
//            if (eeaddressGUI>(CONFIGITEMS + (hudoptions*2*2))){ // hit end address
            if (eeaddressGUI>=(CONFIGITEMS)){ // hit end address config only
              ReadConfigMSPMillis=0;
            }
          }
        }

        if(cmd_internal == OSD_READ_CMD) {
          if(size == 1) {
          }
          else {
//            debug[1]++;
            confCheck=0;
            readerror=0;
            readcounter++;
            for(int i = 0; i < CONFIGITEMS; i++){
              int xx = read8();
              if (i==0){
                confCheck=xx;
              }
              if (i==GetSetting("S_AMPDIVIDERRATIO")){
                xx=0;
              }

              if (i>0){
                if (WriteConfig>0){
                  if ((xx!=readcheck[i])){
                    readerror=1;
                  }
                }
              }
              if (confCheck>0){
                SetConfigItem(i, xx);
              }
            }

            if (readerror==0){
              WriteConfig=0;
              ReadConfig=0;
            }
          
 if (MW_OSD_EEPROM_Version!=confCheck){
   noLoop();
   JOptionPane.showConfirmDialog(null,"GUI version does not match OSD version - a different version is required.", "Version Mismatch Warning", JOptionPane.PLAIN_MESSAGE,JOptionPane.WARNING_MESSAGE);
   loop();      
 }
 
              S16_AMPMAX=(int(confItem[GetSetting("S_AMPMAXH")].value())<<8)+ int(confItem[GetSetting("S_AMPMAXL")].value()); // for 8>>16 bit EEPROM
              SetConfigItem(GetSetting("S_AMPDIVIDERRATIO"), (int) S16_AMPMAX);

            // Send a NULL reply
            //headSerialReply(MSP_OSD, 1);
            //serialize8(OSD_NULL);
            if (FontMode == false){
              toggleMSP_Data = false;
              g_serial.clear();
             }
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

void MWData_Com() {
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
//    println("Display in text area");
    SimControlToggle.setValue(0);
    ReadConfigMSPMillis=1000+millis(); 
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
  }
//  toggleMSP_Data = false; //???????????????????
  ReadConfigMSPMillis=1000+millis(); 
}

  public void WRITEconfigMSP_init(){
    eeaddressGUI=0;  
    CheckCallSign(); 
    EElookuptableReSet();
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
//      System.out.println(eeaddressGUI+i+":"+EElookuptable[i+eeaddressGUI]);
    }
    tailSerialReply();
  }

//  toggleMSP_Data = false; //???????????????????
  WriteConfigMSPMillis=1000+millis(); 
}

  public void EElookuptableReSet(){ // preparing for a write
    confItem[GetSetting("S_AMPMAXL")].setValue(int(confItem[GetSetting("S_AMPDIVIDERRATIO")].value())&0xFF); // for 8>>16 bit EEPROM
    confItem[GetSetting("S_AMPMAXH")].setValue(int(confItem[GetSetting("S_AMPDIVIDERRATIO")].value())>>8);
    for(int i = 0; i < CONFIGITEMS; i++){
      if(i == GetSetting("S_VOLTAGEMIN")){
        EElookuptable[i]=int(confItem[i].value()*10);
      }
      else if(i == GetSetting("S_GPSTZ")){
        EElookuptable[i]=int(confItem[i].value()*10);
      }
      else if(i == GetSetting("S_AMPDIVIDERRATIO")){
        EElookuptable[i]=0;
      }
      else{
        EElookuptable[i]=int(confItem[i].value());
      }

    }
//     for(int i = 0; i < (hudoptions); i++){
//       ConfigLayout[0][i]=CONFIGHUD[int(confItem[GetSetting("S_HUD")].value())][i];
//       ConfigLayout[1][i]=CONFIGHUD[int(confItem[GetSetting("S_HUDOSDSW")].value())][i];
//     }

     int EElookuptableaddress=CONFIGITEMS;
     for(int i = 0; i < (hudoptions); i++){
       EElookuptable[EElookuptableaddress]=int(ConfigLayout[0][i]&0xFF);
       EElookuptableaddress++;
       EElookuptable[EElookuptableaddress]=int(ConfigLayout[0][i]>>8);
       EElookuptableaddress++;
     }
     for(int i = 0; i < (hudoptions); i++){
       EElookuptable[EElookuptableaddress]=int(ConfigLayout[1][i]&0xFF);
       EElookuptableaddress++;
       EElookuptable[EElookuptableaddress]=int(ConfigLayout[1][i]>>8);
       EElookuptableaddress++;
     }
  }
  
  public void EElookuptableSync(){ // Sync settings with EE table
  }
