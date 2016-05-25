final int
  KISSFRAMEINIT=5,
  KISSFRAMELENGTH=154,
  KISSCHECKSUM=99
;

void serialize8kiss(int val) {
  try{
    g_serial.write(val);
  } 
  catch(java.lang.Throwable t) {
    System.out.println( t.getClass().getName() );
    t.printStackTrace();
  }
}    

void synckiss(){

  for (int i=0; i<(KISSFRAMELENGTH); i++) { 
    kisstable[i]=0;
  }
  
// armed 
  if(toggleModeItems[0].getValue()> 0){ //armed
    kisstable[16]=1;
  }

// flight mode
  if(toggleModeItems[1].getValue()> 0){//stab
    kisstable[65]=1;
  }
  else if(toggleModeItems[6].getValue()> 0){//RTH
    kisstable[65]=2;
  }
  else if(toggleModeItems[7].getValue()> 0){//HOLD
    kisstable[65]=3;
  }
  else{
    kisstable[65]=0;
  }  

// bat
  kisstable[17]=int(sVBat * 100)>>8;
  kisstable[18]=int(sVBat * 100);

// throttle
  for (int i=0; i<(8); i++) { 
    kisstable[i*2]=0;
    kisstable[1+(i*2)]=0;
  }
  kisstable[0]=int(map(Throttle_Yaw.arrayValue()[1],1000,2000,-500,500))>>8;
  kisstable[1]=int(map(Throttle_Yaw.arrayValue()[1],1000,2000,-500,500));    

  kisstable[2]=int(map(Pitch_Roll.arrayValue()[0],1000,2000,-500,500))>>8;
  kisstable[3]=int(map(Pitch_Roll.arrayValue()[0],1000,2000,-500,500));

  kisstable[4]=int(map(Pitch_Roll.arrayValue()[1],1000,2000,-500,500))>>8;
  kisstable[5]=int(map(Pitch_Roll.arrayValue()[1],1000,2000,-500,500));

  kisstable[6]=int(map(Throttle_Yaw.arrayValue()[0],1000,2000,-500,500))>>8;
  kisstable[7]=int(map(Throttle_Yaw.arrayValue()[0],1000,2000,-500,500));    


// pitch roll
  kisstable[31]=int(MW_Pitch_Roll.arrayValue()[0])>>8;
  kisstable[32]=int(MW_Pitch_Roll.arrayValue()[0]);
  kisstable[33]=int(MW_Pitch_Roll.arrayValue()[1])>>8;
  kisstable[34]=int(MW_Pitch_Roll.arrayValue()[1]);

// Amperage
  kisstable[148]=millis()>>17;
  kisstable[149]=millis()>>9;
}

void process_kiss_send(){
  synckiss();
  if ((int(SimControlToggle.getValue())!=0)&&(Simtype==4)) {
    g_serial.write(KISSFRAMEINIT);
    g_serial.write(KISSFRAMELENGTH);
    for (int i=0; i<(KISSFRAMELENGTH); i++) { 
      g_serial.write((kisstable[i]&0xFF));
//      println(i+":"+(kisstable[i]&0xFF));
    }
    g_serial.write(KISSCHECKSUM);

  }
  PortWrite = !PortWrite; // toggle TX LED every other    
} 

