
void process_ltm_send(){
//  syncmav();
  if ((int(SimControlToggle.getValue())!=0)&&(Simtype==3)) {
//    if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_HEARTBEAT);
    LTM_sendOrder++;
    switch(LTM_sendOrder) {
      case 1:
//        if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_ATTITUDE);
        break;
      default:  
        LTM_sendOrder=0;
    }
    PortWrite = !PortWrite; // toggle TX LED every other    
  } 
}


