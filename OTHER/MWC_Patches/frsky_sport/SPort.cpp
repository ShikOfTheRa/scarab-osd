#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "SPort.h"

/*
SPort code from https://code.google.com/p/opentx/source/browse/trunk/src/telemetry/frsky_sport.cpp (Many Thanks)
Adapted for MultiiWii by haydent www.httech.com.au
*/

uint8_t frskyRxBuffer[FRSKY_RX_PACKET_SIZE];   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)

uint8_t SPORT_PRESENT = 0;

uint32_t lastPacket = 0;
uint16_t cells[6];
alt_t sport_alt;

void initSPort(void) {
  
    LEDPIN_ON
    SerialEnd(SPORT_SERIAL);
    SerialOpen(SPORT_SERIAL, SPORT_SERIAL_BAUD); 
    delay(500);
    
    for(uint8_t j=1;j<=100;j++){//check 100 times if sport active, takes about 70 but extra is not noticed     
      checkSPort();      
      delayMicroseconds(SPORT_HOST_INTERVAL);     
    }
    
    if(!SPORT_PRESENT){//relase comport if sport device not detected
             SerialEnd(SPORT_SERIAL);
             SerialOpen(SPORT_SERIAL,SERIAL0_COM_SPEED);
    }    
    LEDPIN_OFF
}

void checkSPort(void) {

  #ifdef SPORT_HOST
      static uint32_t lastRequest = 0;

      if((currentTime-lastRequest) > SPORT_HOST_INTERVAL || !SPORT_PRESENT){
          
          SerialWrite(SPORT_SERIAL, START_STOP);//request header
          SerialWrite(SPORT_SERIAL, SPORT_SENSOR_ID);//sensor id
          lastRequest = currentTime;          
      }
  #endif
  
    while (SerialAvailable(SPORT_SERIAL)) {
        uint8_t data = SerialRead(SPORT_SERIAL);
        processSerialData(data); 
     }
    
    if((currentTime-lastPacket) > SPORT_TIMEOUT && lastPacket){
      lastPacket=0;
      memset(cells, 0, sizeof(cells));
      analog.vbat = 0;
      sport_alt.vario = 0;
      sport_alt.EstAlt = 0;
      analog.amperage = 0;
    }

}
  
// Receive buffer state machine state enum
enum FrSkyDataState {
  STATE_DATA_IDLE,
  STATE_DATA_IN_FRAME,
  STATE_DATA_XOR,
};
  
  
void processSerialData(uint8_t data)
{
  static uint8_t numPktBytes = 0;
  static uint8_t dataState = STATE_DATA_IDLE;

  if (data == START_STOP) {
    dataState = STATE_DATA_IN_FRAME;
    numPktBytes = 0;   
  }
  else {
    switch (dataState) {
      case STATE_DATA_XOR:
        frskyRxBuffer[numPktBytes++] = data ^ STUFF_MASK;
        dataState = STATE_DATA_IN_FRAME;
        break;

      case STATE_DATA_IN_FRAME:
        if (data == BYTESTUFF)
          dataState = STATE_DATA_XOR;// XOR next byte
        else
          frskyRxBuffer[numPktBytes++] = data;
        break;
    }
  }

  if (numPktBytes == FRSKY_RX_PACKET_SIZE) {
    processSportPacket(frskyRxBuffer);
    dataState = STATE_DATA_IDLE;
  }
}


bool checkSportPacket(uint8_t *packet)
{
  short crc = 0;
  for (int i=1; i<FRSKY_RX_PACKET_SIZE; i++) {
    crc += packet[i]; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
    crc += crc >> 8; //0-0FF
    crc &= 0x00ff;
  }
  return (crc == 0x00ff);
}

void processSportPacket(uint8_t *packet)
{
  
  //uint8_t  dataId = packet[0];
  uint8_t  prim   = packet[1];
  uint16_t appId  = *((uint16_t *)(packet+2));

  if (!checkSportPacket(packet))
    return;
    
  SPORT_PRESENT = 1;
  lastPacket = currentTime;//maybe
  
  switch (prim)
  {
    case DATA_FRAME:

      if (appId >= ALT_FIRST_ID && appId <= ALT_LAST_ID) {   
        
        #ifdef SPORT_VARIO   
            static int32_t baroAltitudeOffset;
            int32_t baroAltitude = SPORT_DATA_S32(packet);
            if (!baroAltitudeOffset)baroAltitudeOffset = -baroAltitude;
            baroAltitude += baroAltitudeOffset;
            sport_alt.EstAlt = baroAltitude;
        #endif      

      }
      else if (appId >= VARIO_FIRST_ID && appId <= VARIO_LAST_ID) {
        
         #ifdef SPORT_VARIO   
             int16_t varioSpeed = SPORT_DATA_S32(packet); 
             sport_alt.vario = varioSpeed;
            
            // debug[0] = varioSpeed;
         #endif
         
      }      
      else if (appId >= CURR_FIRST_ID && appId <= CURR_LAST_ID) {
        
        #ifdef SPORT_FCS 
             #if !defined(POWERMETER)
                 static uint32_t lastRead = currentTime;
                 static uint32_t currentConsumption;
                 uint16_t current = SPORT_DATA_U32(packet);//10ths amp, 10 = 1 amp
                 //current = 10;//simulate 1 amp
                 current *= 100;// convert 10ths amp to milliamps
                                           
                 currentConsumption += ((currentTime-lastRead) * (uint32_t)(current)) / 100000;
;
                 lastRead = currentTime;    
                 
                 analog.amperage = (uint16_t) current;
                 analog.intPowerMeterSum = (uint16_t) (currentConsumption / 36000);
               
                 //debug[0] = current;
             #endif
      
         #endif    
         
      }
      else if (appId >= VFAS_FIRST_ID && appId <= VFAS_LAST_ID) {
        
        #ifdef SPORT_FCS             
             #if !defined(VBAT) && !defined(SPORT_FLVSS)
                 uint16_t vfas = SPORT_DATA_U32(packet);
                 analog.vbat = (int16_t) (vfas / 10);           
             #endif
         #endif   
         
      }    
      else if (appId >= CELLS_FIRST_ID && appId <= CELLS_LAST_ID) {
        
        #ifdef SPORT_FLVSS
        
            uint32_t cell_data = SPORT_DATA_U32(packet);
            uint8_t battnumber = cell_data & 0xF;         
         
           if(battnumber < 6){//currently only support upto 6 cells
              cells[battnumber] = ((cell_data & 0x000FFF00) >> 8) *2 /10;
              cells[battnumber+1] = ((cell_data & 0xFFF00000) >> 20) *2 /10;   
           }       
          
         #ifndef VBAT
             int16_t sum = 0;
             for (int i=0; i < 6; i++){		
                      sum += cells[i];               
                      //debug[i] = cells[i];
             }             
             analog.vbat = (int16_t) (sum / 10);
         #endif
             
       #endif
         
       }   
       
      break;

  }
}

