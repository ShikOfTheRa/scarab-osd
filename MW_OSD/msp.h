#ifndef __MSP_H
#define __MSP_H

// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_NAV_STATUS           121   //out message	     Returns navigation status

#define MSP_CELLS                130   //out message         FrSky SPort Telemtry

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_ALARMS               242   //in message          poll for alert text

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

// Cleanflight/Betaflight specific
#define MSP_PID_CONTROLLER       59    //in message          no param
#define MSP_SET_PID_CONTROLLER   60    //out message         sets a given pid controller

// Cleanflight specific
#define MSP_LOOP_TIME            73    //out message         Returns FC cycle time i.e looptime 
#define MSP_SET_LOOP_TIME        74    //in message          Sets FC cycle time i.e looptime parameter

// Baseflight specific
#define MSP_CONFIG               66    //out message         baseflight-specific settings that aren't covered elsewhere
#define MSP_SET_CONFIG           67    //in message          baseflight-specific settings save

// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1333
// ---------------------------------------------------------------------------------------

// Private MSP for use with the GUI
#define MSP_OSD                  220   //in message          starts epprom send to OSD GUI
// Subcommands
#define OSD_NULL                 0
#define OSD_READ_CMD             1
#define OSD_WRITE_CMD            2
#define OSD_GET_FONT             3
#define OSD_SERIAL_SPEED         4
#define OSD_RESET                5
#define OSD_DEFAULT              6
#define OSD_SENSORS              7
#define OSD_WRITE_CMD_EE         8
#define OSD_READ_CMD_EE          9
// End private MSP for use with the GUI

#define REQ_MSP_IDENT     (1 <<  0)
#define REQ_MSP_STATUS    (1 <<  1)
#define REQ_MSP_RAW_IMU   (1 <<  2)
#define REQ_MSP_RC        (1 <<  3)
#define REQ_MSP_ALTITUDE  (1 <<  4)
#define REQ_MSP_COMP_GPS  (1 <<  5)
#define REQ_MSP_ATTITUDE  (1 <<  6)
#define REQ_MSP_RAW_GPS   (1 <<  7)
#define REQ_MSP_ANALOG    (1 <<  8)
#define REQ_MSP_RC_TUNING (1 <<  9)
#define REQ_MSP_PID       (1 << 10)
#define REQ_MSP_BOX       (1 << 11)
#define REQ_MSP_FONT      (1 << 12)
#define REQ_MSP_DEBUG     (1 << 13)
#define REQ_MSP_CELLS     (1 << 14)
#define REQ_MSP_NAV_STATUS      32768  // (1 << 15)
#define REQ_MSP_CONFIG          32768  // (1 << 15)
#define REQ_MSP_MISC            65536  // (1 << 16)
#define REQ_MSP_ALARMS          131072 // (1 << 17)
#define REQ_MSP_PID_CONTROLLER  262144 // (1 << 18)
#define REQ_MSP_LOOP_TIME       524288 // (1 << 19) 

// TODO: $$$ abstract the serial bits
#if defined MAVLINK
  #define SERIALBUFFERSIZE 75
#elif defined NAZA
  #define SERIALBUFFERSIZE 75
#elif defined GPSOSD
  #define SERIALBUFFERSIZE 100
#else
  #define SERIALBUFFERSIZE 150
#endif


class MSPClass {
  public:
    MSPClass(HardwareSerial &serial);

    void Receive(uint8_t loops);

    void BuildRequests(void);
    void SendRequests(void);
    void SetRequests(void);

    // writes a request to the serial buffer
    void WriteRequest(uint8_t mspCommand, uint8_t txDataSize);

    void ConfigExit(void);
  private:
    uint32_t read32();
    uint16_t read16();
    uint8_t read8();

    void write8(uint8_t t);
    void write16(uint16_t t);
    void writeChecksum(void);

    void serialMSPCheck();
    void handleRawRC(void);
    void serialMenuCommon(void);
    void settingsSerialRequest(void);
    void settingswriteSerialRequest(void);
    void setFCProfile(void);

    void configExit(void);
    void configSave();

    HardwareSerial* _serial;

    uint8_t _MSPcmdsend=0;
    uint32_t _modeMSPRequests;
    uint32_t _queuedMSPRequests;

    uint8_t _serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
    uint8_t _receiverIndex;
    uint8_t _dataSize;
    uint8_t _cmdMSP;
    uint8_t _rcvChecksum;
    uint8_t _readIndex;
    int8_t _menudir;
    uint8_t _txChecksum;
   
    uint16_t _eeaddress = 0;
    uint8_t _eedata = 0;
    uint8_t _settingsMode = 0;

};

extern MSPClass MSP;

#endif
