
void processSerialData(uint8_t data);
void processSportPacket(uint8_t *packet);
void initSPort();
void checkSPort();

extern uint16_t cells[6];
extern alt_t sport_alt;
extern uint8_t SPORT_PRESENT;

#define SPORT_SERIAL_BAUD 57600

#define START_STOP         0x7e
#define BYTESTUFF          0x7d
#define STUFF_MASK         0x20

#define DATA_FRAME         0x10

#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f

#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f

#define CELLS_FIRST_ID          0x0300
#define CELLS_LAST_ID           0x030f

#define FRSKY_RX_PACKET_SIZE 9

#define SPORT_TIMEOUT 1000000 // 1 sec

#define SPORT_DATA_S32(packet)  (*((int32_t *)(packet+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(packet+4)))


//following code is sport host related
//all sensor id's requested by x8r reciever
//0, 161, 34, 131, 228, 69, 198, 103, 72, 233, 106, 203, 172, 13, 142, 47, 208, 113, 242, 83, 52, 149, 22, 183, 152, 57, 186, 27

#define SPORT_HOST_INTERVAL   120000// 120000 //interval between sensor requests //x8r 12 millis
#define SPORT_SENSOR_ID 0xA1 //161 cell monitor //currently only coded for one option

