// NXP SC16IS7{4,5,6}0

#define     IS7x0_REG_RHR        0x00   // R   00
#define     IS7x0_REG_THR        0X00   // W   00
#define     IS7x0_REG_IER        0X01   // R/W 08
#define     IS7x0_REG_FCR        0X02   // W   10
#define     IS7x0_REG_IIR        0X02   // R   10
#define     IS7x0_REG_LCR        0X03   // R/W 18
#define     IS7x0_REG_MCR        0X04   // R/W 20
#define     IS7x0_REG_LSR        0X05   // R   28
#define     IS7x0_REG_MSR        0X06   // R   30
#define     IS7x0_REG_SPR        0X07   // R/W 38
#define     IS7x0_REG_TCR        0X06   // R/W 30
#define     IS7x0_REG_TLR        0X07   // R/W 38
#define     IS7x0_REG_TXLVL      0X08   // R   40
#define     IS7x0_REG_RXLVL      0X09   // R   48
#define     IS7x0_REG_IODIR      0X0A   // R/W 50
#define     IS7x0_REG_IOSTATE    0X0B   // R/W 58
#define     IS7x0_REG_IOINTENA   0X0C   // R/W 60
#define     IS7x0_REG_IOCONTROL  0X0E   // R/W 70
#define     IS7x0_REG_EFCR       0X0F   // R/W 78

// Divisor registers
// Can only be accessed when LCR[7] (IS7x0_LCR_DIVLATEN) = 1 and LCR != 0xbf
#define     IS7x0_REG_DLL        0x00   // R/W 00
#define     IS7x0_REG_DLH        0X01   // R/W 08

// Enhanced register set
// Can only be accessed when LCR == 0xbf

#define     IS7x0_REG_EFR        0X02   // R/W
#define     IS7x0_REG_XON1       0X04   // R/W
#define     IS7x0_REG_XON2       0X05   // R/W
#define     IS7x0_REG_XOFF1      0X06   // R/W
#define     IS7x0_REG_XOFF2      0X07   // R/W

// Bits in IER
#define     IS7x0_IER_CTS        0X80
#define     IS7x0_IER_RTS        0X40
#define     IS7x0_IER_XOFF       0X20
#define     IS7x0_IER_SLEEP      0X10
#define     IS7x0_IER_MODEM      0X08
#define     IS7x0_IER_LINE       0X04
#define     IS7x0_IER_THR        0X02
#define     IS7x0_IER_RHR        0X01

// Bits in FCR
#define     IS7x0_FCR_RXTRG_SFT  6
#define     IS7x0_FCR_TXTRG_SFT  4
#define     IS7x0_FCR_TXFIFO_RST 0x04
#define     IS7x0_FCR_RXFIFO_RST 0x02
#define     IS7x0_FCR_FIFO_EN    0x01

// Bits in IIR
#define     IS7x0_IIR_FIFOEN7    0x80
#define     IS7x0_IIR_FIFOEN6    0x40
#define     IS7x0_IIR_INTMSK     0x3E
#define       IS7x0_IIR_LINESTAT   0x06
#define       IS7x0_IIR_RXTIMO     0x0C
#define       IS7x0_IIR_RHR        0x04
#define       IS7x0_IIR_THR        0x02
#define       IS7x0_IIR_MODEMSTAT  0x00
#define       IS7x0_IIR_IOPINS     0x30
#define       IS7x0_IIR_XOFF       0x10
#define       IS7x0_IIR_CTSRTS     0x20
#define     IS7x0_IIR_INTSTAT    0x01

// Bits in LCR
#define     IS7x0_LCR_DIVLATEN   0x80
#define     IS7x0_LCR_SETBRK     0x40
#define     IS7x0_LCR_PARMSK     0x38
#define       IS7x0_LCR_PARNONE    0x00
#define       IS7x0_LCR_PARODD     0x08
#define       IS7x0_LCR_PAREVEN    0x18
#define       IS7x0_LCR_PARONE     0x28
#define       IS7x0_LCR_PARZERO    0x38
#define     IS7x0_LCR_STOP2      0x04
#define     IS7x0_LCR_WLENMSK    0x03
#define       IS7x0_LCR_WLEN5      0x00
#define       IS7x0_LCR_WLEN6      0x01
#define       IS7x0_LCR_WLEN7      0x02
#define       IS7x0_LCR_WLEN8      0x03

// Bits in MCR
#define     IS7x0_MCR_CLKDIV     0x80
#define     IS7x0_MCR_IRDAEN     0x40
#define     IS7x0_MCR_XONANY     0x20
#define     IS7x0_MCR_LOOPBACK   0x10
#define     IS7x0_MCR_TCRTLR_EN  0x04
#define     IS7x0_MCR_RTS        0x02
#define     IS7x0_MCR_DTR        0x01

// Bits in LSR
#define     IS7x0_LSR_FIFOERR    0x80   // At least one char with error
#define     IS7x0_LSR_TXEMPTY    0x40   // THR and TSR empty
#define     IS7x0_LSR_THREMPTY   0x20   // THR empty
#define     IS7x0_LSR_BRKINT     0x10   // Break interrupt
#define     IS7x0_LSR_FERR       0x08   // Framing Error
#define     IS7x0_LSR_PERR       0x04   // Parity Error
#define     IS7x0_LSR_OERR       0x02   // Overrun Error
#define     IS7x0_LSR_DATAINRCVR 0x01	// At least one char in RX FIFO

// Bits in TLR
#define     IS7x0_TLR_RX_SFT     4
#define     IS7x0_TLR_TX_SFT     0

// Bits in IOCONTROL
#define     IS7x0_IOC_RESET      0x08

// Bits in EFR
#define     IS7x0_EFR_ENH        0x10

// Bits in EFCR
#define     IS7x0_EFCR_IRDA_FAST 0x80
#define     IS7x0_EFCR_RTSINVER  0x20
#define     IS7x0_EFCR_RTSCON    0x10
#define     IS7x0_EFCR_TXDISABLE 0x04
#define     IS7x0_EFCR_RXDISABLE 0x02
#define     IS7x0_EFCR_9BIT      0x01
