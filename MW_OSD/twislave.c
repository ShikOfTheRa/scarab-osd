/*
  twislave.c - TWI/I2C slave optimized library for I2C-UART Bridge
      Heavily modified from ...

  twi.c - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified 2016 by jflyper
*/

#include "Config.h" // Look for I2C_UB_SUPPORT
#ifdef I2C_UB_SUPPORT

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <compat/twi.h>
#include "Arduino.h" // for digitalWrite
#include "sc16is7x0.h"
#include "ubreg.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "pins_arduino.h"
#include "twislave.h"

#define USE_QLEN
#define TX_PRELOAD
#define RX_PREPTR

// For debugging
//#define DEBUG

#ifdef DEBUG
#define DebugPin1 3
#define DebugPin2 4
#define DebugPin3 5
//#define DebugPin4
#define digitalDebug(pin, state) digitalWrite(pin, state)
#else
#define digitalDebug(pin, state)
#endif

static void (*twis_onRegisterWrite)(uint8_t, uint8_t) = NULL;

static uint8_t twis_txBuffer[TWI_TX_BUFFER_LENGTH];
static volatile uint8_t twis_txBufferIndex;
static volatile uint8_t twis_txBufferLength;

static uint8_t twis_txQueue[TWI_TX_QUEUE_SIZE];
static volatile uint8_t twis_txqin;
static volatile uint8_t twis_txqout;

#ifdef USE_QLEN
static volatile uint8_t twis_txqlen;
#define TWI_TX_QLEN twis_txqlen
#define TWI_TX_QROOM (TWI_TX_QUEUE_SIZE - twis_txqlen)
#else
#define TWI_TX_QLEN ((twis_txqin - twis_txqout) & (TWI_TX_QUEUE_SIZE - 1))
#define TWI_TX_QROOM (TWI_TX_QUEUE_SIZE - TWI_TX_QLEN - 1)
#endif

static uint8_t twis_rxQueue[TWI_RX_QUEUE_SIZE];
static volatile uint8_t twis_rxqin;
static volatile uint8_t twis_rxqout;

#ifdef USE_QLEN
static volatile uint8_t twis_rxqlen;
#define TWI_RX_QLEN twis_rxqlen
#define TWI_RX_QROOM (TWI_RX_QUEUE_SIZE - twis_rxqlen)
#else
#define TWI_RX_QLEN ((twis_rxqin - twis_rxqout) & (TWI_RX_QUEUE_SIZE - 1))
#define TWI_RX_QROOM (TWI_RX_QUEUE_SIZE - TWI_RX_QLEN - 1)
#endif

static volatile uint8_t twis_txMode;
#define TXMODE_BUFFER 0
#define TXMODE_QUEUE  1

//static uint8_t twis_rxBuffer[TWI_RX_BUFFER_LENGTH];
static uint8_t twis_rxBuffer[2]; // Only used for non-FIFO register writes
static volatile uint8_t twis_rxBufferIndex;

//static volatile uint8_t twis_error;

#define UBVERSION "UB\x01\x00"

/* 
 * Function twis_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twis_init(void)
{
#ifdef DEBUG
pinMode(DebugPin1, OUTPUT);
pinMode(DebugPin2, OUTPUT);
pinMode(DebugPin3, OUTPUT);
//pinMode(DebugPin4, OUTPUT);
#endif

  // initialize state
  //twis_state = TWI_READY;
  //twis_sendStop = true;		// default value
  //twis_inRepStart = false;

  // initialize internal variables
  twis_txqin = 0;
  twis_txqout = 0;
  twis_rxqin = 0;
  twis_rxqout = 0;
  
#if 0
  // activate internal pullups for twi.
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
#endif

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

/* 
 * Function twis_disable
 * Desc     disables twi pins
 * Input    none
 * Output   none
 */
void twis_disable(void)
{
  // disable twi module, acks, and twi interrupt
  TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));

  // Bus may still be active; go passive.
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
}

/* 
 * Function twis_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void twis_setAddress(uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

#if 0
/* 
 * Function twis_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t twis_transmit(const uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_TX_BUFFER_LENGTH < length){
    return 1;
  }

  // set length and copy data into tx buffer
  twis_txBufferLength = length;
  for(i = 0; i < length; ++i){
    twis_txBuffer[i] = data[i];
  }

  twis_txMode = TXMODE_BUFFER;
  
  return 0;
}
#endif

uint8_t twis_txenq(const uint8_t* data, uint8_t length)
{
  uint8_t room = TWI_TX_QROOM;

  if (length > room)
    length = room;

  for (int i = 0 ; i < length ; i++) {
    twis_txQueue[twis_txqin] = *data++;
    twis_txqin = (twis_txqin + 1) % TWI_TX_QUEUE_SIZE;
#ifdef USE_QLEN
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      twis_txqlen++;
    }
#endif
  }

  return length;
}

int twis_available(void)
{
  return TWI_RX_QLEN;
}

int twis_read(void)
{
  int data;

  if (TWI_RX_QLEN) {
    data = twis_rxQueue[twis_rxqout];
    twis_rxqout = (twis_rxqout + 1) % TWI_RX_QUEUE_SIZE;
#ifdef USE_QLEN
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      twis_rxqlen--;
    }
#endif
    return data;
  }

  return -1;
}

int twis_peek(void)
{
  return TWI_RX_QLEN ? twis_rxQueue[twis_rxqout] : -1;
}

/* 
 * Function twis_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twis_attachRegisterWriteHandler( void (*function)(uint8_t, uint8_t) )
{
  twis_onRegisterWrite = function;
}

/* 
 * Function twis_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
#define TWREPLYACK (_BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA))
#define TWREPLY (_BV(TWEN) | _BV(TWIE) | _BV(TWINT))

#if 0
void twis_reply(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = TWREPLYACK;
  }else{
    TWCR = TWREPLY;
  }
}
#else
// Optimizer should delete unused case
#define twis_reply(ack) { \
  if(ack) { \
    TWCR = TWREPLYACK;\
  }else{ \
    TWCR = TWREPLY;\
  } \
}

#endif

/* 
 * Function twis_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twis_stop(void)
{
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }
}

/* 
 * Function twis_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
#define TWRELBUS (_BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT))
#if 0
void twis_releaseBus(void)
{
  // release bus
  TWCR = TWRELBUS;
}
#else
#define twis_releaseBus() { TWCR = TWRELBUS; }
#endif

ISR(TWI_vect)
{
  static uint8_t _reg;
  static bool preloaded = false;
  static uint8_t tdata;
  static bool prepointed = false;
  static uint8_t *rdatap;

  top:;
  switch(TW_STATUS){

    /*
     * Slave Receiver
     */

    case TW_SR_SLA_ACK:   // 0x60 addressed, returned ack
    sr_sla_ack:;
      digitalDebug(DebugPin1, HIGH);

      // indicate that rx buffer can be overwritten and ack
      twis_rxBufferIndex = 0;
      twis_reply(1);
      digitalDebug(DebugPin1, LOW);
      break;

    case TW_SR_DATA_ACK:       // 0x80 data received, returned ack
      sr_data_ack:;

      if (twis_rxBufferIndex == 0) {
        // First byte
        _reg = TWDR >> 3;         // Ignore channel
        twis_rxBuffer[0] = _reg;  // Constant index for speed
        twis_rxBufferIndex++;
        twis_reply(1);
#ifdef RX_PREPOINTER
        if (TWI_RX_QROOM) {
          rdatap = &twis_rxQueue[twis_rxqin];
          prepointed = true;
        } else {
          prepointed = false;
        }
#endif
      } else {
        // Following bytes
        if (_reg == IS7x0_REG_THR) {
#ifdef RX_PREPOINTER
          if (prepointed) {
            *rdatap = TWDR;
            twis_reply(1);
            twis_rxqin = (twis_rxqin + 1) % TWI_RX_QUEUE_SIZE;
#ifdef USE_QLEN
            twis_rxqlen++;
#endif
            if (TWI_RX_QROOM) {
              rdatap = &twis_rxQueue[twis_rxqin];
            } else {
              prepointed = false;
            }
          } else
#endif
          if (TWI_RX_QROOM) {
            twis_rxQueue[twis_rxqin] = TWDR;
            twis_reply(1);
            twis_rxqin = (twis_rxqin + 1) % TWI_RX_QUEUE_SIZE;
#ifdef USE_QLEN
            twis_rxqlen++;
#endif
#ifdef RX_PREPOINTER
            if (TWI_RX_QROOM) {
              rdatap = &twis_rxQueue[twis_rxqin];
              prepointed = true;
            }
#endif
          } else {
            twis_reply(0);
          }
        } else {
          // Non-FIFO register writes. Should be single byte data.
          if (twis_rxBufferIndex == 1) {
            twis_rxBuffer[1] = TWDR;
            twis_rxBufferIndex++;
            twis_reply(1);
          } else {
            twis_reply(0);
          }
        }
      }

      break;

    case TW_SR_STOP: // 0xA0 stop or repeated start condition received

      digitalDebug(DebugPin2, HIGH);

      // ack future responses and leave slave receiver state
      //twis_releaseBus();

      //_reg = twis_rxBuffer[0] >> 3; // Ignore channel // This is done already

      if (twis_rxBufferIndex == 1) {
        // Register (sub-address) designation cycle for a future read.
        twis_releaseBus();

        digitalDebug(DebugPin2, LOW);

        break;
      }

      // Process register writes:
      // Call user defined callback

      if (_reg != IS7x0_REG_THR && twis_onRegisterWrite) {
        twis_onRegisterWrite(_reg, twis_rxBuffer[1]);
      }

      twis_releaseBus();

      digitalDebug(DebugPin2, LOW);

#if 0
      if (TWCR & _BV(TWINT))
        goto top;
#endif
      break;

    case TW_SR_DATA_NACK:       // 0x88 data received, returned nack
      sr_data_nack:;
      // nack back at master
      twis_reply(0);
      break;

    case TW_ST_SLA_ACK:          // 0xA8 addressed, returned ack
      st_sla_ack:;

      /*
       * Fast response case
       */
      switch (_reg) {
      case IS7x0_REG_RHR:
        if (TWI_TX_QLEN) {
#ifdef TX_PRELOAD
          if (preloaded)
            TWDR = tdata;
          else
            TWDR = twis_txQueue[twis_txqout];
#else
          TWDR = twis_txQueue[twis_txqout];
#endif
          twis_reply(1);
          twis_txqout = (twis_txqout + 1) % TWI_TX_QUEUE_SIZE;
#ifdef USE_QLEN
          twis_txqlen--;
#endif

#ifdef TX_PRELOAD
          if (TWI_TX_QLEN) {
            tdata = twis_txQueue[twis_txqout];
            preloaded = true;
          } else {
            preloaded = false;
          }
#endif
        } else {
          TWDR = 0;
          twis_reply(0);
        }
        goto out;

      case IS7x0_REG_IIR:
        TWDR = 0x01; // XXX Temporary, should return IIR
        twis_reply(1);
        goto out;

      case IS7x0_REG_RXLVL:
        TWDR = TWI_TX_QLEN;
        twis_reply(1);
        goto out;

      case IS7x0_REG_TXLVL:
        TWDR = TWI_RX_QROOM;
        twis_reply(1);
        goto out;

      case UB_REG_ID:
        //memcpy(twis_txBuffer, (void *)"UB\x01\x00", 4);
        twis_txBuffer[0] = UBVERSION[0];
        twis_txBuffer[1] = UBVERSION[1];
        twis_txBuffer[2] = UBVERSION[2];
        twis_txBuffer[3] = UBVERSION[3];
        twis_txBufferLength = 4;
        twis_txBufferIndex = 0;
        twis_txMode = TXMODE_BUFFER; // XXX Will be deleted
        goto st_data_ack_other;

      default:
        twis_txBufferLength = 1;
        twis_txBuffer[0] = 0x00;
        goto st_data_ack_other;
      }

      break; // All gone at this point, but for consistency.

    case TW_ST_DATA_ACK: // 0xB8 byte sent, ack returned

      /*
       * Fast response case
       */
      if (_reg == IS7x0_REG_RHR) {
        if (TWI_TX_QLEN) {
#ifdef TX_PRELOAD
          if (preloaded)
            TWDR = tdata;
          else
            TWDR = twis_txQueue[twis_txqout];
#else
          TWDR = twis_txQueue[twis_txqout];
#endif
          twis_reply(1);
          twis_txqout = (twis_txqout + 1) % TWI_TX_QUEUE_SIZE;
#ifdef USE_QLEN
          twis_txqlen--;
#endif

#ifdef TX_PRELOAD
          if (TWI_TX_QLEN) {
            tdata = twis_txQueue[twis_txqout];
            preloaded = true;
          } else {
            preloaded = false;
          }
#endif
        } else {
          TWDR = 0;
          twis_reply(0);
        }
        goto out;
      }

      // XXX FIFO case (else-clause) can be deleted if fast response works.
      // copy data to output register

    st_data_ack_other:;

      if (twis_txMode == TXMODE_BUFFER) {
        TWDR = twis_txBuffer[twis_txBufferIndex++];
        // if there is more to send, ack, otherwise nack
        if(twis_txBufferIndex < twis_txBufferLength){
          twis_reply(1);
        }else{
          twis_reply(0);
        }
      } else {
        if (TWI_TX_QLEN) {
          TWDR = twis_txQueue[twis_txqout];
          twis_reply(1);
          twis_txqout = (twis_txqout + 1) % TWI_TX_QUEUE_SIZE;
#ifdef USE_QLEN
          twis_txqlen--;
#endif
        } else {
          TWDR = 0;
          twis_reply(0);
        }
      }
      break;

    case TW_ST_DATA_NACK: // 0xC0 received nack, we are done 
    case TW_ST_LAST_DATA: // 0xC8 received ack, but we are done already!
      // ack future responses
      twis_reply(1);
      break;

    // All
    case TW_NO_INFO:   // 0xF8 no state information
      break;

    case TW_BUS_ERROR: // 0x00 bus error, illegal stop/start
      // Dont remove this. Hangs the master at startup?
      twis_stop();
      break;

    // Rare cases
    case TW_SR_GCALL_ACK: // 0x70 addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // 0x68 lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // 0x78 lost arbitration, returned ack
      goto sr_sla_ack;
    case TW_SR_GCALL_DATA_ACK: // 0x90 data received generally, returned ack
      goto sr_data_ack;
    case TW_SR_GCALL_DATA_NACK: // 0x98 data received generally, returned nack
      goto sr_data_nack;
    case TW_ST_ARB_LOST_SLA_ACK: // 0xB0 arbitration lost, returned ack
      goto st_sla_ack;
  }
  out:;
}
#endif // I2C_UB_SUPPORT
