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

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
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

//static volatile uint8_t twis_state;
//static volatile uint8_t twis_slarw;
//static volatile uint8_t twis_sendStop;			// should the transaction end with a stop
//static volatile uint8_t twis_inRepStart;			// in the middle of a repeated start

//static void (*twis_onSlaveTransmit)(void);
static void (*twis_onSlaveReceive)(uint8_t*, int) = NULL;

static uint8_t twis_txBuffer[TWI_TX_BUFFER_LENGTH];
static volatile uint8_t twis_txBufferIndex;
static volatile uint8_t twis_txBufferLength;

static uint8_t twis_txQueue[TWI_TX_QUEUE_LENGTH];
static volatile uint8_t twis_txqin;
static volatile uint8_t twis_txqout;
#define TWI_TX_QLEN ((twis_txqin - twis_txqout) & (TWI_TX_QUEUE_LENGTH - 1))
#define TWI_TX_QROOM (TWI_TX_QUEUE_LENGTH - TWI_TX_QLEN)

static uint8_t twis_rxQueue[TWI_RX_QUEUE_LENGTH];
static volatile uint8_t twis_rxqin;
static volatile uint8_t twis_rxqout;
#define TWI_RX_QLEN ((twis_rxqin - twis_rxqout) & (TWI_RX_QUEUE_LENGTH - 1))
#define TWI_RX_QROOM (TWI_RX_QUEUE_LENGTH - TWI_RX_QLEN)

static volatile uint8_t twis_txMode;
#define TXMODE_BUFFER 0
#define TXMODE_QUEUE  1

//static uint8_t twis_rxBuffer[TWI_RX_BUFFER_LENGTH];
static uint8_t twis_rxBuffer[2]; // Only used for non-FIFO register writes
static volatile uint8_t twis_rxBufferIndex;

static volatile uint8_t twis_error;

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
  
  // activate internal pullups for twi.
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

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

#if 0
  // deactivate internal pullups for twi.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
#endif

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
 * Function twis_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 *          sendStop: Boolean indicating whether to send a stop at the end
 * Output   number of bytes read
 */
uint8_t twis_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 0;
  }

  // wait until twi is ready, become master receiver
  while(TWI_READY != twis_state){
    continue;
  }
  twis_state = TWI_MRX;
  twis_sendStop = sendStop;
  // reset error state (0xFF.. no error occured)
  twis_error = 0xFF;

  // initialize buffer iteration vars
  twis_masterBufferIndex = 0;
  twis_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled. 
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twis_slarw = TW_READ;
  twis_slarw |= address << 1;

  if (true == twis_inRepStart) {
    // if we're in the repeated start state, then we've already sent the start,
    // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    // We need to remove ourselves from the repeated start state before we enable interrupts,
    // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    // up. Also, don't enable the START interrupt. There may be one pending from the 
    // repeated start that we sent ourselves, and that would really confuse things.
    twis_inRepStart = false;			// remember, we're dealing with an ASYNC ISR
    do {
      TWDR = twis_slarw;
    } while(TWCR & _BV(TWWC));
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
  }
  else
    // send start condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  // wait for read operation to complete
  while(TWI_MRX == twis_state){
    continue;
  }

  if (twis_masterBufferIndex < length)
    length = twis_masterBufferIndex;

  // copy twi buffer to data
  for(i = 0; i < length; ++i){
    data[i] = twis_masterBuffer[i];
  }
	
  return length;
}
#endif

#if 0
/* 
 * Function twis_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          sendStop: boolean indicating whether or not to send a stop at the end
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
uint8_t twis_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 1;
  }

  // wait until twi is ready, become master transmitter
  while(TWI_READY != twis_state){
    continue;
  }
  twis_state = TWI_MTX;
  twis_sendStop = sendStop;
  // reset error state (0xFF.. no error occured)
  twis_error = 0xFF;

  // initialize buffer iteration vars
  twis_masterBufferIndex = 0;
  twis_masterBufferLength = length;
  
  // copy data to twi buffer
  for(i = 0; i < length; ++i){
    twis_masterBuffer[i] = data[i];
  }
  
  // build sla+w, slave device address + w bit
  twis_slarw = TW_WRITE;
  twis_slarw |= address << 1;
  
  // if we're in a repeated start, then we've already sent the START
  // in the ISR. Don't do it again.
  //
  if (true == twis_inRepStart) {
    // if we're in the repeated start state, then we've already sent the start,
    // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    // We need to remove ourselves from the repeated start state before we enable interrupts,
    // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    // up. Also, don't enable the START interrupt. There may be one pending from the 
    // repeated start that we sent outselves, and that would really confuse things.
    twis_inRepStart = false;			// remember, we're dealing with an ASYNC ISR
    do {
      TWDR = twis_slarw;				
    } while(TWCR & _BV(TWWC));
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
  }
  else
    // send start condition
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);	// enable INTs

  // wait for write operation to complete
  while(wait && (TWI_MTX == twis_state)){
    continue;
  }
  
  if (twis_error == 0xFF)
    return 0;	// success
  else if (twis_error == TW_MT_SLA_NACK)
    return 2;	// error: address send, nack received
  else if (twis_error == TW_MT_DATA_NACK)
    return 3;	// error: data send, nack received
  else
    return 4;	// other twi error
}
#endif

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

#if 0
  // ensure we are currently a slave transmitter
  if(TWI_STX != twis_state){
    return 2;
  }
#endif
  
  // set length and copy data into tx buffer
  twis_txBufferLength = length;
  for(i = 0; i < length; ++i){
    twis_txBuffer[i] = data[i];
  }

  twis_txMode = TXMODE_BUFFER;
  
  return 0;
}

uint8_t twis_txenq(const uint8_t* data, uint8_t length)
{
  uint8_t room = TWI_TX_QROOM;
  if (length > room)
    length = room;

  for (int i = 0 ; i < length ; i++) {
    twis_txQueue[twis_txqin] = *data++;
    twis_txqin = (twis_txqin + 1) % TWI_TX_QUEUE_LENGTH;
  }

  return length;
}

uint8_t twis_txqlen()
{
  return TWI_TX_QLEN;
}

uint8_t twis_txtxq()
{
#if 0
  // ensure we are currently a slave transmitter
  if(TWI_STX != twis_state){
    return 2;
  }
#endif

  twis_txMode = TXMODE_QUEUE;

  return 0;
}

int twis_available(void)
{
  return TWI_RX_QLEN;
}

int twis_read(void)
{
  int data;

  if (TWI_RX_QLEN) {
    data = twis_rxQueue[twis_txqout];
    twis_txqout = (twis_txqout + 1) % TWI_RX_QUEUE_LENGTH;
    return data;
  }

  return -1;
}

int twis_peek(void)
{
  return TWI_RX_QLEN ? twis_rxQueue[twis_txqout] : -1;
}

/* 
 * Function twis_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twis_attachSlaveRxEvent( void (*function)(uint8_t*, int) )
{
  twis_onSlaveReceive = function;
}

#if 0
/* 
 * Function twis_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twis_attachSlaveTxEvent( void (*function)(void) )
{
  twis_onSlaveTransmit = function;
}
#endif

/* 
 * Function twis_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
void twis_reply(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

#if 0
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

  // update twi state
  twis_state = TWI_READY;
}
#endif

/* 
 * Function twis_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twis_releaseBus(void)
{
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

#if 0
  // update twi state
  twis_state = TWI_READY;
#endif
}

ISR(TWI_vect)
{
  static uint8_t _reg;

  top:;
  switch(TW_STATUS){

    /*
     * Slave Receiver
     */

    case TW_SR_SLA_ACK:   // 0x60 addressed, returned ack
    sr_sla_ack:;
      digitalDebug(DebugPin1, HIGH);
#if 0
      // enter slave receiver mode
      twis_state = TWI_SRX;
#endif
      // indicate that rx buffer can be overwritten and ack
      twis_rxBufferIndex = 0;
      twis_reply(1);
      digitalDebug(DebugPin1, LOW);
      break;

    case TW_SR_DATA_ACK:       // 0x80 data received, returned ack
      sr_data_ack:;

twis_reply(1);

#ifdef notdef
      if (twis_rxBufferIndex == 0) {
        // First byte
        _reg = TWDR >> 3;         // Ignore channel
        twis_rxBuffer[0] = _reg;  // Constant index for speed
        twis_rxBufferIndex++;
        twis_reply(1);
      } else {
        // Following bytes
        if (_reg == 0) {
          if (TWI_RX_QROOM) {
            twis_rxQueue[twis_rxqin] = TWDR;
            twis_rxqin = (twis_rxqin + 1) % TWI_RX_QUEUE_LENGTH;
            twis_reply(1);
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
#endif // notdef

#if 0
      // if there is still room in the rx buffer
      if(twis_rxBufferIndex < TWI_RX_BUFFER_LENGTH){
        // put byte in buffer and ack
        twis_rxBuffer[twis_rxBufferIndex++] = TWDR;
        twis_reply(1);
      }else{
        // otherwise nack
        twis_reply(0);
      }
#endif

      break;

    case TW_SR_STOP: // 0xA0 stop or repeated start condition received

      twis_releaseBus();

#ifdef notdef
      digitalDebug(DebugPin2, HIGH);

      // ack future responses and leave slave receiver state
      //twis_releaseBus();

      //_reg = twis_rxBuffer[0] >> 3; // Ignore channel

      if (twis_rxBufferIndex == 1) {
        // Register (sub-address) designation cycle for a future read.
        twis_releaseBus();

        digitalDebug(DebugPin2, LOW);

        break;
      }

      // Process register writes:
      // Call user defined callback

      if (_reg != IS7x0_REG_THR) {
          if (twis_onSlaveReceive)
              twis_onSlaveReceive(twis_rxBuffer, twis_rxBufferIndex);
          // since we submit rx buffer to "wire" library, we can reset it
          twis_rxBufferIndex = 0;
      }

      twis_releaseBus();

      digitalDebug(DebugPin2, LOW);

#if 0
      if (TWCR & _BV(TWINT))
        goto top;
#endif

#endif //notdef

      break;

    case TW_SR_DATA_NACK:       // 0x88 data received, returned nack
      sr_data_nack:;
      // nack back at master
      twis_reply(0);
      break;

    case TW_ST_SLA_ACK:          // 0xA8 addressed, returned ack
      st_sla_ack:;
#if 0
      // enter slave transmitter mode
      twis_state = TWI_STX;
#endif

      /*
       * Fast response case
       */
      switch (_reg) {
      case IS7x0_REG_RHR:
        if (TWI_TX_QLEN) {
          TWDR = twis_txQueue[twis_txqout];
          twis_reply(1);
          twis_txqout = (twis_txqout + 1) % TWI_TX_QUEUE_LENGTH;
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
        memcpy(twis_txBuffer, (void *)"UB\x01\x00", 4);
        twis_txBufferLength = 4;
        twis_txBufferIndex = 0;
        twis_txMode = TXMODE_BUFFER; // XXX Will be deleted
        goto st_data_ack_other;

      default:
        twis_txBufferLength = 1;
        twis_txBuffer[0] = 0x00;
        goto st_data_ack_other;
      }

// XXX No more upper layer processing for transmitter handling
#if 0
      // ready the tx buffer index for iteration
      twis_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twis_txBufferLength = 0;

      // request for data to transmit.
      digitalDebug(DebugPin3, HIGH);

      twis_onSlaveTransmit(_reg);

      digitalDebug(DebugPin3, LOW);
      // if they didn't change buffer & length, initialize it

      if(twis_txMode == TXMODE_BUFFER && 0 == twis_txBufferLength){
        twis_txBufferLength = 1;
        twis_txBuffer[0] = 0x00;
      }
#endif

      // Fall through

    case TW_ST_DATA_ACK: // 0xB8 byte sent, ack returned

      /*
       * Fast response case
       */
      if (_reg == IS7x0_REG_RHR) {
        if (TWI_TX_QLEN) {
          TWDR = twis_txQueue[twis_txqout];
          twis_reply(1);
          twis_txqout = (twis_txqout + 1) % TWI_TX_QUEUE_LENGTH;
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
          twis_txqout = (twis_txqout + 1) % TWI_TX_QUEUE_LENGTH;
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
#if 0
      // leave slave receiver state
      twis_state = TWI_READY;
#endif
      break;

    // All
    case TW_NO_INFO:   // 0xF8 no state information
// XXX should consider releasing the bus
//twis_releaseBus();
      break;

// Can't happen?
    case TW_BUS_ERROR: // 0x00 bus error, illegal stop/start
// XXX should consider releasing the bus
//twis_releaseBus();
#if 0
      twis_error = TW_BUS_ERROR;
      twis_stop();
#endif
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

