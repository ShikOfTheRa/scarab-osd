/*
  twislave.c - TWI/I2C optimized for NXP SC16IS7xx emulation,
      Heavily modified from ...

  twi.h - TWI/I2C library for Wiring & Arduino
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

  Modified 2016 by jflyper
*/

#ifndef twis_h
#define twis_h

  #include <inttypes.h>

  //#define ATMEGA8

  #ifndef TWI_FREQ
  #define TWI_FREQ 100000L
  #endif

  #ifndef TWI_TX_BUFFER_LENGTH
  #define TWI_TX_BUFFER_LENGTH 8
  #endif

  #ifndef TWI_TX_QUEUE_SIZE
  #define TWI_TX_QUEUE_SIZE 32
  #endif

  #ifndef TWI_RX_QUEUE_SIZE
  #define TWI_RX_QUEUE_SIZE 32
  #endif

  #define TWI_READY 0
  #define TWI_MRX   1
  #define TWI_MTX   2
  #define TWI_SRX   3
  #define TWI_STX   4
  
  void twis_init(void);
  void twis_disable(void);
  void twis_setAddress(uint8_t);
  //uint8_t twis_transmit(const uint8_t*, uint8_t);
  uint8_t twis_txenq(const uint8_t*, uint8_t);
  //uint8_t twis_txqlen(void);
  //uint8_t twis_txtxq();

  int twis_available(void);
  int twis_peek(void);
  int twis_read(void);

  void twis_attachRegisterWriteHandler( void (*)(uint8_t, uint8_t) );
  //void twis_attachSlaveTxEvent( void (*)(void) );
  //void twis_reply(uint8_t);
  void twis_stop(void);
  //void twis_releaseBus(void);

#endif

