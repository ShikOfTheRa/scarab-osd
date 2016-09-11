/*
  WireUB.cpp - I2C UART Bridge library, heavily modified from...

  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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

  Modified 2016 by jflyper for UART Bridge Slave
*/

#include "Config.h" // Look for I2C_UB_SUPPORT

#ifdef I2C_UB_SUPPORT

#define EMBEDDED // Not a standalone bridge

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include <avr/io.h>
  #include "twislave.h"
}

#include "Arduino.h"
#include "WireUB.h"
#include "sc16is7x0.h"
#include "ubreg.h"

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t reg;
uint8_t reg_iir;
uint8_t reg_ier;
uint8_t reg_brh;
int8_t irqpin;

unsigned long writeTimo;
unsigned long lastWrite;

// XXX Should handle this somewhere (here or ISR)
uint8_t rhrThreshold = 4;  // upBuffer
uint8_t thrThreshold = 4;  // dnBuffer

#define ACTIVELOW_ON(pin) pinMode(pin, OUTPUT)
#define ACTIVELOW_OFF(pin) pinMode(pin, INPUT)
#define ACTIVELOW_INIT(pin) {\
  digitalWrite(pin, LOW);\
  ACTIVELOW_OFF(pin);}

// Constructors ////////////////////////////////////////////////////////////////

TwoWireUB::TwoWireUB()
{
}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWireUB::begin(void)
{
  if (irqpin >= 0)
    ACTIVELOW_INIT(irqpin);

  reg_iir = IS7x0_IIR_INTSTAT;

  twis_init();
}

void TwoWireUB::begin(uint8_t address, int pin)
{
  twis_setAddress(address);

  irqpin = pin;

  //twis_attachSlaveTxEvent(onRequestService);
  twis_attachRegisterWriteHandler(onRegisterWrite);

  begin();
}

void TwoWireUB::begin(int address, int pin)
{
  begin((uint8_t)address, pin);
}

void TwoWireUB::end(void)
{
  twis_disable();
}

void TwoWireUB::setClock(uint32_t frequency)
{
  TWBR = ((F_CPU / frequency) - 16) / 2;
}

void TwoWireUB::setWriteTimo(unsigned long timo)
{
  writeTimo = timo;
}

void TwoWireUB::setThreshold(int rxlevel, int txlevel)
{
   // Sanity check?
   rhrThreshold = rxlevel;
   thrThreshold = txlevel;
}

size_t TwoWireUB::write(const uint8_t data)
{
  lastWrite = micros();
  twis_txenq(&data, 1);
  return 1;
}

size_t TwoWireUB::write(const uint8_t *data, size_t quantity)
{
  lastWrite = micros();
  twis_txenq(data, quantity);
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWireUB::available(void)
{
  return twis_available();
}

int TwoWireUB::read(void)
{
  return twis_read();
}

int TwoWireUB::peek(void)
{
  return twis_peek();
}

void TwoWireUB::flush(void)
{
  // XXX: to be implemented.
}

// Called from ISR upon non-FIFO register writes.
void TwoWireUB::onRegisterWrite(uint8_t reg, uint8_t data)
{
  uint8_t fcr;

  switch (reg) {
  case IS7x0_REG_FCR:
    // Should handle FIFO reset
    fcr = data;
    break;

  case IS7x0_REG_IOCONTROL:
    // Should handle software reset
    break;

#ifndef EMBEDDED
  case UB_REG_BRH:
    reg_brh = data;
    break;

  case UB_REG_BRL:
    // Calling Serial.begin() on already began Serial shouldn't be a problem.
    Serial.begin(((reg_brh << 8) | data) * 150);
    break;
#endif

  default:
    break;
  }
}

// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWireUB WireUB = TwoWireUB();
#endif // I2C_UB_SUPPORT
