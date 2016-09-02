/*
  I2C UART Bridge Slave

  Modified from:

  TwoWire.h - TWI/I2C library for Arduino & Wiring
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

#ifndef TwoWireUB_h
#define TwoWireUB_h

#include <inttypes.h>
#include "Stream.h"

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class TwoWireUB : public Stream
{
  private:

    static void onRegisterWrite(uint8_t, uint8_t);

    static void begin(void);

  public:
    TwoWireUB();
    void begin(uint8_t, int);
    void begin(int, int);
    void end();
    void setClock(uint32_t);
    void setWriteTimo(unsigned long);
    void setThreshold(int, int);

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;
};

extern TwoWireUB WireUB;

#endif

