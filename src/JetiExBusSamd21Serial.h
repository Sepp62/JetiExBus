/**************************************************************** 
Jeti Sensor EX Bus Telemetry C++ Library

JetiExBusSamd21Serial - Samd21 serial implementation for half duplex 
single wire DMA operation, compatible with Seeedstudio XIAO m0.

this is an addon from nichtgedacht
Version history: 1.0 initial
                 2.0 using DMA for UART

JetiExBus Library is: 
---------------------------------------------------------------------
Copyright (C) 2018 Bernd Wokoeck
  
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.

**************************************************************/

#ifndef JETIEXBUSSAMD21SERIAL_H
#define JETIEXBUSSAMD21SERIAL_H

#if defined (__SAMD21__)

#include "JetiExBusSerial.h"

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// DMA descriptors
// outside of object because of alignment 
volatile DmacDescriptor dmaDescriptor[2] __attribute__((aligned(16)));
volatile DmacDescriptor dmaWriteback[2] __attribute__((aligned(16)));

// SAMD21
//////////

class JetiExBusSamd21Serial : public JetiExBusSerial
{
	friend void DMAC_Handler(void);
	
public:
	virtual void   begin(uint32_t baud, uint32_t format);
	virtual size_t write(const uint8_t *buffer, size_t size);
	virtual int    available(void);
	virtual int    read(void);

protected:
  enum
  {
    TX_RINGBUF_SIZE = 128, 
    RX_RINGBUF_SIZE = 128,
  };

  // tx buffer
  volatile uint8_t   m_txBuf[ TX_RINGBUF_SIZE ]; 
  volatile uint8_t   m_txHead;
  volatile uint8_t   m_txTail;

  // rx buffer
  volatile uint8_t   m_rxBuf[ RX_RINGBUF_SIZE ]; 
  volatile uint8_t   m_rxHead;
  volatile uint8_t   m_rxTail;

  // receiver state
  volatile bool       m_bSending;
};

#endif // __SAMD21__  
#endif // JETIEXBUSSAMD21SERIAL_H
