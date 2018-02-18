/* 
  Jeti Sensor EX Bus Telemetry C++ Library
  
  JetiExBusAtMegaSerial - AtMega serial implementation for half duplex operation
                          compatible with AtMega 32u4   
  ---------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/09/2018  created

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

#ifndef JETIEXBUSATMEGASERIAL_H
#define JETIEXBUSATMEGASERIAL_H

#if defined (ARDUINO_ARCH_AVR) && !defined(__AVR_ATmega328PB__)

#include "JetiExBusSerial.h"

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#if defined (__AVR_ATmega32U4__)
#define _BV(bit) (1 << (bit))
#define UCSRA UCSR1A
#define UCSRB UCSR1B
#define UCSRC UCSR1C

#define UCSZ2 UCSZ12
#define RXEN RXEN1
#define TXEN TXEN1
#define UCSZ0 UCSZ10
#define UCSZ1 UCSZ11
#define UCSZ2 UCSZ12

#define UPM0 UPM10
#define UPM1 UPM11

#define UBRRH UBRR1H
#define UBRRL UBRR1L

#define RXCIE RXCIE1
#define UDRIE UDRIE1
#define TXCIE TXCIE1

#define TXB8 TXB81

#define UDR UDR1
#define UDRIE UDRIE1

#define USART_RX_vect USART1_RX_vect
#define USART_TX_vect USART1_TX_vect
#define USART_UDRE_vect USART1_UDRE_vect
#else
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C

#define RXEN RXEN0
#define TXEN TXEN0
#define UCSZ0 UCSZ00
#define UCSZ1 UCSZ01
#define UCSZ2 UCSZ02

#define UPM0 UPM00
#define UPM1 UPM01

#define UBRRH UBRR0H
#define UBRRL UBRR0L

#define RXCIE RXCIE0
#define UDRIE UDRIE0
#define TXCIE TXCIE0

#define TXB8 TXB80
#define UDR UDR0
#define UDRIE UDRIE0

#endif // __AVR_ATmega32U4__

// ATMega
//////////

extern "C" void USART_UDRE_vect(void) __attribute__ ((signal)); // make C++ class accessible for ISR
extern "C" void USART_RX_vect(void) __attribute__ ((signal)); // make C++ class accessible for ISR
extern "C" void USART_TX_vect(void) __attribute__ ((signal)); // make C++ class accessible for ISR

class JetiExBusAtMegaSerial : public JetiExBusSerial
{
	friend void USART_UDRE_vect(void);
	friend void USART_RX_vect(void);
	friend void USART_TX_vect(void);

public:
	virtual void   begin(uint32_t baud, uint32_t format);
	virtual size_t write(const uint8_t *buffer, size_t size);
	virtual int    available(void) { return m_rxNumChar != 0; }
	virtual int    read(void);

protected:
	enum
	{
		TX_RINGBUF_SIZE = 64, 
		RX_RINGBUF_SIZE = 64,
	};

	// tx buffer
	volatile uint8_t   m_txBuf[ TX_RINGBUF_SIZE ]; 
	volatile uint8_t * m_txHeadPtr;
	volatile uint8_t * m_txTailPtr;
	volatile uint8_t    m_txNumChar;

	// rx buffer
	volatile uint8_t   m_rxBuf[ RX_RINGBUF_SIZE ]; 
	volatile uint8_t * m_rxHeadPtr;
	volatile uint8_t * m_rxTailPtr;
	volatile uint8_t   m_rxNumChar;

	volatile uint8_t * IncBufPtr8( volatile uint8_t * ptr, volatile uint8_t * ringBuf, size_t bufSize );

	// receiver state
	volatile bool       m_bSending;
};
  
#endif // ARDUINO_ARCH_AVR

#endif // JETIEXBUSATMEGASERIAL_H