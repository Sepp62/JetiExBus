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

#if defined (ARDUINO_ARCH_AVR)

#include "JetiExBusAtMegaSerial.h"

#include <avr/io.h>
#include <avr/interrupt.h>

// #define LED_DEBUG

JetiExBusAtMegaSerial * _pInstance = 0;   // instance pointer to find the serial object from ISR

JetiExBusSerial * JetiExBusSerial::CreatePort(int comPort)
{
	return new JetiExBusAtMegaSerial();
}

void JetiExBusAtMegaSerial::begin(uint32_t baud, uint32_t format)
{
  // init UART-registers
  // https://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Der_UART#Die_UART-Register
  UCSRA = 0x00;												// u2x mode off (1 << U2X0);
  UCSRB = _BV(RXEN)  | _BV(RXCIE);							// Rx enable
  UCSRC = _BV(UCSZ0) | _BV(UCSZ1);                          // 8-bit data, 1 stop, no parity

  // wormfood.net/avrbaudcalc.php 
#if F_CPU == 16000000L  // for the 16 MHz clock on most Arduino boards
  UBRRH = 0x00;
  UBRRL = 0x07; // 125000 Bit/s
#elif F_CPU == 8000000L   // for the 8 MHz internal clock (Pro Mini 3.3 Volt) 
  UBRRH = 0x00;
  UBRRL = 0x03; // 125000 Bit/s
#else
  #error Unsupported clock speed
#endif  

  // TX and RX pins goes high, when disabled
  pinMode( 0, INPUT_PULLUP );
  pinMode( 1, INPUT_PULLUP );

  // init tx ring buffer 
  m_txHeadPtr = m_txBuf;
  m_txTailPtr = m_txBuf;
  m_txNumChar = 0;

  // init rx ring buffer 
  m_rxHeadPtr = m_rxBuf;
  m_rxTailPtr = m_rxBuf;
  m_rxNumChar = 0;

  m_bSending = false;
  _pInstance = this; // there is a single instance only

#ifdef LED_DEBUG
  pinMode( 13, OUTPUT);
  digitalWrite(13, LOW ); 
#endif
}


// Read key from Jeti box
int JetiExBusAtMegaSerial::read(void)
{
  uint8_t c = 0;

  if( m_rxNumChar ) // atomic operation
  {
    cli();
    c = *(_pInstance->m_rxTailPtr);
    m_rxNumChar--; 
    m_rxTailPtr = IncBufPtr8( m_rxTailPtr, m_rxBuf, RX_RINGBUF_SIZE );
    sei();
  }
  return c;
}

// Send one byte  
size_t JetiExBusAtMegaSerial::write(const uint8_t *buffer, size_t size)
{
  cli();

  size_t ret = size;

  for( uint8_t i = 0; i < size; i++ )
  {
	  if (m_txNumChar < TX_RINGBUF_SIZE)
	  {
		  *m_txHeadPtr = buffer[i];                                           // write data to buffer
		  m_txNumChar++;                                                      // increase number of characters in buffer
		  m_txHeadPtr = IncBufPtr8(m_txHeadPtr, m_txBuf, TX_RINGBUF_SIZE);    // increase ringbuf pointer
	  }
	  else
	  {
		  ret = i;
		  break;
	  }
  }

  // enable transmitter
  if( !m_bSending )
  {
    m_bSending    = true;
    uint8_t ucsrb = UCSRB;
    ucsrb        &= ~( (1<<RXEN) | (1<<RXCIE) ); // disable receiver and receiver interrupt
    ucsrb        |=    (1<<TXEN) | (1<<UDRIE);   // enable transmitter and tx register empty interrupt 
    UCSRB         = ucsrb;

    // digitalWrite( 13, HIGH ); // show transmission
  }
  sei();

  return ret;
}

volatile uint8_t * JetiExBusAtMegaSerial::IncBufPtr8( volatile uint8_t * ptr, volatile uint8_t * pRingBuf, size_t bufSize )
{
  ptr++;
  if( ptr >= ( pRingBuf + bufSize ) )
    return pRingBuf; // wrap around
  else
    return ptr;
}
  
// ISR - transmission complete 
ISR( USART_TX_vect )
{
  // enable receiver
  uint8_t ucsrb = UCSRB; 
  ucsrb        &= ~( (1<<TXEN) | (1<<TXCIE) ); // disable transmitter and tx interrupt when there is nothing more to send
  ucsrb        |=    (1<<RXEN) | (1<<RXCIE);   // enable receiver with interrupt
  UCSRB        = ucsrb;

  // clear receiver buffer
  _pInstance->m_rxHeadPtr = _pInstance->m_rxBuf;
  _pInstance->m_rxTailPtr = _pInstance->m_rxBuf;
  _pInstance->m_rxNumChar = 0;

#ifdef LED_DEBUG
  digitalWrite( 13, LOW ); 
#endif
}


// ISR - send buffer empty
ISR( USART_UDRE_vect )
{
  if( _pInstance->m_txNumChar != 0 )
  {
    UDR = *(_pInstance->m_txTailPtr);

    _pInstance->m_txNumChar--; 
    _pInstance->m_txTailPtr = _pInstance->IncBufPtr8( _pInstance->m_txTailPtr, _pInstance->m_txBuf, JetiExBusAtMegaSerial::TX_RINGBUF_SIZE );
  }
  else
  {
    // enable TX complete interrupt to get a signal for end of transmission
    uint8_t ucsrb = UCSRB; 
    ucsrb        &= ~(1<<UDRIE); 
    ucsrb        |=  (1<<TXCIE);
    UCSRB        = ucsrb;
    _pInstance->m_bSending = false;
  }
}

// ISR - receiver buffer full
ISR( USART_RX_vect )
{
	*(_pInstance->m_rxHeadPtr) = UDR;   // save data to buffer
	_pInstance->m_rxNumChar++;          // increase number of characters in buffer
	_pInstance->m_rxHeadPtr = _pInstance->IncBufPtr8( _pInstance->m_rxHeadPtr, _pInstance->m_rxBuf, _pInstance->RX_RINGBUF_SIZE );    // increase ringbuf pointer

#ifdef LED_DEBUG
   digitalWrite( 13, HIGH ); 
#endif 
}

#endif // ARDUINO_ARCH_AVR