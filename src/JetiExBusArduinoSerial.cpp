/*
Jeti Sensor EX Bus Telemetry C++ Library

JetiExBusArduinoSerial - serial implementation for
                         half duplex operation for generic HardwareSerial
						 plus some small uC-specific adaptions,
						 this code works for ATMega328PB
						 (connect RX/TX to pin 12/11 for UART1)
---------------------------------------------------------------------

Copyright (C) 2018 Bernd Wokoeck

Version history:
0.90   02/13/2018  created
0.94   02/17/2018  generic arduino HardwareSerial support for AtMega328PB

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

#if defined (__AVR_ATmega328PB__)

// #define LED_DEBUG

#include "JetiExBusArduinoSerial.h"

static volatile bool      _bTimerRunning = false;
static volatile uint8_t * _pUCSRB = 0; // points to UCSR0B/0xC1 or UCSR1B/0xC9

// Static functions are specific for microcontroller
////////////////////////////////////////////////////
void _TxOn()
{
	*_pUCSRB |= (1 << TXEN1);
}

void _TxOff()
{
	*_pUCSRB &= ~(1 << TXEN1);
}

void _InitRegisters(int comPort)
{
	// TX/RX Pins on ATMEGA328PB: RX0-Pin = 0, TX0-Pin = 1, RX1-Pin	= 12, TX1-Pin = 11
	// set "transmitter enable register" and disabled TX pin to "input" 
	switch (comPort)
	{
	case 0: _pUCSRB = &UCSR0B; pinMode(1, INPUT); digitalWrite(1, true); break;
	default:
	case 1: _pUCSRB = &UCSR1B; pinMode(11, INPUT); digitalWrite(11, true); break;
	}
}

void _Setup4msTimer()
{
	// Use Arduino Timer 2 --> do not use Arduino tone()-function anymore
	cli();
	TCCR2A = 0; // Stop and reset
	TCCR2B = 0;
	TCNT2 = 0;
	// https://github.com/bigjosh/TimerShot/blob/master/TimerShot.ino
	OCR2A = 0;
	OCR2B = 0xff - 64;  // set pulse width 64 --> ~4ms
	TCCR2A = (1 << WGM20) | (1 << WGM21); // mode 7 fast PWM.
	TCCR2B = (1 << WGM22) | (1 << CS20) | (1 << CS21) | (1 << CS22); // prescaler 1024, mode 7 fast PWM.
	TCNT2 = OCR2B - 1; // now start countdown 
	TIMSK2 |= (1 << OCIE2A);
	sei();
}

ISR(TIMER2_COMPA_vect)
{
	 if (_bTimerRunning)
	 {
		_TxOff(); // transmitter off
		_bTimerRunning = false;
#ifdef LED_DEBUG
		digitalWrite(13, false);
#endif
	}
}

// Generic code using HardwareSerial
////////////////////////////////////
JetiExBusSerial * JetiExBusSerial::CreatePort(int comPort) // Comport 0 = Serial, Comport 1 = Serial1
{
#ifdef LED_DEBUG
	pinMode(13, OUTPUT);
    digitalWrite(13, false);
#endif
	
	return new JetiExBusArduinoSerial(comPort);
}

JetiExBusArduinoSerial::JetiExBusArduinoSerial(int comPort) : m_pSerial(0), m_comPort( comPort )
{
	switch (comPort)
	{
	case 0: m_pSerial = &Serial; break;
	default:
	case 1: m_pSerial = &Serial1; break;
	}
	_InitRegisters(comPort);
}

void JetiExBusArduinoSerial::begin(uint32_t baud, uint32_t format)
{
	m_pSerial->begin(baud, format);
	_TxOff();
}

size_t JetiExBusArduinoSerial::write(const uint8_t *buffer, size_t size)
{
	if( !_bTimerRunning )
	{
		_TxOn();
		_Setup4msTimer();
		_bTimerRunning = true;
#ifdef LED_DEBUG
		digitalWrite(13, true);
#endif
	}
	return m_pSerial->write(buffer, size);
}

#endif // __AVR_ATmega328PB__