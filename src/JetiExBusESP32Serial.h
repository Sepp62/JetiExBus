/* 
  Jeti Sensor EX Bus Telemetry C++ Library

  JetiExBusESP32Serial - serial implementation for
						 half duplex operation with ESP32
  ---------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/13/2018  created

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

#ifndef JETIEXBUSESP32SERIAL_H
#define JETIEXBUSESP32SERIAL_H

#include "JetiExBusSerial.h"

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#if defined (ESP32)

#define UART_TXD_IDX(u) ((u==0)?U0TXD_OUT_IDX:((u==1)?U1TXD_OUT_IDX:((u==2)?U2TXD_OUT_IDX:0)))

class ESP32HardwareSerial : public HardwareSerial
{
public:
	ESP32HardwareSerial(int uart_nr) : HardwareSerial(uart_nr) { m_txPin = getTxDefaultPin(); }

    // see ...\Arduino\hardware\espressif\esp32\cores\esp32\esp32-hal-uart.c
	void uartDetachTx()
	{
		if (m_txPin >= 0)
		{
			pinMatrixOutDetach(m_txPin, true, true); // ATTENTION: Bug in esp32-hal-uart.c, function uses "pin" not "signal"
		}
	}

	void uartAttachTx()
	{
		if (m_txPin >= 0)
		{
			pinMode(m_txPin, OUTPUT);
			pinMatrixOutAttach(m_txPin, UART_TXD_IDX(_uart_nr), false, false);
		}
	}

protected:
	int8_t getTxDefaultPin() // pin defaults, see HardwareSerial::begin(...)
	{
		if (_uart_nr == 0)      { return 1; }
		else if (_uart_nr == 1) { return 10; }
		else if (_uart_nr == 2) { return 17; }
		return -1;
	}
	int8_t m_txPin;
};

class JetiExBusESP32Serial : public JetiExBusSerial
{
public:
	JetiExBusESP32Serial(int comPort = 2);

	virtual void   begin(uint32_t baud, uint32_t format) { m_pSerial->begin(baud, format); }
	virtual size_t write(const uint8_t *buffer, size_t size);
	virtual int    available(void) { return m_pSerial->available(); }
	virtual int    read(void) { return m_pSerial->read(); }

protected:
	ESP32HardwareSerial * m_pSerial;
};
  
#endif // ESP32

#endif // JETIEXBUSESP32SERIAL_H