/* 
  Jeti Sensor EX Bus Telemetry C++ Library

  JetiExBusESP32Serial - serial implementation for
						 half duplex operation with ESP32
  ---------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/13/2018  created
  0.93   02/16/2018  ESP32 uart initialization changed
  0.94   02/17/2018  generic arduino HardwareSerial support for AtMega328PB
                     Connect UART1 to Pin 11/12 (TX/RX)

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

#ifndef JETIEXBUSARDUINOSERIAL_H
#define JETIEXBUSARDUINOSERIAL_H

#include "JetiExBusSerial.h"

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#if defined (__AVR_ATmega328PB__)

class JetiExBusArduinoSerial : public JetiExBusSerial
{
public:
	JetiExBusArduinoSerial(int comPort = 2);

	virtual void   begin(uint32_t baud, uint32_t format);
	virtual size_t write(const uint8_t *buffer, size_t size);
	virtual int    available(void) { return m_pSerial->available(); }
	virtual int    read(void) { return m_pSerial->read(); }

protected:
	HardwareSerial * m_pSerial;
	int m_comPort;
};
  
#endif // __AVR_ATmega328PB__

#endif // JETIEXBUSARDUINOSERIAL_H