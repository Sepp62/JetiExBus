/* 
  Jeti Sensor EX Telemetry C++ Library
  
  JetiExSerial - EX Bus serial I/O implementation
  --------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/08/2018  created
                     !!! read TeensyReadme.txt !!!

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

#ifndef JETIEXBUSSERIAL_H
#define JETIEXBUSSERIAL_H

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

class JetiExBusSerial
{
public:
  static JetiExBusSerial * CreatePort( int comPort ); // comPort: 0=default, Teensy: 1..3

  virtual void   begin(uint32_t baud, uint32_t format) = 0;
  virtual size_t write(const uint8_t *buffer, size_t size) = 0;
  virtual int    available(void) = 0;
  virtual int    read(void) = 0;
};

// Teensy
/////////
#if defined ( CORE_TEENSY )

  class JetiExBusTeensySerial : public JetiExBusSerial
  {
  public:
    JetiExBusTeensySerial( int comPort = 2 );

	virtual void   begin(uint32_t baud, uint32_t format) { m_pSerial->begin( baud, format ); }
	virtual size_t write(const uint8_t *buffer, size_t size) { return m_pSerial->write(buffer, size); }
    virtual int    available(void) { return m_pSerial->available(); }
	virtual int    read(void) { return m_pSerial->read(); }
  
  protected:
    HardwareSerial * m_pSerial;
  };

  
#endif // CORE_TEENSY

#endif // JETIEXBUSSERIAL_H