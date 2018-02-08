/* 
  Jeti Sensor EX Telemetry C++ Library
  
  DemoSensor - get some demeo values for telemetry display
  --------------------------------------------------------------------
  
  Copyright (C) 2015 Bernd Wokoeck
  
  Version history:
  1.00   11/22/2015  created
  
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

#ifndef DEMOSENSOR_H
#define DEMOSENSOR_H

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

class DemoSensor
{
public:
  DemoSensor();

  long GetVoltage();
  long GetAltitude();
  long GetTemp();
  long GetClimb();
  long GetFuel();
  long GetRpm();

  long GetVal( int idx ); // values 7..18

protected:
  enum
  {
    NVALS = 12,
  };

  float volt;
  unsigned long tiVolt;

  float alt;
  unsigned long tiAlt;

  float temp;
  unsigned long tiTemp;

  float climb;
  unsigned long tiClimb;

  float fuel;
  unsigned long tiFuel;

  float rpm;
  unsigned long tiRpm;

  float vals[NVALS];
  unsigned long tiVals[NVALS];
};

#endif // DEMOSENSOR
