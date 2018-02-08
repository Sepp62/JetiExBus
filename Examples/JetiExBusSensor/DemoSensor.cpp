/* 
  Jeti Sensor EX Telemetry C++ Library
  
  JetiExSerial - EX serial output implementation
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

#include "DemoSensor.h"

  /*
  jetiEx.SetSensorValue( ID_VOLTAGE,   81 );  // 8.1 volt
  jetiEx.SetSensorValue( ID_ALTITUDE, 250 );  // 250 m
  jetiEx.SetSensorValue( ID_TEMP,     300 );  // 300 degrees celsius
  jetiEx.SetSensorValue( ID_CLIMB,    153 );  // 1.53 m/s
  jetiEx.SetSensorValue( ID_FUEL,     80 );   // 80 %
  jetiEx.SetSensorValue( ID_RPM,      5000 ); // 5000/min
  */


DemoSensor::DemoSensor() : volt( 81 ), tiVolt( 0 ), alt( 250 ), tiAlt( 0 ), temp( 300 ), tiTemp( 0 ),
                           climb( 153 ), tiClimb( 0 ), fuel( 80 ), tiFuel( 0 ), rpm( 100 ), tiRpm( 0 ) 
{
  for( int i = 1; i <= NVALS; i++ )
    vals[i-1] = 10*i;
  memset( tiVals, 0, sizeof( tiVals) );  
}

long DemoSensor::GetVoltage()
{
  if( ( tiVolt + 100 ) <= millis() )
  {
    tiVolt = millis(); 
    volt -= .02;
    if( volt < 3.3 )
      volt = 81;
  }
  return volt;
}

long DemoSensor::GetAltitude()
{
  if( ( tiAlt + 100 ) <= millis() )
  {
    tiAlt = millis(); 
    alt -= 1;
    if( alt < 100 )
      alt = 250;
  }
  return alt;
}

long DemoSensor::GetTemp()
{
  if( ( tiTemp + 100 ) <= millis() )
  {
    tiTemp = millis(); 
    temp += 1;
    if( temp > 700 )
      temp = 300;
  }
  return temp;
}
  
long DemoSensor::GetClimb()
{
  if( ( tiClimb + 100 ) <= millis() )
  {
    tiClimb = millis(); 
    climb += .01;
    if( climb > 260 )
      climb = 153;
  }
  return climb;
}
  
long DemoSensor::GetFuel()
{
  if( ( tiFuel + 100 ) <= millis() )
  {
    tiFuel = millis(); 
    fuel -= .05;
    if( fuel < 5.0 )
      fuel = 80;
  }
  return fuel;
}

long DemoSensor::GetRpm()
{
  if( ( tiRpm + 100 ) <= millis() )
  {
    tiRpm = millis(); 
    rpm += 3;
    if( rpm > 900 )
      rpm = 100;
  }
  return rpm;
}


// values 7..18 --> idx 0..12
long DemoSensor::GetVal( int idx )
{
  if( ( tiVals[idx] + 100 ) <= millis() )
  {
    tiVals[idx] = millis(); 
    vals[idx] += .05;
    if( vals[idx] > ((idx+1)*10 + 9.9) )
      vals[idx] = (idx+1)*10;
  }
  return vals[idx];
}
