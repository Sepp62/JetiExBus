
/* 
  Jeti  EX Bus C++ Library for Teensy 3.x
  -------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/04/2018  created
  0.91   02/09/2018  Support for AtMega32u4 added
  0.92   02/14/2018  Support for ESP32 added
  0.93   02/16/2018  ESP32 uart initialization changed

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

#include "JetiExBusProtocol.h"
#include "DemoSensor.h"

JetiExBusProtocol exBus;
DemoSensor        demoSensor;

enum
{
	ID_VOLTAGE = 1,
	ID_ALTITUDE,
	ID_TEMP,
	ID_CLIMB,
	ID_FUEL,
	ID_RPM,
	ID_GPSLON,
	ID_GPSLAT,
	ID_DATE,
	ID_TIME,
	ID_VAL11, ID_VAL12, ID_VAL13, ID_VAL14, ID_VAL15, ID_VAL16, ID_VAL17, ID_VAL18,
};

// sensor definition (max. 31 for DC/DS-16)
// name plus unit must be < 20 characters
// precision = 0 --> 0, precision = 1 --> 0.0, precision = 2 --> 0.00
JETISENSOR_CONST sensors[] PROGMEM =
{
	// id             name          unit         data type             precision 
	{ ID_VOLTAGE,    "Voltage",    "V",         JetiSensor::TYPE_14b, 1 },
	{ ID_ALTITUDE,   "Altitude",   "m",         JetiSensor::TYPE_14b, 0 },
	{ ID_TEMP,       "Temp",       "\xB0\x43",  JetiSensor::TYPE_14b, 0 }, // °C
	{ ID_CLIMB,      "Climb",      "m/s",       JetiSensor::TYPE_14b, 2 },
	{ ID_FUEL,       "Fuel",       "%",         JetiSensor::TYPE_14b, 0 },
	{ ID_RPM,        "RPM x 1000", "/min",      JetiSensor::TYPE_14b, 1 },

	{ ID_GPSLON,     "Longitude",  " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_GPSLAT,     "Latitude",   " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_DATE,       "Date",       " ",         JetiSensor::TYPE_DT,  0 },
	{ ID_TIME,       "Time",       " ",         JetiSensor::TYPE_DT,  0 },

	{ ID_VAL11,      "V11",        "U11",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL12,      "V12",        "U12",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL13,      "V13",        "U13",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL14,      "V14",        "U14",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL15,      "V15",        "U15",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL16,      "V16",        "U16",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL17,      "V17",        "U17",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL18,      "V18",        "U18",       JetiSensor::TYPE_14b, 0 },
	{ 0 } // end of array
};


void setup()
{
	Serial.begin( 115200 );
	while (!Serial)
		;

	exBus.SetDeviceId(0x76, 0x32); // 0x3276
	exBus.Start("EX Bus", sensors, 2 );

	exBus.SetJetiboxText(0, "Line1");
	exBus.SetJetiboxText(1, "Line2");
}

void loop()
{
	char buf[20];

	// channel data
	if (false)
	//	 if ( exBus.HasNewChannelData() )
	{
		int i;
		for (i = 0; i < exBus.GetNumChannels(); i++)
		{
			sprintf(buf, "chan-%d: %d", i, exBus.GetChannel(i));
			Serial.println(buf);
		}
	}
	// get JETI buttons
	uint8_t bt = exBus.GetJetiboxKey();
	if( bt )
	{
		Serial.print( "bt - "); Serial.println(bt);
    }

	// sensor data
	exBus.SetSensorValue(ID_VOLTAGE, demoSensor.GetVoltage());
	exBus.SetSensorValue(ID_ALTITUDE, demoSensor.GetAltitude());
	exBus.SetSensorValue(ID_TEMP, demoSensor.GetTemp());
	exBus.SetSensorValue(ID_CLIMB, demoSensor.GetClimb());
	exBus.SetSensorValue(ID_FUEL, demoSensor.GetFuel());
	exBus.SetSensorValue(ID_RPM, demoSensor.GetRpm());

	exBus.SetSensorValueGPS(ID_GPSLON, true, 11.55616f); // E 11° 33' 22.176"
	exBus.SetSensorValueGPS(ID_GPSLAT, false, 48.24570f); // N 48° 14' 44.520"
	exBus.SetSensorValueDate(ID_DATE, 29, 12, 2015);
	exBus.SetSensorValueTime(ID_TIME, 19, 16, 37);

	exBus.SetSensorValue(ID_VAL11, demoSensor.GetVal(4));
	exBus.SetSensorValue(ID_VAL12, demoSensor.GetVal(5));
	exBus.SetSensorValue(ID_VAL13, demoSensor.GetVal(6));
	exBus.SetSensorValue(ID_VAL14, demoSensor.GetVal(7));
	exBus.SetSensorValue(ID_VAL15, demoSensor.GetVal(8));
	exBus.SetSensorValue(ID_VAL16, demoSensor.GetVal(9));
	exBus.SetSensorValue(ID_VAL17, demoSensor.GetVal(10));
	exBus.SetSensorValue(ID_VAL18, demoSensor.GetVal(11));

	// run protocol state machine
	exBus.DoJetiExBus();
}
