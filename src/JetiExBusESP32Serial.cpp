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

#if defined (ESP32)

#include "JetiExBusESP32Serial.h"

ESP32HardwareSerial Esp32Serial(2); // use port number 2

hw_timer_t *  _timer = NULL;
portMUX_TYPE  _timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool _bTimerRunning = false;

void IRAM_ATTR _onTimer() {
	portENTER_CRITICAL_ISR(&_timerMux);
	timerAlarmDisable(_timer);
	_bTimerRunning = false;
	Esp32Serial.uartDetachTx();
	// digitalWrite(15, false);
	portEXIT_CRITICAL_ISR(&_timerMux);
}

JetiExBusSerial * JetiExBusSerial::CreatePort(int comPort)
{
	// pinMode(15, OUTPUT);
	// digitalWrite(15, false);

	return new JetiExBusESP32Serial(comPort);
}

JetiExBusESP32Serial::JetiExBusESP32Serial(int comPort) : m_pSerial(0)
{
	m_pSerial = &Esp32Serial;
	m_pSerial->uartDetachTx();
}


size_t JetiExBusESP32Serial::write(const uint8_t *buffer, size_t size)
{
	if( !_bTimerRunning )
	{
		portENTER_CRITICAL(&_timerMux);
		// https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
		_timer = timerBegin(0, 160, true);               // timer 0, prescaler 160 --> 80MHz/160 = 0,5 MHz = 2us, count up
		timerAttachInterrupt(_timer, &_onTimer, true);   // attach handler, trigger on edge 
		timerAlarmWrite(_timer, 2000L, true);            // 2000 * 2us = 4ms, single shot
		timerAlarmEnable(_timer); // start
		_bTimerRunning = true;
		m_pSerial->uartAttachTx();
		// digitalWrite(15, true);
		portEXIT_CRITICAL(&_timerMux);
	}
	return m_pSerial->write(buffer, size);
}


#endif // ESP32