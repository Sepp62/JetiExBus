/* 
  Jeti EX Bus C++ Library
  
  JetiExBus - EX Bus protocol implementation for Teensy 3.x
  -------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/04/2018  created
  
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
#ifndef JETIEXBUS_H
#define JETIEXBUS_H

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "JetiExProtocolBuf.h"
#include "JetiExBusSerial.h"

#define JETIEXBUS_COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x]))))) // number of elements in an array

class JetiExBusProtocol : public JetiExProtocolBuf
{
public:
	JetiExBusProtocol();

	void    Start( const char * name, JETISENSOR_CONST * pSensorArray, int comPort = 2 );   // call once in setup(), comPort: 0=Default, Teensy: 1..3
	void    DoJetiExBus();              // call periodically in loop()

	uint32_t HasNewChannelData() { bool b = m_bNewChannelData; m_bNewChannelData = false; return b; }
	uint8_t  GetNumChannels() { return m_nNumChannels; }
	uint16_t GetChannel(uint8_t nChannel);
	
	void     SetJetiboxText(int lineNo, const char* text);
	uint8_t  GetJetiboxKey() { uint8_t b = m_nButtons; m_nButtons = 0; return b; }

protected:

	JetiExBusSerial * m_pSerial;

	// packet processing
	enum enPacketState
	{
		WAIT_HDR_START = 0,
		WAIT_HDR_TYPE = 1,
		WAIT_LEN = 2,
		WAIT_END = 3,
	};

	uint8_t		  m_exFrameCnt;

	enPacketState m_state;
	bool          m_bChannelData;
	bool          m_bReleaseBus;
	uint8_t       m_nPacketLen;
	uint8_t       m_nBytes;
	uint8_t       m_nPacketId;
	uint8_t       m_exBusBuffer[64];  // 7 bytes header + 2 bytes crc + 24*2bytes channels
	void ResetPacket() { m_state = WAIT_HDR_START; m_bChannelData = false;  m_bReleaseBus = false;  m_nPacketLen = 0; m_nBytes = 0; m_nPacketId = 0; }

	// channel and button data
	uint8_t  m_nButtons;
	bool     m_bNewChannelData;
	uint8_t  m_nNumChannels;
	uint16_t m_channelValues[24];

	// jetibox text buffer
	char m_textBuffer[32];

	// helpers
	void DecodeChannelData();
	void SendJetiBoxData();
	void SendTelemetryData();
	bool ReceiveCRCCheck();
	uint16_t crc_ccitt_update(uint16_t crc, uint8_t data);

	// debug 
	void DumpPacket();
    void DumpChar(char c);
};


#endif // JETIEXBUS_H