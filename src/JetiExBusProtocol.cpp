/* 
  Jeti EX Bus C++ Library
  
  JetiExBus - EX Bus protocol implementation for Teensy 3.x
  -------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/04/2018  created
  0.95   03/17/2018  Synchronization (IsBusReleased) for time consuming operations

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

// #define JEXTIEXBUS_PROTOCOL_DEBUG

JetiExBusProtocol::JetiExBusProtocol() : m_pSerial(NULL), m_exFrameCnt(0), m_nButtons(0), m_bNewChannelData(false), m_nNumChannels(0), m_bBusReleased(false)
{
	ResetPacket();
	memset(m_channelValues, 0, sizeof(m_channelValues));
}

void JetiExBusProtocol::Start(const char * name, JETISENSOR_CONST * pSensorArray, int comPort)
{
	// init jetibox text memory
	memset(m_textBuffer, ' ', sizeof(m_textBuffer));

	// init EX protocol handler
	JetiExProtocolBuf::Init(name, pSensorArray);

	// init EX bus serial port 
	m_pSerial = JetiExBusSerial::CreatePort( comPort );
	m_pSerial->begin( 125000, SERIAL_8N1 );

	ResetPacket();
	m_exFrameCnt = 0;
}


uint16_t JetiExBusProtocol::GetChannel(uint8_t nChannel)
{
	if( nChannel < JETIEXBUS_COUNT_OF( m_channelValues ) )
	  return m_channelValues[ nChannel ];
	return 0;
}


void JetiExBusProtocol::DoJetiExBus()
{
	// master header data
	// 0x3x 0x01 --> release bus
	// 0x3x 0x03 --> keep allocated
	// 0x3e 0x0x --> channel data
	// 0x3d 0x01 --> EX telemetry or jetibox request
	while (m_pSerial->available())
	{
		int c = m_pSerial->read();
		 // DumpChar( (char)c );

		if (m_state == WAIT_HDR_START)
		{
			if (c == 0x3d || c == 0x3e)
			{
				m_state = WAIT_HDR_TYPE;
				m_bChannelData = (c == 0x3e) ? true : false;
				m_exBusBuffer[0] = c;
				// Serial.println("start");
			}
		}
		else if (m_state == WAIT_HDR_TYPE)
		{
			if (c == 0x01 || c == 0x03 )
			{
				m_state = WAIT_LEN;
				m_bReleaseBus = (c == 0x01) ? true : false;
				m_exBusBuffer[1] = c;
				// Serial.println("type");
			}
			else
			{
				m_state = WAIT_HDR_START; // --> Error
			}
		}
		else if ( m_state == WAIT_LEN)
		{
			m_state = WAIT_END;
			m_nPacketLen = (uint8_t)c;
			m_exBusBuffer[2] = c;
			m_nBytes = 3;
			if( m_nPacketLen > sizeof( m_exBusBuffer) )
			{
				m_state = WAIT_HDR_START; // --> Error
			}
			// Serial.print("len - "); Serial.println( m_nPacketLen );
		}
		else if ( m_state == WAIT_END )
		{
			m_exBusBuffer[m_nBytes++] = c;
			if ( m_nBytes == m_nPacketLen )
			{
				if (ReceiveCRCCheck() )
				{
					//  DumpPacket();
					m_nPacketId = m_exBusBuffer[3];
					
					// packet contains channel data 
					if (m_bChannelData && ( m_exBusBuffer[4] == 0x31 ) )
					{
						DecodeChannelData();
					}
					// packet is a telemetry request
					else if( m_exBusBuffer[ 4 ] == 0x3a && m_bReleaseBus )
					{
						SendTelemetryData();
						m_bBusReleased = true;
					}
					// packet is a Jetibox request
					else if (m_exBusBuffer[4] == 0x3b && m_bReleaseBus )
					{
						SendJetiBoxData();
						m_bBusReleased = true;
					}
				}
				m_state = WAIT_HDR_START;
			}
		}
	}
}


void JetiExBusProtocol::DecodeChannelData()
{
	// Serial.print("c");

	m_nNumChannels = m_exBusBuffer[5] / 2;  // number of channels
	
	uint16_t * pChannel = (uint16_t*)&m_exBusBuffer[6];  // first channel data position
	for (int i = 0; i < m_nNumChannels; i++)
		m_channelValues[i] = *(pChannel++);
	
	m_bNewChannelData = true;
}


void JetiExBusProtocol::SendJetiBoxData()
{
	// Serial.print("j");

	// store buttons
	m_nButtons = m_exBusBuffer[6]; 
	if (m_nButtons == 0xF0)
		m_nButtons = 0;
	
	// send jetibox packet
	m_nBytes = 40;
	m_exBusBuffer[0] = 0x3b;
	m_exBusBuffer[1] = 0x01;
	m_exBusBuffer[2] = m_nBytes;
	m_exBusBuffer[3] = m_nPacketId;
	m_exBusBuffer[4] = 0x3b; // jetibox
	m_exBusBuffer[5] = 32;   // SUB_LEN
	memcpy(&m_exBusBuffer[6], m_textBuffer, 32 );

	uint16_t crcCalc = 0;
	for (int i = 0; i < m_nBytes - 2; i++)
		crcCalc = crc_ccitt_update(crcCalc, m_exBusBuffer[i]);

	m_exBusBuffer[38] = (uint8_t)(crcCalc & 0xFF);
	m_exBusBuffer[39] = (uint8_t)(crcCalc >> 8);

	// DumpPacket();

	m_pSerial->write(m_exBusBuffer, m_nBytes);
}


void JetiExBusProtocol::SendTelemetryData()
{
	// Serial.print("t");

	// copy telemetry data
	uint8_t len = SetupExFrame( m_exFrameCnt++, &m_exBusBuffer[6] );

	m_nBytes = 8 + len;
	m_exBusBuffer[0] = 0x3b;
	m_exBusBuffer[1] = 0x01;
	m_exBusBuffer[2] = m_nBytes;
	m_exBusBuffer[3] = m_nPacketId;
	m_exBusBuffer[4] = 0x3a; // telemetry data
	m_exBusBuffer[5] = len;  // SUB_LEN

	// crc
	uint16_t crcCalc = 0;
	for (int i = 0; i < m_nBytes - 2; i++)
		crcCalc = crc_ccitt_update(crcCalc, m_exBusBuffer[i]);

	m_exBusBuffer[m_nBytes - 2] = (uint8_t)(crcCalc & 0xFF);
	m_exBusBuffer[m_nBytes - 1] = (uint8_t)(crcCalc >> 8);
	
    // DumpPacket();

	m_pSerial->write(m_exBusBuffer, m_nBytes);

}

bool JetiExBusProtocol::ReceiveCRCCheck()
{
	// calc crc...
	uint16_t crcCalc = 0;
	for (int i = 0; i < m_nBytes-2; i++)
		crcCalc = crc_ccitt_update(crcCalc, m_exBusBuffer[i]);

	// ...and compare with crc from packet
	uint16_t crcPacket  = m_exBusBuffer[m_nBytes - 1] << 8;
	         crcPacket |= m_exBusBuffer[m_nBytes - 2];

	return (crcCalc == crcPacket);
}


// Jeti Box
///////////
void JetiExBusProtocol::SetJetiboxText(int lineNo, const char* text)
{
	if (text == 0)
		return;

	char * pStart = 0;
	switch (lineNo)
	{
	default:
	case 0: pStart = m_textBuffer; break;
	case 1: pStart = m_textBuffer + 16; break;
	}

	bool bPadding = false;
	for (int i = 0; i < 16; i++)
	{
		if (text[i] == '\0')
			bPadding = true;

		if (!bPadding)
			pStart[i] = text[i];
		else
			pStart[i] = ' ';
	}
}


// CRC calculation
//////////////////
uint16_t JetiExBusProtocol::crc_ccitt_update(uint16_t crc, uint8_t data)
{
	uint16_t ret_val;
	data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
	data ^= data << 4;
	ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
		       ^ (uint8_t)(data >> 4)
		       ^ ((uint16_t)data << 3));
	return ret_val;
}

//////////////////////
// debug 
//////////////////////
#ifdef JEXTIEXBUS_PROTOCOL_DEBUG
	void JetiExBusProtocol::DumpPacket()
	{
		Serial.println("");
		Serial.println("--- dump start ---");
		for (int i = 0; i < m_nBytes; i++)
			DumpChar(m_exBusBuffer[i]);
		Serial.println("");
		Serial.println("--- dump end ---");
	}

	void JetiExBusProtocol::DumpChar(char c)
	{
		char buf[5];
		int idx = 0;

		buf[idx++] = '0';
		buf[idx++] = 'x';
		itoa(((int)c) & 0x00FF, &buf[idx], 16);
		idx += 2;
		buf[idx++] = ' ';

		if (idx > 0 && idx < (int)sizeof( buf ) )
			buf[idx++] = '\0';

		Serial.println(buf);
	}
#else
  void JetiExBusProtocol::DumpPacket() {}
  void JetiExBusProtocol::DumpChar(char c) {}
#endif // JEXTIEXBUS_PROTOCOL_DEBUG