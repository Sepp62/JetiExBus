/*
Jeti Sensor EX Telemetry C++ Library

JetiExProtocolBuf - EX protocol implementation for EX Bus
------------------------------------------------------------

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

#include "JetiExProtocolBuf.h"

// JetiSensor work data
///////////////////////
JetiSensor::JetiSensor( int arrIdx, JetiExProtocolBuf * pProtocol )
  : m_id( 0 ), m_value( -1 ), m_bActive( true ), m_textLen( 0 ), m_unitLen( 0 ), m_dataType( 0 ), m_precision( 0 ), m_bufLen( 0 )
{
  // sensor state
  m_bActive = (pProtocol->m_activeSensors[arrIdx >> 3] & (1 << (arrIdx & 7))) ? true : false;
  if( !m_bActive )
    return;

  // copy constant data 
  JetiSensorConst constData;
  memcpy_P( &constData, &pProtocol->m_pSensorsConst[ arrIdx ], sizeof(JetiSensorConst) );

  m_dataType = constData.dataType; 
  m_id       = constData.id;

  // value
  m_value = pProtocol->m_pValues[ arrIdx ].m_value;

  // copy to combined sensor/value buffer
  copyLabel( (const uint8_t*)constData.text, (const uint8_t*)constData.unit, m_label, sizeof( m_label ), &m_textLen, &m_unitLen );

  // 0...2 decimal places
  switch( constData.precision )
  {
  case 1: m_precision = 0x20; break; 
  case 2: m_precision = 0x40; break; 
  }

  // set needed space in EX frame buffer
  switch( m_dataType )
  {
  case TYPE_6b:  m_bufLen = 2; break;  //  1 byte id and data type + 1 byte value (incl. sign and prec)
  case TYPE_14b: m_bufLen = 3; break;  //  1 byte id and data type + 2 byte value (incl. sign and prec)
  case TYPE_22b: m_bufLen = 4; break;  //  1 byte id and data type + 3 byte value (incl. sign and prec)
  case TYPE_DT:  m_bufLen = 4; break;  //  1 byte id and data type + 3 byte value
  case TYPE_30b: m_bufLen = 5; break;  //  1 byte id and data type + 4 byte value
  case TYPE_GPS: m_bufLen = 5; break;  //  1 byte id and data type + 4 byte value
  }
}

// JetiExProtocolBuf
/////////////////
JetiExProtocolBuf::JetiExProtocolBuf() :
  m_nameLen( 0 ), m_pSensorsConst( 0 ), m_pValues( 0 ), m_nSensors( 0 ), m_sensorIdx( 0 ), m_dictIdx( 0 ), 
  m_devIdLow( DEVICE_ID_LOW ), m_devIdHi( DEVICE_ID_HI )
{
  m_name[0] = '\0';
  memset( m_activeSensors, 255, sizeof(m_activeSensors) ); // default: all sensors active
  memset( m_sensorMapper, 0, sizeof( m_sensorMapper ) );
}

void JetiExProtocolBuf::Init( const char * name, JETISENSOR_CONST * pSensorArray )
{
  // call it once only !
  if( m_nameLen != 0 )
    return;

  // sensor name
  strncpy( m_name, name, sizeof( m_name ) - 1 );
  m_nameLen = strlen( name );

  // map sensor values
  if( m_nSensors == 0 ) // dont do it more than once
    InitSensorMapper( pSensorArray );

  // init sensor value array
  m_pValues = new JetiValue[ m_nSensors ];

  // reset state machine
  m_sensorIdx = m_dictIdx = 0;
}

void JetiExProtocolBuf::SetSensorValue( uint8_t id, int32_t value )
{
  if( m_pValues && id < sizeof( m_sensorMapper ) )
    m_pValues[ m_sensorMapper[ id ] ].m_value = value;
}

void JetiExProtocolBuf::SetSensorValueGPS( uint8_t id, bool bLongitude, float value )
{
  // Jeti doc: If the lowest bit of a decimal point (Bit 5) equals log. 1, the data represents longitude. According to the highest bit (30) of a decimal point it is either West (1) or East (0).
  // Jeti doc: If the lowest bit of a decimal point (Bit 5) equals log. 0, the data represents latitude. According to the highest bit (30) of a decimal point it is either South (1) or North (0).
  // Byte 0: lo of minute, Byte 1: hi of minute, Byte 2: lo von degree, Byte 3: hi of degree 
  union
  {
    int32_t vInt;
    char    vBytes[4];
  } gps;

  // i.e.:
  // E 11° 33' 22.176" --> 11.55616 --> 11° 33.369' see http://www.gpscoordinates.eu/convert-gps-coordinates.php
  // N 48° 14' 44.520" --> 48.24570 --> 48° 14.742'
  float deg, frac = modff( value, &deg );
  uint16_t deg16 = (uint16_t)fabs( deg );
  uint16_t min16 = (uint16_t)fabs( frac * 0.6f * 100000 );
  gps.vInt = 0;
  gps.vBytes[0]  = min16 & 0xFF;
  gps.vBytes[1]  = ( min16 >> 8 ) & 0xFF;
  gps.vBytes[2]  = deg16 & 0xFF;                      // degrees 0..255
  gps.vBytes[3]  = ( deg16 >> 8 ) & 0x01;             // degrees 256..359
  gps.vBytes[3] |= bLongitude  ? 0x20 : 0;
  gps.vBytes[3] |= (value < 0) ? 0x40 : 0;
  
  SetSensorValue( id, gps.vInt );
}

void JetiExProtocolBuf::SetSensorValueDate( uint8_t id, uint8_t day, uint8_t month, uint16_t year )
{
  // Jeti doc: If the lowest bit of a decimal point equals log. 1, the data represents date
  // Jeti doc: (decimal representation: b0-7 day, b8-15 month, b16-20 year - 2 decimals, number 2000 to be added).
  // doc seems to be wrong, this is working: b0-b7 year, b16-b20: day
  union
  {
    int32_t vInt;
    char    vBytes[4];
  } date;

  if( year >= 2000 )
    year -= 2000;

  date.vInt = 0;
  date.vBytes[0]  = year;
  date.vBytes[1]  = month;
  date.vBytes[2]  = day & 0x1F;
  date.vBytes[2] |= 0x20;
  
  SetSensorValue( id, date.vInt );
}

void JetiExProtocolBuf::SetSensorValueTime( uint8_t id, uint8_t hour, uint8_t minute, uint8_t second )
{
  // If the lowest bit of a decimal point equals log. 0, the data represents time
  // (decimal representation: b0-7 seconds, b8-15 minutes, b16-20 hours).
  union
  {
    int32_t vInt;
    char    vBytes[4];
  } date;

  date.vInt = 0;
  date.vBytes[0]  = second;
  date.vBytes[1]  = minute;
  date.vBytes[2]  = hour & 0x1F;
  
  SetSensorValue( id, date.vInt );
}

void JetiExProtocolBuf::SetSensorActive( uint8_t id, bool bEnable, JETISENSOR_CONST * pSensorArray )
{
  if( m_nSensors == 0 && pSensorArray ) // dont do it more than once
    InitSensorMapper( pSensorArray );

  if( id < sizeof( m_sensorMapper ) )
  {
    int idx = m_sensorMapper[ id ];
    if( bEnable )
      m_activeSensors[ idx >>3 ] |=   1 << (idx & 7);
    else
      m_activeSensors[ idx >>3 ] &= ~(1 << (idx & 7));
  }

  // restart sending dictionary
  m_sensorIdx = m_dictIdx = 0;
}

void JetiExProtocolBuf::InitSensorMapper( JETISENSOR_CONST * pSensorArray )
{ 
  // map sensor id to index to give quick access by sensor ID
  int i;
  m_nSensors = 0;
  m_pSensorsConst = pSensorArray;
  memset( m_sensorMapper, 0, sizeof( m_sensorMapper ) );
  for( i = 0; i < MAX_SENSORS; i++ )
  {
    // get sensor id and check for end of array
    JetiSensorConst sensorConst;
    memcpy_P( &sensorConst, &m_pSensorsConst[i], sizeof(sensorConst) );
    if( sensorConst.id == 0 )
      break;

    if( sensorConst.id < sizeof( m_sensorMapper ) )
      m_sensorMapper[ sensorConst.id ] = i;
    m_nSensors++;
  }
}


uint8_t JetiExProtocolBuf::SetupExFrame(uint8_t frameCnt, uint8_t * exBuffer )
{
	uint8_t n = 0;

	// sensor name in frame 0
	if (frameCnt == 0)
	{                                                                // sensor name
		exBuffer[1] = 0x00;  			                             // 2Bit packet type(0-3) 0x40=Data, 0x00=Text 
		exBuffer[7] = 0x00;                                          // 8Bit id 
		exBuffer[8] = m_nameLen << 3;                                // 5Bit description, 3Bit unit length (use one space character)
		memcpy(&exBuffer[9], m_name, m_nameLen);                     // copy label plus unit to ex buffer starting from pos 9
		n = m_nameLen + 9;
	}
	// sensor dictionary: use the first few frames with even numbers to transfer 
	else if (((frameCnt / 2) <= m_nSensors && (frameCnt % 2) == 0))
	{
		for (int nDict = 0; nDict < m_nSensors; nDict++)
		{
			JetiSensor sensor(m_dictIdx, this);
			if (++m_dictIdx >= m_nSensors)
				m_dictIdx = 0;

			if (sensor.m_bActive)
			{
				exBuffer[1] = 0x00;                                          // 2Bit packet type(0-3) 0x40=Data, 0x00=Text
				exBuffer[7] = sensor.m_id;  	                             // 8Bit id
				exBuffer[8] = (sensor.m_textLen << 3) | sensor.m_unitLen;	 // 5Bit description, 3Bit unit length 
				n = sensor.jetiCopyLabel(exBuffer, 9) + 9;                 // copy label plus unit to ex buffer starting from pos 9
				break;
			}
		}
	}
	// send EX values in all other frames
	else
	{
		int bufLen;
		int nVal =                                                             // count values 
		exBuffer[1] = 0x40;						                               // 2Bit Type(0-3) 0x40=Data, 0x00=Text
		n = 7;				                                                   // start at 8th byte in buffer

		do
		{
			bufLen = 0;                                                              // last value buffer length    
			JetiSensor sensor(m_sensorIdx, this);
			if (++m_sensorIdx >= m_nSensors)                                         // wrap index when array is at the end
				m_sensorIdx = 0;

			if (sensor.m_bActive && sensor.m_value != -1)                            // -1 is "invalid"
			{
				if (sensor.m_id > 15)
				{
					exBuffer[n++] = 0x0 | (sensor.m_dataType & 0x0F);               // sensor id > 15 --> put id to next byte
					exBuffer[n++] = sensor.m_id;
				}
				else
					exBuffer[n++] = (sensor.m_id << 4) | (sensor.m_dataType & 0x0F);  // 4Bit id, 4 bit data type (i.e. int14_t)

				bufLen = sensor.m_bufLen;
				n += sensor.jetiEncodeValue(exBuffer, n);
			}
			if (++nVal >= m_nSensors)                                                // dont send twice in a frame
				break;
		} while (n < (26 - bufLen));                                               // jeti spec says max 29 Bytes per buffer
	}

	// complete some more EX frame data
	exBuffer[0] = 0x2F;			// start of packet          
	exBuffer[1] |= n - 1;			// frame length to Byte 2, packet length omitting 0x7e and crc8
	exBuffer[2] = MANUFACTURER_ID_LOW; exBuffer[3] = MANUFACTURER_ID_HI;  // sensor ID
	exBuffer[4] = m_devIdLow;          exBuffer[5] = m_devIdHi;
	exBuffer[6] = 0x00;         // reserved (key for encryption)

								  // calculate crc
	uint8_t crc = 0;
	for (uint8_t c = 1; c < n; c++)
		crc = update_crc(exBuffer[c], crc);
	exBuffer[n] = crc;

	// return length
	return n + 1;
}


// **************************************
// Helpers
// **************************************
// merge name and unit and terminate with '\0'
void JetiSensor::copyLabel( const uint8_t * name, const uint8_t * unit,  uint8_t * dest, int dest_size, uint8_t * nameLen, uint8_t * unitLen )
{
  int maxchar = dest_size -1 ;

  uint8_t i = 0, j = 0;
  while( name[i] != '\0' && j < maxchar )
    dest[ j++ ] = name[ i++ ];
  *nameLen = i;

  i = 0;
  while( unit[i] != '\0' && j < maxchar )
    dest[ j++ ] = unit[ i++ ];
  *unitLen = i;

  dest[ j ] = '\0';
}

// encode sensor value to jeti ex format and copy to buffer
uint8_t JetiSensor::jetiEncodeValue( uint8_t * exbuf, uint8_t n )
{
  switch( m_dataType )
  {
  case TYPE_6b:
    exbuf[n]  = ( m_value & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );                   // 5 bit value and sign 
    exbuf[n] |= m_precision;                                                         // precision in bit 5/6 (0, 20, 40)
    return 1;
	
  case TYPE_14b:
    exbuf[n]      = m_value & 0xFF;                                                  // lo byte
    exbuf[n + 1]  = ( (m_value >> 8) & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );        // 5 bit hi byte and sign 
    exbuf[n + 1] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 2;

  case TYPE_22b:
    exbuf[n]      = m_value & 0xFF;                                                  // lo byte
    exbuf[n + 1]  = (m_value >> 8 ) & 0xFF;                                          // mid byte
    exbuf[n + 2]  = ( (m_value >> 16) & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );       // 5 bit hi byte and sign 
    exbuf[n + 2] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 3;

  case TYPE_DT:
    exbuf[n]      = m_value & 0xFF;                                                  // value has been prepared by SetSensorValueDate/Time 
    exbuf[n + 1]  = (m_value >> 8 ) & 0xFF;
    exbuf[n + 2]  = ( (m_value >> 16) & 0xFF) | ((m_value < 0) ? 0x80 :0x00 );
    return 3;

  case TYPE_30b:
    exbuf[n]      = m_value & 0xFF;                                                  // lo byte
    exbuf[n + 1]  = (m_value >> 8 ) & 0xFF;
    exbuf[n + 2]  = (m_value >> 16 ) & 0xFF;
    exbuf[n + 3]  = ( (m_value >> 24) & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );       // 5 bit hi byte and sign 
    exbuf[n + 3] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 4;

  case TYPE_GPS:
    exbuf[n]      = m_value & 0xFF;                                                  // value has been prepared by SetSensorValueGPS 
    exbuf[n + 1]  = (m_value >> 8 ) & 0xFF;                                          
    exbuf[n + 2]  = (m_value >> 16) & 0xFF;
    exbuf[n + 3]  = (m_value >> 24) & 0xFF;
    return 4;
  }
  return 0;
}

// copy sensor label to ex buffer
uint8_t JetiSensor::jetiCopyLabel( uint8_t * exbuf, uint8_t n )
{
  uint8_t i = 0;

  while( m_label[i] != '\0' )
    exbuf[ n++ ] = m_label[ i++ ];

  return( i ) ; // number of bytes copied
}


// **************************************
// Jeti helpers
// **************************************

// Published in "JETI Telemetry Protocol EN V1.06"
//* Jeti EX Protocol: Calculate 8-bit CRC polynomial X^8 + X^2 + X + 1
uint8_t JetiExProtocolBuf::update_crc (uint8_t crc, uint8_t crc_seed)
{
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

//* Calculate CRC8 Checksum over EX-Frame, Original code by Jeti
uint8_t JetiExProtocolBuf::jeti_crc8(uint8_t *exbuf, unsigned char framelen)
{
	uint8_t crc = 0;
	uint8_t c;

	for (c = 2; c<framelen; c++)
		crc = update_crc(exbuf[c], crc);
	return (crc);
}
