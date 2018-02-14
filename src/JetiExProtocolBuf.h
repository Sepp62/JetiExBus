/* 
  Jeti Sensor EX Telemetry C++ Library
  
  JetiExProtocolBuf - EX protocol buffer implementation 
  -----------------------------------------------------
  
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
#ifndef JETIEXPROTOCOLBUF_H
#define JETIEXPROTOCOLBUF_H

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#ifndef ESP32
  #include <new.h>
#endif 

// Definition of Jeti sensor (aka "Equipment")

// constant data
////////////////
typedef struct /*_JetiSensorConst*/
{
  uint8_t id;
  char    text[20];
  char    unit[7];
  uint8_t dataType;
  uint8_t precision;
}
JetiSensorConst;
typedef const JetiSensorConst JETISENSOR_CONST; 


// dynamic sensor data
//////////////////////
class JetiValue
{
  friend class JetiSensor;
  friend class JetiExProtocolBuf;
public:

  JetiValue() : m_value( -1 ) {}

protected:
  // value
  int32_t m_value;
};

// complete data for a sensor to fill ex frame buffer
/////////////////////////////////////////////////////
class JetiExProtocolBuf;
class JetiSensor
{
public:
  // Jeti data types
  enum enDataType
  {
    TYPE_6b   = 0, // int6_t  Data type 6b (-31 ¸31)
    TYPE_14b  = 1, // int14_t Data type 14b (-8191 ¸8191)
    TYPE_22b  = 4, // int22_t Data type 22b (-2097151 ¸2097151)
    TYPE_DT   = 5, // int22_t Special data type – time and date
    TYPE_30b  = 8, // int30_t Data type 30b (-536870911 ¸536870911) 
    TYPE_GPS  = 9, // int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree. 
  }
  EN_DATA_TYPE;

  JetiSensor( int arrIdx, JetiExProtocolBuf * pProtocol );

  // sensor id
  uint8_t m_id;

  // value
  int32_t m_value;

  // value
  uint8_t m_bActive;

  // label/description of value
  uint8_t m_label[ 20 ];
  uint8_t m_textLen;
  uint8_t m_unitLen;

  // format
  uint8_t  m_dataType;
  uint8_t  m_precision; 
  uint8_t  m_bufLen;

  // helpers
  void    copyLabel( const uint8_t * text, const uint8_t * unit,  uint8_t * label, int label_size, uint8_t * textLen, uint8_t * unitLen );
  uint8_t jetiCopyLabel( uint8_t * exbuf, uint8_t n );
  uint8_t jetiEncodeValue( uint8_t * exbuf, uint8_t n );
};

// Definition of Jeti EX protocol
/////////////////////////////////
class JetiExProtocolBuf
{
  friend class JetiSensor;
public:

  JetiExProtocolBuf();

  void Init( const char * name,  JETISENSOR_CONST * pSensorArray ); 

  void SetDeviceId( uint8_t idLo, uint8_t idHi ) { m_devIdLow = idLo; m_devIdHi = idHi; } // adapt it, when you have multiple sensor devices connected to your REX
  void SetSensorValue( uint8_t id, int32_t value );
  void SetSensorValueGPS( uint8_t id, bool bLongitude, float value );
  void SetSensorValueDate( uint8_t id, uint8_t day, uint8_t month, uint16_t year );
  void SetSensorValueTime( uint8_t id, uint8_t hour, uint8_t minute, uint8_t second );
  void SetSensorActive( uint8_t id, bool bEnable, JETISENSOR_CONST * pSensorArray );

protected:
  enum
  {
    MAX_SENSORS   = 32, // increase up to 255 if necessary, 31 is max for DC16/DS/16

    MAX_SENSORBYTES = (MAX_SENSORS / 8) + ( MAX_SENSORS % 8 ? 1:0)
  };

  uint8_t   SetupExFrame(uint8_t frameCnt, uint8_t * exBuffer);

  void InitSensorMapper( JETISENSOR_CONST * pSensorArray );

  // sensor name
  char               m_name[ 20 ];
  uint8_t            m_nameLen;

  // sensor array
  JETISENSOR_CONST * m_pSensorsConst;               // array to constant sensor definitions
  JetiValue        * m_pValues;                     // sensor value array, same order as constant data array
  int                m_nSensors;                    // number of sensors
  uint8_t            m_sensorIdx;                   // current index to sensor array to send value
  uint8_t            m_dictIdx;                     // current index to sensor array to send sensor dictionary
  uint8_t            m_sensorMapper[ MAX_SENSORS ]; // id to idx lookup table to accelerate SetSensorValue()
  uint8_t            m_activeSensors[ MAX_SENSORBYTES ]; // bit array for active sensor bit field

  //crypt and crc
  enum
  {
    // Jeti Duplex EX Ids: Manufacturer and device
    MANUFACTURER_ID_LOW = 0x09, // 0xA409 (Jeti recommended range for 3rd party sensors is 0xA400 – 0xA41F)
    MANUFACTURER_ID_HI  = 0xA4,
    DEVICE_ID_LOW       = 0x76, // random number: 0x3276
    DEVICE_ID_HI        = 0x32,
    POLY                = 0x07, // constant for for "crypt"
  };
  uint8_t m_devIdLow;
  uint8_t m_devIdHi;
  uint8_t update_crc (uint8_t crc, uint8_t crc_seed);
  uint8_t jeti_crc8(uint8_t *exbuf, unsigned char framelen);
};

#endif // JETIEXPROTOCOLBUF_H
