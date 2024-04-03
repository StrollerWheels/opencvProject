#include <iostream>

#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x5555```
  uint8_t ucIdPacket; ///< Идентификатор пакета (пока один id = 0x01)
  float fRotation; ///< Rotation coefficient, absolute
  float fTranslation; ///< Translation coefficient, absolute
  /** OrangePi is increased marker, in next response packet
      from stroller control unit it is repeated in same field */
  uint16_t ucMarker; 
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInStroller;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x9999```
  uint8_t ucIdPacket; ///< Идентификатор пакета ответа (пока один id = 0x02)
  float fPeriod; ///< Period, in seconds
  float fAmplitude; ///< Amplitude, in meters
  /** OrangePi is increased marker, in next response packet
      from stroller control unit it is repeated in same field */
  uint16_t ucMarker;
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInOrangePi;
#pragma pack(pop)