/**
 ******************************************************************************
 * @file           : ProtocolAruco.h
 * @brief          **Протокол информационнго взаимодействия**
 * @details Протокол прикладного уровня взаимодействия между Блоком управления коляской и Блоком ориентации
 * Физический и канальный уровень - SPI.
 * Блок управления коляской (БУК) - плата stm32f4discovery
 * Блок ориентации (БО) - одноплатный компьютер OrangePi 3 LTS
 ******************************************************************************
 * @attention
 * - **Протокол должен быть синхронизирован и в БУК и в БО**
 * - **Необходимо учитывать, чтобы файл одинакого компилировался GCC GNU11 и G++ 11.04**
 ******************************************************************************
  @authors [Novikov Andrey](https://t.me/AndreyNikolaevichPerm)
  @version 1.0
  @date 30.04.2024
 ******************************************************************************
 */

#ifndef PROTOCOLARUCO_H_
#define PROTOCOLARUCO_H_

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x5555```
  uint8_t ucIdPacket; ///< Идентификатор пакета (пока один id = 0x01)
  float fRotation; ///< Rotation coefficient, absolute
  float fTranslation; ///< Translation coefficient, absolute
  /** OrangePi is increased marker, in next response packet
      from stroller control unit it is repeated in same field */
  uint16_t usMarker; 
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
  uint16_t usMarker;
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInAruco;
#pragma pack(pop)


#endif /* PROTOCOLARUCO_H_ */