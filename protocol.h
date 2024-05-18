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

#define PREAMBULE_IN_STROLLER (0x5555)
#define PREAMBULE_IN_ARUCO (0x9999)
#define ID_PACKET_IN_STROLLER (0x01)
#define ID_PACKET_IN_ARUCO (0x02)
/// On reset the stroller assign s a field value "ucIdPacket" equal to this constant three packets in a row
#define ID_PACKET_RESET_WAS (0x03)


#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x5555```
  uint8_t ucIdPacket; ///< Идентификатор пакета (= 0x01)
  /// @note Rotation and translation coefficients, shift command start to be taken in the stroller driver at the closest point, phase PI/2
  float fRotation; ///< Rotation coefficient, absolute
  float fTranslation; ///< Translation coefficient, absolute
  /** @code
   * shiftValue = ssShift * (0.1 * Amplitude)
   * @endcode */
  int16_t ssShift; ///< Shift the skating path; >0 - forward, <0 - back
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInStroller;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x9999```
  /** Идентификатор пакета ответа (= 0x02);
  if there was a stroller reset, the ID will be @ID_PACKET_RESET_WAS three packets in a row */
  uint8_t ucIdPacket; ///< Идентификатор пакета ответа (= 0x02);
  float fPeriod; ///< Period, in seconds;
  float fAmplitude; ///< Amplitude, in meters
  uint16_t nHallError; ///< Count of Hall errors
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInAruco;
#pragma pack(pop)


#endif /* PROTOCOLARUCO_H_ */
