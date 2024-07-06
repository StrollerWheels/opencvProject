/**
 ******************************************************************************
 * @file           : ProtocolAruco.h
 * @brief          **Протокол информационнго взаимодействия**
 * @details Протокол прикладного уровня взаимодействия между Блоком управления колёсами и Блоком ориентации
 * Физический и канальный уровень - SPI.
 * Блок управления колёсами (БУК) - плата stm32f4discovery
 * Блок ориентации (БО) - одноплатный компьютер OrangePi 3 LTS
 *
 * Блок управления колёсами (БУК) - Wheel control unit (WCU)
 * Блок ориентации (БО) - orientation unit (OU)
 * Packet preambule to WCU ```0x5555```, to OU ```0x9999```
 *
 * ***Communication algorithm***
 * GPIO.0 - closest point, GPIO.1 - far point
 * Orientation unit                           Wheel control unit
 *                              <------------ GPIO.0 = GPIO.1 = HIGH
 * TProtocolPullUpToMarker      ------------>
 * TProtocolInWcuReceiptRequest <===========> TProtocolInOuConfirmation
 *                             marker movement
 *                              <------------ GPIO.0 = HIGH ; GPIO.1 = LOW
 *                              <------------ GPIO.0 = GPIO.1 = HIGH
 * TProtocoInWcuPullUpToMarker  ------------>
 * TProtocolInWcuReceiptRequest <===========> TProtocolInOuConfirmation
 *                             marker movement
 *                              <------------ GPIO.0 = HIGH ; GPIO.1 = LOW
 *                              <------------ GPIO.0 = GPIO.1 = HIGH
 *                            target calculation
 * TProtocolInWcuStart          ------------>
 * TProtocolInWcuReceiptRequest <===========> TProtocolInOuConfirmation
 *                            far point movement
 *                              <------------ GPIO.0 = LOW ; GPIO.1 = HIGH
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 * TProtocolInWcuMotionCmd      <===========> TProtocolInOuCondition
 * TProtocolInWcuReceiptRequest <===========> TProtocolInOuConfirmation
 *                            closest point movement
 *                              <------------ GPIO.0 = HIGH ; GPIO.1 = LOW
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 *                            far point movement
 *                              <------------ GPIO.0 = LOW ; GPIO.1 = HIGH
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 * TProtocolInWcuMotionCmd      <===========> TProtocolInOuCondition
 * TProtocolInWcuReceiptRequest <===========> TProtocolInOuConfirmation
 * ...
 * ...
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


/** @name
 * ### Preambules and packets ID
 * @{
 */
#define PREAMBULE_IN_WCU (0x5555)
#define PREAMBULE_IN_OU (0x9999)
#define ID_PACKET_IN_WCU_MOTION_CMD (0x01)
#define ID_PACKET_IN_OU_CONDITION (0x11)
#define ID_PACKET_IN_WCU_PULL_UP_TO_MARKER (0x02)
#define ID_PACKET_IN_WCU_RECEIPT_REQUEST (0x03)
#define ID_PACKET_IN_OU_CONFIRMATION (0x13)
/// On reset the stroller assign s a field value "ucIdPacket" equal to this constant three packets in a row
#define ID_PACKET_IN_OU_RESET_WAS (0x03)
/// @}


/// Regular packet with a motion command. It is sent after the far point
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x5555```
  uint8_t ucIdPacket; ///< Идентификатор пакета (= 0x01)
  /** @note Rotation and translation coefficients,
  shift command start to be taken in the stroller driver at the closest point, phase PI/2 */
  float fRotation; ///< Rotation coefficient, absolute
  float fTranslation; ///< Translation coefficient, absolute
  /** @code
   * shiftValue = ssShift * (0.1 * Amplitude)
   * @endcode */
  int16_t ssShift; ///< Shift the skating path; >0 - forward, <0 - back
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInWcuMotionCmd;
#pragma pack(pop)


/// Response packet to a motion command packet
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x9999```, @ref PREAMBULE_IN_OU
  /** Идентификатор пакета ответа (= 0x11);
  if there was a stroller reset, the ID will be @ID_PACKET_RESET_WAS three packets in a row */
  uint8_t ucIdPacket; ///< Идентификатор пакета ответа (= 0x02);
  float fPeriod; ///< Period, in seconds;
  float fAmplitude; ///< Amplitude, in meters
  uint16_t nHallError; ///< Count of Hall errors
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInOuCondition;
#pragma pack(pop)


/** A packet with the distance you need to drive to the marker.
Only at the begginning of the skating is used */
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< @ref PREAMBULE_IN_WCU
  uint8_t ucIdPacket; ///< 0x02, @ref ID_PACKET_IN_WCU_PULL_UP_TO_MARKER
  float fDistance; ///< Distance in meters
  uint16_t crc16; ///< @ref crc16.c
}TProtocolPullUpToMarker;
#pragma pack(pop)


/// Package receipt request from
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< 0x5555, @ref PREAMBULE_IN_WCU
  uint8_t ucIdPacket; ///< 0x03, @ref ID_PACKET_IN_WCU_RECEIPT_REQUEST
  uint8_t ucPacketRequestId; ///< Unused. Needed for reply packet
  uint16_t crc16; ///< @ref crc16.c
}TProtocolInWcuReceiptRequest;
#pragma pack(pop)


// Package receipt request from
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< @ref PREAMBULE_IN_OU
  uint8_t ucIdPacket; ///< 0x13, @ref ID_PACKET_IN_OU_CONFIRMATION
  /** Reception result code.
  If the reception is successful, the received packet id; else 0xFF */
  uint8_t ucResultCode;
  uint16_t crc16; ///< @ref crc16.c
}TProtocolInOuConfirmation;
#pragma pack(pop)


#endif /* PROTOCOLARUCO_H_ */
