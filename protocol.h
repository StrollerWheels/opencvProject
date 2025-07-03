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
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 *                              <------------ GPIO.0 = GPIO.1 = HIGH
 *                    target calculation
 * TProtocolInScuMotionCmd      <===========> TProtocolInOuConfirmation
 *                                    far point movement
 *                              <------------ GPIO.0 = LOW ; GPIO.1 = HIGH
 *                        frame capture
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 * TProtocolInScuMotionCmd      <===========> TProtocolInOuCondition
 *                                     closest point movement
 *                              <------------ GPIO.0 = HIGH ; GPIO.1 = LOW
 *                        frame capture
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 *                                     far point movement
 *                              <------------ GPIO.0 = LOW ; GPIO.1 = HIGH
 *                              <------------ GPIO.0 = GPIO.1 = LOW
 * TProtocolInScuMotionCmd      <===========> TProtocolInOuCondition
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


#define MAX_SIZE_ARUCO_PACKET (sizeof(TProtocolInScuMotionCmd)) ///< Максимальный размер валидного пакета

/** @name
 * ### Packet preambules
 * @{
 */
#define PREAMBULE_IN_WCU (0x5555)
#define PREAMBULE_IN_OU (0x9999)
/// @}


/// Packet ID in Aruco protocol
#ifdef __cplusplus
#define ID_PACKET_IN_WCU_MOTION_CMD (0x01)              ///<
#define ID_PACKET_IN_WCU_IMPOSSIBLE_CALC_SETPOINT (0x81) ///<
#define ID_PACKET_IN_WCU_SIDEWAYS_DRIFT_EXCEEDED_PERMISSIBLE (0x82) ///<
#define ID_PACKET_IN_WCU_ROTATION_ANGLE_EXCEEDED_PERMISSIBLE (0x83) ///<
#define ID_PACKET_IN_WCU_MARKER_IS_GONE (0x84) ///<
#define ID_PACKET_IN_WCU_MANY_MISSES (0x85) ///<
#define ID_PACKET_IN_WCU_CAMERA_PROBLEM (0x86) ///<
#define ID_PACKET_IN_WCU_NO_PACKET_FROM_WCU (0x87)
#define ID_PACKET_IN_OU_CONDITION (0x11) ///<
#else
typedef enum
{
  ID_PACKET_IN_WCU_MOTION_CMD = 0x01,              ///<
  ID_PACKET_IN_WCU_IMPOSSIBLE_CALC_SETPOINT = 0x81,///<
  ID_PACKET_IN_WCU_SIDEWAYS_DRIFT_EXCEEDED_PERMISSIBLE = 0x82, ///<
  ID_PACKET_IN_WCU_ROTATION_ANGLE_EXCEEDED_PERMISSIBLE = 0x83, ///<
  ID_PACKET_IN_WCU_MARKER_IS_GONE = 0x84, ///<
  ID_PACKET_IN_WCU_MANY_MISSES = 0x85, ///<
  ID_PACKET_IN_WCU_CAMERA_PROBLEM = 0x86, ///<
	ID_PACKET_IN_WCU_NO_PACKET_FROM_WCU = 0x87, ///<
  ID_PACKET_IN_OU_CONDITION = 0x11, ///<
}TEnumeIdPAcketAruco;
#endif


/// Message to OU from WCU by USART
#ifdef __cplusplus
#define MSG_IN_OU_SHUTDOWN (0x01)
#else
typedef enum
{
  MSG_IN_OU_SHUTDOWN = 0x01,              ///<
}TEnumeMsgToOuUsart;
#endif




/// Regular packet with a motion command. It is sent after the far point
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x5555```
  /// Идентификатор пакета = ID_PACKET_IN_WCU_MOTION_CMD
  #ifdef __cplusplus
  uint8_t eIdPacketAruco;
  #else
  TEnumeIdPAcketAruco eIdPacketAruco;
  #endif
  /** @note Rotation and translation coefficients,
  shift command start to be taken in the stroller driver at the closest point, phase PI/2 */
  float fRotation; ///< Rotation coefficient, absolute
  float fTranslation; ///< Translation coefficient, absolute
  /** @code
   * shiftValue = ssShift * (0.1 * Amplitude)
   * @endcode */
  int16_t ssShift; ///< Shift the skating path; >0 - forward, <0 - back
  float fDistance; ///< Marker distance, in meters
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInScuMotionCmd;
#pragma pack(pop)


/// Response packet to a motion command packet
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x9999```, @ref PREAMBULE_IN_OU
  /// Идентификатор пакета = ID_PACKET_IN_WCU_MOTION_CMD
  #ifdef __cplusplus
  uint8_t eIdPacketAruco;
  #else
  TEnumeIdPAcketAruco eIdPacketAruco;
  #endif
  float fPeriod; ///< Period, in seconds;
  float fAmplitude; ///< Amplitude, in meters
  uint16_t nHallError; ///< Count of Hall errors
  float fReserve; ///< Reserve field
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TProtocolInOuCondition;
#pragma pack(pop)


/// Message from WCU to OU by USART
#pragma pack(push,1)
typedef struct
{
  uint16_t usPreambule; ///< Преамбула пакета, ```= 0x9999```, @ref PREAMBULE_IN_OU
  /// Идентификатор пакета = ID_PACKET_IN_WCU_MOTION_CMD
  #ifdef __cplusplus
  uint8_t eMsgToOuUsart;
  #else
  TEnumeMsgToOuUsart eMsgToOuUsart;
  #endif
  uint16_t crc16; ///< Контрольная сумма, см. @ref crc16.c
}TMessageInOuByUsart;
#pragma pack(pop)



#endif /* PROTOCOLARUCO_H_ */
