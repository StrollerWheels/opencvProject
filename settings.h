/**
 ******************************************************************************
 * @file           : settings.h
 * @brief          **File with project settings** 
  ******************************************************************************
 * @attention 
 ******************************************************************************
  @authors [Novikov Andrey](https://t.me/AndreyNikolaevichPerm)
  @version 1.0
  @date 16.07.2024
 ******************************************************************************
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "protocol.h"

/* = SAFETY = ATTENTION = SAFETY = SAFETY = ATTENTION = SAFETY = SAFETY = ATTENTION = SAFETY = SAFETY = ATTENTION = SAFETY = SAFETY = ATTENTION = SAFETY = */
/** @name 
 * ### __SECURITY SETTINGS__
 * IMPORTANT SECURITY SETTINGS
 * @{
 */
//#define SAFETY_USE (0)
#define SAFETY_MAX_SIDEWAYS_M (0.21) ///< Maximum permissible sideways movement, in meters
#define SAFETY_MAX_ROTATION_ANGLE_RAD (18.f / DEGRES_IN_RAD) ///< Maximum permissible rotation angle (18 degrees), in radians
#define SAFETY_COUNT_ITERATION_MARKER_NOT_IDENTIFIED (2) ///< Number of iterations when the marker is not identified, at which the Alert
#define SAFETY_MAX_MARKER_MISSES (40.f / 100.f) ///< Maximum permissible percentage of marker misses
#define SAFETY_COUNT_ITERATION_TO_CALC_MISSES (3) ///< Number of iterations to calculate the percentage of marker misses
#define SAFETY_MISS_RATE_YAW_RAD (8.f / DEGRES_IN_RAD ) ///< 
#define SAFETY_TIME_TO_CALC_SETPOINT_SEC (20) ///< 
#define SAFETY_MIN_PERCENTAGE_FRAMES_TO_CALC_SETPOINT (67.f / 100.f) ///< Minimal permissible percentage of frames for setpoint calculation
#define SAFETY_MAX_COUNT_ITERATION_NO_COMMUNICATION_WCU (3) ///< Maximum permissible number ofiteration of no communication with the WCU
/// @}
#define DEBUG_ON (1)


constexpr double PI = 3.141592653589793;
constexpr double DEGRES_IN_RAD = 180 / PI;

constexpr float FLOAT_EPSILON = 0.00001;

const float MARKER_LENGTH = 0.15; ///< Marker side length
const int CAMERA_NUMBER = 1;     ///< Camera number in OS
const int NO_PIN_FORWARD = 0;
const int NO_PIN_BACK = 1;
const int NO_PIN_SHUTDOWN = 10;
const int SPI_CHANNEL = 1;
const int SPI_PORT = 0;
const int SPI_BAUDRATE = 100000;
const int SPI_MODE = 0;

const float COEF_PROPORTIONAL_YAW = 4.f;
const float COEF_INTEGRAL_YAW = 0.01;
const float COEF_DIFFERENTIAL_YAW = 0.f;
const float COEF_PROPORTIONAL_X = /***/1.f;
const float COEF_INTEGRAL_X = /***/0.004;
const float COEF_DIFFERENTIAL_X = 0.f;

const size_t COUNT_FRAMES_TO_CALC = 5;
const size_t COUNT_FRAMES_TO_CALC_TARGET = 50;
const size_t COUNT__OF_DATA_PACKET_SENDS = 7;

const size_t COUNT_MEASUREMENT_FOR_MOVING_AVG = 3;
constexpr float MISS_RATE_YAW_GRAD = /*400.999999999; /*/ 7.999999999;
constexpr float MISS_RATE_YAW_RAD = MISS_RATE_YAW_GRAD / DEGRES_IN_RAD;
constexpr float MISS_RATE_X_METER = /*100; /*/ 0.2;
constexpr float MISS_RATE_Z_METER = /*100;          /*/ 0.4;
constexpr float IMPOSSIBLE_YAW_X_Z_VALUE = 100.f; ///< Moving average is initiated by this value, impossible value
/** Value for conditional branching, just in case because of the specifics of storing float in memory */
constexpr float IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT = (IMPOSSIBLE_YAW_X_Z_VALUE - FLOAT_EPSILON); ///< Moving average is initiated by this value
const float TARGET_DISTANCE_VALUE_METER = TARGET_DISTANCE_TO_MARKER_AT_CLOSEST_POINT_METERS; /// @note Zoom in. when the camera will replace
const float MAX_RECOMMENDED_DISTANCE_METER = TARGET_DISTANCE_TO_MARKER_AT_CLOSEST_POINT_METERS + 0.25;
const float MINIMAL_DISTANCE_VALUE_METER = 0.61; /// @note Zoom in. when the camera will replace
const float MINIMAL_DISTANCE_VALUE_METER_TWO_SHIFT = 0.55; /// @note Zoom in. when the camera will replace

#endif // SETTINGS_H_