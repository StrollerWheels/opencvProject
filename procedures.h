/**
 ******************************************************************************
 * @file           : procedures.h
 * @brief          **Related function, header file** 
 * @details Related functions that do not affect the program logic
 ******************************************************************************
 * @attention 
 ******************************************************************************
  @authors [Novikov Andrey](https://t.me/AndreyNikolaevichPerm)
  @version 1.0
  @date 16.07.2024
 ******************************************************************************
 */
 
#ifndef PROCEDURES_H_
#define PROCEDURES_H_


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>


#define OUT


enum class TEnumRiscBehavior
{
  RISC_WRONG_INPUT_COMBINATION = 0,
  RISC_CAMERA_PROBLEM,
  RISC_INVALID_CAMERA_FILE,
  RISC_NOT_SETUP_SPI,
  RISC_CANNOT_SEND_SPI_PACKET,
  RISC_CANNOT_CALC_TRAGET,
  RISK_ALERT_SIDEWAYS,
  RISK_ALERT_ROTATION_ANGLE,
  RISK_ALERT_MARKER_NOT_IDENTIFIED,
  RISK_ALERT_MORE_MISSES,
  RISK_ALERT_NO_PACKET_FROM_WCU,
};


typedef struct
{
  float fYaw;
  float fX;
  float fZ;
  float fDistance;
}TTargetValues;




void vInitializationSystem(std::ofstream &xFileToSave, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients,
                                    OUT cv::VideoCapture &xCaptureFrame, OUT cv::Mat &xMarkerPoints);

void vRiscBehavior(TEnumRiscBehavior eErrorCode, std::string sError);

bool bCaptureFrame(cv::VideoCapture &xCapture, OUT cv::Mat &frame, size_t nAttempts);

bool bTelemetryCalculation(cv::Mat &xFrame, OUT double &yaw, OUT double &x, OUT double &z);

void vGetYawRollPitch(double q0, double qx, double qy, double qz, OUT double &yaw, OUT double &roll, OUT double &pitch);

void vDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, float &fMovingCorrelatedAvgX, float &fMovingAvgZ, std::ofstream &xFileToSave,
                    float &fCoefRot, float &fCoefTransl, int16_t ssCoefShift, float fTargetYaw, float fTargetX);

void vMyCout (char *str);

bool bSendPacketToStroller(uint8_t ucId, float &fCoefRotation, float &fCoefTranslation, int16_t ssShift, float fDistance, OUT float &fPeriod, OUT float &fAmplitude);

#endif // PROCEDURES_H_                      