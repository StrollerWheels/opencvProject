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
 

enum class TEnumRiscBehavior
{
  RISC_WRONG_INPUT_COMBINATION = 0,
  RISC_CAMERA_PROBLEM,
  RISC_INVALID_CAMERA_FILE,
  RISC_NOT_SETUP_SPI,
  RISC_CANNOT_SEND_SPI_PACKET,
  RISC_CANNOT_CALC_TRAGET,
};


void vInitializationSystem(ofstream &xFileToSave, OUT Mat &xCameraMatrix, OUT Mat &xDistCoefficients,
                                    OUT VideoCapture &xCaptureFrame, OUT Mat &xMarkerPoints);

void vRiscBehavior(TEnumRiscBehavior eErrorCode, string sError);

bool bCaptureFrame(VideoCapture &xCapture, OUT Mat &frame, size_t nAttempts);

void vGetYawRollPitch(double q0, double qx, double qy, double qz, OUT double &yaw, OUT double &roll, OUT double &pitch);

void vDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, float &fMovingAvgZ, ofstream &xFileToSave,
                      float &fCoefRot, float &fCoefTransl, int16_t ssCoefShift, float fTargetYaw, float fTargetX);