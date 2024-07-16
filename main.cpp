#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <fstream>
#include <omp.h>
#include <fcntl.h>
#include <stdlib.h>
#include <queue>
#include <vector>
#include <cmath>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "settings.h"
#include "crc.h"
#include "protocol.h"
#include "procedures.h"

// 3 - All_save_in_file
#define DEBUG_SOFT (5)

#define OUT

#define TARGET_IS_CURRENT_POSITION


enum class TEnumStatePosition
{
  STATE_NONE = 0,
  STATE_FORWARD,
  STATE_BACK,
};


using namespace std;
using namespace cv;

ofstream xFileToSave;                  ///< Debugging telemetry recording
Mat xCameraMatrix, xDistCoefficients;  ///< Camera calibration settings
VideoCapture xCaptureFrame;            ///< Object to capture a frame
cv::Mat xMarkerPoints(4, 1, CV_32FC3); ///< Coordinates of marker corners relative to the marker center
uint32_t nMeasurement(0);              // Count of measurement

#ifdef DEBUG_SOFT
float fCoefTranslationDebug(0.f), fCoefRotationDebug(0.f); ///< To display in the terminal when debugging
int16_t ssCoefShiftDebug(0.f);
#endif

static vector<float> xAvgPeriodYaw_(0);                             // Vector of yaw moving average for one period, is calculated at the far point
static vector<float> xAvgPeriodX_(0);                               // Vector of X moving average for one period, is calculated at the far point
static vector<float> xAvgPeriodZ_(0);                               // Vector of Z moving average for one period, is calculated at the far point
static float fMovingAvgYaw_(IMPOSSIBLE_YAW_X_Z_VALUE), fMovingAvgX_(IMPOSSIBLE_YAW_X_Z_VALUE);             ///< Moving average value
static float fMovingAvgZ_(IMPOSSIBLE_YAW_X_Z_VALUE);
static float fTargetYaw_(IMPOSSIBLE_YAW_X_Z_VALUE);
static float fTargetX_(IMPOSSIBLE_YAW_X_Z_VALUE);
static float fTargetDistance_(IMPOSSIBLE_YAW_X_Z_VALUE);
static queue<Mat> pxFramesForward_;
static queue<Mat> pxFramesBack_;
static queue<Mat> pxFramesToCalc_;                                         ///< Captured frames at the points of trajectory extremum
static TEnumStatePosition eStatePosition_(TEnumStatePosition::STATE_NONE); ///< Wheelchair position: at the nearest or farthest point from the marker, or between them
static bool isFirstRunAfterReset_(true);
static bool isResetWas_(false);


static bool prvYawTranslationCalculation(TEnumStatePosition &eStatePosition_, queue<Mat> &pxFramesToCalc, cv::Mat &xMarkerPoints, 
                                         Mat &xCameraMatrix, Mat &xDistCoefficients, OUT float &fAvgYaw, OUT float &fAvgX, 
                                         OUT float &fAvgZ, float &fTargetYaw, float &fMovingYaw, float fMovingX, float fMovingZ);
static void prvMovingAvgAndSendPacket(TEnumStatePosition &eStatePosition_, float &fAvgYaw, float &fAvgX, float &fAvgZ, OUT float &fMovingAvgYaw,
                                      OUT float &fMovingAvgX, OUT float &fMovingAvgZ, VideoCapture &xCaptureFrame, OUT Mat &xFrameCommon);                                      
static void prvRoataionTranslationCalculation(float &fYaw, float &fX, float fDistance, float &fTargetYaw, float &fTargetX,
                                              float &fTargetDistance, OUT float &fCoefRotation, OUT float &fCoefTranslation,
                                              OUT int16_t &ssCoefShift, float &fPeriod, float &fAmplitude);
static bool prvSendPacketToStroller(float &fCoefRotation, float &fCoefTranslation, uint16_t usShift, OUT float &fPeriod, OUT float &fAmplitude);
bool prvCalculateTarget(queue<Mat> &pxFramesToCalc, OUT float &fYaw, OUT float &fX, OUT float &fZ,
                        cv::Mat &xMarkerPoints, Mat &xCameraMatrix, Mat &xDistCoefficients);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int main(int argc, char *argv[])
{
  Mat xFrameCommon, xFrameTemp;                                                                              // Captured frames
  float fAvgYaw(IMPOSSIBLE_YAW_X_Z_VALUE), fAvgX(IMPOSSIBLE_YAW_X_Z_VALUE), fAvgZ(IMPOSSIBLE_YAW_X_Z_VALUE); // Arithmetic average

  vInitializationSystem(xFileToSave, xCameraMatrix, xDistCoefficients, xCaptureFrame, xMarkerPoints);

  // Just in case, while the control WCU initializes
  while (((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW)) ||
         ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH)))
  {
    if (bCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
      vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
  }         

  // Main infinite loop
  for (;;)
  {
    while (pxFramesForward_.empty() == false)
      pxFramesForward_.pop();
    while (pxFramesBack_.empty() == false)
      pxFramesBack_.pop();
    while (pxFramesToCalc_.empty() == false)
      pxFramesToCalc_.pop();

    while (((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == LOW)) ||
           ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == HIGH)))
    {
      if (bCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Pause to allow the levels on the pins to be set
    this_thread::sleep_for(2ms);

//  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
    // Target caluclation
    if (isFirstRunAfterReset_ == true)
    {
      isResetWas_ = true;
      isFirstRunAfterReset_ = false;
      // Frames capture
      while ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW) &&
           (pxFramesToCalc_.size() < COUNT_FRAMES_TO_CALC_TARGET))
      {
        if (bCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
          vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame")
        else
          pxFramesToCalc_.push_back(xFrameCommon);
      }
      if ((pxFramesToCalc_.size() < COUNT_FRAMES_TO_CALC_TARGET) ||
          (prvCalculateTarget(pxFramesToCalc_, OUT fMovingAvgYaw_, OUT fMovingAvgX_, OUT fMovingAvgZ_,
                         xMarkerPoints, xCameraMatrix, xDistCoefficients) == false))
        vRiscBehavior(TEnumRiscBehavior::RISC_CANNOT_CALC_TRAGET, "Unable to calculation the targets");
      // You can remove it: the vector is already cleared at the beginning of the loop
      pxFramesToCalc_.clear();

      // Targets initialization
      fTargetYaw_ = fMovingAvgYaw;
      fTargetX_ = fMovingAvgX;
      fTargetDistance_ = TARGET_DISTANCE_VALUE_METER;

      // Memorization of telemetry vectors
      xAvgPeriodYaw_.clear();
      xAvgPeriodX_.clear();
      xAvgPeriodZ_.clear();
      while (xAvgPeriodYaw_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG)
        xAvgPeriodYaw_.push_back(fMovingAvgYaw);
      while (xAvgPeriodX_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG)
        xAvgPeriodX_.push_back(fMovingAvgX);
      while (xAvgPeriodZ_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG)
        xAvgPeriodZ_.push_back(fMovingAvgZ); 

      // Memorization of forward values
      TEnumStatePosition eStateTemp(TEnumStatePosition::STATE_FORWARD);
      float fTemp(fMovingAvgYaw_), fTemp1(fMovingAvgX_), float fTemp2(fMovingAvgZ_), fTemp3(0.f);
      Mat xFrameTemp; 
      // At the closest point we only memorize the arithmetic mean value
      prvMovingAvgAndSendPacket(eStateTemp, fTemp, fTemp1, fTemp2, 
                                OUT fTemp3, OUT fTemp3, OUT fTemp3, xCaptureFrame, OUT xFrameTemp);
      
      // Sending three identical pacckets because there are often transmission errors
      fTemp = fTemp1 = fTemp2 = fTemp3 = 0.f;
      int16_t ssTemp(0);    
      for (size_t i = 0; i < 3; i++)
      {
        prvSendPacketToStroller(fTemp, fTemp1, ssTemp, fTemp2, fTemp3);
        this_thread::sleep_for(5ms);
      }
#ifndef TARGET_IS_CURRENT_POSITION
      fTargetYaw_ = fTargetX_ = 0.f;
#endif

      continue;
    }
//  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =    

    // Closes point to the marker
    while ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW) &&
           (pxFramesForward_.size() < COUNT_FRAMES_TO_CALC))
    {
      if (bCaptureFrame(xCaptureFrame, xFrameTemp, 10) == false)
        vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
      pxFramesForward_.push(xFrameTemp);
      cout << "Capture closes was" << endl;
    }

    // Farthest point from the marker
    while ((digitalRead(NO_PIN_BACK) == HIGH) && (digitalRead(NO_PIN_FORWARD) == LOW) &&
           (pxFramesBack_.size() < COUNT_FRAMES_TO_CALC))
    {
      if (bCaptureFrame(xCaptureFrame, xFrameTemp, 10) == false)
        vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
      pxFramesBack_.push(xFrameTemp);
      cout << "Capture farthest was" << endl;
    }

    // Point defenition
    if ((pxFramesForward_.empty() == false) && (pxFramesBack_.empty() == true))
      eStatePosition_ = TEnumStatePosition::STATE_FORWARD;

    if ((pxFramesForward_.empty() == true) && (pxFramesBack_.empty() == false))
      eStatePosition_ = TEnumStatePosition::STATE_BACK;

    if (pxFramesForward_.empty() == pxFramesBack_.empty())
      eStatePosition_ = TEnumStatePosition::STATE_NONE;

    // Position calculation preparation
    switch (eStatePosition_)
    {
    case TEnumStatePosition::STATE_FORWARD:
      if (pxFramesForward_.size() < COUNT_FRAMES_TO_CALC)
        pxFramesToCalc_.push(xFrameCommon);      
      while (pxFramesForward_.empty() == false)
      {
        pxFramesToCalc_.push(pxFramesForward_.front());
        pxFramesForward_.pop();
      }
      break;
    case TEnumStatePosition::STATE_BACK:
      if (pxFramesBack_.size() < COUNT_FRAMES_TO_CALC)
        pxFramesToCalc_.push(xFrameCommon);
      while (pxFramesBack_.empty() == false)
      {
        pxFramesToCalc_.push(pxFramesBack_.front());
        pxFramesBack_.pop();
      }
      break;
    default:
      continue;
    }

    cout << "Count of frames = " + std::to_string(pxFramesToCalc_.size()) << endl;
    if (eStatePosition_ == TEnumStatePosition::STATE_FORWARD)
      cout << "= = = = = = =" << endl;

    // Calculation of the arithmetic mean ratation angle (Yaw), X and Z of measurement at one far or closest point 
    if ((prvYawTranslationCalculation(eStatePosition_, pxFramesToCalc_, xMarkerPoints, xCameraMatrix, xDistCoefficients, OUT fAvgYaw,
                                      OUT fAvgX, OUT fAvgZ, fTargetYaw_, fMovingAvgYaw_, fMovingAvgX_, fMovingAvgZ_) == false) &&
        (fMovingAvgYaw_ < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fMovingAvgX_ < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) &&
        (fMovingAvgZ_ < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
    {
      fAvgYaw = fMovingAvgYaw_;
      fAvgX = fMovingAvgX_;
      fAvgZ = fMovingAvgZ_;
      cout << "Failed measurement" << endl;
    }

    // Caluclation of moving average at the far point and sending a packet of trajectory correction coefficients to the WCU
 *  // At the closest point we only memorize the arithmetic mean value
    prvMovingAvgAndSendPacket(eStatePosition_, fAvgYaw, fAvgX, fAvgZ, OUT fMovingAvgYaw_, OUT fMovingAvgX_,
                              OUT fMovingAvgZ_, xCaptureFrame, xFrameCommon);
  }

  return 0;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Calculation of the arithmetic mean ratation angle (Yaw), X and Z of measurement at one far or closest point 
 */
static bool prvYawTranslationCalculation(TEnumStatePosition &eStatePosition_, queue<Mat> &pxFramesToCalc, cv::Mat &xMarkerPoints, 
                                         Mat &xCameraMatrix, Mat &xDistCoefficients, OUT float &fAvgYaw, OUT float &fAvgX, 
                                         OUT float &fAvgZ, float &fTargetYaw, float &fMovingYaw, float fMovingX, float fMovingZ)
{
  bool ret = true;
  Mat xFrameTemp1;
  vector<int> xIdDetectMarker;                             // Vector of identifiers of the detected markers
  vector<vector<Point2f>> xCornersMarker, xRejectedMarker; // Vector of detected marker corners
  aruco::DetectorParameters xDetectorParams;
    /***///xDetectorParams.cornerRefinementMethod = CORNER_REFINE_APRILTAG;
  aruco::Dictionary dictionary = aruco::getPredefinedDictionary(/*aruco::DICT_ARUCO_MIP_36h12*/cv::aruco::DICT_5X5_50);/***/  
  size_t nMarkers(0);                                                  // Number of found markers (must be 1)
  double yaw(0), roll(0), pitch(0);
  float fSumYaw(0.f), fSumX(0.f), fSumZ(0.f);
  uint32_t ulYawNo(0), ulX_No(0), ulZ_No(0);
  float fCorrelatedX(fMovingX);
  static aruco::ArucoDetector xDetector_(dictionary, xDetectorParams); // Detection of markers in an image
  static bool isFirst_(true);

  while (pxFramesToCalc.empty() == false)
  {
    // Position calculation
    xFrameTemp1 = pxFramesToCalc.front();
    try
    {
      xDetector_.detectMarkers(xFrameTemp1, xCornersMarker, xIdDetectMarker, xRejectedMarker); /// @warning Was exception!!!
    }
    catch (...)
    {
      cout << "There is been an exception: xDetector_.detectMarkers()" << endl;
    }
    /*terminate called after throwing an instance of 'cv::Exception'
    what():  OpenCV(4.9.0-dev) /home/orangepi/opencv-4.x/modules/objdetect/src/aruco/aruco_detector.cpp:872: error: (-215:Assertion failed) !_image.empty() in function 'detectMarkers'*/
    pxFramesToCalc.pop();
    nMarkers = xCornersMarker.size();
    vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if (xIdDetectMarker.empty() == false)
      solvePnP(xMarkerPoints, xCornersMarker.at(0), xCameraMatrix, xDistCoefficients, rvecs.at(0), tvecs.at(0), false);
    else
      continue;

    // Quaternion calculation
    double r[] = {rvecs.at(0)[0], rvecs.at(0)[1], rvecs.at(0)[2]};
    double t[] = {tvecs.at(0)[0], tvecs.at(0)[1], tvecs.at(0)[2]};
    double lenVecRotation = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    r[0] = r[0] / lenVecRotation;
    r[1] = r[1] / lenVecRotation;
    r[2] = r[2] / lenVecRotation;
    double angle = lenVecRotation / 2.f;
    double quat[] = {cos(angle), sin(angle) * r[0], sin(angle) * r[1], sin(angle) * r[2]};

    // Euler angles calculation
    vGetYawRollPitch(quat[0], quat[1], quat[2], quat[3], OUT yaw, OUT roll, OUT pitch);

    // Yaw sum calculation
    if (((fabsf(static_cast<float>(yaw) - fMovingYaw) < MISS_RATE_YAW_RAD) && (fabsf(fMovingYaw) < 3.f * PI)) ||
        (fabsf(fMovingYaw > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))))
    {
      fSumYaw = fSumYaw + yaw;
      ulYawNo++;
    }
    else
    {
#ifdef DEBUG_SOFT
      cout << "Missed yaw is " << yaw * DEGRES_IN_RAD << endl;
#endif
    }

    // X sum calculation  
    fCorrelatedX = static_cast<float>(tvecs.at(0)[0]);
    if ((MovingAvgYaw < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fTargetYaw < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) &&
        (fMovingZ < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      fCorrelatedX = fCorrelatedX - (MovingAvgYaw - fTargetYaw) * (fMovingAvgZ * 0.04364); // 0.04364 = tg(1) * 2.5

    if (((fabsf(fCorrelatedX - fMovingX) < MISS_RATE_X_METER) && (fabsf(fMovingX) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)) || 
        (fabsf(fMovingX) > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
    {
      fSumX = fSumX + fCorrelatedX;
      ulX_No++;
    }
    else
    {
#ifdef DEBUG_SOFT
      cout << "Missed X is " << tvecs.at(0)[0] << " , correlated X is " << fCorrelatedX << endl;
#endif
    }

    // Z sum calculation
    if (((fabsf(static_cast<float>(tvecs.at(0)[2]) - fMovingZ) < MISS_RATE_Z_METER) && 
         (fabsf(fMovingZ) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)) || (fabsf(fMovingZ) > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
    {
      fSumZ = fSumZ + tvecs.at(0)[2];
      ulZ_No++;
    }
    else
    {
#ifdef DEBUG_SOFT
      if (eStatePosition_ == TEnumStatePosition::STATE_FORWARD)
        cout << "Missed Z is " << tvecs.at(0)[2] << endl;
#endif
    }

#if DEBUG_SOFT > 2
    xFileToSave << std::to_string(yaw * DEGRES_IN_RAD) + "	" << fCorrelatedX << + " " +
                << std::to_string(tvecs.at(0)[2]) << +" " + std::to_string(fMovingYaw * DEGRES_IN_RAD) << endl;
#endif
  }

  // Averaging
  if ((ulYawNo != 0) && (ulX_No != 0) && (ulZ_No != 0))
  {
    fAvgYaw = fSumYaw / static_cast<float>(ulYawNo);
    fAvgX = fSumX / static_cast<float>(ulX_No);    
    fAvgZ = fSumZ / static_cast<float>(ulZ_No);
  }
  else
    ret = false;

  return ret;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Moving average and send packet to stroller
 *  @details Caluclation of moving average at the far point and sending a packet of trajectory correction coefficients to the WCU
 *  @note At the closest point we only memorize the arithmetic mean value
 */
static void prvMovingAvgAndSendPacket(TEnumStatePosition &eStatePosition_, float &fAvgYaw, float &fAvgX, float &fAvgZ, OUT float &fMovingAvgYaw,
                                      OUT float &fMovingAvgX, OUT float &fMovingAvgZ, VideoCapture &xCaptureFrame, OUT Mat &xFrameCommon)
{
  float fCoefRotation(0.f), fCoefTranslation(0.f); // Coefficients of rotation and translation
  int16_t ssCoefShift(0);  
  static float fYawForward_(0.f), fX_Forward_(0.f), fZ_Forward_(0.f); // Calculated values at the point closest to the marker
  static float fPeriod_(0.f), fAmplitude_(0.f);
  /***/ static size_t nTemp(0), nTemp1(0);

  switch (eStatePosition_)
  {
  // At the closest point memorize values
  case TEnumStatePosition::STATE_FORWARD:
    // Memorize values
    fYawForward_ = fAvgYaw;
    fX_Forward_ = fAvgX;
    fZ_Forward_ = fAvgZ;

    // Waiting for movement to start
    while ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW))
    {
      if (bCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Checking the correctness of the combination on the input
    if ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH))
      vRiscBehavior(TEnumRiscBehavior::RISC_WRONG_INPUT_COMBINATION, "Momentary movements between points of extrema");

    eStatePosition_ = TEnumStatePosition::STATE_NONE;

    /***/ nTemp++;
    break;

  // At the far point orientation calculation
  case TEnumStatePosition::STATE_BACK:
  {
    float fTemp(IMPOSSIBLE_YAW_X_Z_VALUE);

    /***/ nTemp1++;
    // Yaw calculation
    if ((fabsf(fYawForward_) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fabsf(fAvgYaw) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      fTemp = (4.f * fYawForward_ + fAvgYaw) / 5.f;
    if ((fabsf(fYawForward_) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fabsf(fAvgYaw) > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      fTemp = fYawForward_;
    if ((fabsf(fYawForward_) > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fabsf(fAvgYaw) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      fTemp = fAvgYaw;
    if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
      xAvgPeriodYaw_.push_back(fTemp);

    // X calculation
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if ((fabsf(fX_Forward_) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fabsf(fAvgX) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      fTemp = (4.f * fX_Forward_ + fAvgX) / 5.f;
    if ((fabsf(fX_Forward_) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fabsf(fAvgX) > (IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT - 1.f)))
      fTemp = fX_Forward_;
    if ((fabsf(fX_Forward_) > (IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT - 1.f)) && (fabsf(fAvgX) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      fTemp = fAvgX;
    if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
      xAvgPeriodX_.push_back(fTemp);

    // Z calculation
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if (fabsf(fZ_Forward_) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
      fTemp = fZ_Forward_;
    if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
      xAvgPeriodZ_.push_back(fTemp);

    // Yaw vector alignment
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if ((xAvgPeriodYaw_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG) && (xAvgPeriodYaw_.empty() == false))
    {
      auto iter = xAvgPeriodYaw_.end() - 1;
      while (((iter + 1) != xAvgPeriodYaw_.begin()) && (*iter > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
        --iter;
      if (*iter < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
        fTemp = *iter;
      if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
        for (auto i = xAvgPeriodYaw_.size(); i < COUNT_MEASUREMENT_FOR_MOVING_AVG; i++)
          xAvgPeriodYaw_.push_back(fTemp);
    }
    while (xAvgPeriodYaw_.size() > COUNT_MEASUREMENT_FOR_MOVING_AVG)
      xAvgPeriodYaw_.erase(xAvgPeriodYaw_.begin());

    // X vector alignment
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if ((xAvgPeriodX_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG) && (xAvgPeriodX_.empty() == false))
    {
      auto iter = xAvgPeriodX_.end() - 1;
      while (((iter + 1) != xAvgPeriodX_.begin()) && (*iter > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
        --iter;
      if (*iter < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
        fTemp = *iter;
      if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
        for (auto i = xAvgPeriodX_.size(); i < COUNT_MEASUREMENT_FOR_MOVING_AVG; i++)
          xAvgPeriodX_.push_back(fTemp);
    }
    while (xAvgPeriodX_.size() > COUNT_MEASUREMENT_FOR_MOVING_AVG)
      xAvgPeriodX_.erase(xAvgPeriodX_.begin());

    // Z vector alignment
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if ((xAvgPeriodZ_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG) && (xAvgPeriodZ_.empty() == false))
    {
      auto iter = xAvgPeriodZ_.end() - 1;
      while (((iter + 1) != xAvgPeriodZ_.begin()) && (*iter > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
        --iter;
      if (*iter < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT)
        fTemp = *iter;
      if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) 
        for (auto i = xAvgPeriodZ_.size(); i < COUNT_MEASUREMENT_FOR_MOVING_AVG; i++)
          xAvgPeriodZ_.push_back(fTemp);
    }
    while (xAvgPeriodZ_.size() > COUNT_MEASUREMENT_FOR_MOVING_AVG)
      xAvgPeriodZ_.erase(xAvgPeriodZ_.begin());

    // Average calculation
    float fSumTemp = 0.f;
    size_t i = 1;
    for (auto &v : xAvgPeriodYaw_)
      fSumTemp += v * (i++);
    fMovingAvgYaw = fSumTemp / ((pow(COUNT_MEASUREMENT_FOR_MOVING_AVG, 2.f) + COUNT_MEASUREMENT_FOR_MOVING_AVG) / 2);
    fSumTemp = 0.f;
    i = 1;
    for (auto &v : xAvgPeriodX_)
      fSumTemp += v * (i++);
    fMovingAvgX = fSumTemp / ((pow(COUNT_MEASUREMENT_FOR_MOVING_AVG, 2.f) + COUNT_MEASUREMENT_FOR_MOVING_AVG) / 2);
    fSumTemp = 0.f;
    i = 1;
    for (auto &v : xAvgPeriodZ_)
      fSumTemp += v * (i++);
    fMovingAvgZ = fSumTemp / ((pow(COUNT_MEASUREMENT_FOR_MOVING_AVG, 2.f) + COUNT_MEASUREMENT_FOR_MOVING_AVG) / 2);

    // If the measurement is successful, we take it inti account 
    if ((xAvgPeriodYaw_.back() < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (xAvgPeriodX_.back() < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) &&
        (xAvgPeriodZ_.back() < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
      nMeasurement++;

    // if moving averages are calculated, then calculation of trajectory adjustment coefficients 
    if ((fabsf(fMovingAvgYaw) > FLOAT_EPSILON) && (fabsf(fMovingAvgX) > FLOAT_EPSILON) && (fabsf(fMovingAvgZ) > FLOAT_EPSILON)
        (fabsf(fMovingAvgYaw) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fabsf(fMovingAvgX) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) &&
        (fabsf(fMovingAvgZ) < IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
    {
      float fDistance = sqrt(pow(fMovingAvgX, 2.f) + pow(fMovingAvgZ, 2.f));
      prvRoataionTranslationCalculation(fMovingAvgYaw, fMovingAvgX, fDistance, fTargetYaw_, fTargetX_, fTargetDistance_,
                                        OUT fCoefRotation, OUT fCoefTranslation, OUT ssCoefShift, fPeriod_, fAmplitude_);
      if (((1.5 * fMovingAvgX) > fMovingAvgZ) || (fZ_Forward_ > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
        ssCoefShift = -1;      
    }        

    // The moving average Z is recalculated
    if (ssCoefShift != 0)
    {
      while (xAvgPeriodZ_.empty() == false)
        xAvgPeriodZ_.erase(xAvgPeriodZ_.begin());
    }

    /***/ float temp(0.f), temp1(0.f);    
    for (size_t i = 0; i < 3; i++)
    {
      prvSendPacketToStroller(/*temp, temp, temp1,*/ fCoefRotation, fCoefTranslation, ssCoefShift, fPeriod_, fAmplitude_);
      this_thread::sleep_for(5ms);
    }

    // Waiting for movement to start
    while ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH))
    {
      if (bCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Checking the correctness of the combination on the input
    if ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW))
      vRiscBehavior(TEnumRiscBehavior::RISC_WRONG_INPUT_COMBINATION, "Momentary movements between points of extrema");

    eStatePosition_ = TEnumStatePosition::STATE_NONE;

#ifdef DEBUG_SOFT
  vDebugFunction(fMovingAvgYaw, fMovingAvgX, fMovingAvgZ, xFileToSave, fCoefRotation, fCoefTranslation, ssCoefShift, fTargetYaw_, fTargetX_);
#endif
  }
  break;
  default:
    break;
  }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvRoataionTranslationCalculation(float &fYaw, float &fX, float fDistance, float &fTargetYaw, float &fTargetX,
                                              float &fTargetDistance, OUT float &fCoefRotation, OUT float &fCoefTranslation,
                                              OUT int16_t &ssCoefShift, float &fPeriod, float &fAmplitude)
{
  static float fIntegralErrorYaw(0.f), fIntegralErrorX(0.f);
  static bool isShiftCalcNow(false);  
  float fErrorYaw = fTargetYaw - fYaw;
  float fErrorX = fTargetX - fX;

  if ((fYaw > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT) && (fX > IMPOSSIBLE_YAW_X_Z_VALUE_FLOAT))
    goto return_prvRoataionTranslationCalculation;

  // Calculation integral errors
  fIntegralErrorYaw = fIntegralErrorYaw + fErrorYaw;
  fIntegralErrorX = fIntegralErrorX + fErrorX;
  // Calculation translation and rotation coefficients
  fCoefRotation = fErrorYaw * COEF_PROPORTIONAL_YAW + fIntegralErrorYaw * COEF_INTEGRAL_YAW * fPeriod;
  fCoefTranslation = fErrorX * COEF_PROPORTIONAL_X + fIntegralErrorX * COEF_INTEGRAL_X * fPeriod;

  // Calculation shift coefficient
  if (fDistance < MINIMAL_DISTANCE_VALUE_METER)
    ssCoefShift = (-1) * static_cast<int16_t>(floor((fTargetDistance - fDistance) / (0.1) / fAmplitude));
  if (fDistance > fTargetDistance)
    ssCoefShift = floor((fDistance - fTargetDistance) / (0.1) / fAmplitude);
  if (abs(ssCoefShift) > 1)/***/
    ssCoefShift = ssCoefShift > 0 ? 1 : -1;
  if (isShiftCalcNow == false)
    ssCoefShift = 0;
  isShiftCalcNow = isShiftCalcNow == true ? false : true;

  if (fabsf(fCoefRotation) > 0.5)
  {
    cout << "!!! Rotation coefficient more than 50 per cent !!!" << endl;
    fCoefRotation = fCoefRotation > 0.f ? 0.5 : -0.5;
  }
  if (fabsf(fCoefTranslation) > 0.5)
  {
    cout << "!!! Translation coefficient more than 50 per cent !!!" << endl;
    fCoefTranslation = fCoefTranslation > 0.f ? 0.5 : -0.5;
  }

return_prvRoataionTranslationCalculation:
  return;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static bool prvSendPacketToStroller(float &fCoefRotation, float &fCoefTranslation, int16_t ssShift, OUT float &fPeriod, OUT float &fAmplitude)
{
  bool ret(true);
  int res(0);
  string sError;
  static TProtocolInWcuMotionCmd xPacketOut_;
  TProtocolInOuCondition *pxPacketIn = reinterpret_cast<TProtocolInOuCondition *>(&xPacketOut_);

  xPacketOut_.usPreambule = PREAMBULE_IN_WCU;
  xPacketOut_.ucIdPacket = ID_PACKET_IN_WCU_MOTION_CMD;
  xPacketOut_.fRotation = fCoefRotation;
  xPacketOut_.fTranslation = fCoefTranslation;
  xPacketOut_.ssShift = ssShift;
  xPacketOut_.crc16 = crc16citt(reinterpret_cast<unsigned char *>(&xPacketOut_), sizeof(xPacketOut_) - 2);

  errno = 0;
  if (wiringPiSPIDataRW(SPI_CHANNEL, reinterpret_cast<unsigned char *>(&xPacketOut_), sizeof(xPacketOut_)) < 0)
    vRiscBehavior(TEnumRiscBehavior::RISC_CANNOT_SEND_SPI_PACKET, strerror(errno));

  /***/ static size_t all = 0;
  all++;
  if (pxPacketIn->crc16 != crc16citt(reinterpret_cast<unsigned char *>(pxPacketIn), sizeof(xPacketOut_) - 2))
  {
    /***/ static size_t err = 0;
    ret = false;
    cout << " ! ! ! CRC16 error ! ! !  error count is " << ++err << " from " << all << " packets" << endl;
  }
  else
  {
    if ((pxPacketIn->ucIdPacket == ID_PACKET_IN_OU_RESET_WAS) && (isResetWas_ == false))
    {
      isFirstRunAfterReset_ = true;
    } else {
      isResetWas_ = fasle;
    }
    fPeriod = pxPacketIn->fPeriod;
    fAmplitude = pxPacketIn->fAmplitude;
  } 

  return ret;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Calculation of setpoint values by median average
 * 
*/
bool prvCalculateTarget(queue<Mat> &pxFramesToCalc, OUT float &fYaw, OUT float &fX, OUT float &fZ,
                        cv::Mat &xMarkerPoints, Mat &xCameraMatrix, Mat &xDistCoefficients)
{
  bool isRetSuccess(true);
  Mat xFrameTemp1;
  size_t nAllFrames = pxFramesToCalc.size();
  vector<int> xIdDetectMarker;                             // Vector of identifiers of the detected markers
  vector<vector<Point2f>> xCornersMarker, xRejectedMarker; // Vector of detected marker corners
  aruco::DetectorParameters xDetectorParams;
    /***///xDetectorParams.cornerRefinementMethod = CORNER_REFINE_APRILTAG;
  aruco::Dictionary dictionary = aruco::getPredefinedDictionary(/*aruco::DICT_ARUCO_MIP_36h12*/cv::aruco::DICT_5X5_50);/***/
  static aruco::ArucoDetector xDetector_(dictionary, xDetectorParams); // Detection of markers in an image
  size_t nMarkers(0);                                                  // Number of found markers (must be 1)
  double yaw(0), roll(0), pitch(0);
  vector<float> xYawDetectArray;
  vector<float> xX_DetectArray;
  vector<float> xZ_DetectArray;

  float fSumYaw(0.f), fSumX(0.f), fSumZ(0.f);
  uint32_t ulYawNo(0), ulX_No(0), ulZ_No(0);

  while (pxFramesToCalc.empty() == false)
  {
    // Position calculation
    xFrameTemp1 = pxFramesToCalc.front();
    try
    {
      xDetector_.detectMarkers(xFrameTemp1, xCornersMarker, xIdDetectMarker, xRejectedMarker); /// @warning Was exception!!!
    }
    catch (...)
    {
      cout << "There is been an exception: xDetector_.detectMarkers()" << endl;
    }
    /*terminate called after throwing an instance of 'cv::Exception'
    what():  OpenCV(4.9.0-dev) /home/orangepi/opencv-4.x/modules/objdetect/src/aruco/aruco_detector.cpp:872: error: (-215:Assertion failed) !_image.empty() in function 'detectMarkers'*/
    pxFramesToCalc.pop();
    nMarkers = xCornersMarker.size();
    vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if (xIdDetectMarker.empty() == false)
      solvePnP(xMarkerPoints, xCornersMarker.at(0), xCameraMatrix, xDistCoefficients, rvecs.at(0), tvecs.at(0), false);
    else
      continue;

    // Quaternion calculation
    double r[] = {rvecs.at(0)[0], rvecs.at(0)[1], rvecs.at(0)[2]};
    double t[] = {tvecs.at(0)[0], tvecs.at(0)[1], tvecs.at(0)[2]};
    double lenVecRotation = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    r[0] = r[0] / lenVecRotation;
    r[1] = r[1] / lenVecRotation;
    r[2] = r[2] / lenVecRotation;
    double angle = lenVecRotation / 2.f;
    double quat[] = {cos(angle), sin(angle) * r[0], sin(angle) * r[1], sin(angle) * r[2]};

    // Euler angles calculation
    vGetYawRollPitch(quat[0], quat[1], quat[2], quat[3], OUT yaw, OUT roll, OUT pitch);

    xYawDetectArray.push_back(yaw);
    xX_DetectArray.push_back(tvecs.at(0)[0]);
    xZ_DetectArray.push_back(tvecs.at(0)[2]);
  }

  // Median search
  constexpr float SEARCH_RANGE_START_YAW = -10.f / DEGRES_IN_RAD;
  constexpr float SEARCH_RANGE_YAW = 20.f / DEGRES_IN_RAD;
  constexpr float SEARCH_WINDOW_WIDTH_YAW = MISS_RATE_YAW_RAD > (15.f / DEGRES_IN_RAD) ? (7.f / DEGRES_IN_RAD) : MISS_RATE_YAW_RAD;
  constexpr float SEARCH_STEP_YAW = 2.f / DEGRES_IN_RAD;
  const float SEARCH_RANGE_START_X = -0.5;
  const float SEARCH_RANGE_X = 1.f;
  constexpr float SEARCH_WINDOW_WIDTH_X = MISS_RATE_X_METER > 0.5 ? 0.2 : MISS_RATE_X_METER;
  const float SEARCH_STEP_X = 0.1;
  const float SEARCH_RANGE_START_Z = 0.3;
  const float SEARCH_RANGE_Z = 1.f;
  constexpr float SEARCH_WINDOW_WIDTH_Z = MISS_RATE_Z_METER > 0.5 ? 0.2 : MISS_RATE_Z_METER;
  const float SEARCH_STEP_Z = 0.1;
  std::map<size_t, float> xCountMedianYaw;
  std::map<size_t, float> xCountMedianX;
  std::map<size_t, float> xCountMedianZ;
  size_t count(0);
  // Counting the number of median values
  for (float value = SEARCH_RANGE_START_YAW; value <= (SEARCH_RANGE_START_YAW + SEARCH_RANGE_YAW); value += SEARCH_STEP_YAW)
  {
    count = 0;
    for (auto &v : xYawDetectArray)
      if ((v > (value - SEARCH_STEP_YAW)) && (v < (value + SEARCH_STEP_YAW)))
        count++;
    xCountMedianYaw[count] = value;
  }
  for (float value = SEARCH_RANGE_START_X; value <= (SEARCH_RANGE_START_X + SEARCH_RANGE_X); value += SEARCH_STEP_X)
  {
    count = 0;
    for (auto &v : xX_DetectArray)
      if ((v > (value - SEARCH_STEP_X)) && (v < (value + SEARCH_STEP_X)))
        count++;
    xCountMedianX[count] = value;
  }
  for (float value = SEARCH_RANGE_START_Z; value <= (SEARCH_RANGE_START_Z + SEARCH_RANGE_Z); value += SEARCH_STEP_Z)
  {
    count = 0;
    for (auto &v : xZ_DetectArray)
      if ((v > (value - SEARCH_STEP_Z)) && (v < (value + SEARCH_STEP_Z)))
        count++;
    xCountMedianZ[count] = value;
  }
  // Finding the average values
  auto medianValue = (xCountMedianYaw::end()-1)->second;
  float fSum(0.f);
  count = 0;
  for (auto &v : xYaw_DetectArray)
    if ((v > (medianValue - MISS_RATE_YAW_RAD)) && (v < (medianValue + MISS_RATE_YAW_RAD)))
    {
      count++;
      fSum = fSum + v;
    } 
  if (count > 0)
    fYaw = fSum / count;
  else
    isRetSuccess = false;

  cout << "The percentage of calculated frames to setpoint is " << (static_cast<float>(count) / static_cast<float>nAllFrames) << endl;
  cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *" << endl;

  medianValue = (xCountMedianX::end()-1)->second;
  fSum = 0.f;
  count = 0;  
  for (auto &v : xX_DetectArray)
    if ((v > (medianValue - MISS_RATE_X_METER)) && (v < (medianValue + MISS_RATE_X_METER)))
    {
      count++;
      fSum = fSum + v;
    } 
  if (count > 0)
    fX = fSum / count;
  else
    isRetSuccess = false;
  medianValue = (xCountMedianZ::end()-1)->second;
  fSum = 0.f;
  count = 0;  
  for (auto &v : xZ_DetectArray)
    if ((v > (medianValue - MISS_RATE_Z_METER)) && (v < (medianValue + MISS_RATE_Z_METER)))
    {
      count++;
      fSum = fSum + v;
    } 
  if (count > 0)
    fZ = fSum / count;
  else
    isRetSuccess = false;
  
#if DEBUG_SOFT > 2
    xFileToSave << std::to_string(fYaw * DEGRES_IN_RAD) + "	" << std::to_string(fX)
                << +" " << std::to_string(fZ) << endl;
#endif  

  return isRetSuccess;
}

