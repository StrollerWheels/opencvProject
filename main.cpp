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

#include "crc.h"
#include "protocol.h"

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

enum class TEnumRiscBehavior
{
  RISC_WRONG_INPUT_COMBINATION = 0,
  RISC_CAMERA_PROBLEM,
  RISC_INVALID_CAMERA_FILE,
  RISC_NOT_SETUP_SPI,
  RISC_CANNOT_SEND_SPI_PACKET,
};

using namespace std;
using namespace cv;

constexpr double PI = 3.141592653589793;
constexpr double DEGRES_IN_RAD = 180 / PI;

const float FLOAT_EPSILON = 0.00001;

const float MARKER_LENGTH = 0.15; ///< Marker side length
const int CAMERA_NUMBER = 1;     ///< Camera number in OS
const int NO_PIN_FORWARD = 0;
const int NO_PIN_BACK = 1;
const int SPI_CHANNEL = 1;
const int SPI_PORT = 0;
const int SPI_BAUDRATE = 100000;
const int SPI_MODE = 0;

const float COEF_PROPORTIONAL_YAW = 2.f;
const float COEF_INTEGRAL_YAW = 0.01;
const float COEF_DIFFERENTIAL_YAW = 0.f;
const float COEF_PROPORTIONAL_X = /***/2.f;
const float COEF_INTEGRAL_X = /***/0.01;
const float COEF_DIFFERENTIAL_X = 0.f;

const size_t COUNT_FRAMES = 5;

const size_t COUNT_MEASUREMENT_FOR_MOVING_AVG = 3;
constexpr float MISS_RATE_YAW_GRAD = /*400.999999999; /*/ 6.999999999;
constexpr float MISS_RATE_YAW_RAD = MISS_RATE_YAW_GRAD / DEGRES_IN_RAD;
const float MISS_RATE_X_METER = /*100; /*/ 0.2;
const float MISS_RATE_Z_METER = /*100;          /*/ 0.3;
const float IMPOSSIBLE_YAW_X_Z_VALUE = 100.f; ///< Moving average is initiated by this value
const float TARGET_DISTANCE_VALUE_METER = 0.6;
const float MINIMAL_DISTANCE_VALUE_METER = 0.55;

ofstream xFileToSave;                  ///< Debugging telemetry recording
Mat xCameraMatrix, xDistCoefficients;  ///< Camera calibration settings
VideoCapture xCaptureFrame;            ///< Object to capture a frame
cv::Mat xMarkerPoints(4, 1, CV_32FC3); ///< Coordinates of marker corners relative to the marker center
uint32_t nMeasurement(0);              // Count of measurement

/***/ static queue<Mat> pxFramesToCalcTarget_;  /// @attention Temporary

#ifdef DEBUG_SOFT
float fCoefTranslationDebug(0.f), fCoefRotationDebug(0.f); ///< To display in the terminal when debugging
int16_t ssCoefShiftDebug(0.f);
#endif

static void prvInitializationSystem(ofstream &xFileToSave, OUT Mat &xCameraMatrix, OUT Mat &xDistCoefficients,
                                    OUT VideoCapture &xCaptureFrame, OUT Mat &xMarkerPoints);
static void prvRiscBehavior(TEnumRiscBehavior eErrorCode, string sError);
static bool prvReadCameraParameters(std::string filename, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients);
static bool prvCaptureFrame(VideoCapture &xCapture, OUT Mat &frame, size_t nAttempts);
static bool prvYawTranslationCalculation(TEnumStatePosition &eStatePosition_, queue<Mat> &pxFramesToCalc, cv::Mat &xMarkerPoints, Mat &xCameraMatrix, Mat &xDistCoefficients,
                                         OUT float &fAvgYaw, OUT float &fAvgX, OUT float &fAvgZ, float &fMovingYaw, float fMovingX, float fMovingZ);
static void prvGetYawRollPitch(double q0, double qx, double qy, double qz, OUT double &yaw, OUT double &roll, OUT double &pitch);
static void prvMovingAvgAndSendPacket(TEnumStatePosition &eStatePosition_, float &fAvgYaw, float &fAvgX, float &fAvgZ, OUT float &fMovingAvgYaw,
                                      OUT float &fMovingAvgX, OUT float &fMovingAvgZ, VideoCapture &xCaptureFrame, OUT Mat &xFrameCommon);                                      
static void prvRoataionTranslationCalculation(float &fYaw, float &fX, float fDistance, float &fTargetYaw, float &fTargetX,
                                              float &fTargetZ, OUT float &fCoefRotation, OUT float &fCoefTranslation,
                                              OUT int16_t &ssCoefShift, float &fPeriod, float &fAmplitude);
static bool prvSendPacketToStroller(float &fCoefRotation, float &fCoefTranslation, uint16_t usShift, OUT float &fPeriod, OUT float &fAmplitude);
void prvDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, float &fMovingAvgZ, ofstream &xFileToSave,
                      float &fCoefRot, float &fCoefTransl, int16_t ssCoefShift, float fTargetYaw, float fTargetX);

bool prvCalculateTarget(queue<Mat> &pxFramesToCalc, OUT float &fYaw, OUT float &fX, OUT float &fZ,
                        cv::Mat &xMarkerPoints, Mat &xCameraMatrix, Mat &xDistCoefficients);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int main(int argc, char *argv[])
{
  prvInitializationSystem(xFileToSave, xCameraMatrix, xDistCoefficients, xCaptureFrame, xMarkerPoints);

  for (;;)
  {
    Mat xFrameCommon, xFrameTemp;                                                                              // Captured frames
    float fAvgYaw(IMPOSSIBLE_YAW_X_Z_VALUE), fAvgX(IMPOSSIBLE_YAW_X_Z_VALUE), fAvgZ(IMPOSSIBLE_YAW_X_Z_VALUE); // Arithmetic average
    static float fMovingAvgYaw_(IMPOSSIBLE_YAW_X_Z_VALUE), fMovingAvgX_(IMPOSSIBLE_YAW_X_Z_VALUE);             ///< Moving average value
    static float fMovingAvgZ_(IMPOSSIBLE_YAW_X_Z_VALUE);
    static queue<Mat> pxFramesForward_;
    static queue<Mat> pxFramesBack_;
    static queue<Mat> pxFramesToCalc_;                                         ///< Captured frames at the points of trajectory extremum
    static TEnumStatePosition eStatePosition_(TEnumStatePosition::STATE_NONE); ///< Wheelchair position: at the nearest or farthest point from the marker, or between them

    while (pxFramesForward_.empty() == false)
      pxFramesForward_.pop();
    while (pxFramesBack_.empty() == false)
      pxFramesBack_.pop();
    while (pxFramesToCalc_.empty() == false)
      pxFramesToCalc_.pop();

    while (((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == LOW)) ||
           ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == HIGH)))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        prvRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Pause to allow the levels on the pins to be set
    this_thread::sleep_for(2ms);

    // Closes point to the marker
    while ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW) &&
           (pxFramesForward_.size() < COUNT_FRAMES))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameTemp, 10) == false)
        prvRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
      pxFramesForward_.push(xFrameTemp);
      if (nMeasurement <= COUNT_MEASUREMENT_FOR_MOVING_AVG) /***/
        pxFramesToCalcTarget_.push(xFrameTemp); /***/
      cout << "Capture closes was" << endl;
    }

    // Farthest point from the marker
    while ((digitalRead(NO_PIN_BACK) == HIGH) && (digitalRead(NO_PIN_FORWARD) == LOW) &&
           (pxFramesBack_.size() < COUNT_FRAMES))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameTemp, 10) == false)
        prvRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
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
      if (pxFramesForward_.size() < COUNT_FRAMES)
        pxFramesToCalc_.push(xFrameCommon);
      if (nMeasurement <= COUNT_MEASUREMENT_FOR_MOVING_AVG) /***/
        pxFramesToCalcTarget_.push(xFrameCommon); /***/
      while (pxFramesForward_.empty() == false)
      {
        pxFramesToCalc_.push(pxFramesForward_.front());
        pxFramesForward_.pop();
      }
      break;
    case TEnumStatePosition::STATE_BACK:
      if (pxFramesBack_.size() < COUNT_FRAMES)
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

    if ((prvYawTranslationCalculation(eStatePosition_, pxFramesToCalc_, xMarkerPoints, xCameraMatrix, xDistCoefficients, OUT fAvgYaw,
                                      OUT fAvgX, OUT fAvgZ, fMovingAvgYaw_, fMovingAvgX_, fMovingAvgZ_) == false) &&
        (fMovingAvgYaw_ < IMPOSSIBLE_YAW_X_Z_VALUE) && (fMovingAvgX_ < IMPOSSIBLE_YAW_X_Z_VALUE) &&
        (fMovingAvgZ_ < IMPOSSIBLE_YAW_X_Z_VALUE))
    {
      fAvgYaw = fMovingAvgYaw_;
      fAvgX = fMovingAvgX_;
      fAvgZ = fMovingAvgZ_;
      cout << "Failed measurement" << endl;
    }

    prvMovingAvgAndSendPacket(eStatePosition_, fAvgYaw, fAvgX, fAvgZ, OUT fMovingAvgYaw_, OUT fMovingAvgX_,
                              OUT fMovingAvgZ_, xCaptureFrame, xFrameCommon);
  }

  return 0;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvInitializationSystem(ofstream &xFileToSave, OUT Mat &xCameraMatrix, OUT Mat &xDistCoefficients,
                                    OUT VideoCapture &xCaptureFrame, OUT Mat &xMarkerPoints)
{
  wiringPiSetup();
  pinMode(NO_PIN_FORWARD, INPUT);
  pinMode(NO_PIN_BACK, INPUT);

  errno = 0;
  if (wiringPiSPISetupMode(SPI_CHANNEL, SPI_PORT, SPI_BAUDRATE, SPI_MODE) < 0)
    prvRiscBehavior(TEnumRiscBehavior::RISC_NOT_SETUP_SPI, strerror(errno));

  xFileToSave.open("../Angles.ods");
  xFileToSave << "AvgYaw	AvgX AvgZ MovingAvgYaw" << endl;

  if (prvReadCameraParameters("../Calibr_1920x1080.xml", xCameraMatrix, xDistCoefficients) == false)
    prvRiscBehavior(TEnumRiscBehavior::RISC_INVALID_CAMERA_FILE, "The settings file cannot be opened");

  // Open camera
  do
  {
    xCaptureFrame.open(CAMERA_NUMBER, cv::CAP_V4L2);
    if (xCaptureFrame.isOpened() == false)
      cout << "Cannot open camera" << endl;
    this_thread::sleep_for(1000ms);
  } while (xCaptureFrame.isOpened() == false);

  // Camera setup
  xCaptureFrame.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  xCaptureFrame.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  xCaptureFrame.set(cv::CAP_PROP_FPS, 5);
  double dWidth = xCaptureFrame.get(cv::CAP_PROP_FRAME_WIDTH);   // get the width of frames of the video
  double dHeight = xCaptureFrame.get(cv::CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
  double dFps = xCaptureFrame.get(cv::CAP_PROP_FPS);
  cout << "camera width = " << dWidth << ", height = " << dHeight << ", FPS = " << dFps << endl;

  // Set coordinate system
  xMarkerPoints.ptr<Vec3f>(0)[0] = Vec3f(-MARKER_LENGTH / 2.f, MARKER_LENGTH / 2.f, 0);
  xMarkerPoints.ptr<Vec3f>(0)[1] = Vec3f(MARKER_LENGTH / 2.f, MARKER_LENGTH / 2.f, 0);
  xMarkerPoints.ptr<Vec3f>(0)[2] = Vec3f(MARKER_LENGTH / 2.f, -MARKER_LENGTH / 2.f, 0);
  xMarkerPoints.ptr<Vec3f>(0)[3] = Vec3f(-MARKER_LENGTH / 2.f, -MARKER_LENGTH / 2.f, 0);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvRiscBehavior(TEnumRiscBehavior eErrorCode, string sError)
{
  switch (eErrorCode)
  {
  case TEnumRiscBehavior::RISC_WRONG_INPUT_COMBINATION:
    cout << " ! ! ! Wrong input combination ! ! ! " << endl;
    break;
  case TEnumRiscBehavior::RISC_CAMERA_PROBLEM:
    cout << " ! ! ! Camera problems ! ! ! " << endl;
    break;
  case TEnumRiscBehavior::RISC_INVALID_CAMERA_FILE:
    cout << " ! ! ! Invalid camera file ! ! ! " << endl;
  case TEnumRiscBehavior::RISC_NOT_SETUP_SPI:
    cout << " ! ! ! Not setup SPI ! ! ! " << endl;
  case TEnumRiscBehavior::RISC_CANNOT_SEND_SPI_PACKET:
    cout << " ! ! ! PACKET SENDING ERROR ! ! ! " << endl;
  default:
    cout << " ! ! ! Uncertain behavior ! ! ! " << endl;
    break;
  }

  cout << sError << endl;

  for (;;)
    asm("NOP");
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static bool prvReadCameraParameters(std::string filename, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["camera_matrix"] >> xCameraMatrix;
  fs["distortion_coefficients"] >> xDistCoefficients;
  return true;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Frame capture
 */
static bool prvCaptureFrame(VideoCapture &xCapture, OUT Mat &frame, size_t nAttempts)
{
  bool ret(true);

  if (xCapture.read(frame) == false)
  {
    ret = false;
    // First open the camera
    do
    {
      this_thread::sleep_for(200ms);
      xCapture.open(CAMERA_NUMBER, cv::CAP_V4L2);
      if (xCapture.isOpened() == false)
      {
        cout << "Cannot open camera" << endl;
        continue;
      }

      cout << "Cannot read a xFrameCommon" << endl;
    } while ((xCapture.read(frame) == false) && (--nAttempts > 0));
  }

  if (nAttempts > 0)
    ret = true;

  return ret;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Yaw and translation calculation
 */
static bool prvYawTranslationCalculation(TEnumStatePosition &eStatePosition_, queue<Mat> &pxFramesToCalc, cv::Mat &xMarkerPoints, 
                                         Mat &xCameraMatrix, Mat &xDistCoefficients, OUT float &fAvgYaw, OUT float &fAvgX, 
                                         OUT float &fAvgZ, float &fMovingYaw, float fMovingX, float fMovingZ)
{
  bool ret = true;
  Mat xFrameTemp1;
  vector<int> xIdDetectMarker;                             // Vector of identifiers of the detected markers
  vector<vector<Point2f>> xCornersMarker, xRejectedMarker; // Vector of detected marker corners
  aruco::DetectorParameters xDetectorParams;
    /***///xDetectorParams.cornerRefinementMethod = CORNER_REFINE_APRILTAG;
  aruco::Dictionary dictionary = aruco::getPredefinedDictionary(/*aruco::DICT_ARUCO_MIP_36h12*/cv::aruco::DICT_5X5_50);/***/
  static aruco::ArucoDetector xDetector_(dictionary, xDetectorParams); // Detection of markers in an image
  size_t nMarkers(0);                                                  // Number of found markers (must be 1)
  double yaw(0), roll(0), pitch(0);
  float fSumYaw(0.f), fSumX(0.f), fSumZ(0.f);
  uint32_t ulYawNo(0), ulX_No(0), ulZ_No(0);
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
    prvGetYawRollPitch(quat[0], quat[1], quat[2], quat[3], yaw, roll, pitch);

    // Yaw sum calculation
    if (((fabsf(static_cast<float>(yaw) - fMovingYaw) < MISS_RATE_YAW_RAD) && (fabsf(fMovingYaw) < 3.f * PI)) ||
        (fabsf(fMovingYaw > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f))))
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
    if (((fabsf(static_cast<float>(tvecs.at(0)[0]) - fMovingX) < MISS_RATE_X_METER) && (fabsf(fMovingX) < 99.f)) ||
        (fabsf(fMovingX) > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
    {
      fSumX = fSumX + tvecs.at(0)[0];
      ulX_No++;
    }
    else
    {
#ifdef DEBUG_SOFT
      cout << "Missed X is " << tvecs.at(0)[0] << endl;
#endif
    }

    // Z sum calculation
    if (((fabsf(static_cast<float>(tvecs.at(0)[2]) - fMovingZ) < MISS_RATE_Z_METER) && (fabsf(fMovingZ) < 99.f)) ||
        (fabsf(fMovingZ) > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
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
    xFileToSave << std::to_string(yaw * DEGRES_IN_RAD) + "	" << std::to_string(tvecs.at(0)[0])
                << +" " << std::to_string(tvecs.at(0)[2]) << +" " + std::to_string(fMovingYaw * DEGRES_IN_RAD) << endl;
#endif
  }

  // Averaging
  if ((ulYawNo != 0) && (ulX_No != 0))
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
static void prvGetYawRollPitch(double q0, double qx, double qy, double qz, OUT double &yaw, OUT double &roll, OUT double &pitch)
{
  double dTestValue = qx * qy + qz * q0;

  if (dTestValue > 0.499) // then-- singularity at north pole
  {
    yaw = (2 * atan2(qx, q0));
    roll = 0;
    pitch = PI / 2;
  }
  if (dTestValue < -0.499) // then-- singularity at south pole
  {
    yaw = (-2 * atan2(qx, q0));
    roll = 0;
    pitch = PI / 2;
  }
  if ((dTestValue <= 0.499) || (dTestValue >= -0.499))
  {
    yaw = atan2(2 * (qy * q0 - qx * qz), 1 - 2 * (qy * qy + qz * qz));
    roll = atan2(2 * (qx * q0 - qy * qz), 1 - 2 * (qx * qx + qz * qz));
    pitch = asin(2 * dTestValue);
  }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Moving average and send packet to stroller
 */
static void prvMovingAvgAndSendPacket(TEnumStatePosition &eStatePosition_, float &fAvgYaw, float &fAvgX, float &fAvgZ, OUT float &fMovingAvgYaw,
                                      OUT float &fMovingAvgX, OUT float &fMovingAvgZ, VideoCapture &xCaptureFrame, OUT Mat &xFrameCommon)
{
  float fCoefRotation(0.f), fCoefTranslation(0.f); // Coefficients of rotation and translation
  int16_t ssCoefShift(0);  
  static float fYawForward_(0.f), fX_Forward_(0.f), fZ_Forward_(0.f); // Calculated values iat the point closest to the marker
  static vector<float> xAvgPeriodYaw_(0);                             // Vector of yaw moving average for one period, is calculated at the far point
  static vector<float> xAvgPeriodX_(0);                               // Vector of X moving average for one period, is calculated at the far point
  static vector<float> xAvgPeriodZ_(0);                               // Vector of Z moving average for one period, is calculated at the far point
  static float fPeriod_(0.f), fAmplitude_(0.f);
  static float fTargetYaw_(0.f), fTargetX_(0.f);
  static float fTargetZ_(TARGET_DISTANCE_VALUE_METER);

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
      if (prvCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        prvRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Checking the correctness of the combination on the input
    if ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH))
      prvRiscBehavior(TEnumRiscBehavior::RISC_WRONG_INPUT_COMBINATION, "Momentary movements between points of extrema");

    eStatePosition_ = TEnumStatePosition::STATE_NONE;

    /***/ nTemp++;
    break;

  // At the far point orientation calculation
  case TEnumStatePosition::STATE_BACK:
  {
    float fTemp(IMPOSSIBLE_YAW_X_Z_VALUE);

    /***/ nTemp1++;
    // Yaw calculation
    if ((fabsf(fYawForward_) < IMPOSSIBLE_YAW_X_Z_VALUE) && (fabsf(fAvgYaw) < IMPOSSIBLE_YAW_X_Z_VALUE))
      fTemp = (4.f * fYawForward_ + fAvgYaw) / 5.f;
    if ((fabsf(fYawForward_) < IMPOSSIBLE_YAW_X_Z_VALUE) && (fabsf(fAvgYaw) > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
      fTemp = fYawForward_;
    if ((fabsf(fYawForward_) > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)) && (fabsf(fAvgYaw) < IMPOSSIBLE_YAW_X_Z_VALUE))
      fTemp = fAvgYaw;
    if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE)
      xAvgPeriodYaw_.push_back(fTemp);

    // X calculation
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if ((fabsf(fX_Forward_) < IMPOSSIBLE_YAW_X_Z_VALUE) && (fabsf(fAvgX) < IMPOSSIBLE_YAW_X_Z_VALUE))
      fTemp = (3.f * fX_Forward_ + fAvgX) / 4.f;
    if ((fabsf(fX_Forward_) < IMPOSSIBLE_YAW_X_Z_VALUE) && (fabsf(fAvgX) > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
      fTemp = fX_Forward_;
    if ((fabsf(fX_Forward_) > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)) && (fabsf(fAvgX) < IMPOSSIBLE_YAW_X_Z_VALUE))
      fTemp = fAvgX;
    if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE)
      xAvgPeriodX_.push_back(fTemp);

    // Z calculation
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if (fabsf(fZ_Forward_) < IMPOSSIBLE_YAW_X_Z_VALUE)
      fTemp = fZ_Forward_;
    if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE)
      xAvgPeriodZ_.push_back(fTemp);

    // Yaw vector alignment
    fTemp = IMPOSSIBLE_YAW_X_Z_VALUE;
    if ((xAvgPeriodYaw_.size() < COUNT_MEASUREMENT_FOR_MOVING_AVG) && (xAvgPeriodYaw_.empty() == false))
    {
      auto iter = xAvgPeriodYaw_.end() - 1;
      while (((iter + 1) != xAvgPeriodYaw_.begin()) && (*iter > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
        --iter;
      if (*iter < IMPOSSIBLE_YAW_X_Z_VALUE)
        fTemp = *iter;
      if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE)
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
      while (((iter + 1) != xAvgPeriodX_.begin()) && (*iter > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
        --iter;
      if (*iter < IMPOSSIBLE_YAW_X_Z_VALUE)
        fTemp = *iter;
      if (fTemp < IMPOSSIBLE_YAW_X_Z_VALUE)
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
      while (((iter + 1) != xAvgPeriodZ_.begin()) && (*iter > (IMPOSSIBLE_YAW_X_Z_VALUE - 1.f)))
        --iter;
      if (*iter < IMPOSSIBLE_YAW_X_Z_VALUE)
        fTemp = *iter;
      // The first Z measurement os not into account 
      /// @todo Correct if you first approach the a certain distance
      if ((fTemp < IMPOSSIBLE_YAW_X_Z_VALUE) && (nMeasurement > 0)) 
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

    // Calculation of motion coefficients for stroller control unit
    if ((fTargetYaw_ < IMPOSSIBLE_YAW_X_Z_VALUE) && (fTargetX_ < IMPOSSIBLE_YAW_X_Z_VALUE))
      nMeasurement++;
    if (nMeasurement >= COUNT_MEASUREMENT_FOR_MOVING_AVG)
    {
#ifdef TARGET_IS_CURRENT_POSITION
      if (nMeasurement == COUNT_MEASUREMENT_FOR_MOVING_AVG)
      {
        prvCalculateTarget(pxFramesToCalcTarget_, OUT fMovingAvgYaw, OUT fMovingAvgX, OUT fMovingAvgZ,
                           xMarkerPoints, xCameraMatrix, xDistCoefficients);
        fTargetYaw_ = fMovingAvgYaw;
        fTargetX_ = fMovingAvgX;
        nMeasurement++;
      }
#endif
      if ((fabsf(fMovingAvgYaw) > FLOAT_EPSILON) && (fabsf(fMovingAvgX) > FLOAT_EPSILON) &&
          (fabsf(fMovingAvgYaw) < IMPOSSIBLE_YAW_X_Z_VALUE) && (fabsf(fMovingAvgX) < IMPOSSIBLE_YAW_X_Z_VALUE))
      {
        float fDistance = sqrt(pow(fMovingAvgX, 2.f) + pow(fMovingAvgZ, 2.f));
        prvRoataionTranslationCalculation(fMovingAvgYaw, fMovingAvgX, fDistance, fTargetYaw_, fTargetX_, fTargetZ_,
                                          OUT fCoefRotation, OUT fCoefTranslation, OUT ssCoefShift, fPeriod_, fAmplitude_);
        if (((1.5 * fMovingAvgX) > fMovingAvgZ) || (fZ_Forward_ > (IMPOSSIBLE_YAW_X_Z_VALUE - FLOAT_EPSILON)))
          ssCoefShift = -1;
      }
    }        

    // The moving average Z is recalculated
    if (ssCoefShift != 0)
    {
      while (xAvgPeriodZ_.empty() == false)
        xAvgPeriodZ_.erase(xAvgPeriodZ_.begin());
    }

    /***/ float temp = 0.f;
    int16_t temp1 = 0.f;
    prvSendPacketToStroller(/*temp, temp, temp1,*/ fCoefRotation, fCoefTranslation, ssCoefShift, fPeriod_, fAmplitude_);

    // In case there was a Reset packet
    if (nMeasurement < COUNT_MEASUREMENT_FOR_MOVING_AVG)
    {
      fMovingAvgYaw = IMPOSSIBLE_YAW_X_Z_VALUE;
      fMovingAvgX = IMPOSSIBLE_YAW_X_Z_VALUE;
      fMovingAvgZ = IMPOSSIBLE_YAW_X_Z_VALUE;
    }

    // Waiting for movement to start
    while ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        prvRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Checking the correctness of the combination on the input
    if ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW))
      prvRiscBehavior(TEnumRiscBehavior::RISC_WRONG_INPUT_COMBINATION, "Momentary movements between points of extrema");

    eStatePosition_ = TEnumStatePosition::STATE_NONE;

#ifdef DEBUG_SOFT
  prvDebugFunction(fMovingAvgYaw, fMovingAvgX, fMovingAvgZ, xFileToSave, fCoefRotation, fCoefTranslation, ssCoefShift, fTargetYaw_, fTargetX_);
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
                                              float &fTargetZ, OUT float &fCoefRotation, OUT float &fCoefTranslation,
                                              OUT int16_t &ssCoefShift, float &fPeriod, float &fAmplitude)
{
  static float fIntegralErrorYaw(0.f), fIntegralErrorX(0.f);
  static bool isShiftCalcNow(false);
  float fErrorYaw = fTargetYaw - fYaw;
  float fErrorX = fTargetX - fX;

  if ((fYaw == IMPOSSIBLE_YAW_X_Z_VALUE) && (fX == IMPOSSIBLE_YAW_X_Z_VALUE))
    goto return_prvRoataionTranslationCalculation;

  // Calculation integral errors
  fIntegralErrorYaw = fIntegralErrorYaw + fErrorYaw;
  fIntegralErrorX = fIntegralErrorX + fErrorX;
  // Calculation translation and rotation coefficients
  fCoefRotation = fErrorYaw * COEF_PROPORTIONAL_YAW + fIntegralErrorYaw * COEF_INTEGRAL_YAW * fPeriod;
  fCoefTranslation = fErrorX * COEF_PROPORTIONAL_X + fIntegralErrorX * COEF_INTEGRAL_X * fPeriod;

  // Calculation shift coefficient
  if (fDistance < MINIMAL_DISTANCE_VALUE_METER)
    ssCoefShift = (-1) * static_cast<int16_t>(floor((fTargetZ - fDistance) / (0.1) / fAmplitude));
  if (fDistance > fTargetZ)
    ssCoefShift = floor((fDistance - fTargetZ) / (0.1) / fAmplitude);
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
  static TProtocolInStroller xPacketOut_;
  TProtocolInAruco *pxPacketIn = reinterpret_cast<TProtocolInAruco *>(&xPacketOut_);
  static bool isResetWas_(true);

  xPacketOut_.usPreambule = 0x5555;
  xPacketOut_.ucIdPacket = 0x01;
  xPacketOut_.fRotation = fCoefRotation;
  xPacketOut_.fTranslation = fCoefTranslation;
  xPacketOut_.ssShift = ssShift;
  xPacketOut_.crc16 = crc16citt(reinterpret_cast<unsigned char *>(&xPacketOut_), sizeof(xPacketOut_) - 2);

  errno = 0;
  if (wiringPiSPIDataRW(SPI_CHANNEL, reinterpret_cast<unsigned char *>(&xPacketOut_), sizeof(xPacketOut_)) < 0)
    prvRiscBehavior(TEnumRiscBehavior::RISC_CANNOT_SEND_SPI_PACKET, strerror(errno));

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
    if ((pxPacketIn->ucIdPacket == ID_PACKET_RESET_WAS) && (isResetWas_ == false))
    {
      nMeasurement = 0;
      isResetWas_ = true;
    }
    fPeriod = pxPacketIn->fPeriod;
    fAmplitude = pxPacketIn->fAmplitude;
  }

  if (nMeasurement >= COUNT_MEASUREMENT_FOR_MOVING_AVG)
    isResetWas_ = false;

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
    prvGetYawRollPitch(quat[0], quat[1], quat[2], quat[3], yaw, roll, pitch);

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
  
  cout << "The percentage of calculated frames is " << (static_cast<float>(count) / static_cast<float>nAllFrames) << endl;
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


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/** @brief Debugging information
 */
void prvDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, float &fMovingAvgZ, ofstream &xFileToSave,
                      float &fCoefRot, float &fCoefTransl, int16_t ssCoefShift, float fTargetYaw, float fTargetX)
{
  float fDistance = sqrt(pow(fMovingAvgX, 2.f) + pow(fMovingAvgZ, 2.f));

  cout << "Target yaw = " << fTargetYaw << " ( " << (fTargetYaw * DEGRES_IN_RAD) << " degres)"  << 
                " ;  target X = " << fTargetX << " ( " << (fTargetX * 100.f) << " cm);" << endl;
  cout << "Moving Yaw = " << fMovingAvgYaw << " ( " << (fMovingAvgYaw * DEGRES_IN_RAD) << " degres);" << endl;
  cout << "Moving X = " << fMovingAvgX << " ( " << (fMovingAvgX * 100.f) << " cm);" << endl;
  cout << "Moving distance = " << fDistance << " ( " << (fDistance * 100.f) << " cm);" << endl;
  cout << "Coef rotation = " << fCoefRot << " ; Coef translation = " << fCoefTransl << " ; Coef shift = " << ssCoefShift << endl;
  cout << "=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=" << endl;

//  Saving the position to a file
#if DEBUG_SOFT < 3
  xFileToSave << std::to_string(fMovingAvgYaw * DEGRES_IN_RAD) + "	" + std::to_string(fMovingAvgX) << endl;
#endif
}
