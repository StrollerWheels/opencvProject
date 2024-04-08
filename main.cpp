#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <fstream>
#include <omp.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>

#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "crc.h"
#include "protocol.h"

#define DEBUG_SOFT

// #define TARGET_IS_CURRENT_POSITION

typedef enum
{
  STATE_NONE = 0,
  STATE_FORWARD,
  STATE_BACK,
} TEnumStatePosition;

typedef enum
{
  RISC_WRONG_INPUT_COMBINATION = 0,
  RISC_CAMERA_PROBLEM,
  RISC_INVALID_CAMERA_FILE,
  RISC_NOT_SETUP_SPI,
  RISC_CANNOT_SEND_SPI_PACKET,
} TEnumRiscBehavior;

using namespace std;
using namespace cv;

const double PI = 3.141592653589793;
const double DEGRES_IN_RAD = 180 / PI;

const float MARKER_LENGTH = 0.1; ///< Marker side length
const int CAMERA_NUMBER = 1;     ///< Camera number in OS
const int NO_PIN_FORWARD = 0;
const int NO_PIN_BACK = 1;
const int SPI_CHANNEL = 1;
const int SPI_PORT = 0;
const int SPI_BAUDRATE = 1000000;
const int SPI_MODE = 0;

const float COEF_PROPORTIONAL_YAW = 5.f;
const float COEF_INTEGRAL_YAW = 1.f;
const float COEF_DIFFERENTIAL_YAW = 0.f;
const float COEF_PROPORTIONAL_X = 3.f;
const float COEF_INTEGRAL_X = 0.5;
const float COEF_DIFFERENTIAL_X = 0.f;

const size_t COUNT_FRAMES = 10;

ofstream xFileToSave;                  ///< Debugging telemetry recording
Mat xCameraMatrix, xDistCoefficients;  ///< Camera calibration settings
VideoCapture xCaptureFrame;            ///< Object to capture a frame
cv::Mat xMarkerPoints(4, 1, CV_32FC3); ///< Coordinates of marker corners relative to the marker center

static bool prvReadCameraParameters(std::string filename, cv::Mat &xCameraMatrix, cv::Mat &xDistCoefficients);
static void prvGetYawRollPitch(double q0, double qx, double qy, double qz, double &yaw, double &roll, double &pitch);
static bool prvCaptureFrame(VideoCapture &xCapture, Mat &frame, size_t nAttempts);
static void prvRiscBehavior(TEnumRiscBehavior eErrorCode, string sError);
static void prvRoataionTranslationCalculation(float &fYaw, float &fX, float &fTargetYaw, float &fTargetX,
                                              float &fCoefRotation, float &fCoefTranslation);
static bool prvSendPacketToStroller(float &fCoefRotation_, float &fCoefTranslation_, float &fPeriod, float &fAmplitude);
static void prvInitializationSystem(ofstream &xFileToSave, Mat &xCameraMatrix, Mat &xDistCoefficients,
                                    VideoCapture &xCaptureFrame, Mat &xMarkerPoints);
static bool prvYawTranslationCalculation(queue<Mat> &pxFramesToCalc, cv::Mat &xMarkerPoints, Mat &xCameraMatrix,
                                         Mat &xDistCoefficients, float &fAvgYaw, float &fAvgX);
static void prvMovingAvgAndSendPacket(TEnumStatePosition &eStatePosition, float &fAvgYaw, float &fAvgX, float &fMovingAvgYaw,
                                      float &fMovingAvgX, VideoCapture &xCaptureFrame, Mat &xFrameCommon);
void prvDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, ofstream &xFileToSave);

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
int main(int argc, char *argv[])
{
  prvInitializationSystem(xFileToSave, xCameraMatrix, xDistCoefficients, xCaptureFrame, xMarkerPoints);

  for (;;)
  {
    static float fMovingAvgYaw_(0.f), fMovingAvgX_(0.f);             ///< Moving average value
    static float fPeriod_(0.f), fAmplitude_(0.f);                    ///< Skating period and amplitude
    Mat xFrameCommon, xFrameTemp;                                    ///< Captured frames
    static queue<Mat> pxFramesForward, pxFramesBack, pxFramesToCalc; ///< Captured frames at the points of trajectory extremum
    double yaw(0), pitch(0), roll(0);                                ///< Yaw, pitch, roll, in radian
    static TEnumStatePosition eStatePosition(STATE_NONE);            ///< Wheelchair position: at the nearest or farthest point from the marker, or between them
    float fAvgYaw(0.f), fAvgX(0.f);

    while (pxFramesForward.empty() == false)
      pxFramesForward.pop();
    while (pxFramesBack.empty() == false)
      pxFramesBack.pop();
    while (pxFramesToCalc.empty() == false)
      pxFramesToCalc.pop();

    while (((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == LOW)) ||
           ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == HIGH)))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        prvRiscBehavior(RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }

    // Closes point to the marker
    while ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW) &&
           (pxFramesForward.size() < COUNT_FRAMES))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameTemp, 10) == false)
        prvRiscBehavior(RISC_CAMERA_PROBLEM, "Unable to capture a frame");
      pxFramesForward.push(xFrameTemp);
      cout << "Capture closes was" << endl;
    }

    // Farthest point from the marker
    while ((digitalRead(NO_PIN_BACK) == HIGH) && (digitalRead(NO_PIN_FORWARD) == LOW) &&
           (pxFramesBack.size() < COUNT_FRAMES))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameTemp, 10) == false)
        prvRiscBehavior(RISC_CAMERA_PROBLEM, "Unable to capture a frame");
      pxFramesBack.push(xFrameTemp);
      cout << "Capture farthest was" << endl;
    }

    // Point defenition
    if ((pxFramesForward.empty() == false) && (pxFramesBack.empty() == true))
      eStatePosition = STATE_FORWARD;
    if ((pxFramesForward.empty() == true) && (pxFramesBack.empty() == false))
      eStatePosition = STATE_BACK;
    if ((pxFramesForward.empty() == false) && (pxFramesBack.empty() == false))
      eStatePosition = STATE_NONE;

    // Position calculation preparation
    switch (eStatePosition)
    {
    case STATE_FORWARD:
      if (pxFramesForward.size() < COUNT_FRAMES)
        pxFramesToCalc.push(xFrameCommon);
      while (pxFramesForward.empty() == false)
      {
        pxFramesToCalc.push(pxFramesForward.front());
        pxFramesForward.pop();
      }
      break;
    case STATE_BACK:
      if (pxFramesBack.size() < COUNT_FRAMES)
        pxFramesToCalc.push(xFrameCommon);
      while (pxFramesBack.empty() == false)
      {
        pxFramesToCalc.push(pxFramesBack.front());
        pxFramesBack.pop();
      }
      break;
    default:
      continue;
    }

    cout << "Count of frames = " + std::to_string(pxFramesToCalc.size()) << endl;

    if (prvYawTranslationCalculation(pxFramesToCalc, xMarkerPoints, xCameraMatrix, xDistCoefficients, fAvgYaw, fAvgX) == false)
    {
      fAvgYaw = fMovingAvgYaw_;
      fAvgX = fMovingAvgX_;
      cout << "Failed measurement" << endl;
    }

    prvMovingAvgAndSendPacket(eStatePosition, fAvgYaw, fAvgX, fMovingAvgYaw_, fMovingAvgX_, xCaptureFrame, xFrameCommon);

#ifdef DEBUG_SOFT
    prvDebugFunction(fMovingAvgYaw_, fMovingAvgX_, xFileToSave);
#endif
  }

  return 0;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
static bool prvReadCameraParameters(std::string filename, cv::Mat &xCameraMatrix, cv::Mat &xDistCoefficients)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["camera_matrix"] >> xCameraMatrix;
  fs["distortion_coefficients"] >> xDistCoefficients;
  return true;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
void prvGetYawRollPitch(double q0, double qx, double qy, double qz, double &yaw, double &roll, double &pitch)
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

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/** @brief Frame capture
 */
static bool prvCaptureFrame(VideoCapture &xCapture, Mat &frame, size_t nAttempts)
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

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
static void prvRiscBehavior(TEnumRiscBehavior eErrorCode, string sError)
{
  switch (eErrorCode)
  {
  case RISC_WRONG_INPUT_COMBINATION:
    cout << " ! ! ! Wrong input combination ! ! ! " << endl;
    break;
  case RISC_CAMERA_PROBLEM:
    cout << " ! ! ! Camera problems ! ! ! " << endl;
    break;
  case RISC_INVALID_CAMERA_FILE:
    cout << " ! ! ! Invalid camera file ! ! ! " << endl;
  case RISC_NOT_SETUP_SPI:
    cout << " ! ! ! Not setup SPI ! ! ! " << endl;
  case RISC_CANNOT_SEND_SPI_PACKET:
    cout << " ! ! ! PACKET SENDING ERROR ! ! ! " << endl;
  default:
    cout << " ! ! ! Uncertain behavior ! ! ! " << endl;
    break;
  }

  cout << sError << endl;

  for (;;)
    asm("NOP");
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
static void prvRoataionTranslationCalculation(float &fYaw, float &fX, float &fTargetYaw, float &fTargetX,
                                              float &fCoefRotation, float &fCoefTranslation)
{
  static float fIntegralErrorYaw(0.f), fIntegralErrorX(0.f);
  float fErrorYaw = fTargetYaw - fYaw;
  float fErrorX = fTargetX - fX;

  fIntegralErrorYaw = fIntegralErrorYaw + fErrorYaw;
  fIntegralErrorX = fIntegralErrorX + fErrorX;

  fCoefRotation = fErrorYaw * COEF_PROPORTIONAL_YAW + fIntegralErrorYaw * COEF_INTEGRAL_YAW;
  fCoefTranslation = fErrorX * COEF_PROPORTIONAL_X + fIntegralErrorX * COEF_INTEGRAL_X;

  if (fabsf(fCoefRotation) > 50.f)
  {
    cout << "!!! Rotation coefficient more than 50 per cent !!!" << endl;
    fCoefRotation = fCoefRotation > 0.f ? 50.f : -50.f;
  }
  if (fabsf(fCoefTranslation) > 50.f)
  {
    cout << "!!! Translation coefficient more than 50 per cent !!!" << endl;
    fCoefTranslation = fCoefTranslation > 0.f ? 50.f : -50.f;
  }
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
static bool prvSendPacketToStroller(float &fCoefRotation, float &fCoefTranslation, float &fPeriod, float &fAmplitude)
{
  static uint16_t usMarker(0.f);
  bool ret(true);
  static TProtocolInStroller xPacketOut = {.usPreambule = 0x5555, .ucIdPacket = 0x01, .fRotation = fCoefRotation, .fTranslation = fCoefTranslation};
  TProtocolInOrangePi *pxPacketIn = reinterpret_cast<TProtocolInOrangePi *>(&xPacketOut);
  int res(0);
  string sError;

  xPacketOut.ucMarker = usMarker++;
  xPacketOut.crc16 = crc16citt(reinterpret_cast<unsigned char *>(&xPacketOut), sizeof(xPacketOut) - 2);

  errno = 0;
  if (wiringPiSPIDataRW(SPI_CHANNEL, reinterpret_cast<unsigned char *>(&xPacketOut), sizeof(xPacketOut)) < 0)
    prvRiscBehavior(RISC_CANNOT_SEND_SPI_PACKET, strerror(errno));

  if ((pxPacketIn->crc16 != crc16citt(reinterpret_cast<unsigned char *>(pxPacketIn), sizeof(xPacketOut) - 2)) ||
      (pxPacketIn->ucMarker != (usMarker - 1)))
  {
    ret = false;
  }
  else
  {
    fPeriod = pxPacketIn->fPeriod;
    fAmplitude = pxPacketIn->fAmplitude;
  }

  return ret;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
static void prvInitializationSystem(ofstream &xFileToSave, Mat &xCameraMatrix, Mat &xDistCoefficients,
                                    VideoCapture &xCaptureFrame, Mat &xMarkerPoints)
{
  wiringPiSetup();
  pinMode(NO_PIN_FORWARD, INPUT);
  pinMode(NO_PIN_BACK, INPUT);

  errno = 0;
  if (wiringPiSPISetupMode(SPI_CHANNEL, SPI_PORT, SPI_BAUDRATE, SPI_MODE) < 0)
    prvRiscBehavior(RISC_NOT_SETUP_SPI, strerror(errno));

  xFileToSave.open("../Angles.ods");
  xFileToSave << "Yaw	X" << endl;

  if (prvReadCameraParameters("../Calibr_1920x1080.xml", xCameraMatrix, xDistCoefficients) == false)
    prvRiscBehavior(RISC_INVALID_CAMERA_FILE, "The settings file cannot be opened");

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

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/** @brief Yaw and translation calculation
 */
static bool prvYawTranslationCalculation(queue<Mat> &pxFramesToCalc, cv::Mat &xMarkerPoints, Mat &xCameraMatrix,
                                         Mat &xDistCoefficients, float &fAvgYaw, float &fAvgX)
{
  bool ret = true;
  Mat xFrameTemp1;
  vector<int> xIdDetectMarker;                             // Vector of identifiers of the detected markers
  vector<vector<Point2f>> xCornersMarker, xRejectedMarker; // Vector of detected marker corners
  aruco::DetectorParameters xDetectorParams;
  aruco::Dictionary dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  static aruco::ArucoDetector xDetector_(dictionary, xDetectorParams); // Detection of markers in an image
  size_t nMarkers(0);                                                  // Number of found markers (must be 1)
  double yaw(0), roll(0), pitch(0);
  float fSumYaw(0.f), fSumX(0.f);
  uint32_t ulFrameNo(0);

  while (pxFramesToCalc.empty() == false)
  {
    // Position calculation
    xFrameTemp1 = pxFramesToCalc.front();
    xDetector_.detectMarkers(xFrameTemp1, xCornersMarker, xIdDetectMarker, xRejectedMarker);
    pxFramesToCalc.pop();
    nMarkers = xCornersMarker.size();
    vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if (xIdDetectMarker.empty() == false)
    {
      solvePnP(xMarkerPoints, xCornersMarker.at(0), xCameraMatrix, xDistCoefficients, rvecs.at(0), tvecs.at(0), false);
      ulFrameNo++;
    }
    else
    {
      continue;
    }

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

    // Sum calculation
    fSumX = fSumX + tvecs.at(0)[0];
    fSumYaw = fSumYaw + yaw;
  }

  // Averaging
  if (ulFrameNo != 0)
  {
    fAvgYaw = fSumYaw / static_cast<float>(ulFrameNo);
    fAvgX = fSumX / static_cast<float>(ulFrameNo);
  }

  if (ulFrameNo == 0)
    ret = false;

  return ret;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/** @brief Moving average and send packet to stroller
 */
static void prvMovingAvgAndSendPacket(TEnumStatePosition &eStatePosition, float &fAvgYaw, float &fAvgX, float &fMovingAvgYaw,
                                      float &fMovingAvgX, VideoCapture &xCaptureFrame, Mat &xFrameCommon)
{
  static float fYawForward_(0.f), fX_Forward_(0.f); // Calculated values iat the point closest to the marker
  static float fPeriod_(0.f), fAmplitude_(0.f);
  static bool isFirstMeasurement_(true); // To lock the start position
  static float fTargetYaw_(0.f), fTargetX_(0.f);
  float fCoefRotation(0.f), fCoefTranslation(0.f); // Coefficients of rotation and translation

  switch (eStatePosition)
  {
  case STATE_FORWARD:
    fYawForward_ = fAvgYaw;
    fX_Forward_ = fAvgX;
    // Waiting for movement to start
    while ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        prvRiscBehavior(RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }
    // Checking the correctness of the combination on the input
    if ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH))
      prvRiscBehavior(RISC_WRONG_INPUT_COMBINATION, "Momentary movements between points of extrema");
    eStatePosition = STATE_NONE;
    break;
  case STATE_BACK:
    fMovingAvgYaw = (2.f * fYawForward_ + fAvgYaw) / 3.f;
    fMovingAvgX = (2.f * fX_Forward_ + fAvgX) / 3.f;

#ifdef TARGET_IS_CURRENT_POSITION
    if (isFirstMeasurement_ == true)
    {
      isFirstMeasurement_ = false;
      fTargetYaw_ = fMovingAvgYaw;
      fTargetX_ = fMovingAvgX;
    }
#endif

    prvRoataionTranslationCalculation(fMovingAvgYaw, fMovingAvgX, fTargetYaw_, fTargetX_, fCoefRotation, fCoefTranslation);

    prvSendPacketToStroller(fCoefRotation, fCoefTranslation, fPeriod_, fAmplitude_);

    // Waiting for movement to start
    while ((digitalRead(NO_PIN_FORWARD) == LOW) && (digitalRead(NO_PIN_BACK) == HIGH))
    {
      if (prvCaptureFrame(xCaptureFrame, xFrameCommon, 10) == false)
        prvRiscBehavior(RISC_CAMERA_PROBLEM, "Unable to capture a frame");
    }
    // Checking the correctness of the combination on the input
    if ((digitalRead(NO_PIN_FORWARD) == HIGH) && (digitalRead(NO_PIN_BACK) == LOW))
      prvRiscBehavior(RISC_WRONG_INPUT_COMBINATION, "Momentary movements between points of extrema");
    eStatePosition = STATE_NONE;
    break;
  default:
    break;
  }
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/** @brief Debugging information
 */
void prvDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, ofstream &xFileToSave)
{
  cout << "Yaw = " << fMovingAvgYaw << " ( " << (fMovingAvgYaw * DEGRES_IN_RAD) << " degres);" << endl;
  cout << "X = " << fMovingAvgX << " ( " << (fMovingAvgX * 100.f) << " cm);" << endl;
  cout << "=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=" << endl;
  // this_thread::sleep_for(3333ms);
  //  Saving the position to a file
  xFileToSave << std::to_string(fMovingAvgYaw * DEGRES_IN_RAD) + "	" + std::to_string(fMovingAvgX) << endl;
}
