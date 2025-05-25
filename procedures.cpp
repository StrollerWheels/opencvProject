/**
 ******************************************************************************
 * @file           : procedures.cpp
 * @brief          **Related function**
 * @details Related functions that do not affect the program logic
 ******************************************************************************
 * @attention
 ******************************************************************************
  @authors [Novikov Andrey](https://t.me/AndreyNikolaevichPerm)
  @version 1.0
  @date 16.07.2024
 ******************************************************************************
 */

#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>

#include "procedures.h"
#include "settings.h"
#include "crc.h"

using namespace std;
using namespace cv;

static size_t cameraNo_ = 0;

extern cv::Mat xMarkerPoints;
extern Mat xCameraMatrix, xDistCoefficients; ///< Camera calibration settings
extern VideoCapture xCaptureFrame;           ///< Object to capture a frame

static bool prvReadCameraParameters(std::string filename, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients);

static void prvAlertWCU(uint8_t ucEvent);

static void *prvCheckShutdown(void *args);

/**
 * @brief Initialization function
 */
void vInitializationSystem(std::ofstream &xFileToSave, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients,
                           OUT cv::VideoCapture &xCaptureFrame, OUT cv::Mat &xMarkerPoints)
{
  pthread_t xThreadCheckPinShutdown;

  wiringPiSetup();
  pinMode(NO_PIN_FORWARD, INPUT);
  pinMode(NO_PIN_BACK, INPUT);

  pthread_create (&xThreadCheckPinShutdown, NULL, prvCheckShutdown, NULL);
  pthread_detach(xThreadCheckPinShutdown);

  errno = 0;
  if (wiringPiSPISetupMode(SPI_CHANNEL, SPI_PORT, SPI_BAUDRATE, SPI_MODE) < 0)
    vRiscBehavior(TEnumRiscBehavior::RISC_NOT_SETUP_SPI, strerror(errno));

#if DEBUG_ON > 0
  xFileToSave.open("../Angles.ods");
  xFileToSave << "AvgYaw	AvgX AvgZ MovingAvgYaw" << endl;
#endif

  if (prvReadCameraParameters("/home/Calibr_1920x1080.xml", xCameraMatrix, xDistCoefficients) == false)
    vRiscBehavior(TEnumRiscBehavior::RISC_INVALID_CAMERA_FILE, "The settings file cannot be opened");

  // Open camera
  size_t nAttemptOpen = 10;
  do
  {
    xCaptureFrame.open(CAMERA_NUMBER_0, cv::CAP_V4L2);
    if (xCaptureFrame.isOpened() == false)
#if DEBUG_ON == 1
      cout << "Cannot open camera 0" << endl;
#else
      asm("NOP");
#endif
    this_thread::sleep_for(100ms);
  } while ((xCaptureFrame.isOpened() == false) && (nAttemptOpen-- > 2));
  if (nAttemptOpen < 3)
  {
    nAttemptOpen = 10;
    do
    {
      xCaptureFrame.open(CAMERA_NUMBER_1, cv::CAP_V4L2);
      if (xCaptureFrame.isOpened() == false)
#if DEBUG_ON == 1
        cout << "Cannot open camera 1" << endl;
#else
        asm("NOP");
#endif
      this_thread::sleep_for(100ms);
    } while ((xCaptureFrame.isOpened() == false) && (nAttemptOpen-- > 2));
    if (nAttemptOpen < 3)
    {
      vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to open a camera");
    }
    else
    {
      cameraNo_ = CAMERA_NUMBER_1;
      cout << "Open camera 1 (/dev/video1)" <<endl;
    }
  }
  else
  {
    cameraNo_ = CAMERA_NUMBER_0;
    cout << "Open camera 0 (/dev/video0)" <<endl;
  }

  // Camera setup
  xCaptureFrame.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  xCaptureFrame.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  xCaptureFrame.set(cv::CAP_PROP_FPS, 5);
  double dWidth = xCaptureFrame.get(cv::CAP_PROP_FRAME_WIDTH);   // get the width of frames of the video
  double dHeight = xCaptureFrame.get(cv::CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
  double dFps = xCaptureFrame.get(cv::CAP_PROP_FPS);
#if DEBUG_ON == 1
  cout << "camera width = " << std::to_string(dWidth) << ", height = " << std::to_string(dHeight) << ", FPS = " << std::to_string(dFps) << endl;
#endif

  // Set coordinate system
  xMarkerPoints.ptr<Vec3f>(0)[0] = Vec3f(-MARKER_LENGTH / 2.f, MARKER_LENGTH / 2.f, 0);
  xMarkerPoints.ptr<Vec3f>(0)[1] = Vec3f(MARKER_LENGTH / 2.f, MARKER_LENGTH / 2.f, 0);
  xMarkerPoints.ptr<Vec3f>(0)[2] = Vec3f(MARKER_LENGTH / 2.f, -MARKER_LENGTH / 2.f, 0);
  xMarkerPoints.ptr<Vec3f>(0)[3] = Vec3f(-MARKER_LENGTH / 2.f, -MARKER_LENGTH / 2.f, 0);
}

/**
 * @brief Risc management function
 */
void vRiscBehavior(TEnumRiscBehavior eErrorCode, string sError)
{
  switch (eErrorCode)
  {
  case TEnumRiscBehavior::RISC_WRONG_INPUT_COMBINATION:
#if DEBUG_ON == 1
    cout << " ! ! ! Wrong input combination ! ! ! " << endl;
#endif
    /***/ return;
    break;
  case TEnumRiscBehavior::RISC_CAMERA_PROBLEM:
#if DEBUG_ON == 1
    cout << " ! ! ! Camera problems ! ! ! " << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_CAMERA_PROBLEM);
    break;
  case TEnumRiscBehavior::RISC_INVALID_CAMERA_FILE:
#if DEBUG_ON == 1
    cout << " ! ! ! Invalid camera file ! ! ! " << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_CAMERA_PROBLEM);
  case TEnumRiscBehavior::RISC_NOT_SETUP_SPI:
#if DEBUG_ON == 1
    cout << " ! ! ! Not setup SPI ! ! ! " << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_NO_PACKET_FROM_WCU);
  case TEnumRiscBehavior::RISC_CANNOT_SEND_SPI_PACKET:
#if DEBUG_ON == 1
    cout << " ! ! ! PACKET SENDING ERROR ! ! ! " << endl;
#endif
    break;
  case TEnumRiscBehavior::RISC_CANNOT_CALC_TRAGET:
#if DEBUG_ON == 1
    cout << " ! ! ! CANNOT TO CALCULATION THE TARGETS ! ! ! " << endl;
#endif
#if DEBUG_ON == 1
    cout << sError << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_IMPOSSIBLE_CALC_SETPOINT);
  case TEnumRiscBehavior::RISK_ALERT_SIDEWAYS:
#if DEBUG_ON == 1
    cout << sError << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_SIDEWAYS_DRIFT_EXCEEDED_PERMISSIBLE);
  case TEnumRiscBehavior::RISK_ALERT_ROTATION_ANGLE:
#if DEBUG_ON == 1
    cout << sError << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_ROTATION_ANGLE_EXCEEDED_PERMISSIBLE);
  case TEnumRiscBehavior::RISK_ALERT_MARKER_NOT_IDENTIFIED:
#if DEBUG_ON == 1
    cout << sError << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_MARKER_IS_GONE);
  case TEnumRiscBehavior::RISK_ALERT_MORE_MISSES:
#if DEBUG_ON == 1
    cout << sError << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_MANY_MISSES);
  case TEnumRiscBehavior::RISK_ALERT_NO_PACKET_FROM_WCU:
#if DEBUG_ON == 1
    cout << sError << endl;
#endif
    prvAlertWCU(ID_PACKET_IN_WCU_NO_PACKET_FROM_WCU);
  default:
#if DEBUG_ON == 1
    cout << " ! ! ! Uncertain behavior ! ! ! " << endl;
#endif
    break;
  }

#if DEBUG_ON == 1
  cout << sError << endl;
#endif

  for (;;)
    asm("NOP");
}

/**
 * @brief Frame capture
 */
bool bCaptureFrame(VideoCapture &xCapture, OUT Mat &frame, size_t nAttempts)
{
  bool ret(true);

  if (xCapture.read(frame) == false)
  {
    ret = false;
    // First open the camera
    do
    {
      this_thread::sleep_for(200ms);
      xCapture.open(cameraNo_, cv::CAP_V4L2);
      if (xCapture.isOpened() == false)
      {
#if DEBUG_ON == 1
        cout << "Cannot open camera" << endl;
#endif
        vRiscBehavior(TEnumRiscBehavior::RISC_CAMERA_PROBLEM, "Unable to capture a frame");
        continue;
      }

#if DEBUG_ON == 1
      cout << "Cannot read a xFrameCommon" << endl;
#endif
    } while ((xCapture.read(frame) == false) && (--nAttempts > 0));
  }

  if (nAttempts > 0)
    ret = true;

  return ret;
}

/**
 * @brief
 *
 * @param[in] xFrame
 * @param[out] yaw
 * @param[out] x
 * @param[out] distance
 * @return true
 * @return false
 */
bool bTelemetryCalculation(cv::Mat &xFrame, OUT double &yaw, OUT double &x, OUT double &z)
{
  aruco::Dictionary dictionary = aruco::getPredefinedDictionary(/*aruco::DICT_ARUCO_MIP_36h12*/ cv::aruco::DICT_5X5_50); /***/
  aruco::DetectorParameters xDetectorParams;
  static aruco::ArucoDetector xDetector_(dictionary, xDetectorParams); // Detection of markers in an image
  vector<int> xIdDetectMarker;                                         // Vector of identifiers of the detected markers
  vector<vector<Point2f>> xCornersMarker, xRejectedMarker;             // Vector of detected marker corners
  bool ret(true);
  double r[3];
  double t[3];
  double lenVecRotation;
  double angle;
  double quat[4];
  double roll, pitch;

  try
  {
    xDetector_.detectMarkers(xFrame, xCornersMarker, xIdDetectMarker, xRejectedMarker); /// @warning Was exception!!!
  }
  catch (...)
  {
#if DEBUG_ON == 1
    cout << "There is been an exception: xDetector_.detectMarkers()" << endl;
#endif
  }
  /*terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.9.0-dev) /home/orangepi/opencv-4.x/modules/objdetect/src/aruco/aruco_detector.cpp:872: error: (-215:Assertion failed) !_image.empty() in function 'detectMarkers'*/
  auto nMarkers = xCornersMarker.size();
  vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

  if (xIdDetectMarker.empty() == false)
    solvePnP(xMarkerPoints, xCornersMarker.at(0), xCameraMatrix, xDistCoefficients, rvecs.at(0), tvecs.at(0), false);
  else
  {
    ret = false;
    goto return_bTelemetryCalculation;
  }

  // Quaternion calculation
  r[0] = rvecs.at(0)[0];
  r[1] = rvecs.at(0)[1];
  r[2] = rvecs.at(0)[2];
  t[0] = tvecs.at(0)[0];
  t[1] = tvecs.at(0)[1];
  t[2] = tvecs.at(0)[2];
  lenVecRotation = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  r[0] = r[0] / lenVecRotation;
  r[1] = r[1] / lenVecRotation;
  r[2] = r[2] / lenVecRotation;
  angle = lenVecRotation / 2.f;
  quat[0] = cos(angle);
  quat[1] = sin(angle) * r[0];
  quat[2] = sin(angle) * r[1];
  quat[3] = sin(angle) * r[2];
  // Euler angles calculation
  vGetYawRollPitch(quat[0], quat[1], quat[2], quat[3], OUT yaw, OUT roll, OUT pitch);
  x = t[0];
  z = t[2];

return_bTelemetryCalculation:
  return ret;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vGetYawRollPitch(double q0, double qx, double qy, double qz, OUT double &yaw, OUT double &roll, OUT double &pitch)
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

/**
 * @brief Output debugging information and save it to a file
 */
void vDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, float &fMovingCorrelatedAvgX, float &fMovingAvgZ, std::ofstream &xFileToSave,
                    float &fCoefRot, float &fCoefTransl, int16_t ssCoefShift, float fTargetYaw, float fTargetX)
{
  float fDistance = sqrt(pow(fMovingAvgX, 2.f) + pow(fMovingAvgZ, 2.f));

#if DEBUG_ON == 1
  cout << "Target yaw = " + std::to_string(fTargetYaw) + " ( " + std::to_string((fTargetYaw * DEGRES_IN_RAD)) + " degres)" +
              " ;  target X = " + std::to_string(fTargetX) + " ( " + std::to_string((fTargetX * 100.f)) + " cm);"
       << endl;
  cout << "Moving Yaw = " + std::to_string(fMovingAvgYaw) + " ( " + std::to_string((fMovingAvgYaw * DEGRES_IN_RAD)) + " degres);" << endl;
  cout << "Moving X = " + std::to_string(fMovingAvgX) + " ( " + std::to_string((fMovingAvgX * 100.f)) + " cm);" << endl;
  cout << "Moving correlated X = " + std::to_string(fMovingCorrelatedAvgX) + " ( " + std::to_string((fMovingCorrelatedAvgX * 100.f)) + " cm);" << endl;
  cout << "Moving Z = " + std::to_string(fMovingAvgZ) + " ( " + std::to_string((fMovingAvgZ * 100.f)) + " cm);" << endl;
  cout << "Moving distance = " + std::to_string(fDistance) + " ( " + std::to_string((fDistance * 100.f)) + " cm);" << endl;
  cout << "Coef rotation = " + std::to_string(fCoefRot) + " ; Coef translation = " + std::to_string(fCoefTransl) + " ; Coef shift = " + std::to_string(ssCoefShift) << endl;
  cout << "=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=	=" << endl;
#endif
//  Saving the position to a file
#if DEBUG_SOFT < 3
#if DEBUG_ON > 0
  xFileToSave << std::to_string(fMovingAvgYaw * DEGRES_IN_RAD) + "	" + std::to_string(fMovingAvgX) << endl;
#endif
#endif
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
bool bSendPacketToStroller(uint8_t ucId, float &fCoefRotation, float &fCoefTranslation, int16_t ssShift, float fDistance, OUT float &fPeriod, OUT float &fAmplitude)
{
  bool ret(true);
  int res(0);
  string sError;
  static TProtocolInScuMotionCmd xPacketOut_;
  TProtocolInOuCondition *pxPacketIn = reinterpret_cast<TProtocolInOuCondition *>(&xPacketOut_);
  static size_t nErrorPacketRow__(0);

  xPacketOut_.usPreambule = PREAMBULE_IN_WCU;
  xPacketOut_.eIdPacketAruco = ucId;
  xPacketOut_.fRotation = fCoefRotation;
  xPacketOut_.fTranslation = fCoefTranslation;
  xPacketOut_.ssShift = ssShift;
  xPacketOut_.fDistance = fDistance;
  xPacketOut_.crc16 = crc16citt(reinterpret_cast<unsigned char *>(&xPacketOut_), sizeof(xPacketOut_) - 2);

  errno = 0;
  if (wiringPiSPIDataRW(SPI_CHANNEL, reinterpret_cast<unsigned char *>(&xPacketOut_), sizeof(xPacketOut_)) < 0)
    vRiscBehavior(TEnumRiscBehavior::RISC_CANNOT_SEND_SPI_PACKET, strerror(errno));

  /***/ static size_t all__ = 0;
  all__++;
  if (pxPacketIn->crc16 != crc16citt(reinterpret_cast<unsigned char *>(pxPacketIn), sizeof(xPacketOut_) - 2))
  {
    /***/ static size_t err__ = 0;
    ret = false;
#if DEBUG_ON == 1
    cout << " ! ! ! CRC16 error ! ! !  error count is " << std::to_string(++err__) << " from " << std::to_string(all__) << " packets" << endl;
#endif
    if (ucId == ID_PACKET_IN_WCU_MOTION_CMD)
      nErrorPacketRow__++;
  }
  else
  {
    fPeriod = pxPacketIn->fPeriod;
    fAmplitude = pxPacketIn->fAmplitude;
    if (ucId == ID_PACKET_IN_WCU_MOTION_CMD)
      nErrorPacketRow__ = 0;
  }

  // SAFETY CHECK: Bad connection
  if ((nErrorPacketRow__ >= COUNT__OF_DATA_PACKET_SENDS * SAFETY_MAX_COUNT_ITERATION_NO_COMMUNICATION_WCU) && (ucId == ID_PACKET_IN_WCU_MOTION_CMD))
    vRiscBehavior(TEnumRiscBehavior::RISK_ALERT_NO_PACKET_FROM_WCU, " ! ! ! Bad connection with WCU ! ! ! ");

  return ret;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/**
 * @brief Read camera parameters
 */
bool prvReadCameraParameters(std::string filename, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["camera_matrix"] >> xCameraMatrix;
  fs["distortion_coefficients"] >> xDistCoefficients;
  return true;
}

void prvAlertWCU(uint8_t ucEventId)
{
  float fTempIn1(0.f), fTempIn2(0.f), fTempIn3(0.f), fTempOut1(0.f), fTempOut2(0.f);
  int16_t ssTempIn(0.f);

  for (;;)
  {
    bSendPacketToStroller(ucEventId, fTempIn1, fTempIn2, ssTempIn, fTempIn3, OUT fTempOut1, OUT fTempOut2);
    this_thread::sleep_for(2000ms);
  }
}


/**
 * @brief Shutdown either by pin level or by USART message
 * @note Is executed in a separate thread
*/
static void *prvCheckShutdown (void *args)
{
  static size_t nConfirmation__(0);
  static auto fdUsart__(0);
  static size_t nBytes__(0);
  static uint8_t pucDataUsartIn__[100];
  static TMessageInOuByUsart *pxMsgToOu__ = (TMessageInOuByUsart*)pucDataUsartIn__;

  this_thread::sleep_for(5000ms);

  if ((fdUsart__ = serialOpen ("/dev/ttyS3", 115200)) < 0)  
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    
  pinMode(NO_PIN_SHUTDOWN, INPUT);

  for (;;)
  {
    if (digitalRead(NO_PIN_SHUTDOWN) == HIGH)
    {
      nConfirmation__++;
    } else {
      nConfirmation__ = 0;
    }

    // Either by pin shutdown
    if (nConfirmation__ > 5000)/***/
    {
      this_thread::sleep_for(380000ms);
      if (xCaptureFrame.isOpened() == true)
        xCaptureFrame.release();
      std::system("shutdown now");
    }

    // Or by message shutdown
    nBytes__ = read(fdUsart__, pucDataUsartIn__, 100);
    if (nBytes__ > 0)
    {
      nBytes__ = 0;
      if ((pxMsgToOu__->usPreambule != PREAMBULE_IN_OU) || 
          (crc16citt(pucDataUsartIn__, sizeof(TMessageInOuByUsart) - 2) != pxMsgToOu__->crc16))
          continue;
      switch (pxMsgToOu__->eMsgToOuUsart)
      {
        case MSG_IN_OU_SHUTDOWN :
          std::system("shutdown now");
          break;
        default :
          break;
      }
    }

    this_thread::sleep_for(750ms);
  }

  pthread_exit(0);
}