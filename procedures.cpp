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
 

 static bool prvReadCameraParameters(std::string filename, OUT cv::Mat &xCameraMatrix, OUT cv::Mat &xDistCoefficients);
 
 

/**
 * @brief Initialization function 
 */
void vInitializationSystem(OUT ofstream &xFileToSave, OUT Mat &xCameraMatrix, OUT Mat &xDistCoefficients,
                             OUT VideoCapture &xCaptureFrame, OUT Mat &xMarkerPoints)
{
  wiringPiSetup();
  pinMode(NO_PIN_FORWARD, INPUT);
  pinMode(NO_PIN_BACK, INPUT);

  errno = 0;
  if (wiringPiSPISetupMode(SPI_CHANNEL, SPI_PORT, SPI_BAUDRATE, SPI_MODE) < 0)
    vRiscBehavior(TEnumRiscBehavior::RISC_NOT_SETUP_SPI, strerror(errno));

  xFileToSave.open("../Angles.ods");
  xFileToSave << "AvgYaw	AvgX AvgZ MovingAvgYaw" << endl;

  if (prvReadCameraParameters("../Calibr_1920x1080.xml", xCameraMatrix, xDistCoefficients) == false)
    vRiscBehavior(TEnumRiscBehavior::RISC_INVALID_CAMERA_FILE, "The settings file cannot be opened");

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


/**
 * @brief Risc management function 
 */
void vRiscBehavior(TEnumRiscBehavior eErrorCode, string sError)
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
  case TEnumRiscBehavior::RISC_CANNOT_CALC_TRAGET:
    cout << " ! ! ! CANNOT TO CALCULATION THE TARGETS ! ! ! " << endl;
    cout << sError << endl;
    for(;;){}
  default:
    cout << " ! ! ! Uncertain behavior ! ! ! " << endl;
    break;
  }

  cout << sError << endl;

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
void vDebugFunction(float &fMovingAvgYaw, float &fMovingAvgX, float &fMovingAvgZ, ofstream &xFileToSave,
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