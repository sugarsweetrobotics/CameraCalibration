// -*- C++ -*-
/*!
 * @file  CameraCalibration.cpp
 * @brief Camera calibration component using OpenCV 2.4.8
 * @date $Date$
 *
 * $Id$
 */

#include "CameraCalibration.h"

// Module specification
// <rtc-template block="module_spec">
static const char* cameracalibration_spec[] =
  {
    "implementation_id", "CameraCalibration",
    "type_name",         "CameraCalibration",
    "description",       "Camera calibration component using OpenCV 2.4.8",
    "version",           "1.0.1",
    "vendor",            "Kenichi Ohara, Meijo University, Yuki Suga, Sugar Sweet Robotics",
    "category",          "ImageProcessing",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.image_num", "10",
    "conf.default.chess_pattern_row", "7",
    "conf.default.chess_pattern_column", "10",
    "conf.default.chess_pattern_size", "24.0",
    "conf.default.output_file_name", "calib.yml",
    // Widget
    "conf.__widget__.image_num", "text",
    "conf.__widget__.chess_pattern_row", "text",
    "conf.__widget__.chess_pattern_column", "text",
    "conf.__widget__.chess_pattern_size", "text",
    "conf.__widget__.output_file_name", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
CameraCalibration::CameraCalibration(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_InImageIn("InImage", m_InImage),
    m_CameraCaptureServicePort("CameraCaptureService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
CameraCalibration::~CameraCalibration()
{
}



RTC::ReturnCode_t CameraCalibration::onInitialize()
{
  is_alive = true;
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("InImage", m_InImageIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_CameraCaptureServicePort.registerConsumer("CameraCaptureService", "Img::CameraCaptureService", m_CameraCaptureService);
  
  // Set CORBA Service Ports
  addPort(m_CameraCaptureServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("image_num", m_image_num, "10");
  bindParameter("chess_pattern_row", m_chess_pattern_row, "7");
  bindParameter("chess_pattern_column", m_chess_pattern_column, "10");
  bindParameter("chess_pattern_size", m_chess_pattern_size, "24.0");
  bindParameter("output_file_name", m_output_file_name, "calib.yml");

  width = 0;
  height = 0;
  channels = 0;
	found_num = 0;
	false_num = 0;
	base = 0;
	
  
  // </rtc-template>
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CameraCalibration::onFinalize()
{
  is_alive = false;
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t CameraCalibration::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraCalibration::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t CameraCalibration::onActivated(RTC::UniqueId ec_id)
{
  is_active = true;

#ifdef WIN32
  return onInitCalib(ec_id);
#elif __APPLE__
  return RTC::RTC_OK;
#else
  return onInitCalib(ec_id);
#endif
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraCalibration::onInitCalib(RTC::UniqueId ec_id) {
	//Show the calibration setting
	printf("======================Camera calibration setting================\n");
	printf("Image number                               : %d\n", m_image_num);
	printf("Chess pattern row number                   : %d\n", m_chess_pattern_row);
	printf("Chess pattern column number                : %d\n", m_chess_pattern_column);
	printf("Chess pattern size[mm]                     : %3.1f\n", m_chess_pattern_size);
	printf("Output file name stored calibration results: %s\n", m_output_file_name.c_str());
	printf("================================================================\n");

	//Memory allocation and initialize parameters for calibration
	pattern_size = cvSize(m_chess_pattern_row, m_chess_pattern_column);
	objects = (CvPoint3D32f *)cvAlloc (sizeof(CvPoint3D32f) * m_chess_pattern_row * m_chess_pattern_column * m_image_num);
	corners = (CvPoint2D32f *)cvAlloc (sizeof(CvPoint2D32f) * m_chess_pattern_row * m_chess_pattern_column * m_image_num);
	intrinsic = cvCreateMat (3, 3, CV_64FC1);
	rotation = cvCreateMat (1, 3, CV_32FC1);
	translation = cvCreateMat (1, 3, CV_32FC1);
	distortion = cvCreateMat (5, 1, CV_64FC1);
	
	//Initialization of 3D space corrdinate for chess pattern
	for( int i(0); i < m_image_num; i++)
	{
		for( int j(0); j < m_chess_pattern_row; j++)
		{
			for( int k(0); k < m_chess_pattern_column; k++)
			{
				objects[ i * m_chess_pattern_row * m_chess_pattern_column + j * m_chess_pattern_column + k].x = j * m_chess_pattern_size;
				objects[ i * m_chess_pattern_row * m_chess_pattern_column + j * m_chess_pattern_column + k].y = k * m_chess_pattern_size;
				objects[ i * m_chess_pattern_row * m_chess_pattern_column + j * m_chess_pattern_column + k].z = 0.0;
				
			}
		}
	}
	
	cvInitMatHeader(&object_points, m_chess_pattern_row * m_chess_pattern_column * m_image_num, 3, CV_32FC1, objects);

	p_count = (int *)malloc(m_image_num * sizeof(int));

	printf("==========================Command list==========================\n");
	printf("t : capture and store the image for calibration\n");
	printf("================================================================\n");
	cvNamedWindow("Live image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow ("Corner Detection Result", CV_WINDOW_AUTOSIZE);

	return RTC::RTC_OK;
}


RTC::ReturnCode_t CameraCalibration::onDeactivated(RTC::UniqueId ec_id)
{
  is_active = false;
#ifdef WIN32
  return onFiniCalib(ec_id);
#elif __APPLE__
  return RTC::RTC_OK;
#else
  return onFiniCalib(ec_id);
#endif
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CameraCalibration::onFiniCalib(RTC::UniqueId ec_id) {
	//Close the window for live image
	cvDestroyWindow("Live image");
	cvDestroyWindow("Corner Detection Result");
	
	//Free allocated memory
	cvFree(&objects);
	cvFree(&corners);
	free(p_count);
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&rotation);
	cvReleaseMat(&translation);
	cvReleaseMat(&distortion);

	//Clear parameters
	found_num = 0;
	
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraCalibration::onProcess(RTC::UniqueId ec_id) {
	//Inport data check
	if(m_InImageIn.isNew())
	{
		m_InImageIn.read();
		//***********************************************************************************
		//***********************************************************************************
		//Following part is the template for common camera interface
		//Don't change following part
		//***********************************************************************************
		//***********************************************************************************    		
		width = m_InImage.data.image.width;
		height = m_InImage.data.image.height;
		channels = (m_InImage.data.image.format == Img::CF_GRAY) ? 1 :
			   (m_InImage.data.image.format == Img::CF_RGB || m_InImage.data.image.format == Img::CF_PNG || m_InImage.data.image.format == Img::CF_JPEG) ? 3 :
			   (m_InImage.data.image.raw_data.length()/width/height);
		RTC_TRACE(("Capture image size %d x %d", width, height));
		RTC_TRACE(("Channels %d", channels));
		
		if(channels == 3)
			image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 3);
		else
			image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1);

		long data_length = m_InImage.data.image.raw_data.length();
		long image_size = width * height * channels;

		if( m_InImage.data.image.format == Img::CF_RGB )
		{
			for(int i(0); i<height; ++i)
				memcpy(&image->imageData[i*image->widthStep],&m_InImage.data.image.raw_data[i*width*channels],sizeof(unsigned char)*width*channels);
			if(channels == 3)
				cvCvtColor(image, image, CV_RGB2BGR);
		}
		else if( m_InImage.data.image.format == Img::CF_JPEG || m_InImage.data.image.format == Img::CF_PNG )
		{
			printf("Sorry, cannot use compressed image in current version!!");
			printf("Please change input image color format to RGB");
			return RTC::RTC_ERROR;
		}
		//***********************************************************************************
		//End of the template part
		//***********************************************************************************
		//‰æ‘œ‚ð•\Ž¦
		cvShowImage("Live image", image);
		int key = cvWaitKey(5);

		//===================================================================================
		//Calibration Process
		//===================================================================================
		if( key == 't')
		{
			found = cvFindChessboardCorners (image, pattern_size, &corners[found_num * m_chess_pattern_row * m_chess_pattern_column], &corner_count);
			if (found)
			{
				//Modifying the corner position to subpixel precision
				IplImage *src_gray = cvCreateImage (cvGetSize (image), IPL_DEPTH_8U, 1);
				cvCvtColor (image, src_gray, CV_BGR2GRAY);
				cvFindCornerSubPix (src_gray, &corners[ found_num * m_chess_pattern_row * m_chess_pattern_column ], corner_count,
					cvSize (3, 3), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
				cvDrawChessboardCorners (image, pattern_size, &corners[ found_num * m_chess_pattern_row * m_chess_pattern_column ], corner_count, found);
				p_count[found_num] = corner_count;
				printf("Captured image: %d \n", found_num);
				cvShowImage ("Corner Detection Result", image);
				cvWaitKey (100);
				found_num++;
				if( found_num == m_image_num )
				{
					//Calibration based on the result of corner detection
					cvInitMatHeader (&image_points, m_image_num * m_chess_pattern_row * m_chess_pattern_column, 1, CV_32FC2, corners);
					cvInitMatHeader (&point_counts, m_image_num, 1, CV_32SC1, p_count);
					
					//Intrinsic parameter estimation
					printf("Estimate intrinsic parameter\n" ); 
					cvCalibrateCamera2 (&object_points, &image_points, &point_counts, cvSize (width, height), intrinsic, distortion);
					
					//Extrinsic parameter estimation
					printf("Estimate extrinsic parameter\n" ); 
					cvGetRows (&image_points, &sub_image_points, base * m_chess_pattern_row * m_chess_pattern_column, (base + 1) * m_chess_pattern_row * m_chess_pattern_column);
					cvGetRows (&object_points, &sub_object_points, base * m_chess_pattern_row * m_chess_pattern_column, (base + 1) * m_chess_pattern_row * m_chess_pattern_column);
					cvFindExtrinsicCameraParams2 (&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);
					
					//Output calibration result to YAML type file.
					printf("Output calibration result to %s\n", m_output_file_name.c_str() );
					fs = cvOpenFileStorage (m_output_file_name.c_str(), 0, CV_STORAGE_WRITE);
					cvWriteInt (fs, "image_width", width);
					cvWriteInt (fs, "image_height", height);
					cvWrite (fs, "camera_matrix", intrinsic);
					cvWrite (fs, "rotation", rotation);
					cvWrite (fs, "translation", translation);
					cvWrite (fs, "distortion_coefficients", distortion);
					cvReleaseFileStorage (&fs);
					
					found_num = 0;
				}
				cvReleaseImage(&src_gray);
			}
			else
			{
				printf("Cannot detect the corners from the image!\n");
			}
		}
		if(image != NULL)
		{
			cvReleaseImage(&image);
		}
	}
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraCalibration::onExecute(RTC::UniqueId ec_id)
{
#ifdef WIN32
  return onProcess(0);
#elif __APPLE__
  return RTC::RTC_OK;
#else
  return onProcess(0);
#endif
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CameraCalibration::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraCalibration::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraCalibration::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraCalibration::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraCalibration::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void CameraCalibrationInit(RTC::Manager* manager)
  {
    coil::Properties profile(cameracalibration_spec);
    manager->registerFactory(profile,
                             RTC::Create<CameraCalibration>,
                             RTC::Delete<CameraCalibration>);
  }
  
};


