#include "VideoInputImageStreamPGR.h"
#include <cv.h>
#include <boost/thread.hpp>
#include "lsd_slam/util/undistorter.h"


VideoInputImageStreamPGR::VideoInputImageStreamPGR()
	: haveCalib(false)
	, undistorter(nullptr)
{
	//readSetting.symbolicLink = symbolicDeviceName;
	width_ = 0;
	height_ = 0;
	imageBuffer = new lsd_slam::NotifyBuffer<lsd_slam::TimestampedMat>(8);
}


VideoInputImageStreamPGR::~VideoInputImageStreamPGR()
{
	delete imageBuffer;
	camera.StopCapture();
	
	camera.Disconnect();
}


void VideoInputImageStreamPGR::run()
{
	boost::thread thread(boost::ref(*this));
}


void VideoInputImageStreamPGR::setCalibration(const std::string& file)
{
	if (file == "")
	{
		printf("NO camera calibration file!\n");
	}
	else
	{
		undistorter = lsd_slam::Undistorter::getUndistorterForFile(file.c_str());

		if (!undistorter)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			assert("Failed to read camera calibration from file... wrong syntax?");
		}

		fx_ = undistorter->getK().at<double>(0, 0);
		fy_ = undistorter->getK().at<double>(1, 1);
		cx_ = undistorter->getK().at<double>(2, 0);
		cy_ = undistorter->getK().at<double>(2, 1);

		width_ = undistorter->getOutputWidth();
		height_ = undistorter->getOutputHeight();
	}

	haveCalib = true;
}


void VideoInputImageStreamPGR::operator()()
{
	if (!haveCalib)
	{
		assert("no calibration");
		return;
	}

	FlyCapture2::Error error;
	FlyCapture2::CameraInfo camInfo;

	// Connect the camera
	error = camera.Connect(0);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "Failed to connect to camera" << std::endl;
		return;
	}

	// Get the camera info and print it out
	error = camera.GetCameraInfo(&camInfo);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "Failed to get camera info from camera" << std::endl;
		return;
	}
	std::cout << camInfo.vendorName << " "
		<< camInfo.modelName << " "
		<< camInfo.serialNumber << std::endl;

	FlyCapture2::GigEImageSettingsInfo imageSettingsInfo;
	error = camera.GetGigEImageSettingsInfo(&imageSettingsInfo);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "Camera info reading failed" << std::endl;;
		return;
	}

	FlyCapture2::GigEImageSettings imageSettings;
	imageSettings.offsetX = 0;
	imageSettings.offsetY = 4;
	imageSettings.height = 592;// imageSettingsInfo.maxHeight;
	imageSettings.width = imageSettingsInfo.maxWidth;
	imageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RGB8;

	std::cout << "Setting GigE image settings..." << std::endl;

	error = camera.SetGigEImageSettings(&imageSettings);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "Camera settings failed" << std::endl;
		return;
	}

	FlyCapture2::Mode imageMode;
	imageMode = FlyCapture2::MODE_4;
	error = camera.SetGigEImagingMode(imageMode);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "Imaging mode setting failed" << std::endl;
		return;
	}

	error = camera.StartCapture();
	if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
	{
		std::cout << "Bandwidth exceeded" << std::endl;
		return;
	}
	else if (error != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "Failed to start image capture" << std::endl;
		return;
	}


	while (1)
	{
		lsd_slam::TimestampedMat bufferItem;
		bufferItem.timestamp = lsd_slam::Timestamp::now();

		// Get the image
		FlyCapture2::Image rawImage;
		FlyCapture2::Error error = camera.RetrieveBuffer(&rawImage);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			std::cout << "capture error" << std::endl;
			continue;
		}

		// convert to rgb
		FlyCapture2::Image rgbImage;
		rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
		cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);

		// convert to OpenCV IplImage*
		IplImage* frame = cvCloneImage(&(IplImage)image);

		if (undistorter != nullptr)
		{
			assert(undistorter->isValid());
			cv::Mat m = cv::cvarrToMat(frame, false);
			undistorter->undistort(m, bufferItem.data);
		}
		else
		{
			bufferItem.data = cv::cvarrToMat(frame, false);
		}

		//bufferItem.data = cv::Mat(frame, true);
		imageBuffer->pushBack(bufferItem);
	}

	exit(1);
}
