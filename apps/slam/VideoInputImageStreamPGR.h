#pragma once

//#include "videoInput.h"
#include "lsd_slam/io_wrapper/input_image_stream.h"
#include <FlyCapture2.h>


namespace lsd_slam
{
	class Undistorter;
}


class VideoInputImageStreamPGR
	: public lsd_slam::InputImageStream
{
public:
	VideoInputImageStreamPGR();
	~VideoInputImageStreamPGR();

	void run();
	void setCalibration(const std::string& file);
	void operator()();

private:
	//ReadSetting readSetting;
	FlyCapture2::Camera camera;
	bool haveCalib;
	lsd_slam::Undistorter* undistorter;
};
