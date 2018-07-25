// Image Capture algorithm 

#include "SerialComm.h"
// #include "ImgCapture.h"

#include <iostream>
#include <thread>
#include <mutex>

std::mutex captureFlag_mutex;
int captureNum;
int captureFlag;

bool programDone;
std::mutex programDone_mutex;

#include <opencv2/opencv.hpp> // opencv api
// #include <opencv2/imgproc/imgproc.h>
#include <librealsense2/rs.hpp> // Realsense API

rs2::pipeline piper;
rs2::colorizer color_map;

void displayLoop()
{
	const auto window_name = "Display Image";
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

	while (cv::waitKey(1) < 0 && cvGetWindowHandle(window_name))
	{
		captureFlag_mutex.lock();
		bool myFlag = captureFlag;
		captureFlag = false;
		captureFlag_mutex.unlock();

		if (myFlag) 
		{
			rs2::frameset data = piper.wait_for_frames();

			rs2::frame depth = color_map(data.get_depth_frame());
			rs2::frame color = data.get_color_frame();

			auto vf_depth = depth.as<rs2::video_frame>();

			const int h_depth = vf_depth.get_height();
			const int w_depth = vf_depth.get_width();

			cv::Mat depthImage(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
			imshow(window_name, depthImage);
			
			// save the images now
			std::string filename = "./data/example/depth/";
			filename.append(std::to_string(captureNum));
			filename.append("depth.png");
			imwrite(filename, depthImage);


			auto vf_color = color.as<rs2::video_frame>();

			const int h_color = vf_color.get_height();
			const int w_color = vf_color.get_width();

			cv::Mat colorImage(cv::Size(w_color, h_color), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
			
			// save the images now
			filename = "./data/example/color/";
			filename.append(std::to_string(captureNum));
			filename.append("color.png");
			imwrite(filename, colorImage);
		}

		programDone_mutex.lock();
		bool myDone = programDone;  
		programDone_mutex.unlock();

		if (myDone)
		{
			break;
		}
	}
}


void RunAlgorithm()
{
	for (captureNum = 1; captureNum <= 24; captureNum++)
	{
		std::cout << captureNum << "/24" << std::endl;
		// run the stepper motor 15 degrees
		StepperSpin();

		// set the flag for the other thread
		captureFlag_mutex.lock();
		captureFlag = true;
		captureFlag_mutex.unlock();
		usleep(1500000);
	}

	programDone_mutex.lock();
	programDone = true;
	programDone_mutex.unlock();
}

// compile string: 
// gcc main.cpp ImgCapture.cpp SerialComm.cpp -o exec `pkg-config --cflags --libs opencv` -lrealsense2 -std=c++11 -lstdc++

int main(int argc, char *argv[])
{
	int serial_rc = SerialInit();

	if (serial_rc == 0)
	{
		std::cout << "Serial Initialisation Complete." << std::endl;
	}
	else 
	{
		std::cout << "Serial Initialisation Failed." << std::endl;
	}

	piper.start();

	// let auto-exposure and initialisation happen
	for (auto i = 0; i < 30; ++i)
	{
		piper.wait_for_frames();
	}

	std::thread displayThread(displayLoop);
	RunAlgorithm();

	displayThread.join();
	piper.stop();

	return 0;
}    
