// Image Capture algorithm 

#include "SerialComm.h"
// #include "ImgCapture.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <sys/stat.h>
#include <sys/types.h>

std::mutex captureFlag_mutex;
int captureNum;
int captureFlag;

bool programDone;
std::mutex programDone_mutex;

#include <opencv2/opencv.hpp> // opencv api
#include <librealsense2/rs.hpp> // Realsense API

rs2::pipeline piper;
rs2::colorizer color_map;



std::string filedir;

void displayLoop(std::string outfilename, rs2::align align);

void RunAlgorithm();

void SaveFrameAsImage(rs2::frame myFrame, std::string windowname, std::string filedir, std::string imageType, int capturenum);

// compile string: 
// gcc main.cpp  SerialComm.cpp -o exec `pkg-config --cflags --libs opencv` -lpthread -lrealsense2 -std=c++11 -lstdc++

int main(int argc, char *argv[])
{

	// read in the file directory where this should be saved
	if (argc > 1)
	{
		filedir = argv[1];
	}
	else
	{
		std::cout << "Output path not specified" << std::endl;
		std::cout << "\t Usage: ./exec ./data/dataset/" << std::endl;
		return -1;
	}


	// Initialise Micro-controller communication
	int serial_rc = SerialInit();

	if (serial_rc == 0)
	{
		std::cout << "Serial Initialisation Complete." << std::endl;
	}
	else 
	{
		std::cout << "Serial Initialisation Failed." << std::endl;
	}


	// TODO: after testing, the system should only function if everything connects

	// Start streaming images from the camera
	rs2::pipeline_profile profile = piper.start();

	rs2_stream align_to = RS2_STREAM_COLOR;

	rs2::align align(align_to);

	// let auto-exposure and initialisation happen
	for (auto i = 0; i < 30; ++i)
	{
		piper.wait_for_frames();
	}

	// start displaying what the camera sees
	std::thread displayThread(displayLoop, filedir, align);

	// run the image capture algorithm
	RunAlgorithm();
	

	// wait for the display thread to return
	displayThread.join();

	// stop streaming from the camera (safe-disconnect)
	piper.stop();

	// TODO: disconnect serial.

	return 0;
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

		// wait for the image to be captured and saved
		usleep(1500000);
	}

	// the loop is finished, tell the other thread to stop displaying
	programDone_mutex.lock();
	programDone = true;
	programDone_mutex.unlock();
}

void displayLoop(std::string outfiledir, rs2::align align)
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

			auto processed = align.process(data);

			rs2::frame depth = color_map(data.get_depth_frame());
			rs2::frame color = data.get_color_frame();
			// rs2::frame color = processed.first(RS2_STREAM_COLOR);
			// rs2::frame depth = processed.get_depth_frame();

			SaveFrameAsImage(depth, window_name, outfiledir, "depth", captureNum);
			SaveFrameAsImage(color, window_name, outfiledir, "color", captureNum);

			// auto vf_depth = depth.as<rs2::video_frame>();

			// const int h_depth = vf_depth.get_height();
			// const int w_depth = vf_depth.get_width();

			// cv::Mat depthImage(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
			// imshow(window_name, depthImage);
			
			// // save the images now
			// std::string filename = "./data/example/depth/";
			// filename.append(std::to_string(captureNum));
			// filename.append("depth.png");
			// imwrite(filename, depthImage);


			// auto vf_color = color.as<rs2::video_frame>();

			// const int h_color = vf_color.get_height();
			// const int w_color = vf_color.get_width();

			// cv::Mat colorImage(cv::Size(w_color, h_color), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
			
			// // save the images now
			// filename = "./data/example/color/";
			// filename.append(std::to_string(captureNum));
			// filename.append("color.png");
			// imwrite(filename, colorImage);
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

void SaveFrameAsImage(rs2::frame myFrame, std::string windowname, std::string filedir, std::string imageType, int capturenum)
{
	auto vidFrame = myFrame.as<rs2::video_frame>();

	// get frame dimensions
	const int h = vidFrame.get_height();
	const int w = vidFrame.get_width();

	// make an OpenCV material to display
	cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)myFrame.get_data(), cv::Mat::AUTO_STEP);
	cvtColor(image, image, cv::COLOR_BGR2RGB);

	// show the image
	imshow(windowname, image);

	// create the directory
	std::string sys_cmd = "mkdir -p ";
	sys_cmd.append(filedir);
	std::string colorDirCmd = sys_cmd;
	colorDirCmd.append("color");
	system(colorDirCmd.c_str());
	std::string depthDirCmd = sys_cmd;
	depthDirCmd.append("depth");
	system(depthDirCmd.c_str());

	
	// save the image
	std::string filename;
	if (imageType == "depth")
	{
		filename = filedir.append("depth/");
		filename.append(std::to_string(capturenum));
		filename.append(".png");
	}
	else if (imageType == "color")
	{
		filename = filedir.append("color/");
		filename.append(std::to_string(capturenum));
		filename.append(".png");
	}

	imwrite(filename, image);
}

