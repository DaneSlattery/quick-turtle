// Image Capture algorithm 

#include "SerialComm.h"
#include "RealSenseGrabber.h"

#include <iostream>   // i-o terminal
#include <sys/stat.h>
#include <sys/types.h>

// compile string: 
// cmake && make clean && make

int main(int argc, char *argv[])
{	
	const float platformAngle = 15.0f;
	float numRotations = 360.0f/platformAngle;

	// read in the file directory where this should be saved
	std::string filedir;
	if (argc > 1)
	{
		filedir = argv[1];
		std::string sys_cmd = "mkdir -p ";
		sys_cmd.append(filedir);
		system(sys_cmd.c_str());
		std::cout << "Directory created at: " << filedir << std::endl;
	}
	else
	{
		std::cout << "Output path not specified" << std::endl;
		std::cout << "\t Usage: ./quick_turtle /path/to/directory/" << std::endl;
		return -1;
	}

	// Initialise Micro-controller communication
	SerialComm comm;
	int serial_rc = comm.serial_init(); // SerialInit();

	if (serial_rc == 0)
	{
		std::cout << "Serial Initialisation Complete." << std::endl;
	}
	else 
	{
		std::cout << "Serial Initialisation Failed." << std::endl;
	}

	RealSenseGrabber grabber;
	grabber.user_filter_XYZ();


	for (int captureNum = 1; captureNum <= 24; captureNum++)
	{
		std::cout << captureNum << "/24" << std::endl;

		// run the stepper motor 15 degrees, this should return when the arduino does
		comm.stepper_spin();

		// wait for the platform to get in position
		//usleep(1500000);

		// grab a point cloud
		grabber.grab_point_cloud();
		// apply the pass through filters to it
		grabber.apply_filters();
		grabber.display_cloud();

		// save the point cloud
		std::string filename = filedir;
		filename.append(std::to_string(captureNum));
		filename.append(".pcd");
		grabber.save_cloud_to_disk(filename);
		std::cout << "Point cloud saved to: " << filename << std::endl;
	}	

	// close the serial connection
	comm.serial_end();
	return 0;
}    

