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
	// read in the file directory where this should be saved
	std::string filedir;
	
	// need to create flags to see if the file needs to be read, or to create a new file for new data
	// using -c for camera
	// using -f for file
	bool fileInput = false;
	bool cameraInput = false;
	if (argc > 2)
	{

		if ( strcmp (argv[1], "-c")== 0)
		{
			cameraInput = true; // set the flag to be true
			std::cout << "User would like to capture data using the camera" << std::endl;
			filedir = argv[2];
			std::string sys_cmd = "mkdir -p ";
			sys_cmd.append(filedir);
			system(sys_cmd.c_str());
			std::cout << "Directory created at: " << filedir << std::endl;

		} else if (strcmp (argv[1], "-f")== 0)
		{
			fileInput = true;
			std::cout << "user would like to use data already saved to the file" << std::endl;
			filedir = argv[2];
			//opendir[filedir];
			std::cout << "Opening folder " << filedir << std::endl;
		}
		
	}
	else
	{
		std::cout << "Output path not specified" << std::endl;
		std::cout << "\t Usage: ./quick_turtle [data-option] /path/to/directory/" << std::endl;
		return -1;
	}

	if(cameraInput)
	{
		// Since new data is to be captured, the micro needs to be initiated
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
	}
	return 0;
}    

