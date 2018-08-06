// Image Capture algorithm 

#include "SerialComm.h"

#include <iostream>   // i-o terminal
#include <sys/stat.h>
#include <sys/types.h>

#include <librealsense2/rs.hpp> // Realsense API
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

// compile string: 
// cmake && make clean && make

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
std::tuple<uint8_t, uint8_t, uint8_t>  get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
void dimensionFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_out, std::string dimension, const float lower, const float upper);

int main(int argc, char *argv[])
{	
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
	int serial_rc = SerialInit();
	if (serial_rc == 0)
	{
		std::cout << "Serial Initialisation Complete." << std::endl;
	}
	else 
	{
		std::cout << "Serial Initialisation Failed." << std::endl;
	}


	rs2::pointcloud pc;
	rs2::points points;

	// rs2::align align(RS2_STREAM_COLOR);
	rs2::pipeline pipe;

	pipe.start();

	// let auto-exposure and initialisation happen
	for (auto i = 0; i < 30; ++i)
	{
		pipe.wait_for_frames();
	}

	// let user decide filter values
    pcl::visualization::CloudViewer viewer("Cloud Viewer");	

	bool filter_done = false;
	auto frames = pipe.wait_for_frames();
	rs2::frame color = frames.get_color_frame(); //.get_color_frame();
	rs2::depth_frame depth = frames.get_depth_frame();
	
	
	pc.map_to(color);
	points = pc.calculate(depth);

	// load the point cloud
	auto pcl_points = points_to_pcl(points, color);
	viewer.showCloud(pcl_points);
	// viz.showCloud(pcl_points, "cloud");
	// viz.resetCamera();
	// filter values

	float lowerX = -1;
	float upperX = 1;
	float lowerY = -1;
	float upperY = 1;
	float lowerZ = 0.01;
	float upperZ = 1;				
	dimensionFilter(pcl_points, pcl_points, "z", lowerZ, upperZ);

	std::cout << "Filter the point cloud." << std::endl;

	std::string response;
	int state = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	while (!filter_done)
	{
		switch (state)
		{
			case 0: 
			{
				std::cout << "Lower X: ";
				std::cin >> lowerX;
				std::cout << "x = [" << lowerX << ", " << upperX << "] y = [" << lowerY << ", " << upperY << "] z = [" << lowerZ << ", " << upperZ << "]" << std::endl;

				dimensionFilter(pcl_points, temp, "x", lowerX, upperX);
				cout << "num points: " << std::to_string(temp->points.size()) << std::endl;

				viewer.showCloud(temp);
				// viz.spinOnce();
				std::cout << "Happy? [y/n]";

				std::cin >> response;

				if (response == "y") 
				{
					state = 1;
					// copy the cloud data, not the pointer -_-
					pcl::copyPointCloud(*temp, *pcl_points);
				}
				break;
			}
			case 1: 
			{
				std::cout << "Upper X: ";
				std::cin >> upperX;
				std::cout << "x = [" << lowerX << ", " << upperX << "] y = [" << lowerY << ", " << upperY << "] z = [" << lowerZ << ", " << upperZ << "]" << std::endl;


				dimensionFilter(pcl_points, temp, "x", lowerX, upperX);
				cout << "num points: " << std::to_string(temp->points.size()) << std::endl;

				viewer.showCloud(temp);

				// viz.spinOnce();
				std::cout << "Happy? [y/n]";

				std::cin >> response;

				if (response == "y") 
				{
					state = 2;
					pcl::copyPointCloud(*temp, *pcl_points);
				}		
				break;	
			}
			case 2: 
			{
				std::cout << "Lower Y: ";
				std::cin >> lowerY;
				std::cout << "x = [" << lowerX << ", " << upperX << "] y = [" << lowerY << ", " << upperY << "] z = [" << lowerZ << ", " << upperZ << "]" << std::endl;


				dimensionFilter(pcl_points, temp, "y", lowerY, upperY);
				cout << "num points: " << std::to_string(temp->points.size()) << std::endl;

				viewer.showCloud(temp);
				// viz.spinOnce();
				std::cout << "Happy? [y/n]";

				std::cin >> response;

				if (response == "y") 
				{
					state = 3;
					pcl::copyPointCloud(*temp, *pcl_points);
				}
				break;
			}
			case 3: 
			{
				std::cout << "Upper Y: ";
				std::cin >> upperY;
				std::cout << "x = [" << lowerX << ", " << upperX << "] y = [" << lowerY << ", " << upperY << "] z = [" << lowerZ << ", " << upperZ << "]" << std::endl;

				dimensionFilter(pcl_points, temp, "y", lowerY, upperY);
				cout << "num points: " << std::to_string(temp->points.size()) << std::endl;

				viewer.showCloud(temp);
				// viz.spinOnce();
				std::cout << "Happy? [y/n]";

				std::cin >> response;

				if (response == "y") 
				{
					state = 4;
					pcl::copyPointCloud(*temp, *pcl_points);
				}
				break;
			}
			case 4: 
			{
				std::cout << "Lower Z: ";
				std::cin >> lowerZ;
				std::cout << "x = [" << lowerX << ", " << upperX << "] y = [" << lowerY << ", " << upperY << "] z = [" << lowerZ << ", " << upperZ << "]" << std::endl;

				dimensionFilter(pcl_points, temp, "z", lowerZ, upperZ);
				cout << "num points: " << std::to_string(temp->points.size()) << std::endl;

				viewer.showCloud(temp);
				// viz.spinOnce();
				std::cout << "Happy? [y/n]";

				std::cin >> response;

				if (response == "y") 
				{
					state = 5;
					pcl::copyPointCloud(*temp, *pcl_points);
				}
				break;
			}
			case 5: 
			{
				std::cout << "Upper Z: ";
				std::cin >> upperZ;
				std::cout << "x = [" << lowerX << ", " << upperX << "] y = [" << lowerY << ", " << upperY << "] z = [" << lowerZ << ", " << upperZ << "]" << std::endl;

				dimensionFilter(pcl_points, temp, "z", lowerZ, upperZ);
				cout << "num points: " << std::to_string(temp->points.size()) << std::endl;

				viewer.showCloud(temp);
				// viz.spinOnce();
				std::cout << "Happy? [y/n]";

				std::cin >> response;

				if (response == "y") 
				{
					filter_done = true;
					pcl::copyPointCloud(*temp, *pcl_points);
				}

				break;
			}
			default: 
			{
				break;
			}
		}


	}



	for (int captureNum = 1; captureNum <= 24; captureNum++)
	{
		std::cout << captureNum << "/24" << std::endl;

		// run the stepper motor 15 degrees
		StepperSpin();
		// wait for the platform to get in position
		usleep(1500000);
		
		// capture and save the point-cloud
		auto frames = pipe.wait_for_frames();
		rs2::frame color = frames.get_color_frame(); //.get_color_frame();
		rs2::depth_frame depth = frames.get_depth_frame();
		
		pc.map_to(color);
		points = pc.calculate(depth);

		// load the point cloud
		auto pcl_points = points_to_pcl(points, color);
		cout << "num points: " << std::to_string(pcl_points->points.size()) << std::endl;

  		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

		// create a filter to get rid of bad x values
		dimensionFilter(pcl_points, cloud_filtered, "x", lowerX, upperX);

		// create a filter to get rid of bad y values
		dimensionFilter(cloud_filtered, cloud_filtered, "y", lowerY, upperY);

		// create a filter to get rid of bad z values
		dimensionFilter(cloud_filtered, cloud_filtered, "z", lowerZ, upperZ);

		viewer.showCloud(cloud_filtered);

		std::string filename = filedir;
		filename.append(std::to_string(captureNum));
		filename.append(".pcd");
		pcl::io::savePCDFileASCII(filename, *cloud_filtered);
		std::cout << "Point cloud saved to: " << filename << std::endl;
	}	

	// close the serial connection
	SerialEnd();
	return 0;
}    

void dimensionFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in,  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_out, std::string dimension, const float lower, const float upper)
{
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(point_cloud_in);
		pass.setFilterFieldName(dimension);
		pass.setFilterLimits(lower, upper);
		pass.filter(*point_cloud_out);
}

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);
	}

	return cloud;
}