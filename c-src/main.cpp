// Image Capture algorithm 

#include "SerialComm.h"

#include <iostream>   // i-o terminal
#include <sys/stat.h>
#include <sys/types.h>

#include <librealsense2/rs.hpp> // Realsense API
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

// compile string: 
// cmake && make clean && make

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
std::tuple<uint8_t, uint8_t, uint8_t>  get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

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

		auto pcl_points = points_to_pcl(points, color);
  		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		
		// create a filter to get rid of bad values
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.1, 0.3);

		pass.filter(*cloud_filtered);

		// for (int i = 0; i < cloud_filtered->points.size(); ++i)
		// {
		// 	std::cout << "x: " << cloud_filtered->points[i].x << "y: " << pcl_points->points[i].y << "z: " << pcl_points->points[i].z << std::endl;
		// }

		std::string filename = filedir;
		filename.append(std::to_string(captureNum));
		filename.append(".pcd");
		pcl::io::savePCDFileASCII(filename, *cloud_filtered);
		std::cout << "Point cloud saved to: " << filename << std::endl;
	}	

	
	return 0;
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