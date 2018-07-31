// Image Capture algorithm 

#include "SerialComm.h"
// #include "ImgCapture.h"

#include <iostream>

#include <librealsense2/rs.hpp> // Realsense API
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// compile string: 
// gcc main.cpp  SerialComm.cpp -o exec `pkg-config --cflags --libs opencv` -lpthread -lrealsense2 -std=c++11 -lstdc++

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
std::tuple<uint8_t, uint8_t, uint8_t>  get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

int main(int argc, char *argv[])
{
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

	auto frames = pipe.wait_for_frames();
	
	// auto aligned_frames = align.process(frameset);

	// rs2::video_frame color = aligned_frames.first(RS2_STREAM_COLOR); //.get_color_frame();

	// rs2::depth_frame depth = aligned_frames.get_depth_frame();

	// auto aligned_frames = align.process(frameset);

	rs2::frame color = frames.get_color_frame(); //.get_color_frame();

	rs2::depth_frame depth = frames.get_depth_frame();
	
	pc.map_to(color);
	points = pc.calculate(depth);

	// points.export_to_ply("test.ply", color);

	auto pcl_points = points_to_pcl(points, color);

	pcl::io::savePCDFileASCII("my_point_cloud.pcd", *pcl_points);

	return 0;
}    

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	// std::cout << "test: " << texture_data[0] << std::endl;
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

		if (i == 0)
		{
			std::cout << "R: " << std::get<0>(current_color) << std::endl;
		}

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);
	}

	return cloud;
}