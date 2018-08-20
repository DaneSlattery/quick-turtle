#include "RealSenseGrabber.h"

RealSenseGrabber::RealSenseGrabber()
{
    // // start streaming
    // pipe.start();

    // // let auto-exposure and initialisation happen
	// for (auto i = 0; i < 30; ++i)
	// {
	// 	pipe.wait_for_frames();
	// }
}

int RealSenseGrabber::init()
{
    // start streaming
    pipe.start();

    // let auto-exposure and initialisation happen
	for (auto i = 0; i < 30; ++i)
	{
		pipe.wait_for_frames();
	}
}

RealSenseGrabber::PCLCloudPtr RealSenseGrabber::grab_point_cloud()
{
    // wait for frames from the camera
    rs2::frameset frameSet = pipe.wait_for_frames();
    // extract depth and color
    rs2::frame color = frameSet.get_color_frame();
    rs2::depth_frame depth = frameSet.get_depth_frame();

    // map the point cloud to color
    rs_pc.map_to(color);
    // map the color to depth
    rs_points = rs_pc.calculate(depth);

    // update the current point cloud as a PCL cloud
    pcl_pc = points_to_pcl(rs_points, color);

    return pcl_pc;
}

RealSenseGrabber::PCLCloudPtr RealSenseGrabber::points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
	RealSenseGrabber::PCLCloudPtr cloud(new RealSenseGrabber::PCLCloud);

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

		RealSenseGrabber::texColor current_color;
		current_color = get_tex_color(color, tex_coords[i]);

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);
        cloud->points[i].a = 0;
	}

	return cloud;
}

RealSenseGrabber::texColor RealSenseGrabber::get_tex_color(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return RealSenseGrabber::texColor(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

int RealSenseGrabber::dimension_filter(RealSenseGrabber::PCLCloudPtr point_cloud_in, RealSenseGrabber::PCLCloudPtr point_cloud_out, std::string dimension, const float lower, const float upper)
{
    pcl::PassThrough<RealSenseGrabber::PointT> pass;
    pass.setInputCloud(point_cloud_in);
    pass.setFilterFieldName(dimension);
    pass.setFilterLimits(lower, upper);
    pass.filter(*point_cloud_out);
    return 0;
}

int RealSenseGrabber::apply_filters()
{
    dimension_filter(pcl_pc, pcl_pc, "x", lowerX, upperX);
    dimension_filter(pcl_pc, pcl_pc, "y", lowerY, upperY);
    dimension_filter(pcl_pc, pcl_pc, "z", lowerZ, upperZ);
    return 0;
}


