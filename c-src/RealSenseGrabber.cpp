#include "RealSenseGrabber.h"

RealSenseGrabber::RealSenseGrabber(): viewer("Cloud Viewer")
{
    // start streaming
    pipe.start();

    // let auto-exposure and initialisation happen
	for (auto i = 0; i < 30; ++i)
	{
		pipe.wait_for_frames();
	}

}

int RealSenseGrabber::user_filter_XYZ()
{
    RealSenseGrabber::PCLCloudPtr temp (new PCLCloud);
    bool filterDone = false;


    // put a new point cloud into the system
    grab_point_cloud();
    // filter the obvious bad z values (negative z values can't exist)
    dimension_filter(pcl_pc, pcl_pc, "z", lowerZ, upperZ);
    // show the cloud
    viewer.showCloud(pcl_pc);
    std::cout << "Number of points before filters: " << std::to_string(pcl_pc->points.size()) << std::endl;


    std::cout << "Filter the point cloud..." << std::endl;

    int state = 0;

    while (!filterDone)
    {
        switch (state)
        {
            case 0:
            {
                if (user_filter_capture("x", temp) == 0) state = 1;
                break;
            }
            case 1:
            {
                if (user_filter_capture("y", temp) == 0) state = 2;
                break;
            }  
            case 2:
            {
                if (user_filter_capture("z", temp) == 0) filterDone = true;
                break;
            }
            default:
                break;

        }
    }


    return 0;
}

int RealSenseGrabber::user_filter_capture(std::string dim, RealSenseGrabber::PCLCloudPtr temp)
{
    // capture the user input
    float lower, upper;
    std::cout << "Lower " << dim << ": ";
    std::cin >> lower;
    std::cout << std::endl;
    std::cout << "Upper " << dim << ": ";
    std::cin >> upper;

    // filter out in the specified dimension
    dimension_filter(pcl_pc, temp, dim, lower, upper);
    std::cout << "Number of points after filter: " << std::to_string(temp->points.size()) << std::endl;

    // display the filtered cloud
    viewer.showCloud(temp);

    // check if the defined boundaries are good
    std::string response;
    std::cout << "Happy? [y/n]: ";
    std::cin >> response;

    if (response == "y")
    {
        // copy the temp cloud into the current cloud
        pcl::copyPointCloud(*temp, *pcl_pc);
        if (dim == "x")
        {
            lowerX = lower;
            upperX = upper;
        }
        else if (dim == "y")
        {
            lowerY = lower;
            upperY = upper;
        }
        else if (dim == "z")
        {
            lowerZ = lower;
            upperZ = upper;
        }
        // exit well
        std::cout << dim << " dimension values saved." << std::endl;
        return 0;
    }
    // exit fail
    return -1;
}


int RealSenseGrabber::grab_point_cloud()
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
    return 0;
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

int RealSenseGrabber::display_cloud()
{
    viewer.showCloud(pcl_pc);
    return 0;
}

int RealSenseGrabber::save_cloud_to_disk(std::string filename)
{
    pcl::io::savePCDFileASCII(filename, *pcl_pc);
}

