// Serial Interface
//

#ifndef REALSENSE_GRABBER_H
#define REALSENSE_GRABBER_H

#define DEBUG_GRABBER 0

#include <librealsense2/rs.hpp> // Realsense API

#include <pcl/visualization/cloud_viewer.h> // cloud viewer
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h> // allows for transforms


// a self contained grabber class for the realsense camera.
/* This class is intended to separate the realsense and PCL 
* API from the user. They do not have to handle the pointers,
* or the API calls. It acts as an API for them to call.
* The class is modelled on a simple PCL grabber for other cameras.
*/
class RealSenseGrabber
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PCLCloud;
    typedef pcl::PointCloud<PointT>::Ptr PCLCloudPtr;
    
private:
    typedef std::tuple<uint8_t, uint8_t, uint8_t> texColor;
    // realsense point cloud
    rs2::pointcloud rs_pc;
    // realsense point list
	rs2::points rs_points;
    // realsense capture pipeline
    rs2::pipeline pipe;
    // pcl current cloud
    RealSenseGrabber::PCLCloudPtr pcl_pc;

    // filter dimensions
    float lowerX = -1;
	float upperX = 1;
	float lowerY = -1;
	float upperY = 1;
	float lowerZ = 0.01;
	float upperZ = 1;

    RealSenseGrabber::PCLCloudPtr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
    RealSenseGrabber::texColor get_tex_color(rs2::video_frame texture, rs2::texture_coordinate texcoords);

public:
    // constructor
    RealSenseGrabber();
 
    // start the grabber
    int init();

    // pcl pass through filter
    int dimension_filter(RealSenseGrabber::PCLCloudPtr point_cloud_in, RealSenseGrabber::PCLCloudPtr point_cloud_out, std::string dimension, const float lower, const float upper);
    
    // get user input to  filter all dimensions
    int user_filter_XYZ();

    // grab a frame and return a point cloud
    RealSenseGrabber::PCLCloudPtr grab_point_cloud();
    
    // get user input to filter one dimension
    int user_filter_capture(std::string dim, RealSenseGrabber::PCLCloudPtr temp);

    // apply the set filters to the saved point cloud
    int apply_filters();

};

#endif