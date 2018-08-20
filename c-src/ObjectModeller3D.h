// Serial Interface
//

#ifndef OBJECT_MODELLER_3D_H
#define OBJECT_MODELLER_3D_H

#define DEBUG_OBJECT_MODELLER_3D 0

#include "RealSenseGrabber.h"
#include "SerialComm.h"

#include <pcl/visualization/cloud_viewer.h> // cloud viewer
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h> // allows for transforms

// enumerator for the input type
enum class InputType {Camera, Directory};

//
class ObjectModeller3D
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PCLCloud;
    typedef pcl::PointCloud<PointT>::Ptr PCLCloudPtr;
    
private:
    InputType inputType;            //
    int angle;                      //
    int numRotations;               //
    std::string outFileDir;         //
    std::string inFileDir;          //
    SerialComm comm;                //
    RealSenseGrabber rsGrabber;     //
    // the main container
    std::vector<ObjectModeller3D::PCLCloudPtr> alignedClouds; 
    float lowerX = -0.1;
    float upperX = 0.1;

    float lowerZ = -0.1;
    float upperZ = 0.02;
    
    float lowerY = -0.1;
    float upperY = 0.1;

    float voxelLeafSize = 0.002f;

    float statFilterMean = 20;
    float statFilterStdDev = 0.5;

    //
    ObjectModeller3D::PCLCloudPtr get_point_cloud(int cloudNum);
    
    //
    void down_sample(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, float leafSize);
    
    //
    void stat_filter(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, float mean, float stdDev);
    
    //
    int dimension_filter(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, std::string dimension, const float lower, const float upper);
    
    // transform from camera to world axis
    void camera_to_world(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output);

    // perform generalised 6D registration
    float register_point_cloud(ObjectModeller3D::PCLCloudPtr target, ObjectModeller3D::PCLCloudPtr source);

    // 
    float offset_platform_rotation(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, int cloudNum);

public:
    // constructor
    ObjectModeller3D(InputType _inputType, int _angle, std::string fileDir, float _leafSize, float mean, float stdDev, float _lowerX, float _upperX, float _lowerY, float _upperY, float _lowerZ, float _upperZ);

    // create output directory
    int create_output_directory(std::string filename);

    // perform whole model generation
    int generate_model();
};

#endif