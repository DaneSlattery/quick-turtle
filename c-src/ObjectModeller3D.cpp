#include "ObjectModeller3D.h"

#include <iostream>

#include <pcl/io/pcd_io.h> // Allows for .pcd to be read from disk
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/visualization/pcl_visualizer.h>

ObjectModeller3D::ObjectModeller3D(InputType _inputType, int _angle, std::string fileDir, float _leafSize, float mean, float stdDev, float _lowerX, float _upperX, float _lowerY, float _upperY, float _lowerZ, float _upperZ)
{
    inputType = _inputType;
    angle = _angle;
    numRotations = 360/angle;
    inFileDir = fileDir;
    voxelLeafSize = _leafSize;
    statFilterMean = mean;
    statFilterStdDev = stdDev;
    lowerX = _lowerX;
    upperX = _upperX;
    lowerY = _lowerY;
    upperY = _upperY;
    lowerZ = _lowerZ;
    upperZ = _upperZ;

    create_output_directory(inFileDir);

    if (inputType == InputType::Camera)
    {
        rsGrabber.init();
        comm.serial_init();
    }
}

int ObjectModeller3D::create_output_directory(std::string fileDir)
{
    std::string sys_cmd = "mkdir -p ";
    outFileDir = fileDir;
    outFileDir.append("output/");
    sys_cmd.append(outFileDir);
    system(sys_cmd.c_str());
    if (DEBUG_OBJECT_MODELLER_3D) std::cout << sys_cmd << std::endl;
    std::cout << "Directory created at: " << outFileDir << std::endl;
}

int ObjectModeller3D::generate_model()
{
    ObjectModeller3D::PCLCloudPtr cloud (new ObjectModeller3D::PCLCloud);
    ObjectModeller3D::PCLCloudPtr target (new ObjectModeller3D::PCLCloud);
    ObjectModeller3D::PCLCloudPtr og_source (new ObjectModeller3D::PCLCloud);

	pcl::visualization::PCLVisualizer finalViewer("Final");
    
    finalViewer.addCoordinateSystem(0.1f);
	// pcl::visualization::PCLVisualizer currentviewer("Current");
    // currentviewer.addCoordinateSystem(0.1f)

    if (DEBUG_OBJECT_MODELLER_3D) 
    {
        std::cout << "X [" << lowerX << ".." << upperX << "]. Y [" << lowerY << ".." << upperY << "]. Z [" << lowerZ << ".." << upperZ << "]." << std::endl;
        std::cout << "LeafSize: " << voxelLeafSize << ". SFMean: " << statFilterMean << ". SFStdDev: " << statFilterStdDev << "." << std::endl;
    }

    float currentAngle = 0;
    float score = 0;
    float totalScore = 0;

    for (int i = 1; i <= numRotations; i++)
    {
        // read in the point cloud
        cloud = get_point_cloud(i);
        std::cout << i << "/" << numRotations << "... ";

        finalViewer.addPointCloud(cloud, "a");
        finalViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "a");
        finalViewer.spinOnce();
        finalViewer.removePointCloud("a");

        if (DEBUG_OBJECT_MODELLER_3D) 
        {
            std::cout << "Num points cloud before filtering " << og_source->points.size() << std::endl;
        }

        // rigidly transform the point cloud
        camera_to_world(cloud, cloud);

        // finalViewer.addPointCloud(cloud, "a");
        // finalViewer.spinOnce();
        // finalViewer.removePointCloud("a");
        // apply a pass filter
        dimension_filter(cloud, cloud, "x", lowerX, upperX);
        dimension_filter(cloud, cloud, "y", lowerY, upperY);
        dimension_filter(cloud, cloud, "z", lowerZ, upperZ);
        // finalViewer.addPointCloud(cloud, "a");
        // finalViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "a");
        // finalViewer.spin();
        // finalViewer.removePointCloud("a");
        // this is where we create the downsample

        // downsample the cloud
        // pcl::copyPointCloud(*cloud, *og_source);
        down_sample(cloud, cloud, voxelLeafSize);
        // finalViewer.addPointCloud(cloud, "a");
        // finalViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "a");
        // finalViewer.spin();
        // finalViewer.removePointCloud("a");

        // filter the point cloud
        stat_filter(cloud, cloud, statFilterMean, statFilterStdDev);
        // finalViewer.addPointCloud(cloud, "a");
        // finalViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "a");
        // finalViewer.spin();
        // finalViewer.removePointCloud("a");
        // offset the platform rotation
        //  finalViewer.addPointCloud(cloud, "a");
        currentAngle = offset_platform_rotation(cloud, cloud, i);

        // finalViewer.addPointCloud(cloud, "a");
        // finalViewer.spinOnce();
        // finalViewer.removePointCloud("a");

        if (DEBUG_OBJECT_MODELLER_3D) 
        {
            std::cout << "Current angle: " << currentAngle*180/M_PI << std::endl;
            std::cout << "Num points in cloud after filtering " << cloud->points.size() << std::endl;
        }
        
        // perform ICP
        if (i != 1) 
        {
            target = alignedClouds[i-2];
            // finalViewer.addPointCloud(target, "b");
            // finalViewer.spin();
           

            score = register_point_cloud(target, cloud);// , og_source);

       
            totalScore += score;
            if (DEBUG_OBJECT_MODELLER_3D) std::cout << "The score of this registration is: " << score << std::endl;
        }

        // finalViewer.removePointCloud("a");
        // finalViewer.removePointCloud("b");
        // finalViewer.addPointCloud(cloud, "a");
        // finalViewer.addPointCloud(target, "b");
        // finalViewer.spin();
        // finalViewer.removePointCloud("a");
        // finalViewer.removePointCloud("b");

        alignedClouds.push_back(cloud);
        if (DEBUG_OBJECT_MODELLER_3D) std::cout << "Num clouds in alignedClouds: " << alignedClouds.size() << std::endl;

        // display the total model
        std::string cloudName = "PC";
        cloudName.append(std::to_string(i));
        finalViewer.addPointCloud(cloud, cloudName);
        if (i == 1) finalViewer.spin();
        else finalViewer.spinOnce();

        // save the point cloud - save the non-sampled one with the transformations
    
        if (DEBUG_OBJECT_MODELLER_3D) std::cout << std::endl;

    }
    float averageScore = totalScore/(numRotations-1);
    std::cout << std::endl << "The average score of this registration is: " << averageScore << std::endl;

    finalViewer.spin();
    if (inputType == InputType::Camera)
    {
        comm.serial_end();
    }
}

ObjectModeller3D::PCLCloudPtr ObjectModeller3D::get_point_cloud(int cloudNum)
{
    ObjectModeller3D::PCLCloudPtr cloud (new ObjectModeller3D::PCLCloud);

    if (inputType == InputType::Camera)
    {
        // grab a new cloud
        cloud = rsGrabber.grab_point_cloud();
        // rotate the platform
        comm.stepper_spin();
    }
    else 
    { 
        std::string inputFileName = inFileDir;
        inputFileName.append(std::to_string(cloudNum));
        inputFileName.append(".pcd");
        if (DEBUG_OBJECT_MODELLER_3D) std::cout << inputFileName << std::endl;
        pcl::io::loadPCDFile(inputFileName, *cloud);
    }
    return cloud;
}

void ObjectModeller3D::down_sample(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, float leafSize)
{
    pcl::VoxelGrid<ObjectModeller3D::PointT> vox;
    vox.setLeafSize(leafSize, leafSize, leafSize);
    vox.setInputCloud(input);
    vox.filter(*output);
}

void ObjectModeller3D::stat_filter(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, float mean, float stdDev)
{
    pcl::StatisticalOutlierRemoval<ObjectModeller3D::PointT> statFilter;
    statFilter.setMeanK(mean);
    statFilter.setStddevMulThresh(stdDev);
    statFilter.setInputCloud(input);
    statFilter.filter(*output);
}

int ObjectModeller3D::dimension_filter(ObjectModeller3D::PCLCloudPtr point_cloud_in, ObjectModeller3D::PCLCloudPtr point_cloud_out, std::string dimension, const float lower, const float upper)
{
    pcl::PassThrough<RealSenseGrabber::PointT> pass;
    pass.setInputCloud(point_cloud_in);
    pass.setFilterFieldName(dimension);
    pass.setFilterLimits(lower, upper);
    pass.filter(*point_cloud_out);
    return 0;
}

void ObjectModeller3D::camera_to_world(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // rotate to offset the camera angle
    float cameraAngle = 0;
    if (inputType == InputType::Camera)
    {
        cameraAngle = M_PI/4;
    }

    transform.rotate(Eigen::AngleAxisf(cameraAngle, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*input, *output, transform);

    // // reset the transform 
    transform = Eigen::Affine3f::Identity();

    // translate to offset the camera distance away from the object
    // translate the object in x    y and  z
    transform.translation() << 0.0, 0.16, -0.18;
    // apply the initial translation
    pcl::transformPointCloud(*output, *output, transform);
}

float ObjectModeller3D::register_point_cloud(ObjectModeller3D::PCLCloudPtr target, ObjectModeller3D::PCLCloudPtr source) //,  ObjectModeller3D::PCLCloudPtr og_source)
{    
    ObjectModeller3D::PCLCloudPtr out (new ObjectModeller3D::PCLCloud);

    // registration
    pcl::GeneralizedIterativeClosestPoint6D reg6d;
    reg6d.setInputSource(source);
    reg6d.setInputTarget(target);
    
    reg6d.align(*out);

    // transforming the source into the target
    Eigen::Matrix4f fundamentalMatrix = Eigen::Matrix4f::Identity();
    fundamentalMatrix = reg6d.getFinalTransformation();
    pcl::transformPointCloud(*source, *source, fundamentalMatrix);    
    // pcl::transformPointCloud(*og_source, *og_source, fundamentalMatrix);
    // the fitness score should be as close to zero as possible
    return reg6d.getFitnessScore();
}

float ObjectModeller3D::offset_platform_rotation(ObjectModeller3D::PCLCloudPtr input, ObjectModeller3D::PCLCloudPtr output, int cloudNum)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float angleFromOrigin = (angle*(M_PI/180))*(cloudNum-1); // angle between consecutive clouds in radians * the number of times the object was rotated
    transform.rotate(Eigen::AngleAxisf(angleFromOrigin, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*input, *output, transform);
    return angleFromOrigin;
}
