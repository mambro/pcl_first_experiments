#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>



int main (int argc, char** argv)
{

    std::string filename = argv[1];

    std::cout << "Reading " << filename << std::endl;

    // Source RGB cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
    // Output Planar segmented cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);




    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud_rgb) == -1) // load the file
    {
    PCL_ERROR ("Couldn't read file");
    return -1;
    }

    std::cout << "points: " << cloud->points.size () << std::endl;

    pcl::copyPointCloud(*cloud_rgb, *cloud);

    cloud->sensor_orientation_.w() = 0.0;
    cloud->sensor_orientation_.x() = 1.0;
    cloud->sensor_orientation_.y() = 0.0;
    cloud->sensor_orientation_.z() = 0.0;

    cloud_rgb->sensor_orientation_.w() = 0.0;
    cloud_rgb->sensor_orientation_.x() = 1.0;
    cloud_rgb->sensor_orientation_.y() = 0.0;
    cloud_rgb->sensor_orientation_.z() = 0.0;
    
    // Create the segmentation object
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);


    seg.setInputCloud(cloud_rgb);    
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return -1;
    }

    std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

    std::cout << "Model inliers: " << inliers->indices.size () << std::endl;

    // Extract the inliers

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
   
    extract.setInputCloud(cloud_rgb);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    
    // Visualize Point Clouds 
       
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    
    viewer.removeAllPointClouds();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "source cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(cloud_p, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_p,red_color,"segmented cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");
  
    while (!viewer.wasStopped ())
    {
    viewer.spinOnce();
    }

    return (0);

}