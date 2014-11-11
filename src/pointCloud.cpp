/*
 * pointCloud.cpp
 *
 *  Created on: Nov 11, 2014
 *      Author: mafilipp
 */

#include "pointCloud.h"
#include <ros/ros.h>
// Point Cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
// Subscribe to the camera
#include <sensor_msgs/PointCloud2.h>
// Markers
#include <visualization_msgs/Marker.h>

// sito


#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Read File

#include <pcl/point_types.h>

// Spin image
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

// clusterExtraction
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


pointCloud::pointCloud() {
	// TODO Auto-generated constructor stub
//	cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);


}

pointCloud::~pointCloud() {
	// TODO Auto-generated destructor stub
}

void pointCloud::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	//TODO
//	ROS_INFO("I got the camera");
//	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);


	pcl::PCLPointCloud2 pcl_pc;

//	  void toPCL(const sensor_msgs::Image &image, pcl::PCLImage &pcl_image)

	pcl_conversions::toPCL(*input, pcl_pc);

	pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::fromPCLPointCloud2(pcl_pc, cloud);
	//pcl::YOUR_PCL_FUNCTION(cloud,...);

	printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
	BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

void pointCloud::readFile(const std::string& path)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, cloud) == -1) //* load the file
	{
//		std::string error = "Couldn't read file" + path +  "\n";
//		PCL_ERROR (error);
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}
	std::cout << "Loaded "
			<< cloud.width * cloud.height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;
}
