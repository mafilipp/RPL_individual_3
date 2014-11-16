/*
 * ClusterH.h
 *
 *  Created on: Nov 15, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_CLUSTERH_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_CLUSTERH_H_


#include "PointCloudH.h"
// C++
#include <iostream>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// Point Cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>
// Subscribe to the camera
#include <sensor_msgs/PointCloud2.h>
// Markers
#include <visualization_msgs/Marker.h>

// sito
#include <pcl/conversions.h>
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


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;
typedef pcl::Histogram<153> SpinImage;

typedef pcl::Histogram<153> DescriptorType;
typedef std::vector < pcl::PointCloud<pcl::PointXYZ> > vectorPoint;



class ClusterH {
public:
	ClusterH();
	virtual ~ClusterH();
//	vectorPoint* clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector <PointCloudH> * cloud_cluster_vector);
 // pcl::PointCloud<pcl::PointXYZ> m_cloud;				std::vector <PointCloudH> vectorClusterH;
	void clusterExtraction(std::string path_to_coplete_scene_file, std::string path_to_save_clusters);

private:
	std::vector < pcl::PointCloud<pcl::PointXYZ> > m_vector_clusterH;

};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_CLUSTERH_H_ */
