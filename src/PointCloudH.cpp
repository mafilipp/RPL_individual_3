/*
 * PointCloudH.cpp
 *
 *  Created on: Nov 14, 2014
 *      Author: mafilipp
 */

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


PointCloudH::PointCloudH() {
	// TODO Auto-generated constructor stub

}

PointCloudH::~PointCloudH() {
	// TODO Auto-generated destructor stub
}

void PointCloudH::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{

	ROS_INFO("Camera Callback");

	pcl::PCLPointCloud2 pcl_pc;

	pcl_conversions::toPCL(*input, pcl_pc);

	pcl::fromPCLPointCloud2(pcl_pc, m_cloud);

//		  ROS_INFO("cloudH.getCloud()[1].x = %n",m_cloud.points.size());

	ROS_INFO("camera Callback finished");
	ROS_INFO("heigh = 	%d, width = 	%d", m_cloud.height, m_cloud.width);

}

const pcl::PointCloud<pcl::PointXYZ>& PointCloudH::getCloud() const {
	return m_cloud;
}

void PointCloudH::setCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	m_cloud = cloud;
}
// Getters and Setters

