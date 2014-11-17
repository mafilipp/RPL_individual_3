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


// Change dir
#include <string>
#include <sys/param.h>
#include <unistd.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;
typedef pcl::Histogram<153> SpinImage;

typedef pcl::Histogram<153> DescriptorType;


PointCloudH::PointCloudH() {
	// TODO Auto-generated constructor stub
	m_upToDate = false;
//	m_cloud = new pcl::PointCloud<pcl::PointXYZ>;

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
	//	ROS_INFO("cloudH.getCloud()[1].x = %n",m_cloud.points.size());
	ROS_INFO("heigh = %d, width = %d", m_cloud.height, m_cloud.width);

	ROS_INFO("camera Callback finished");

	m_upToDate = true;

}

void PointCloudH::savePclImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_image, std::string path_to_save_image, std::string name)
{
	chdir(path_to_save_image.c_str());
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> (name, *cloud_image, false);
}


void PointCloudH::computeSpin(std::string pathToPcdImage, pcl::PointCloud<SpinImage>::Ptr descriptors)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the spin image for each point.
//	pcl::PointCloud<SpinImage>::Ptr descriptors(new pcl::PointCloud<SpinImage>());

	// Read in the cloud data
	pcl::PCDReader reader;
	reader.read (pathToPcdImage, *cloud);
	std::cout << "PointCloud used for calculating Spin image has: " << cloud->points.size () << " data points." << std::endl; //*

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// Spin image estimation object.
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> si;
	si.setInputCloud(cloud);
	si.setInputNormals(normals);
	// Radius of the support cylinder.
	si.setRadiusSearch(0.02);
	// Set the resolution of the spin image (the number of bins along one dimension).
	// Note: you must change the output histogram size to reflect this.
	si.setImageWidth(8);

	si.compute(*descriptors);
}


// Getters and Setters

const pcl::PointCloud<pcl::PointXYZ>& PointCloudH::getCloud() const {
	return m_cloud;
}

void PointCloudH::setCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	m_cloud = cloud;
}

bool PointCloudH::isUpToDate() const {
	return m_upToDate;
}

void PointCloudH::setUpToDate(bool upToDate) {
	m_upToDate = upToDate;
}
