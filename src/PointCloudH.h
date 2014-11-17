/*
 * PointCloudH.h
 *
 *  Created on: Nov 14, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUDH_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUDH_H_

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


class PointCloudH {
public:
	PointCloudH();
	virtual ~PointCloudH();

	void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void savePclImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_image, std::string path_to_save_image, std::string name);
	void computeSpin(std::string pathToPcdImage, pcl::PointCloud<SpinImage>::Ptr descriptors);


	const pcl::PointCloud<pcl::PointXYZ>& getCloud() const;
	void setCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
	bool isUpToDate() const;
	void setUpToDate(bool upToDate);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudPtr;
	pcl::PointCloud<pcl::PointXYZ> m_cloud;

	bool m_upToDate;
};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUDH_H_ */
