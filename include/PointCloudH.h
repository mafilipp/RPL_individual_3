/*
 * PointCloudH.h
 *
 *  Created on: Nov 14, 2014
 *      Author: mafilipp
 *      Note: This class implement functions that are used for operation with point clouds
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUDH_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUDH_H_

// C++
#include <iostream>
// Change dir
#include <string>
#include <sys/param.h>
#include <unistd.h>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// Subscribe to the camera
#include <sensor_msgs/PointCloud2.h>
// Markers
#include <visualization_msgs/Marker.h>
// Point Cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::Histogram<153> SpinImage;


class PointCloudH {
public:
	PointCloudH();
	virtual ~PointCloudH();

	void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void savePclImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_image, std::string path_to_save_image, std::string name);
	void computeSpin(std::string pathToPcdImage, pcl::PointCloud<SpinImage>::Ptr descriptors);
	void readFile(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
	double euclideanNorm(SpinImage first, SpinImage second);


	// Getters and Setters
	const pcl::PointCloud<pcl::PointXYZ>& getCloud() const;
	void setCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
	bool isUpToDate() const;
	void setUpToDate(bool upToDate);

private:
	pcl::PointCloud<pcl::PointXYZ> m_cloud;
	bool m_upToDate;
};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUDH_H_ */
