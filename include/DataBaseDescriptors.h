/*
 * DataBaseDescriptors.h
 *
 *  Created on: Nov 11, 2014
 *      Author: This class is used for the creation of a Data Base of spin image
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_

// C++
#include <vector>
#include <string>
#include <iostream>
#include <dirent.h>
// ROS
#include <ros/ros.h>
// Point Cloud
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// Spin image
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
// Other
#include "PointCloudH.h"
#include "DataBaseDescriptors.h"
#include "DirectoriesParser.h"

typedef pcl::Histogram<153> SpinImage;


class DataBaseDescriptors {
public:
	DataBaseDescriptors(std::string path_dataBaseFolder_a);
	virtual ~DataBaseDescriptors();
	std::vector<std::string> open(std::string path);
	void calculateDataBaseDescriptors();

	// Setters and Getters
	const std::vector<std::vector<pcl::PointCloud<SpinImage> > >& getDataBaseDescriptors() const;
	void setDataBaseDescriptors(
	const std::vector<std::vector<pcl::PointCloud<SpinImage> > >& dataBaseDescriptors);

private:
	std::vector<std::string> vectorDirectories;
	std::string path_dataBaseFolder;
	PointCloudH pointCloudDBD;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> si;
	pcl::PointCloud<pcl::Histogram<153> >::Ptr dataBaseDescriptorsPtr;

	// Matrix where the different pcd images for every object are stored
	std::vector <std::vector<pcl::PointCloud<SpinImage> > > dataBaseDescriptors;


};


#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_ */
