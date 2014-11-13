/*
 * DataBaseDescriptors.h
 *
 *  Created on: Nov 11, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_

#include <vector>
#include <string>

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

#include "pointCloud.h"
#include "DataBaseDescriptors.h"

#include <boost/lexical_cast.hpp>

using namespace std;
typedef pcl::Histogram<153> SpinImage;


class DataBaseDescriptors {
public:
	DataBaseDescriptors(std::string path_dataBaseFolder_a);
	virtual ~DataBaseDescriptors();
//	std::vector<std::string> open(std::string path = ".");
	std::vector<std::string> open(std::string path);
	void calculateDataBaseDescriptors();

private:
	std::vector<std::string> vectorDirectories;
	std::string path_dataBaseFolder;
	pointCloud pointCloudDBD;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> si;
	pcl::PointCloud<pcl::Histogram<153> >::Ptr dataBaseDescriptorsPtr;
//	std::vector< pcl::PointCloud<pcl::Histogram<153> > > dataBaseDescriptors;
	std::vector<SpinImage> dataBaseDescriptors;



};


#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_ */
