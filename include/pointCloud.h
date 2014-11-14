/*
 * pointCloud.h
 *
 *  Created on: Nov 11, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUD_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUD_H_


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


// A handy typedef.
typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::Histogram<153> DescriptorType;



class pointCloud {
public:
	pointCloud();
	virtual ~pointCloud();

	void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void readFile(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);//, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
	void clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector <std::vector< pcl::PointCloud<pcl::PointXYZ> > > &cloud_cluster_vector);
	void computeSpin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images);
	int findCorrespondence(pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors);



private:
	pcl::PointCloud<pcl::PointXYZ> m_cloud;
	int a;

	//pcl::PointCloud<pcl::PointXYZ> cloud;

//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;


};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_POINTCLOUD_H_ */
