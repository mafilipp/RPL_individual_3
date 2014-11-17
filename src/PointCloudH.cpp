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

// Correspondance
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;
typedef pcl::Histogram<153> SpinImage;



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

void PointCloudH::readFile(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
	{
//		std::string error = "Couldn't read file" + path +  "\n";
//		PCL_ERROR (error);
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;
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


int PointCloudH::findCorrespondence(pcl::PointCloud<SpinImage>::Ptr model_descriptors, pcl::PointCloud<SpinImage>::Ptr scene_descriptors)
{

	// Per far andare il tutto
//	  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
//	  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	//
	  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	  pcl::KdTreeFLANN<DescriptorType> match_search;
	  match_search.setInputCloud (model_descriptors);

	  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	  for (size_t i = 0; i < scene_descriptors->size (); ++i)
	  {
	    std::vector<int> neigh_indices (1);
	    std::vector<float> neigh_sqr_dists (1);
//	    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
//	    {
//	      continue;
//	    }
	    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
	    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
	    {
	      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
	      model_scene_corrs->push_back (corr);
	    }
	  }
	  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

	  return model_scene_corrs->size ();
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
