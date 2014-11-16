/*
 * ClusterH.cpp
 *
 *  Created on: Nov 15, 2014
 *      Author: mafilipp
 */

#include "ClusterH.h"


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

typedef std::vector < pcl::PointCloud<pcl::PointXYZ> > vectorPoint;




ClusterH::ClusterH() {
	// TODO Auto-generated constructor stub

}

ClusterH::~ClusterH() {
	// TODO Auto-generated destructor stub
}






void ClusterH::clusterExtraction(std::string path_to_coplete_scene_file, std::string path_to_save_clusters)
{
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	reader.read (path_to_coplete_scene_file, *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int i=0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.1); // 2cm 0.03 mi trova papera
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	// Change directory to path_to_save_clusters
	chdir(path_to_save_clusters.c_str());

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}

	ROS_INFO("Clustering Done");
	ROS_INFO("Create %d clusters", j);


}
































//vectorPoint* ClusterH::clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector <PointCloudH> * cloud_cluster_vector)
//{
//	// Read in the cloud data
//	//	  pcl::PCDReader reader;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//	//	  reader.read ("/home/mafilipp/Desktop/table_scene_lms400.pcd", *cloud);
//	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
//
//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//	vg.setInputCloud (cloud);
//	vg.setLeafSize (0.01f, 0.01f, 0.01f);
//	vg.filter (*cloud_filtered);
//	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
//
//	// Create the segmentation object for the planar model and set all the parameters
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//	pcl::PCDWriter writer;
//	seg.setOptimizeCoefficients (true);
//	seg.setModelType (pcl::SACMODEL_PLANE);
//	seg.setMethodType (pcl::SAC_RANSAC);
//	seg.setMaxIterations (100);
//	seg.setDistanceThreshold (0.02);
//
//	int i=0, nr_points = (int) cloud_filtered->points.size ();
//	while (cloud_filtered->points.size () > 0.3 * nr_points)
//	{
//		// Segment the largest planar component from the remaining cloud
//		seg.setInputCloud (cloud_filtered);
//		seg.segment (*inliers, *coefficients);
//		if (inliers->indices.size () == 0)
//		{
//			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//			break;
//		}
//
//		// Extract the planar inliers from the input cloud
//		pcl::ExtractIndices<pcl::PointXYZ> extract;
//		extract.setInputCloud (cloud_filtered);
//		extract.setIndices (inliers);
//		extract.setNegative (false);
//
//		// Get the points associated with the planar surface
//		extract.filter (*cloud_plane);
//		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//
//		// Remove the planar inliers, extract the rest
//		extract.setNegative (true);
//		extract.filter (*cloud_f);
//		*cloud_filtered = *cloud_f;
//	}
//
//	// Creating the KdTree object for the search method of the extraction
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud (cloud_filtered);
//
//	std::vector<pcl::PointIndices> cluster_indices;
//	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//	ec.setClusterTolerance (0.02); // 2cm
//	ec.setMinClusterSize (100);
//	ec.setMaxClusterSize (25000);
//	ec.setSearchMethod (tree);
//	ec.setInputCloud (cloud_filtered);
//	ec.extract (cluster_indices);
//
//	// Mie modifiche
////	std::vector< pcl::PointCloud<pcl::PointXYZ> > cloud_cluster_vector;
//
//	vectorPoint vectorClusters[100];
//
//	ROS_INFO("BEFORE SAVE CLUSTER");
//	int j = 0;
//	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
//			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
//		cloud_cluster->width = cloud_cluster->points.size ();
//		cloud_cluster->height = 1;
//		cloud_cluster->is_dense = true;
//
//		// Store the cloud in the vector
//		cloud_cluster_vector->push_back(PointCloudH());
//		vectorClusters[j].push_back(*cloud_cluster);
//
////		cloud_cluster_vector->end()->setCloud(&cloud_cluster);
////							 .->setCloud(*cloud_cluster);
////		cloud_cluster_vector->push_back()->	push_back(*cloud_cluster);
////
////
////		  std::vector <PointCloudH> vectorClusterH;
//
//
//
//		// Solo per salvare l'immagine
//		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//		std::stringstream ss;
//		ss << "cloud_cluster_duck_" << j << ".pcd";
//		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//		j++;
//	}
//
//	return vectorClusters;
//}
