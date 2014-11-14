/*
 * pointCloud.cpp
 *
 *  Created on: Nov 11, 2014
 *      Author: mafilipp
 */

#include "pointCloud.h"
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


// FindCorrespondance
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



typedef pcl::Histogram<153> DescriptorType;


pointCloud::pointCloud() {
	// TODO Auto-generated constructor stub
//	cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

}

pointCloud::~pointCloud() {
	// TODO Auto-generated destructor stub
}

void pointCloud::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{

	ROS_INFO("Camera Callback");

	pcl::PCLPointCloud2 pcl_pc;

	pcl_conversions::toPCL(*input, pcl_pc);

//	pcl::fromPCLPointCloud2(pcl_pc, m_cloud);

//		  ROS_INFO("cloudH.getCloud()[1].x = %n",m_cloud.points.size());

	ROS_INFO("camera Callback finished");

}

void pointCloud::readFile(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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


void pointCloud::clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector< pcl::PointCloud<pcl::PointXYZ> > & cloud_cluster_vector)
{
	// Read in the cloud data
	//	  pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	//	  reader.read ("/home/mafilipp/Desktop/table_scene_lms400.pcd", *cloud);
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
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	// Mie modifiche
//	std::vector< pcl::PointCloud<pcl::PointXYZ> > cloud_cluster_vector;


	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// Store the cloud in the vector
		cloud_cluster_vector.push_back(*cloud_cluster);

		// Solo per salvare l'immagine
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}
}


void pointCloud::computeSpin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images)
{

	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud (cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (kdtree);

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
	normal_estimation.setRadiusSearch (0.03);
	normal_estimation.compute (*normals);

	// Setup spin image computation
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> spin_image_descriptor(8, 0.5, 16);
	// image_width, support_angle_cos, min_pts_neighb
	spin_image_descriptor.setInputCloud (cloud);
	spin_image_descriptor.setInputNormals (normals);

	// Use the same KdTree from the normal estimation
	spin_image_descriptor.setSearchMethod (kdtree);
//	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
	spin_image_descriptor.setRadiusSearch (0.2);

	// Actually compute the spin images
	spin_image_descriptor.compute (*spin_images);
	std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

	// Display and retrieve the spin image descriptor vector for the first point.
	SpinImage first_descriptor = spin_images->points[0];
	std::cout << first_descriptor << std::endl;

}



int pointCloud::findCorrespondence(pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors)
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


// Mia versione
//int findCorrespondence(pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images_model, pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images_scene)
//{
//
//	// Per far andare il tutto
////	  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
////	  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
//
//	//
//	  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
//
//	  pcl::KdTreeFLANN<pcl::Histogram<153> > match_search;
//	  match_search.setInputCloud (spin_images_model);
//
//	  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//	  for (size_t i = 0; i < spin_images_scene->size (); ++i)
//	  {
//	    std::vector<int> neigh_indices (1);
//	    std::vector<float> neigh_sqr_dists (1);
//	    if (!pcl_isfinite (spin_images_scene->at (i).descriptor[0])) //skipping NaNs
//	    {
//	      continue;
//	    }
//	    int found_neighs = match_search.nearestKSearch (spin_images_scene->at (i), 1, neigh_indices, neigh_sqr_dists);
//	    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//	    {
//	      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//	      model_scene_corrs->push_back (corr);
//	    }
//	  }
//	  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
//
//	  return model_scene_corrs->size ();
//}
