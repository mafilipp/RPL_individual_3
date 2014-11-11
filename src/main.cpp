
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

#include <string>
#include "pointCloud.h"


// A handy typedef.
typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;





//======================= roba che non serve al momento
visualization_msgs::Marker sendMarker(float x, float y, float z);
void readFile(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//======================================================


void pointsCallback(const PointCloud::ConstPtr& msg)
{

//	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	//TODO
//	ROS_INFO("I got the camera");
//	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);


	pcl::PCLPointCloud2 pcl_pc;

//	  void toPCL(const sensor_msgs::Image &image, pcl::PCLImage &pcl_image)

	pcl_conversions::toPCL(*input, pcl_pc);

	pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::fromPCLPointCloud2(pcl_pc, cloud);
	//pcl::YOUR_PCL_FUNCTION(cloud,...);

	printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
	BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);


}





void computeSpin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> &si)
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
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
	spin_image_descriptor.setRadiusSearch (0.2);

	// Actually compute the spin images
	spin_image_descriptor.compute (*spin_images);
	std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

	// Display and retrieve the spin image descriptor vector for the first point.
	SpinImage first_descriptor = spin_images->points[0];
	std::cout << first_descriptor << std::endl;
}



void clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// Solo per salvare l'immagine
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}
}


























/////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");
  ros::NodeHandle n;
  // Subscribers
  ros::Subscriber file_sub   = n.subscribe<PointCloud>("points2", 1, pointsCallback);
  ros::Subscriber camera_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cameraCallback);
  // Publishers
  ros::Publisher pub = n.advertise<PointCloud> ("points2", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Rate r(1);
  pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> si;









  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  std::string path = "/home/mafilipp/data/objects/duck/duck_close_90.pcd"; //"/home/mafilipp/Desktop/table_scene_lms400.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudone (new pcl::PointCloud<pcl::PointXYZ>);

  ROS_INFO("start read");

  readFile(path, cloud);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster;// (new pcl::PointCloud<pcl::PointXYZ>);
//  cloud_cluster.push_back(new pcl::PointCloud<pcl::PointXYZ>);   ??????????

//  clusterExtraction(cloud);

  computeSpin(cloud, si);
  ROS_INFO("start read -");



//  pointCloud pc;
//  pc.readFile();
//  ROS_INFO("finish read");

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(1);




  while (n.ok())
  {
	  ros::Time time_st = ros::Time::now ();
	  msg->header.stamp = time_st.toNSec()/1e3;
	  pub.publish (msg);
	  ros::spinOnce ();
	  loop_rate.sleep ();

	  // Publish the marker
	  marker_pub.publish(sendMarker(1.1f,1.1f,1.1f));

  }
}



//==================================== Roba che al momento non serve

visualization_msgs::Marker sendMarker(float x, float y, float z)
{
	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CYLINDER;

	//	// Cycle between different shapes
	//	switch (shape)
	//	{
	//	case visualization_msgs::Marker::CUBE:
	//	  shape = visualization_msgs::Marker::SPHERE;
	//	  break;
	//	case visualization_msgs::Marker::SPHERE:
	//	  shape = visualization_msgs::Marker::ARROW;
	//	  break;
	//	case visualization_msgs::Marker::ARROW:
	//	  shape = visualization_msgs::Marker::CYLINDER;
	//	  break;
	//	case visualization_msgs::Marker::CYLINDER:
	//	  shape = visualization_msgs::Marker::CUBE;
	//	  break;
	//	}

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();


	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	return marker;

}



void readFile(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
	{
//		std::string error = "Couldn't read file" + path +  "\n";
//		PCL_ERROR (error);
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//	return (-1);
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;
//	for (size_t i = 0; i < cloud->points.size (); ++i)
//	std::cout << "    " << cloud->points[i].x
//			  << " "    << cloud->points[i].y
//			  << " "    << cloud->points[i].z << std::endl;
//	return (0);
}


