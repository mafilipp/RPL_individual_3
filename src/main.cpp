
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
#include "DataBaseDescriptors.h"

#include <boost/lexical_cast.hpp>


// A handy typedef.
typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;





//======================= roba che non serve al momento
visualization_msgs::Marker sendMarker(float x, float y, float z);
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


	ROS_INFO("Start callback");
	pcl::PCLPointCloud2 pcl_pc;

//	  void toPCL(const sensor_msgs::Image &image, pcl::PCLImage &pcl_image)

	pcl_conversions::toPCL(*input, pcl_pc);

	pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::fromPCLPointCloud2(pcl_pc, cloud);
	//pcl::YOUR_PCL_FUNCTION(cloud,...);

//	printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
//	BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
//	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

	ROS_INFO("Start saving");

	// Solo per salvare l'immagine
//	pcl::PCDWriter writer;
//	std::stringstream ss;
//	ss << "paperaGiallaTest.pcd";
//	writer.write<pcl::PointXYZ> (ss.str (), cloud, false); //*
//	ROS_INFO("Image saved");

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");


  // Define Publisher and Subscribers
  ros::NodeHandle n;
  // Subscribers
  ros::Subscriber file_sub   = n.subscribe<PointCloud>("points2", 1, pointsCallback);
  ros::Subscriber camera_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cameraCallback);
  // Publishers
  ros::Publisher pub = n.advertise<PointCloud> ("points2", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);



//  //===========================================
//  // Path where to find the file to analize
////  std::string path = "/home/mafilipp/Desktop/table_scene_lms400.pcd";
////  std::string path = "/home/mafilipp/data/objects/duck/duck_close_90.pcd";
//  std::string path = "/home/mafilipp/catkin_ws/src/mafilipp/object_recognition/image/complete_image";
//
//
//  // Read files from database and create a database descriptor
////  std::string path_dataBaseFolder = "/home/mafilipp/data/objects/";
//  std::string path_dataBaseFolder = "/home/mafilipp/catkin_ws/src/mafilipp/object_recognition/image/database_image";
//
//  DataBaseDescriptors dataBaseDescriptors(path_dataBaseFolder);
//  dataBaseDescriptors.calculateDataBaseDescriptors(); // Stored in dataBaseDescriptors.dataBaseDescriptors[i]
//
//  // create a point cloud element -> This contain all the operation that we have to perform
//  pointCloud pointCloud;
//
//  // Define the point cloud coming either from a file or an image
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//  // Define Spin image
//  pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage>::Ptr si;
//
//  // Read from a file the point cloud, and store it in cloud
//  pointCloud.readFile(path, cloud); // -> In cloud I have now the complete image
//
//  // Vector that contain all the clusters
//  std::vector <std::vector< pcl::PointCloud<pcl::PointXYZ> > >cloud_cluster_vector;
//
////  pointCloud.clusterExtraction(cloud, cloud_cluster_vector);
//
//  // To deletw!
//  std::vector< pcl::PointCloud<pcl::PointXYZ> > cloud_cluster;
//  pointCloud.clusterExtraction(cloud, cloud_cluster);
//
//
////  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster;// (new pcl::PointCloud<pcl::PointXYZ>);
////  cloud_cluster.push_back(new pcl::PointCloud<pcl::PointXYZ>);   ??????????
//
//
//  pcl::PointCloud<pcl::Histogram<153> >::Ptr spinImage;
//
//  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType> [cloud_cluster.size()]);
//
//  // Compute all the descriptors for the different clusters
//  for(std::vector< pcl::PointCloud<pcl::PointXYZ> >::iterator it = cloud_cluster.begin(); it != cloud_cluster.end(); ++it)
//  {
//	  pointCloud.computeSpin(cloud,spinImage);
//  }
//
//
//  ROS_INFO("start read -");
//
//
//  // Vettori da usare
//  // Model
////  dataBaseDescriptors.dataBaseDescriptors
//
//
//  // Find Correspondence
//  // Compare the descriptor with the dataBaseDescriptors
//  int match;
////  for (int idDB = 0; idDB < ; idDB++)
////
////  for(std::vector< pcl::PointCloud<DescriptorType>::Ptr >::iterator itModel = cloud_cluster.begin(); itModel != cloud_cluster.end(); ++itModel)
////  {
////	  for(std::vector< pcl::PointCloud<DescriptorType>::Ptr >::iterator itDataBase = dataBaseDescriptors.dataBaseDescriptors[0].begin(); itDataBase != cloud_cluster.end(); ++itDataBase)
////	  {
////		  match = pointCloud.findCorrespondence(*itModel, *itDataBase);
//////		  match = pointCloud.findCorrespondence(pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors);
////
////	  }
//
//
////  }
//
//
////===========================================

  /////======================= ALTRO

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


