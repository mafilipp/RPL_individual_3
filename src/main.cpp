
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

#include "PointCloudH.h"
#include "ClusterH.h"
#include "DirectoriesParser.h"

// A handy typedef.
typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudTF;

typedef std::vector < pcl::PointCloud<pcl::PointXYZ> > vectorPoint;






//======================= roba che non serve al momento
visualization_msgs::Marker sendMarker(float x, float y, float z);
//======================================================


//== Inizializza i valori a un valore default, ed eventualmente cambiali nel lounch

//  // Fuori dal main
//  struct PathPlannerNodeParameters {
//    // the robots radius in [m]
//    double        robot_footprint_radius;
//    // lower bound for detecting obstacles
//    int           obstacle_occupancy_value;
//    // the value to which obstacles should be inflated to
//    int           obstacle_inflation_value;
//    // upper bound for free space
//    int           obstacle_free_space_value;
//    // connectivity of the graph i.e 4 vs. 8 connected
//    int           graph_connectivity;
//  };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");

  //====================== Parameters

  std::string pathToSaveComplete;
  std::string pathToclusters;
  std::string pathToDataBase;
  ros::NodeHandle n_paramters;

  // Load the parameters from parameter server
  n_paramters.getParam("/path_to_save_complete_image_pcl", pathToSaveComplete);
  n_paramters.getParam("/path_to_save_clusters_pcl", pathToclusters);
  n_paramters.getParam("/path_to_data_base", pathToDataBase);

  std::cout << std::endl << std::endl;
  std::cout << "path complete image: " << pathToSaveComplete << std::endl << std::endl;
  std::cout << "path clusters: " << pathToclusters << std::endl << std::endl;


  // Create the Helper object
  PointCloudH cloudH;

  //======================== Node

  // Define Publisher and Subscribers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber camera_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &PointCloudH::cameraCallback, &cloudH);
  // Publishers
  ros::Publisher pub = n.advertise<PointCloudTF> ("points2", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

//  ROS_INFO_STREAM("-> Map resized to "); // Non cambia niente
//  ROS_ERROR("Sorry, could not find a path! Here are the points you requested:"); // scritta rossa


// ========================================= Wait untill we are uptodate

  ros::Rate loop_rate(1);

  while (n.ok() && not cloudH.isUpToDate())
  {
	  ros::spinOnce ();
	  loop_rate.sleep ();

  }

  // Save the image

  //TODO:  point directly to cloudH.getCloud()
  pcl::PointCloud<pcl::PointXYZ> cloud_tmp = cloudH.getCloud();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_Ptr(&cloud_tmp);

  std::string nameSceneComplete = "complete_scene.pcd";

  DirectoriesParser parserH(pathToSaveComplete);

  parserH.removeAllFiles(pathToSaveComplete);

  cloudH.savePclImage(cloud_tmp_Ptr,pathToSaveComplete, nameSceneComplete);

  ROS_INFO("Create the complete image in a file .pcd");

  // Here we have saved the image coming from the camera in the folder, so that we can create clusters with it

  // ====================================== Clusters extraction

  // First remove the old clusters
  parserH.removeAllFiles(pathToclusters);
  // Then create the cluster object
  ClusterH clusterH;
  clusterH.clusterExtraction(pathToSaveComplete + nameSceneComplete, pathToclusters);


  // Create a spin Image vector for all the clusters
  // This is done since maybe we would like to find multiple object rather than only one
  std::vector< pcl::PointCloud<SpinImage>::Ptr > vectorDescriptorsClusters;

  DirectoriesParser clustersParser(pathToclusters);

  std::cout << "Element found in main:" << std::endl;
  std::cout << clustersParser.getVectorElements()->size();

  // 		///////////////////////// Change This togli clusterP e usa direttamente clusterParser
  std::vector<std::string>* clusterP = clustersParser.getVectorElements();

  std::string pathClustersElement;

  for (std::vector<std::string>::iterator it = clustersParser.getVectorElements()->begin(); it != clustersParser.getVectorElements()->end(); ++it)
	  //

  for (std::vector<std::string>::iterator it = clusterP->begin(); it != clusterP->end(); ++it)
  {
	  pcl::PointCloud<SpinImage>::Ptr descriptorsClusters(new pcl::PointCloud<SpinImage>());
	  pathClustersElement = pathToclusters + *it;
	  std::cout << ' ' << pathClustersElement << '\n';
	  cloudH.computeSpin(pathClustersElement, descriptorsClusters);
	  vectorDescriptorsClusters.push_back(descriptorsClusters);
  }
  // 		//////////////////////////


  // =============================== Create Database
  DataBaseDescriptors dataBD(pathToDataBase);
  dataBD.calculateDataBaseDescriptors();

  ROS_INFO("HERE");


  // Compare the cluster with the Data Base in order to detect if an object of the database is present on the scene
//  int PointCloudH::findCorrespondence(pcl::PointCloud<SpinImage>::Ptr model_descriptors, pcl::PointCloud<SpinImage>::Ptr scene_descriptors)


  // Compare the clusters with the database -> Find if there is an object model on the scene and in which cluster it is

  int correspondance = 0;

  std::vector<std::vector<pcl::PointCloud<SpinImage> > > DataBase = dataBD.getDataBaseDescriptors();

  // Iterate through the different objects
  for (std::vector<std::vector<pcl::PointCloud<SpinImage> > >::iterator itMO = DataBase.begin(); itMO != DataBase.end(); ++itMO)
  {
	  // Iterate through all the different model Cloud
	  for (std::vector<pcl::PointCloud<SpinImage> >::iterator itMC = itMO->begin(); itMC != itMO->end(); ++itMC)
	  {
		  // Iterate through all the Spin Image of the object
		  for (pcl::PointCloud<SpinImage>::iterator itMSI = itMC->begin(); itMSI != itMC->end(); ++itMSI)
		  {
			  // Iterate through all the object spin image
			  for(std::vector< pcl::PointCloud<SpinImage>::Ptr >::iterator itC = vectorDescriptorsClusters.begin(); itC != vectorDescriptorsClusters.end(); ++itC)
			  {
//				  diff = cloudH.euclideanDistance(*itMSI,SISCENE);
			  }
		  }
		  std::cout << "1" << std::endl;

	  }

//	  *itM[i]
	  std::cout << "2" << std::endl;


  }




    pcl::PointCloud<pcl::PointXYZ> aaa;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloudBeforeClusteraa(&aaa);



  for (std::vector< pcl::PointCloud<SpinImage>::Ptr >::iterator itCl = vectorDescriptorsClusters.begin(); itCl != vectorDescriptorsClusters.end(); ++itCl)
  {
	  for(int j = 0; j < 4; j ++)
	  {
//		  pcl::PointCloud<SpinImage>::Ptr DBPtr(dataBD.getDataBaseDescriptors());
//		  correspondance = cloudH.findCorrespondence(*itCl, vectorDescriptorsClusters[0]);

	  }
  }


  std::cout << "CORRESPONDANCE FOUND 	" << correspondance << std::endl << std::endl;



//  for ()
//  {
//	  cloudH.computeSpin(std::string pathToPcdImage, descriptorsClusters);
//
//  }




  //================================= Send the marker to cover the figure

  PointCloudTF::Ptr msg (new PointCloudTF);


  while (n.ok())
  {

	  if(cloudH.isUpToDate())
	  {
		  // Publish the marker
		  marker_pub.publish(sendMarker(1.1f,1.1f,1.1f));



//		  cloudBeforeCluster = cloudH.getCloud();

		  //
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//			for (std::vector<int>::const_iterator pit = cloudH.getCloud().begin(); pit != cloudH.getCloud().end (); pit++)
//				cloud_cluster->points.push_back (cloudH.getCloud().points[*pit]); //*
//			cloud_cluster->points = cloudH.getCloud().points;
//			cloud_cluster->width = cloud_cluster->points.size ();
//			cloud_cluster->height = 1;
//			cloud_cluster->is_dense = true;
//		  //

//		  cloudH.savePclImage(cloud_cluster, pathToSavePcl, "prova1.pcd");
//		  ROS_INFO("IMAGE SAVED");
//
////		  clusters = clusterH.clusterExtraction(ptrCloudBeforeCluster, &vectorClusterH);
//
//
//		  std::cout << std::endl << std::endl << std::endl << std::endl;
//		  std::cout << "size -> " << clusters[0].size() << std::endl;
//		  std::cout << std::endl << std::endl << std::endl << std::endl;


	//	  ROS_INFO("heigh = 	%d", cloudH.getCloud().height);
		  std::cout << std::endl << std::endl << "heigh -> " << cloudH.getCloud().height << std::endl;
	//	  msg->points.clear();
		  msg->header.frame_id = "camera_link_out";
	//	  msg->height = 3;
	//	  msg->width = 1;
	//	  msg->points.push_back (pcl::PointXYZ(0.1, 0.1, 0.1));
	//	  msg->points.push_back (pcl::PointXYZ(0.2, 0.2, 0.2));
	//	  msg->points.push_back (pcl::PointXYZ(0.3, 0.2, 0.2));

		  msg->height = cloudH.getCloud().height;
		  msg->width = cloudH.getCloud().width;


		  msg->points.clear();

		  for(int i = 0; i < cloudH.getCloud().size(); i++)
			  msg->points.push_back (cloudH.getCloud().points[i]);


		  ros::Time time_st = ros::Time::now ();
		  msg->header.stamp = time_st.toNSec()/1e3;
		  ROS_INFO("Publishing");
		  pub.publish (msg);

	  }


	  ros::spinOnce ();
	  loop_rate.sleep ();



  }








  //  DataBaseDescriptors clusterDescriptors

  //  ROS_INFO("HERE");
  //  std::cout << pathToDataBase << std::endl;
  //  // Now create the database with all the descriptors
  //  DataBaseDescriptors dataBaseDescriptors(pathToDataBase);
  //  ROS_INFO("HERE2");
  //
  //  dataBaseDescriptors.calculateDataBaseDescriptors(); // Stored in dataBaseDescriptors.dataBaseDescriptors[i]





  //
  //  std::vector <PointCloudH> vectorClusterH;
  //
  //  std::vector <PointCloudH> vectorClusterH1;
  //
  //  vectorClusterH = vectorClusterH1;
  //
  //  // 1st way
  //  pcl::PointCloud<pcl::PointXYZ> cloudBeforeCluster;
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloudBeforeCluster(&cloudBeforeCluster);
  //
  //  vectorPoint * clusters;




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


