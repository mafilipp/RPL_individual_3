
#include <ros/ros.h>
// Point Cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
// Subscribe to the camera
#include <sensor_msgs/PointCloud2.h>
// Markers
#include <visualization_msgs/Marker.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Read File

#include <pcl/point_types.h>

// Spin image
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

// C++
#include <iostream>
#include <string>
#include "DataBaseDescriptors.h"
#include "PointCloudH.h"
#include "ClusterH.h"
#include "DirectoriesParser.h"

// A handy typedef.
typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudTF;
typedef std::vector < pcl::PointCloud<pcl::PointXYZ> > vectorPoint;


visualization_msgs::Marker sendMarker(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, std::string type);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");

  //====================== Parameters

  std::string pathToSaveComplete;
  std::string pathToclusters;
  std::string pathToDataBase;
  PointCloudH cloudH;
  ros::NodeHandle n_paramters;

  // Load the parameters from parameter server
  n_paramters.getParam("/path_to_save_complete_image_pcl", pathToSaveComplete);
  n_paramters.getParam("/path_to_save_clusters_pcl", pathToclusters);
  n_paramters.getParam("/path_to_data_base", pathToDataBase);

  std::cout << std::endl << std::endl;
  std::cout << "path to the complete image: " << pathToSaveComplete << std::endl << std::endl;
  std::cout << "path to the clusters: " << pathToclusters << std::endl << std::endl;

  //======================== Node

  // Define Publisher and Subscribers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber camera_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &PointCloudH::cameraCallback, &cloudH);

  // Publishers
  ros::Publisher pub = n.advertise<PointCloudTF> ("points2", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //======================== Wait untill we have a pointcloud from the camera
  ros::Rate loop_rate(1);

  while (n.ok() && not cloudH.isUpToDate())
  {
	  ros::spinOnce ();
	  loop_rate.sleep ();
  }

  //======================== Save the image

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

  std::cout << "Clusters number:" << std::endl;
  std::cout << clustersParser.getVectorElements()->size();

  std::vector<std::string>* clusterP = clustersParser.getVectorElements();
  std::string pathClustersElement;
  for (std::vector<std::string>::iterator it = clusterP->begin(); it != clusterP->end(); ++it)
  {
	  pcl::PointCloud<SpinImage>::Ptr descriptorsClusters(new pcl::PointCloud<SpinImage>());
	  pathClustersElement = pathToclusters + *it;
	  std::cout << ' ' << pathClustersElement << '\n';
	  cloudH.computeSpin(pathClustersElement, descriptorsClusters);
	  vectorDescriptorsClusters.push_back(descriptorsClusters);
  }


   // =============================== Create Database
   DataBaseDescriptors dataBD(pathToDataBase);
   dataBD.calculateDataBaseDescriptors();

   //================================ Compare the spin image from every cluster with the spin images of the Databare
   // Compare the clusters with the database -> Find if there is an object model on the scene and in which cluster it is

   // Here I wanted to write an algorithm able to find out to which model our spin image belong, but it doesn't work

   // From here to the end, since the whole didn't work, the stuff implemented are not completed..
   int correspondance = 0;
   double diff;
   double minDiff;
   double match = 0;

   std::vector<double> matchRate;
   std::vector<double> bestMatch;

   std::vector<std::vector<pcl::PointCloud<SpinImage> > > DataBase = dataBD.getDataBaseDescriptors();
   ROS_INFO("START FOR LOOP");
   // Iterate through the different model objects
   for (std::vector<std::vector<pcl::PointCloud<SpinImage> > >::iterator itMO = DataBase.begin(); itMO != DataBase.end(); ++itMO)
   {
 	  // Iterate through all the different model Cloud (different pcd files)
 	  for (std::vector<pcl::PointCloud<SpinImage> >::iterator itMC = itMO->begin(); itMC != itMO->end(); ++itMC)
 	  {
 		  match = 0;
 		  // Iterate through all the Spin Image of the object
 		  for (pcl::PointCloud<SpinImage>::iterator itMSI = itMC->begin(); itMSI != itMC->end(); ++itMSI)
 		  {
 			  minDiff = 1000;

 			  // Iterate through all the clusters spin image
 			  for(std::vector< pcl::PointCloud<SpinImage>::Ptr >::iterator itC = vectorDescriptorsClusters.begin(); itC != vectorDescriptorsClusters.end(); ++itC)
 			  {
 				  // Iterate through all the histogram
 				  for(pcl::PointCloud<SpinImage>::iterator itCH = (*itC)->begin(); itCH != (*itC)->end(); ++itCH)
 				  {
 					  diff = cloudH.euclideanNorm(*itMSI,*itCH);
 					  if(diff < minDiff)
 						  minDiff = diff;
 				  }
 			  }
 			  match += minDiff/(*itMSI).descriptorSize();
 		  }
 		  matchRate.push_back(match);
 	  }

 	  double best = 100;

 	  //Find the best matches
 	  for(std::vector<double>::iterator itMa = matchRate.begin(); itMa != matchRate.end(); ++itMa)
 	  {
 		  if(*itMa < best)
 			  best = *itMa;
 	  }

 	  bestMatch.push_back(best);
   }


  //================================= Send the marker to cover the figure with the desired color

  PointCloudTF::Ptr msg (new PointCloudTF);

  while (n.ok())
  {

	  if(cloudH.isUpToDate())
	  {
		  // Publish the message with the image coming from the camera (Debug Purpose)
		  msg->header.frame_id = "camera_link";
		  msg->height = cloudH.getCloud().height;
		  msg->width = cloudH.getCloud().width;
		  msg ->points = cloudH.getCloud().points;
		  ros::Time time_st = ros::Time::now ();
		  msg->header.stamp = time_st.toNSec()/1e3;
		  ROS_INFO("Publishing");
		  pub.publish (msg);

		  float minX, maxX, minY, maxY, minZ, maxZ;
		  minX = 10;
		  minY = 10;
		  minZ = 10;
		  maxX = 0;
		  maxY = 0;
		  maxZ = 0;

		  // Find the boundary of the objects
		  pcl::PointCloud<pcl::PointXYZ> pointCloudXYZtmp = cloudH.getCloud();
		  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = pointCloudXYZtmp.points.begin(); it != pointCloudXYZtmp.points.end(); ++it)
		  {
			  if(it-> x < minX)
				  minX = it->x;
			  if(it-> y < minY)
				  minY = it->y;
			  if(it-> z < minZ)
				  minZ = it->z;
			  if(maxX < it->x)
				  maxX = it->x;
			  if(maxY < it->y)
				  maxY = it->y;
			  if(maxZ < it->z)
				  maxZ = it->z;
		  }

		  std::string object;

		  marker_pub.publish(sendMarker(minX,minY,minZ,maxX,maxY,maxZ, object));

	  }
	  ros::spinOnce ();
	  loop_rate.sleep ();
  }






}


//==================================== Marker Message

visualization_msgs::Marker sendMarker(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, std::string type)
{
	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/camera_link";
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
	marker.pose.position.x = minX;
	marker.pose.position.y = minY;
	marker.pose.position.z = minZ;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	float scaleX, scaleY, scaleZ;
	scaleX = maxX-minX;
	scaleY = maxY-minY;
	scaleZ = maxZ-minZ;

	std::cout << minX << std::endl;
	std::cout << maxX << std::endl;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = scaleX;
	marker.scale.y = scaleY;
	marker.scale.z = scaleZ;

	// Set the color -- be sure to set alpha to something non-zero!

	// Here I would insert a case in order to set the right color with the correspondance to the object detected
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	return marker;
}
