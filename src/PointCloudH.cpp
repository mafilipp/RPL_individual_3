/*
 * PointCloudH.cpp
 *
 *  Created on: Nov 14, 2014
 *      Author: Filippo Martinoni
 *      Note: Implementation of the class PointCloudH
 */

#include "PointCloudH.h"

PointCloudH::PointCloudH() {
	m_upToDate = false;
}

PointCloudH::~PointCloudH() {
}


void PointCloudH::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	ROS_INFO("Camera Callback");

	m_upToDate = false;

	// Get and Filter the image
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*input, pcl_pc);
	pcl::fromPCLPointCloud2(pcl_pc, *cloud_tmp);

	// if we already want to filter the image
//	passThroughFilter(cloud_tmp, cloud_filtered);
//	m_cloud = *cloud_filtered;

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
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file name_file.pcd \n");
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

double PointCloudH::euclideanNorm(SpinImage first, SpinImage second)
{
	double total;
	for(int j = 0; j < first.descriptorSize(); j++)
	{
		total += fabs(first.histogram[j] - second.histogram[j]);
	}
	return total;
}

void PointCloudH::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	  pcl::PassThrough<pcl::PointXYZ> pass;
	  pass.setInputCloud (cloud);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (0.3, 0.7);
	  //pass.setFilterLimitsNegative (true);
	  pass.filter (*cloud_filtered);
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
