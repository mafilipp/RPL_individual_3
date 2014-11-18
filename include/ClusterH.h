/*
 * ClusterH.h
 *
 *  Created on: Nov 15, 2014
 *      Author: Filippo Martinoni
 *      Note: This helper class contains function for the creation of clustering .pcd image, given the path to a .pcd image
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_CLUSTERH_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_CLUSTERH_H_


#include "ClusterH.h"
// C++
#include <iostream>
// ROS
#include <ros/ros.h>
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
// Other
#include "PointCloudH.h"



class ClusterH {
public:
	ClusterH();
	virtual ~ClusterH();
	void clusterExtraction(std::string path_to_coplete_scene_file, std::string path_to_save_clusters);
};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_CLUSTERH_H_ */
