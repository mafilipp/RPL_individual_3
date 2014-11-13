/*
 * DataBaseDescriptors.cpp
 *
 *  Created on: Nov 11, 2014
 *      Author: mafilipp
 */

#include "DataBaseDescriptors.h"

#include <dirent.h>
#include <string>
#include <vector>
#include <iostream>


using namespace std;

DataBaseDescriptors::DataBaseDescriptors(std::string path_dataBaseFolder_a)
{
	path_dataBaseFolder = path_dataBaseFolder_a;
	vectorDirectories = open(path_dataBaseFolder);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
//	vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> > *dataBasePtr1 = new vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> >(vectorDirectories.size());
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
	cloud = cloud1;
	dataBaseDescriptorsPtr = spin_images;
	//	std::vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> > *dataBasePtr = new std::vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> >(10);

	std::cout << vectorDirectories.size() << endl << endl << endl;
	std::vector<SpinImage> * dataBaseDescriptror1 = new std::vector<SpinImage>[vectorDirectories.size()];
	dataBaseDescriptors = dataBaseDescriptror1;
//	*dataBaseDescriptors(std::vector<SpinImage> [5]);
//	int * bigarray = new int[N];
}


//class Class
//{
//   int* array;
//   Class(int x) : array(new int[x]) {};
//};


DataBaseDescriptors::~DataBaseDescriptors()
{
	// TODO Auto-generated destructor stub
}


std::vector<std::string> DataBaseDescriptors::open(std::string path)
{
    DIR*    dir;
    dirent* pdir;
    std::vector<std::string> files;

    dir = opendir(path.c_str());

    while (pdir = readdir(dir))
    {
    	// Avoid to take in consideration . and ..
    	if(pdir->d_name[0] != '.')
    	{
    		files.push_back(pdir->d_name);
    	}
    }
    return files;
}

void DataBaseDescriptors::calculateDataBaseDescriptors()
{

	// Define an array that contain all the file that are in the different folders
	std::vector<std::string> vectorFiles[vectorDirectories.size()];

	int i = 0;
	std::string newPath;

	for(std::vector<string>::iterator it = vectorDirectories.begin(); it != vectorDirectories.end(); ++it)
	{

		newPath = path_dataBaseFolder + *it;
		vectorFiles[i] = open(newPath);
		i++;
	}

	// Create a vector of descriptor with the index that correspond to these of vectorDirecorie
	//TODO

	for (int j = 0; j < vectorDirectories.size(); j++)
	{
		std::cout << "Reading files corresponding to the object ---> " << vectorDirectories[j] << endl << endl;
		std::cout << "Create description for following pcl images:" << endl << endl;

		for(std::vector<string>::iterator it = vectorFiles[j].begin(); it != vectorFiles[j].end(); ++it)
		{
			newPath = path_dataBaseFolder + vectorDirectories[j] + "/" + *it;
			std::cout << "--> " << *it;
			std::cout << '\n';
//			std::cout << newPath << "\n";

			// In new path we have the path to come to the file *it
			// We create the descriptor and push it back to the corresponding vector for the same object

			// First for each .pcl file, open the figure, computer the descriptors
			pointCloudDBD.readFile(newPath, cloud);
			pointCloudDBD.computeSpin(cloud,dataBaseDescriptorsPtr);

			// Store it at the right position
			for(int id = 0; id < dataBaseDescriptorsPtr->points.size(); id++)
			{
				dataBaseDescriptors[j].push_back(dataBaseDescriptorsPtr->points[id]);
//				spin_images->points[0]
			}

			// DEBUGGING
//			std::cout << endl << endl << dataBaseDescriptors[j].size() << std::endl <<  endl;

			i++;
		}

		std::cout << endl;
	}

	cout << "Created the dataBaseDescriptor containing all the descriptors for every object" << endl;

	// todo: use this to compare all the descriptors
	// DEBUGGING
//	  for (std::vector<SpinImage>::iterator it = dataBaseDescriptors[1].begin() ; it != dataBaseDescriptors[1].end(); ++it)
//	  {
//	    std::cout << ' ' << *it;
//	  	  std::cout << '\n';
//	  }



}
