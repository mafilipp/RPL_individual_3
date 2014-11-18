/*
 * DataBaseDescriptors.cpp
 *
 *  Created on: Nov 11, 2014
 *      Author: Filippo Martinoni
 *      Note: Implementation of the class DataBaseDescriptors
 */

#include "DataBaseDescriptors.h"

using namespace std;

DataBaseDescriptors::DataBaseDescriptors(std::string path_dataBaseFolder_a)
{
	path_dataBaseFolder = path_dataBaseFolder_a;
	vectorDirectories = open(path_dataBaseFolder);

	std::cout << endl << "Detected " << vectorDirectories.size() << " objects" << endl;

}


DataBaseDescriptors::~DataBaseDescriptors()
{
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
	std::cout << "vector size = 	%d" << vectorDirectories.size() << std::endl;

	std::vector<std::string> vectorFiles[vectorDirectories.size()];

	int i = 0;
	std::string newPath;

	for(std::vector<string>::iterator it = vectorDirectories.begin(); it != vectorDirectories.end(); ++it)
	{
		newPath = path_dataBaseFolder + *it;
		std::cout << newPath << std::endl << std::endl;
		vectorFiles[i] = open(newPath);
		i++;
	}

	// Create a vector of descriptor with the index that correspond to these of vectorDirecories
	for (int j = 0; j < vectorDirectories.size(); j++)
	{
		std::cout << "Reading files corresponding to the object ---> " << vectorDirectories[j] << endl << endl;
		std::cout << "Create description for following pcl images:" << endl << endl;

		std::vector<pcl::PointCloud<SpinImage> >  vectorTMP;


		for(std::vector<string>::iterator it = vectorFiles[j].begin(); it != vectorFiles[j].end(); ++it)
		{
			pcl::PointCloud<SpinImage>::Ptr tmpPtr (new pcl::PointCloud<SpinImage>);

			newPath = path_dataBaseFolder + vectorDirectories[j] + "/" + *it;
			std::cout << "--> " << *it;
			std::cout << '\n';

			// In new path we have the path to come to the file *it
			// We create the descriptor and push it back to the corresponding vector for the same object
			// For each .pcl file, computer the spin image

			pointCloudDBD.computeSpin(newPath,tmpPtr);
			vectorTMP.push_back(*tmpPtr);

			i++;
		}

		std::cout << endl;
		dataBaseDescriptors.push_back(vectorTMP);
	}
	cout << "Created the dataBaseDescriptor containing all the descriptors for every object" << endl;
}

const std::vector<std::vector<pcl::PointCloud<SpinImage> > >& DataBaseDescriptors::getDataBaseDescriptors() const {
	return dataBaseDescriptors;
}

void DataBaseDescriptors::setDataBaseDescriptors(
		const std::vector<std::vector<pcl::PointCloud<SpinImage> > >& dataBaseDescriptors) {
	this->dataBaseDescriptors = dataBaseDescriptors;
}
