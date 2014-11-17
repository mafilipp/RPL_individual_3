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
//	parser(path_dataBaseFolder_a);

	path_dataBaseFolder = path_dataBaseFolder_a;
	vectorDirectories = open(path_dataBaseFolder);

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
//	vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> > *dataBasePtr1 = new vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> >(vectorDirectories.size());
//	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
//	cloud = cloud1;
//	dataBaseDescriptorsPtr = spin_images;
	//	std::vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> > *dataBasePtr = new std::vector<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> >(10);

//	ROS_INFO("Detected %d objects", vectorDirectories.size());
//	std::vector<SpinImage> * dataBaseDescriptror1 = new std::vector<SpinImage>[vectorDirectories.size()];
//	dataBaseDescriptors = dataBaseDescriptror1;
//	*dataBaseDescriptors(std::vector<SpinImage> [5]);
//	int * bigarray = new int[N];
	std::cout << endl << endl << endl << "Detected " << vectorDirectories.size() << " objects"<< endl << endl << endl;

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
	std::cout << "vector size = 	%d" << vectorDirectories.size() << std::endl;

	std::vector<std::string> vectorFiles[vectorDirectories.size()];

	int i = 0;
	std::string newPath;

	for(std::vector<string>::iterator it = vectorDirectories.begin(); it != vectorDirectories.end(); ++it)
	{
		newPath = path_dataBaseFolder + "/" + *it;
		std::cout << newPath << std::endl << std::endl;
		vectorFiles[i] = open(newPath);
		i++;
	}

	// Create a vector of descriptor with the index that correspond to these of vectorDirecories



	for (int j = 0; j < vectorDirectories.size(); j++)
	{
//		dataBaseDescriptors.back().push_back( ... );
//
//		dataBaseDescriptors.at(...).push_back( ...)

		std::cout << "Reading files corresponding to the object ---> " << vectorDirectories[j] << endl << endl;
		std::cout << "Create description for following pcl images:" << endl << endl;

		std::vector<pcl::PointCloud<SpinImage> >  vectorTMP;


		for(std::vector<string>::iterator it = vectorFiles[j].begin(); it != vectorFiles[j].end(); ++it)
		{

//			pcl::PointCloud<SpinImage> tmp;
//			  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
//			pcl::PointCloud<SpinImage>::Ptr tmpPtr = boost::make_shared <pcl::PointCloud<SpinImage> >(*tmp);

			pcl::PointCloud<SpinImage>::Ptr tmpPtr (new pcl::PointCloud<SpinImage>);

			newPath = path_dataBaseFolder + vectorDirectories[j] + "/" + *it;
			std::cout << "--> " << *it;
			std::cout << '\n';

			// In new path we have the path to come to the file *it
			// We create the descriptor and push it back to the corresponding vector for the same object
			// For each .pcl file, computer the spin image

			pointCloudDBD.computeSpin(newPath,tmpPtr);

			vectorTMP.push_back(*tmpPtr);

			// Store it at the right position
//			for(int id = 0; id < dataBaseDescriptorsPtr->points.size(); id++)
//			{
//				dataBaseDescriptors[j].push_back(dataBaseDescriptorsPtr->points[id]);
//			}

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
