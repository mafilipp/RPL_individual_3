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

DataBaseDescriptors::DataBaseDescriptors(std::string path_dataBaseFolder_a) {
	// TODO Auto-generated constructor stub
	path_dataBaseFolder = path_dataBaseFolder_a;
	vectorDirectories = open(path_dataBaseFolder);
}

DataBaseDescriptors::~DataBaseDescriptors() {
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

//	std::vector<std::string> vectorDirectories;
//	std::string path_dataBaseFolder = "/home/mafilipp/data/objects/";

//	vectorDirectories = open(path_dataBaseFolder);

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
			newPath = path_dataBaseFolder + vectorDirectories[j] + *it;
			std::cout << ' ' << *it;
			std::cout << '\n';
//			std::cout << newPath << "\n";

			// In new path we have the path to come to the file *it
			// We create the descriptor and push it back to the corresponding vector for the same object
			//TODO

			i++;
		}

		std::cout << endl;
	}

}
