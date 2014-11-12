/*
 * DataBaseDescriptors.h
 *
 *  Created on: Nov 11, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_

#include <vector>
#include <string>

using namespace std;

class DataBaseDescriptors {
public:
	DataBaseDescriptors(std::string path_dataBaseFolder_a);
	virtual ~DataBaseDescriptors();
//	std::vector<std::string> open(std::string path = ".");
	std::vector<std::string> open(std::string path);
	void calculateDataBaseDescriptors();

private:
	std::vector<std::string> vectorDirectories;
	std::string path_dataBaseFolder;


};


#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_ */
