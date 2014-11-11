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


class DataBaseDescriptors {
public:
	DataBaseDescriptors();
	virtual ~DataBaseDescriptors();
	std::vector<std::string> DataBaseDescriptors::open(std::string path = ".")

};


#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_DATABASEDESCRIPTORS_H_ */
