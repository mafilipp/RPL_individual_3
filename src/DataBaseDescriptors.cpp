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

DataBaseDescriptors::DataBaseDescriptors() {
	// TODO Auto-generated constructor stub

}

DataBaseDescriptors::~DataBaseDescriptors() {
	// TODO Auto-generated destructor stub
}


std::vector<std::string> DataBaseDescriptors::open(std::string path = ".")
{

    DIR*    dir;
    dirent* pdir;
    std::vector<std::string> files;

    dir = opendir(path.c_str());

    while (pdir = readdir(dir)) {
        files.push_back(pdir->d_name);
    }

    return files;
}

//int main() {
//
//    std::vector<std::string> f;
//
//    f = open(); // or pass which dir to open
//
//    std::fstream file;
//
//    file.open(f[0]);
//
//    return 0;
//}
