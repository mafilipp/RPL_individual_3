/*
 * DirectoriesParser.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: mafilipp
 */

#include "DirectoriesParser.h"
#include <dirent.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>


DirectoriesParser::DirectoriesParser(std::string path) {
	m_path = path;
	m_vectorElements = open(path);
	// TODO Auto-generated constructor stub

}

DirectoriesParser::~DirectoriesParser() {
	// TODO Auto-generated destructor stub
}

std::vector<std::string> DirectoriesParser::open(std::string path)
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

    std::cout << "Element found:" << std::endl;
    for (std::vector<std::string>::iterator it = files.begin() ; it != files.end(); ++it)
    	std::cout << ' ' << *it << '\n';

    return files;
}

const std::vector<std::string>& DirectoriesParser::getVectorElements() const {
	return m_vectorElements;
}

void DirectoriesParser::removeAllFiles(std::string path)
{
    std::vector<std::string> files = open (path);
    std::string currentFile;
    for (std::vector<std::string>::iterator it = files.begin() ; it != files.end(); ++it)
    {
    	currentFile = path + *it;
//    	std::cout << "I would remove" << currentFile << std::endl;
    	remove(currentFile.c_str());
    }


}

void DirectoriesParser::setVectorElements(
		const std::vector<std::string>& vectorElements) {
	m_vectorElements = vectorElements;
}
