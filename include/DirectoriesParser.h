/*
 * DirectoriesParser.h
 *
 *  Created on: Nov 17, 2014
 *      Author: Filippo Martinoni
 *      Note: This class implement function that help with the parse of class elements
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_DIRECTORIESPARSER_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_DIRECTORIESPARSER_H_

#include <dirent.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>

class DirectoriesParser {
public:
	DirectoriesParser(std::string path);
	virtual ~DirectoriesParser();

	std::vector<std::string> open(std::string path);
	void removeAllFiles(std::string path);

	// Getters and setters
	std::vector<std::string>* getVectorElements();
	void setVectorElements(const std::vector<std::string>& vectorElements);

private:
	std::string m_path;
	std::vector<std::string> m_vectorElements;
};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_DIRECTORIESPARSER_H_ */
