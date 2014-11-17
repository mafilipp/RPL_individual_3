/*
 * DirectoriesParser.h
 *
 *  Created on: Nov 17, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_OBJECT_RECOGNITION_SRC_DIRECTORIESPARSER_H_
#define MAFILIPP_OBJECT_RECOGNITION_SRC_DIRECTORIESPARSER_H_

#include <string>
#include <vector>
#include <iostream>

class DirectoriesParser {
public:
	DirectoriesParser(std::string path);
	virtual ~DirectoriesParser();

	std::vector<std::string> open(std::string path);
	void removeAllFiles(std::string path);


	// Getters and setters
	const std::vector<std::string>& getVectorElements() const;
	void setVectorElements(const std::vector<std::string>& vectorElements);

private:
	std::string m_path;
	std::vector<std::string> m_vectorElements;
};

#endif /* MAFILIPP_OBJECT_RECOGNITION_SRC_DIRECTORIESPARSER_H_ */
