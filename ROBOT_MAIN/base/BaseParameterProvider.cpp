/*
 * BaseParameterProvider.cpp
 *
 *  Created on: Apr 27, 2014
 *      Author: root
 */

#include "BaseParameterProvider.h"
#include <stdexcept>
#include "config.h"

BaseParameterProvider *BaseParameterProvider::instance = NULL;
std::string BaseParameterProvider::dir = "";

BaseParameterProvider::BaseParameterProvider(std::string filename) {
	params = new TBUBaseParameters();
	params->load(filename);
}

BaseParameterProvider::~BaseParameterProvider() {
	delete params;
}

BaseParameterProvider* BaseParameterProvider::getInstance()
{
	if(instance==NULL){
		std::string pathToFile = BaseParameterProvider::getFilePath();
		instance = new BaseParameterProvider(pathToFile);
	}
	return instance;
}

void BaseParameterProvider::setDirectory(std::string directory)
{
	dir = directory;
}

std::string BaseParameterProvider::getDirectory()
{
	return dir;
}

std::string BaseParameterProvider::getFilePath()
{
	if (dir == "")
	{
		throw std::runtime_error("You need to set the load directory via BaseParameterProvider::setDirectory once, before calling BaseParameterProvider::getInstance()");
	}

	std::string pathToFile = dir;
	if (*(dir.end()-1) == '/' || *(dir.end()-1) == '\\')
	{
		pathToFile.append(BASE_PARAMETER_FILENAME);
	}
	else
	{
		pathToFile.append(std::string("/") + BASE_PARAMETER_FILENAME);
	}

	return pathToFile;
}

TBUBaseParameters *BaseParameterProvider::getParams()
{
	return params;
}


