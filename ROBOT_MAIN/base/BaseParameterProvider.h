/*
 * BaseParameterProvider.h
 *
 *  Created on: Apr 27, 2014
 *      Author: root
 */

#ifndef BASEPARAMETERPROVIDER_H_
#define BASEPARAMETERPROVIDER_H_

#include "TBUBaseParameters.h"
#include <string>

class BaseParameterProvider {
public:
	BaseParameterProvider(std::string filename);
	virtual ~BaseParameterProvider();

	static BaseParameterProvider* getInstance();
	static void setDirectory(std::string directory);
	static std::string getDirectory();
	static std::string getFilePath();
	TBUBaseParameters *getParams();

private:
	static std::string dir;
	static BaseParameterProvider *instance;
	TBUBaseParameters *params;
};

#endif /* BASEPARAMETERPROVIDER_H_ */
