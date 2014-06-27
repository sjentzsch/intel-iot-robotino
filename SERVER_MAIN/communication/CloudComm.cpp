//============================================================================
// Name        : CloudComm.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "CloudComm.h"

CloudComm* CloudComm::cloudComm = NULL;

CloudComm::CloudComm()
{
	this->cloudServer = NULL;
	this->cloudClient = NULL;
}

CloudComm::~CloudComm()
{
	delete this->cloudServer;
	delete this->cloudClient;
}

CloudComm* CloudComm::getInstance()
{
	if(cloudComm == NULL)
		cloudComm = new CloudComm();
	return cloudComm;
}

void CloudComm::init(CloudServer* cloudServer, CloudClient* cloudClient)
{
	this->cloudServer = cloudServer;
	this->cloudClient = cloudClient;
}

CloudServer* CloudComm::getCloudServer()
{
	return this->cloudServer;
}

CloudClient* CloudComm::getCloudClient()
{
	return this->cloudClient;
}
