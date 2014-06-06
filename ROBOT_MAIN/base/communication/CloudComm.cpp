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
	this->cloudServer = new CloudServer(8190);

	boost::asio::io_service io_service;
	this->cloudClient = new CloudClient(io_service, "localhost", 8190);
	// TODO: a bit nasty, but who cares ...
	boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service));
}

CloudComm::~CloudComm()
{

}

CloudComm* CloudComm::getInstance()
{
	if(cloudComm == NULL)
		cloudComm = new CloudComm();
	return cloudComm;
}

CloudServer* CloudComm::getCloudServer()
{
	return this->cloudServer;
}

CloudClient* CloudComm::getCloudClient()
{
	return this->cloudClient;
}
