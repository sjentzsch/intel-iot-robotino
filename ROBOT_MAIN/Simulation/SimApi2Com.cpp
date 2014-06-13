/*
 * SimApi2Com.cpp
 *
 *  Created on: Apr 19, 2014
 *      Author: root
 */

#include "SimApi2Com.h"
#include "utils/FileLogger.h"

SimApi2Com::~SimApi2Com()
{
}

SimApi2Com::SimApi2Com() : rec::robotino::api2::Com( "Api2Com" )
{
}

bool SimApi2Com::isConnected() const
{
	return true;
}

void SimApi2Com::connectToServer( bool isBlocking )
{
	FileLog::log_NOTICE("[SimApi2Com] Connected event.");
	return;
}

bool SimApi2Com::isLocalConnection() const
{
	return true;
}

void SimApi2Com::processEvents()
{
	return;
}

