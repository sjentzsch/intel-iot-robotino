/*
 * Api2Com.h
 *
 *  Created on: Mar 20, 2014
 *      Author: root
 */

#ifndef API2COM_H_
#define API2COM_H_

#include <iostream>
#include <rec/robotino/api2/Com.h>
#include <rec/robotino/api2/utils.h>

#include "utils/FileLogger.h"

class Api2Com : public rec::robotino::api2::Com
{
public:
	Api2Com()
		: rec::robotino::api2::Com( "Api2Com" )
	{
	}

	void errorEvent( const char* errorString )
	{
		FileLog::log_ERROR("[Api2Com] ERROR: ", errorString);
	}

	void connectedEvent()
	{
		FileLog::log_NOTICE("[Api2Com] Connected event.");
	}

	void connectionClosedEvent()
	{
		FileLog::log_NOTICE("[Api2Com] Connection closed.");
	}

	void logEvent( const char* message, int level )
	{
		FileLog::log_NOTICE("[Api2Com] Event: ", message);
	}

	void pingEvent( float timeMs )
	{
		//std::cout << "Ping: " << timeMs << "ms" << std::endl;
	}
};

#endif /* API2COM_H_ */
