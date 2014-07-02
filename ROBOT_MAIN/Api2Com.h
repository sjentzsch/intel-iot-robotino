//
// Api2Com.h
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Sören Jentzsch
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
