//
// SimApi2Com.cpp
//
// Authors:
//   Stefan Profanter <stefan.profanter@gmail.com>
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Stefan Profanter, Sören Jentzsch
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

