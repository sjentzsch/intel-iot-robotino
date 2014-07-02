//
// CloudClient.h
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

#ifndef CLOUDCLIENT_H_
#define CLOUDCLIENT_H_

#include <iostream>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "utils/FileLogger.h"

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

class CloudClient
{
public:
	CloudClient(boost::asio::io_service& io_service_, ::std::string host_, int port_);
	virtual ~CloudClient();

	bool send(::std::string msg);

private:
	void checkDeadline();

	boost::asio::io_service& io_service;
	tcp::socket socket;
	deadline_timer timer;
	tcp::resolver::iterator endpoint_iterator;

	boost::mutex send_mutex;

	::std::string host;
	int port;
};

#endif /* CLOUDCLIENT_H_ */
