/*
 * CloudClient.h
 *
 *  Created on: Jun 4, 2014
 *      Author: root
 */

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
