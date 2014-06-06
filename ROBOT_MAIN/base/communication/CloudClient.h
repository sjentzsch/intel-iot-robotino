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

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

class CloudClient
{
public:
	CloudClient(boost::asio::io_service& io_service_, ::std::string host_, int port_);
	virtual ~CloudClient();

	void sendTest();

private:
	void checkDeadline();
	void handleConnect(const boost::system::error_code& ec);

	boost::asio::io_service& io_service;
	tcp::socket socket;
	deadline_timer timer;

	::std::string host;
	int port;
};

#endif /* CLOUDCLIENT_H_ */
