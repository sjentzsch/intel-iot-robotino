/*
 * CloudServer.h
 *
 *  Created on: Jun 4, 2014
 *      Author: root
 */

#ifndef CLOUDSERVER_H_
#define CLOUDSERVER_H_

#include <iostream>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <DataProvider.h>

using boost::asio::ip::tcp;

class CloudServer
{
public:
	CloudServer(int port);
	virtual ~CloudServer();

	void handleConnections();

private:
	void handleConnections_impl();

	boost::thread *exec_start;
	int port;
};

#endif /* CLOUDSERVER_H_ */
