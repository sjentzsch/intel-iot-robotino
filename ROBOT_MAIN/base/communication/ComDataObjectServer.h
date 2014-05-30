/*
 * ComDataObjectServer.h
 *
 *  Created on: May 14, 2011
 *      Author: root
 */

#ifndef COMDATAOBJECTSERVER_H_
#define COMDATAOBJECTSERVER_H_

#include <iostream>

//#include "../model/ComDataObject.h"
//#include "../model/WorldModel.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "Communication.h"

using boost::asio::ip::tcp;


class ComDataObjectServer{

private:
	boost::thread *exec_start;
	int port;
	void handleConnections_impl();
public:

	ComDataObjectServer(int port);
	virtual ~ComDataObjectServer();
	void handleConnections();


};

#endif /* COMDATAOBJECTSERVER_H_ */
