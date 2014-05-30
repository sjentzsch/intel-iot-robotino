/*
 * LaserScannerServer.h
 *
 *  Created on: Mar 31, 2014
 *      Author: root
 */

#ifndef LASERSCANNERSERVER_H_
#define LASERSCANNERSERVER_H_

#include "../model/LaserScannerData.h"
#include "../model/DynDataProvider.h"
#include <boost/thread.hpp>

class LaserScannerServer {
private:
	boost::thread *exec_start;
	void handleConnections_impl();
public:
	int port;
	LaserScannerServer(int port);
	virtual ~LaserScannerServer();

	void handleConnections();
};

#endif /* LASERSCANNERSERVER_H_ */
