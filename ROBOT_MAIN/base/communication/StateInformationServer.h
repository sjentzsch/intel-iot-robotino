/*
 * StateInformationServer.h
 *
 *  Created on: Jun 7, 2011
 *      Author: root
 */

#ifndef STATEINFORMATIONSERVER_H_
#define STATEINFORMATIONSERVER_H_
#include "../model/StateInformation.h"
#include "../model/DynDataProvider.h"
#include <boost/thread.hpp>

class StateInformationServer {
private:
	boost::thread *exec_start;
	void handleConnections_impl();
public:
	int port;
	StateInformationServer(int port);
	virtual ~StateInformationServer();

	void handleConnections();
};

#endif /* STATEINFORMATIONSERVER_H_ */
