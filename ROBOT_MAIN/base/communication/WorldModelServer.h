/*
 * WorldModelServer.h
 *
 *  Created on: Apr 24, 2011
 *      Author: peter
 */

#ifndef WORLDMODELSERVER_H_
#define WORLDMODELSERVER_H_

#include <iostream>
#include <limits.h>

#include "../model/WorldModel.h"
#include "Communication.h"
#include "../utils/FileLogger.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::tcp;

class WorldModelServer {
private:
	boost::thread *exec_start;
	int port;
	void handleWorldModelClients_impl();

	int milliseconds;
	boost::thread *exec_deadlineThread;
	boost::mutex mutexTime;
	uint64_t expireTime;
	uint64_t getAbsMsTimePlus(uint64_t addMsTime);
	void deadlineThread();
public:
	WorldModelServer(int port);
	~WorldModelServer();
	void handleWorldModelClients();
};

#endif /* WORLDMODELSERVER_H_ */