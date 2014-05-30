/*
 * WorldModelClientHandler.h
 *
 *  Created on: Jun 8, 2011
 *      Author: root
 */

#ifndef WORLDMODELCLIENTHANDLER_H_
#define WORLDMODELCLIENTHANDLER_H_

#include "WorldModelClient.h"
#include "../model/WorldModel.h"
#include "boost/thread.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include "../utils/FileLogger.h"

using boost::asio::deadline_timer;

class WorldModelClientHandler
{
private:
	static WorldModelClientHandler* handler;

	WorldModelClient* clients[3];
	WorldModelClient* server;

	boost::mutex handlerMutex;

	void check_deadline();

	WorldModelClientHandler();
	virtual ~WorldModelClientHandler();

public:
	static WorldModelClientHandler* getInstance();

	void lock();
	void unlock();
};

#endif /* WORLDMODELCLIENTHANDLER_H_ */
