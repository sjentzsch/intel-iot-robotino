/*
 * WorldModelClient.h
 *
 *  Created on: Apr 24, 2011
 *      Author: peter
 */

#ifndef WORLDMODELCLIENT_H_
#define WORLDMODELCLIENT_H_

#include <iostream>
#include <limits.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "../utils/FileLogger.h"
#include "../model/ModelProvider.h"

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

class WorldModelClient {
private:
	ComDataObject* comDataObject;

	boost::asio::io_service io_service;
	tcp::socket clientHandle;
	boost::thread *exec_deadlineThread;
	int milliseconds;
	bool enabled;
	bool robotWasKicked;
	IP_ADDRESS addr;
	int port;
	int index;

	bool locked;
	boost::mutex lockedMutex;
	boost::condition_variable lockedCond;

	boost::mutex mutexDeadlineThread;
	bool killDeadlineThread;
	boost::mutex mutexTime;
	uint64_t expireTime;

	void handleConnect(const boost::system::error_code& ec);
	uint64_t getAbsMsTimePlus(uint64_t addMsTime);
	void deadlineThread();

public:
	WorldModelClient(ID::ID id, int milliseconds);
	~WorldModelClient();

	void lock();
	void unlock();

	bool getRobotWasKicked();
	void waitForLock();
};

#endif /* WORLDMODELCLIENT_H_ */
