/*
 * TaskManager.h
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#ifndef TaskManager_H_
#define TaskManager_H_

#include <boost/thread.hpp>
#include "config.h"
#include "StateBehaviorController.h"
#include "StateMachineEvents.h"
#include "AsyncStateMachine.h"
#include "SensorServer.h"
#include "DataProvider.h"

class TaskManager
{
public:
	TaskManager(StateBehaviorController* stateBhvContrl, SensorServer *sensorSrv_);
	virtual ~TaskManager();
	void nextTask(); 		// creates a new thread which triggers a new event
	void startSendBeacon();	// creates a new thread which sends beacons periodically
	void serveDrink();
	void setCurrState(std::string newState);
	MsgCustomerOrder getCurrCustomerOrder();

private:
	void nextTask_impl();
	void sendBeacon_impl();

	std::string currState;
	MsgCustomerOrder* currCustomerOrder;
	vector< MsgRobotServed > vecMsgRobotServed;

	boost::thread* nextTask_exec;
	boost::thread* sendBeacon_exec;
	AsyncStateMachine* asyncStateMachine;
	SensorServer* sensorServer;
};

#endif /* TaskManager_H_ */
