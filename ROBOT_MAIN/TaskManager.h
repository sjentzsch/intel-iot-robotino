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
	void nextTask(); //creates a new thread which triggers a new event
	void serveDrink();
	MsgCustomerOrder getCurrCustomerOrder();

private:
	void nextTask_impl();

	std::string currState;
	MsgCustomerOrder* currCustomerOrder;
	vector< MsgRobotServed > vecMsgRobotServed;

	boost::thread* nextTask_exec;
	AsyncStateMachine* asyncStateMachine;
	SensorServer* sensorServer;
};

#endif /* TaskManager_H_ */
