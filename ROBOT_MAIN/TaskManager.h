//
// TaskManager.h
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Sören Jentzsch
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
