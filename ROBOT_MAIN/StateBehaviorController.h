//
// StateBehaviorController.h
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

#ifndef STATEBEHAVIORCONTROLLER_H_
#define STATEBEHAVIORCONTROLLER_H_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/statechart/event.hpp>
#include "config.h"
#include "ISensorControl.h"
#include "AsyncStateMachine.h"
#include "TaskManager.h"


class MotorController;
class SensorServer;
class SensorEventGenerator;
class TaskManager;

using namespace std;
namespace sc = boost::statechart;


class StateBehaviorController {
private:
	AsyncStateMachine* asyncStateMachine;
	MotorController* motorCtrl;
	ISensorControl* sensorCtrl;
	SensorEventGenerator* sensorEvtGen;
	TaskManager *taskManager;

public:
	StateBehaviorController(MotorController *motorCtrl_, SensorServer *sensorSrv_, SensorEventGenerator* sensorEvtGen_);
	virtual ~StateBehaviorController();
	MotorController *getMotorCtrl();
	AsyncStateMachine* getAsyncStateMachine();
	ISensorControl* getSensorControl();
	TaskManager* getTaskManager();

	void initiate(); //very first start of the stateMachine
};

#endif /* STATEBEHAVIORCONTROLLER_H_ */
