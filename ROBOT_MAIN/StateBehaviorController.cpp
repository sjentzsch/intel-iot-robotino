//
// StateBehaviorController.cpp
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

#include "StateBehaviorController.h"
#include "MotorController.h"
#include "AsyncStateMachine.h"
#include "SensorEventGenerator.h"

StateBehaviorController::StateBehaviorController(MotorController *motorCtrl_, SensorServer *sensorSrv_, SensorEventGenerator *sensorEvtGen_):motorCtrl(motorCtrl_),sensorCtrl(sensorSrv_),sensorEvtGen(sensorEvtGen_){
	asyncStateMachine = new AsyncStateMachine(this);
	motorCtrl->setStateBehaviorController(this);
	sensorEvtGen->setStateBehaviorController(this);
	taskManager = new TaskManager(this,sensorSrv_);
}


StateBehaviorController::~StateBehaviorController() {
}

void StateBehaviorController::initiate()
{
	asyncStateMachine->initiate();
#if SIMULATION_MODE == 0
	sensorEvtGen->initiate();
#endif
}

MotorController *StateBehaviorController::getMotorCtrl()
{
	return motorCtrl;
}

AsyncStateMachine* StateBehaviorController::getAsyncStateMachine()
{
	return asyncStateMachine;
}

ISensorControl* StateBehaviorController::getSensorControl()
{
	return sensorCtrl;
}

TaskManager* StateBehaviorController::getTaskManager(){
	return taskManager;
}
