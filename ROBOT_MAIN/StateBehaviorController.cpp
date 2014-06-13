/*
 * StateBehaviorController.cpp
 *
 *  Created on: 23.04.2011
 *      Author: root
 */

#include "StateBehaviorController.h"
#include "MotorController.h"
#include "AsyncStateMachine.h"
#include "SensorEventGenerator.h"

StateBehaviorController::StateBehaviorController(MotorController *motorCtrl_, SensorServer *sensorSrv_, SensorEventGenerator *sensorEvtGen_):motorCtrl(motorCtrl_),sensorCtrl(sensorSrv_),sensorEvtGen(sensorEvtGen_){
	asyncStateMachine = new AsyncStateMachine(this);
	motorCtrl->setStateBehaviorController(this);
	sensorEvtGen->setStateBehaviorController(this);
	taskGuide = new TaskGuide(this,sensorSrv_);
}


StateBehaviorController::~StateBehaviorController() {
}

void StateBehaviorController::initiate()
{
	sensorCtrl->setOdometry(240.0,1120.0,0);

	asyncStateMachine->initiate();
	sensorEvtGen->initiate();
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

TaskGuide* StateBehaviorController::getTaskGuide(){
	return taskGuide;
}
