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
	pathfinder = new PathFinder(asyncStateMachine,sensorSrv_);
	jobHandler = new JobHandler(this,sensorSrv_);
	asyncWorldModelUpdater = new AsyncWorldModelUpdater();
}


StateBehaviorController::~StateBehaviorController() {
	// TODO Auto-generated destructor stub
}

void StateBehaviorController::initiate()
{
	float ROBOT1_START_X;
	float ROBOT1_START_Y;
	float ROBOT2_START_X;
	float ROBOT2_START_Y;
	float ROBOT3_START_X;
	float ROBOT3_START_Y;
	if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
	{
		ROBOT1_START_X = 240.0;
		ROBOT1_START_Y = 1120.0;

		ROBOT2_START_X = 240.0;
		ROBOT2_START_Y = 1680.0;

		ROBOT3_START_X = 240.0;
		ROBOT3_START_Y = 2240.0;
	}
	else
	{
		ROBOT1_START_X = 240.0;
		ROBOT1_START_Y = 8960.0;

		ROBOT2_START_X = 240.0;
		ROBOT2_START_Y = 9520.0;

		ROBOT3_START_X = 240.0;
		ROBOT3_START_Y = 10080.0;
	}

	switch(ModelProvider::getInstance()->getID()){
		case ID::ROBO1: sensorCtrl->setOdometry(ROBOT1_START_X,ROBOT1_START_Y,0); break;// reset odometry at the very first start;
		case ID::ROBO2: sensorCtrl->setOdometry(ROBOT2_START_X,ROBOT2_START_Y,0); break;
		case ID::ROBO3: sensorCtrl->setOdometry(ROBOT3_START_X,ROBOT3_START_Y,0); break;	// phi was -90 before
		case ID::SERVER: break; //do nothing
	}

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

PathFinder* StateBehaviorController::getPathFinder(){
	return pathfinder;
}

JobHandler* StateBehaviorController::getJobHandler(){
	return jobHandler;
}

AsyncWorldModelUpdater * StateBehaviorController::getAsyncWorldModelUpdater(){
	return asyncWorldModelUpdater;
}


