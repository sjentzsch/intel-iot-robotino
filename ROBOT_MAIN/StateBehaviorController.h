/*
 * StateBehaviorController.h
 *
 *  Created on: 23.04.2011
 *      Author: root
 */

#ifndef STATEBEHAVIORCONTROLLER_H_
#define STATEBEHAVIORCONTROLLER_H_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/statechart/event.hpp>
#include "config.h"
#include "ISensorControl.h"
#include "AsyncStateMachine.h"
#include "pathfinder/PathFinder.h"
#include "JobPlanner/JobHandler.h"
#include "AsyncWorldModelUpdater.h"


class MotorController;
class SensorServer;
class SensorEventGenerator;
class JobHandler;

using namespace std;
namespace sc = boost::statechart;


class StateBehaviorController {
private:
	AsyncStateMachine* asyncStateMachine;
	AsyncWorldModelUpdater* asyncWorldModelUpdater;
	MotorController* motorCtrl;
	ISensorControl* sensorCtrl;
	SensorEventGenerator* sensorEvtGen;
	PathFinder* pathfinder;
	JobHandler *jobHandler;

public:
	StateBehaviorController(MotorController *motorCtrl_, SensorServer *sensorSrv_, SensorEventGenerator* sensorEvtGen_);
	virtual ~StateBehaviorController();
	MotorController *getMotorCtrl();
	AsyncStateMachine* getAsyncStateMachine();
	ISensorControl* getSensorControl();
	PathFinder* getPathFinder();
	JobHandler* getJobHandler();
	AsyncWorldModelUpdater * getAsyncWorldModelUpdater();

	void initiate(); //very first start of the stateMachine

};

#endif /* STATEBEHAVIORCONTROLLER_H_ */
