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
#include "TaskGuide.h"


class MotorController;
class SensorServer;
class SensorEventGenerator;
class TaskGuide;

using namespace std;
namespace sc = boost::statechart;


class StateBehaviorController {
private:
	AsyncStateMachine* asyncStateMachine;
	MotorController* motorCtrl;
	ISensorControl* sensorCtrl;
	SensorEventGenerator* sensorEvtGen;
	TaskGuide *taskGuide;

public:
	StateBehaviorController(MotorController *motorCtrl_, SensorServer *sensorSrv_, SensorEventGenerator* sensorEvtGen_);
	virtual ~StateBehaviorController();
	MotorController *getMotorCtrl();
	AsyncStateMachine* getAsyncStateMachine();
	ISensorControl* getSensorControl();
	TaskGuide* getTaskGuide();

	void initiate(); //very first start of the stateMachine
};

#endif /* STATEBEHAVIORCONTROLLER_H_ */
