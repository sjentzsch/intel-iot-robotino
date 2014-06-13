/*
 * TaskGuide.h
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#ifndef TASKGUIDE_H_
#define TASKGUIDE_H_

#include <boost/thread.hpp>
#include "config.h"
#include "StateBehaviorController.h"
#include "StateMachineEvents.h"
#include "AsyncStateMachine.h"
#include "SensorServer.h"

class TaskGuide
{
public:
	TaskGuide(StateBehaviorController* stateBhvContrl, SensorServer *sensorSrv_);
	virtual ~TaskGuide();
	void nextTask(); //creates a new thread which triggers a new event dependent on the current job situation
	void updatePoiType(CameraLightState::CameraLightState lamp); //provides an interface for updating the POI-TYPE dependent on the seen lamp status
	void handleOutOfOrder();
	void startMachineTiming();
	void endMachineTiming(POIType::POIType type);

private:
	boost::thread *nextTask_exec; //thread for finding the path
	AsyncStateMachine* asyncStateMachine;

	void nextTask_impl();

	void handleJob();

	//void triggerEventDeliverPuckToGate(Node* accessNode);
};

#endif /* TASKGUIDE_H_ */
