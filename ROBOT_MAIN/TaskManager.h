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

class TaskManager
{
public:
	TaskManager(StateBehaviorController* stateBhvContrl, SensorServer *sensorSrv_);
	virtual ~TaskManager();
	void nextTask(); //creates a new thread which triggers a new event
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

#endif /* TaskManager_H_ */
