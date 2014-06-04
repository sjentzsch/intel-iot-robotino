/*
 * JobHandler.h
 *
 *  Created on: Jun 11, 2011
 *      Author: root
 */

#ifndef JOBHANDLER_H_
#define JOBHANDLER_H_

#include <boost/thread.hpp>
#include "JobPlanner.h"
#include "config.h"
#include "../StateBehaviorController.h"
#include "../StateMachineEvents.h"
#include "../AsyncStateMachine.h"
#include "../SensorServer.h"
#include "JobPlannerEnums.h"
#include "Jobs.h"

class JobPlanner;

class JobHandler {
public:
	JobHandler(StateBehaviorController* stateBhvContrl, SensorServer *sensorSrv_);
	virtual ~JobHandler();
	void nextTask(); //creates a new thread which triggers a new event dependent on the current job situation
	void updatePoiType(CameraLightState::CameraLightState lamp); //provides an interface for updating the POI-TYPE dependent on the seen lamp status
	void handleOutOfOrder();
	void startMachineTiming();
	void endMachineTiming(POIType::POIType type);

private:
	JobPlanner* jobPlanner;
	Job* currentJob;
	Job* lastJob;
	boost::thread *nextTask_exec; //thread for finding the path
	AsyncStateMachine* asyncStateMachine;
	Grid* grid;

	void nextTask_impl();

	void handleJob(Job* djob);

	/*
	 * Event Triggerings
	 */
	void triggerEventDriveToPoi(POI* poiTo,POI* poiFrom, POIAccessFrom::POIAccessFrom accDir = POIAccessFrom::FRONT);
	void triggerEventDriveToPoiWithPuck(POI* poi,POI* poiFrom, POIAccessFrom::POIAccessFrom accDir = POIAccessFrom::FRONT);

	void triggerEventContinueDeliveringPuck();

	void triggerEventLeavePoiBackwards(POI* poiFrom, POI* targetPOI, POIAccessFrom::POIAccessFrom targetAccDir);
	void triggerEventLeavePoiWithoutPuck(POI* poiFrom,POIAccessFrom::POIAccessFrom accDirFrom,LeaveDirection::LeaveDirection leaveDir,POI* poiTo,POIAccessFrom::POIAccessFrom accDirTo);
	void triggerEventLeavePoiWithPuck(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, POI* poiTo, POIAccessFrom::POIAccessFrom accDirTo);
	void triggerEventLeavePoiWithPuckRotatingOut(POI* poiFrom);
	void triggerEventLeaveRecMachineWithPuck(POI* poiTo, POIAccessFrom::POIAccessFrom accDirTo);

	void triggerEventGrabPuck(POI* poiFrom, POI* poiTo, bool gpmSidewards, bool gpmRec);
	void triggerEventGrabPuckSideOnFront(POI* poiFrom, LeaveDirection::LeaveDirection leaveDir);
	void triggerEventGrabPuckInputZone(POI* poiFrom, POI* poiTo, POIAccessFrom::POIAccessFrom accDirTo);

	void triggerEventDeliverPuckToPoi();
	void triggerEventDeliverPuckToGate(Node* accessNode);

	void triggerEventMoveToAndCheckRPC();

	void triggerEventNoJobFound();

};

#endif /* JOBHANDLER_H_ */
