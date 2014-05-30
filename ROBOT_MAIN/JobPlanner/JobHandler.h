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
#include "../pathfinder/PathFinder.h"
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
	Job * lastJobInCaseOfRandomMovement;
	boost::thread *nextTask_exec; //thread for finding the path
	AsyncStateMachine* asyncStateMachine;
	PathFinder* pathfinder;
	Grid* grid;
	POI* randomPOI;
	Time TimeStampT5_1;
	Time TimeStampT5_2;
//	Time TimeStampT4_1;
//	Time TimeStampT4_2;
//	Time TimeStampT3_1;
//	Time TimeStampT3_2;
	bool foundT1ForRPC;
//	bool T3timingActive;
//	bool T4timingActive;
	bool T5timingActive;
	bool leaveLeft;

	void updateProdMachineType( Job* dJob, CameraLightState::CameraLightState lamp);
	void updateRecMachineType( Job* rJob, CameraLightState::CameraLightState lamp);

	void nextTask_impl();

	void handleJob(Job* djob);

	void handleOutOfOrder_impl();
//	void updatePoiRequired(POI* poi);

	//leaves to poi depending on the job before
	JobProgressStatus::JobProgressStatus leaveProductionMachine(Job* currentJob,Job* lastJob);
	JobProgressStatus::JobProgressStatus prepareJob(Job* djob);

	void removePuckFromProductionMachine(POI* poi);
	void getOutOfMachine(Job* dJob, POI* targetPOI=NULL, POIAccessFrom::POIAccessFrom targetAccDir=POIAccessFrom::FRONT);
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
