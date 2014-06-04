/*
 * JobHandler.cpp
 *
 *  Created on: Jun 11, 2011
 *      Author: root
 */

#include "JobHandler.h"

JobHandler::JobHandler(StateBehaviorController* stateBhvContrl,SensorServer *sensorSrv_):asyncStateMachine(stateBhvContrl->getAsyncStateMachine()) {
	grid = new Grid();
	jobPlanner = new JobPlanner(sensorSrv_);
	nextTask_exec = NULL;
	currentJob = NULL;
	lastJob = NULL;
}

JobHandler::~JobHandler() {
	if(grid != NULL)
		delete grid;
}

void JobHandler::nextTask(){
	while(true){
		try{
			if(nextTask_exec !=NULL){
				nextTask_exec->join();
			}

			nextTask_exec = new boost::thread(&JobHandler::nextTask_impl,this);

			break;
		}catch (std::exception& e) {
		   std::cerr << "[JobHandler] Caught exception: " << e.what() << std::endl;
		}

	}
}


void JobHandler::nextTask_impl()
{
	if(currentJob!=NULL&&currentJob->status==JobProgressStatus::FINISHED_JOB){

		// TODO: deletion of jobs good here?
		if(lastJob != NULL)
			delete lastJob;

		lastJob = currentJob;
		currentJob = NULL;
	}

	while(currentJob==NULL){

		while(ModelProvider::getInstance()->gameStateIsPaused()){
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		}

		currentJob = jobPlanner->giveNewJob(lastJob);

		if(currentJob==NULL){
			boost::this_thread::sleep(boost::posix_time::seconds(2));
		}
	}

	if(currentJob == NULL){
		//trigger no job found (handle this here for the moment, TODO: prob. Statemachine should handle this (Pathfinder as well));
		//triggerEventNoJobFound();
	}
	else{
		handleJob(currentJob);
	}
}

void JobHandler::handleJob(Job* djob){
	switch(djob->status){
		case JobProgressStatus::STARTING:
			FileLog::log(log_Job,"[JobHandler] handleJob Status STARTING");
			//currentJob->status = prepareJob(djob);
			break;
		case JobProgressStatus::STARTED:
			FileLog::log(log_Job,"[JobHandler] handleJob Status STARTED");
			// we moved out of the old PoiTo, release it

			if(lastJob!=NULL && lastJob->POITo->index == djob->POIFrom->index){
				//in this case it is assumed that we have the current puck for the next task as well (TODO: check assumption)
				//omit 1 Machine since we already have the puck
				triggerEventDriveToPoiWithPuck(djob->POITo,lastJob->POITo);
				currentJob->status = JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_2;
			}
			else{
				if(lastJob->jobType == JobType::Explore){
					triggerEventDriveToPoi(currentJob->POIFrom,lastJob->POITo);
				}
				else{
					triggerEventDriveToPoi(djob->POIFrom,NULL, djob->accDir);
				}
				currentJob->status = JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_1;
			}
			break;
		case JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_1:
			FileLog::log(log_Job,"[JobHandler] handleJob Status DRIVEN_TO_TARGET_MACHINE_1");
			// we have driven to the target machine of the new job, at latest release old PoiTo here

			//TRIGGER EVENT FOR GRABBING THE PUCK in front of the machine
			if(djob->POIFrom->type==POIType::INPUT){
				triggerEventGrabPuckInputZone(djob->POIFrom,djob->POITo, POIAccessFrom::FRONT);
				currentJob->status = JobProgressStatus::OUT_OF_MACHINE_1;
			}else{
				//fetch puck from a machine

				// set puck accordingly
				if(djob->jobClass == JobClass::Recycle)
				{
					Job* rjob = djob;
					if(rjob->jobType == JobType::RECYCLE_PUCK_AT_T2_LEFT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T3_LEFT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T4_LEFT_PUCK)
						rjob->POIFrom->left = Puck::NON_EXISTING;
					else if(rjob->jobType == JobType::RECYCLE_PUCK_AT_T2_RIGHT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T3_RIGHT_PUCK  || rjob->jobType == JobType::RECYCLE_PUCK_AT_T4_RIGHT_PUCK)
						rjob->POIFrom->right = Puck::NON_EXISTING;
				}

				//do not set the puck at a production machine as removed yet, it could be that we still have to wait
				//for a green-yellow or red light to turn green (because we leave the puck there to produce)
				else if (djob->POIFrom->type!=POIType::T3 && djob->POIFrom->type!=POIType::T4 && djob->POIFrom->type!=POIType::T5)
					;//removePuckFromProductionMachine(djob->POIFrom);

				if(djob->jobClass == JobClass::Recycle)
				{
					if(djob->accDir == POIAccessFrom::FRONT)
					{
						if(djob->jobType == JobType::RECYCLE_PUCK_AT_T2_LEFT_PUCK || djob->jobType == JobType::RECYCLE_PUCK_AT_T3_LEFT_PUCK || djob->jobType == JobType::RECYCLE_PUCK_AT_T4_LEFT_PUCK)
							triggerEventGrabPuckSideOnFront(djob->POIFrom, LeaveDirection::LEFT);
						else
							triggerEventGrabPuckSideOnFront(djob->POIFrom, LeaveDirection::RIGHT);
					}
					else
						triggerEventGrabPuck(djob->POIFrom, djob->POITo, true, false);
				}
				else if(djob->POIFrom->type == POIType::RECYCLE)
					triggerEventGrabPuck(djob->POIFrom, djob->POITo, false, true);
				else
					triggerEventGrabPuck(djob->POIFrom, djob->POITo, false, false);

				currentJob->status = JobProgressStatus::DONE_AT_MACHINE_1;
			}

			break;
		case JobProgressStatus::DONE_AT_MACHINE_1:
			//TRIGGER EVENT FOR DRIVING TO MACHINE 2
			FileLog::log(log_Job,"[JobHandler] handleJob Status DONE_AT_MACHINE_1");
			if(djob->jobClass == JobClass::Recycle)
			{
				Job* rjob = (Job*)djob;

				if(rjob->accDir == POIAccessFrom::FRONT)
					triggerEventLeavePoiWithPuckRotatingOut(djob->POIFrom);
				else if(rjob->jobType == JobType::RECYCLE_PUCK_AT_T2_LEFT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T3_LEFT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T4_LEFT_PUCK)
					triggerEventLeavePoiWithPuck(djob->POIFrom,POIAccessFrom::LEFT,djob->POITo,POIAccessFrom::FRONT);
				else if(rjob->jobType == JobType::RECYCLE_PUCK_AT_T2_RIGHT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T3_RIGHT_PUCK || rjob->jobType == JobType::RECYCLE_PUCK_AT_T4_RIGHT_PUCK)
					triggerEventLeavePoiWithPuck(djob->POIFrom,POIAccessFrom::RIGHT,djob->POITo,POIAccessFrom::FRONT);
			}else
				triggerEventLeavePoiWithPuck(djob->POIFrom,POIAccessFrom::FRONT,djob->POITo,POIAccessFrom::FRONT);
			currentJob->status = JobProgressStatus::OUT_OF_MACHINE_1;
			break;
		case JobProgressStatus::OUT_OF_MACHINE_1:
			FileLog::log(log_Job,"[JobHandler] handleJob Status OUT_OF_MACHINE_1");

			if(djob->jobClass == JobClass::Explore){
				triggerEventDriveToPoi(djob->POITo,djob->POIFrom);
				if((djob->POITo->type == POIType::DELIVER) || (djob->POITo->type == POIType::INPUT)){
					currentJob->status = JobProgressStatus::FINISHED_JOB;
						break;
				}
				djob->status = JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_2;
				break;
			}
			triggerEventDriveToPoiWithPuck(djob->POITo,djob->POIFrom);
			djob->status = JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_2;
			break;
		case JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_2:
			FileLog::log(log_Job,"[JobHandler] handleJob Status DRIVEN_TO_TARGET_MACHINE_2");
			//TRIGGER EVENT FOR UNLOADING THE PUCK in front of machine
			if(djob->POITo->type == POIType::DELIVER)
				triggerEventDeliverPuckToGate(grid->getAccessNode(djob->POITo));
			else
				triggerEventDeliverPuckToPoi();
			currentJob->status = JobProgressStatus::FINISHED_JOB;
			break;
		case JobProgressStatus::FINISHED_JOB:
			//do nothing
			break;
		case JobProgressStatus::FINISHED_JOB_OUT_OF_ORDER_DURING_FINAL_PROCESSING:
			//do nothing
			break;
		default:
			break;
	}
}

void JobHandler::triggerEventContinueDeliveringPuck(){
	FileLog::log(log_Job,"[JobHandler] EvContinueDeliveringPuck");
	boost::shared_ptr<EvContinueDeliveringPuck> ev(new EvContinueDeliveringPuck());
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventDriveToPoiWithPuck(POI* poiTo,POI* poiFrom,POIAccessFrom::POIAccessFrom accDir){
	FileLog::log(log_Job,"[JobHandler] EvDriveToPOIWithPuck");
	boost::shared_ptr<EvDriveToPoi> ev(new EvDriveToPoi(poiTo,poiFrom,true,accDir));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventDriveToPoi(POI* poiTo,POI* poiFrom,POIAccessFrom::POIAccessFrom accDir){
	FileLog::log(log_Job,"[JobHandler] EvDriveToPOI (", POIAccessFrom::cPOIAccessFrom[accDir], ")");
	boost::shared_ptr<EvDriveToPoi> ev(new EvDriveToPoi(poiTo,poiFrom,false,accDir));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventLeavePoiBackwards(POI* poiFrom, POI* targetPOI, POIAccessFrom::POIAccessFrom targetAccDir){
	FileLog::log(log_Job,"[JobHandler] EvLeavePoiBackwards");
	boost::shared_ptr<EvLeavePoiBackwards> ev(new EvLeavePoiBackwards(poiFrom, targetPOI, targetAccDir));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventLeavePoiWithoutPuck(POI* poiFrom,POIAccessFrom::POIAccessFrom accDirFrom,LeaveDirection::LeaveDirection leaveDir,POI* poiTo,POIAccessFrom::POIAccessFrom accDirTo){
	FileLog::log(log_Job,"[JobHandler] EvLeavePoiWithoutPuck on ", LeaveDirection::cLeaveDirection[leaveDir]);
	boost::shared_ptr<EvLeavePoiWithoutPuck> ev(new EvLeavePoiWithoutPuck(poiFrom,accDirFrom,leaveDir,poiTo,accDirTo));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventLeavePoiWithPuck(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, POI* poiTo, POIAccessFrom::POIAccessFrom accDirTo){
	FileLog::log(log_Job,"[JobHandler] EvLeavePoiWithPuck");
	boost::shared_ptr<EvLeavePoiWithPuck> ev(new EvLeavePoiWithPuck(poiFrom,accDirFrom,poiTo,accDirTo));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventLeavePoiWithPuckRotatingOut(POI* poiFrom){
	FileLog::log(log_Job,"[JobHandler] EvLeavePoiWithPuckRotatingOut");
	boost::shared_ptr<EvLeavePoiWithPuckRotatingOut> ev(new EvLeavePoiWithPuckRotatingOut(poiFrom));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventGrabPuck(POI* poiFrom, POI* poiTo, bool gpmSidewards, bool gpmRec){
	FileLog::log(log_Job,"[JobHandler] EvGrabPuck");
	boost::shared_ptr<EvGrabPuckMachine> ev(new EvGrabPuckMachine(poiFrom, poiTo, gpmSidewards, gpmRec));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventGrabPuckSideOnFront(POI* poiFrom, LeaveDirection::LeaveDirection leaveDir){
	FileLog::log(log_Job,"[JobHandler] EvGrabPuckSideOnFront (", LeaveDirection::cLeaveDirection[leaveDir], ")");
	boost::shared_ptr<EvGrabPuckSideOnFront> ev(new EvGrabPuckSideOnFront(poiFrom, leaveDir));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventGrabPuckInputZone(POI* poiFrom, POI* poiTo, POIAccessFrom::POIAccessFrom accDirTo){
	FileLog::log(log_Job,"[JobHandler] EvGrabPuckInputZone");
	boost::shared_ptr<EvGrabPuckInputZone> ev(new EvGrabPuckInputZone(poiFrom, poiTo, accDirTo));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventDeliverPuckToPoi(){
	FileLog::log(log_Job,"[JobHandler] EvDeliverPuckToPoi");
	boost::shared_ptr<EvDeliverPuckToPoi> ev(new EvDeliverPuckToPoi());
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventDeliverPuckToGate(Node* accessNode){
	FileLog::log(log_Job,"[JobHandler] EvDeliverPuckToGate");
	boost::shared_ptr<EvDeliverPuckToGate> ev(new EvDeliverPuckToGate(accessNode));
	asyncStateMachine->queueEvent(ev);
}

void JobHandler::triggerEventNoJobFound(){
	FileLog::log(log_Job,"[JobHandler] EvNoJobFound");
	boost::shared_ptr<EvNoJobFound> ev(new EvNoJobFound());
	asyncStateMachine->queueEvent(ev);
}
