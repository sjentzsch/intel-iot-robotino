/*
 * JobHandler.cpp
 *
 *  Created on: Jun 11, 2011
 *      Author: root
 */

#include "JobHandler.h"

JobHandler::JobHandler(StateBehaviorController* stateBhvContrl,SensorServer *sensorSrv_):asyncStateMachine(stateBhvContrl->getAsyncStateMachine()),pathfinder(stateBhvContrl->getPathFinder()) {
	grid = new Grid();
	jobPlanner = new JobPlanner(sensorSrv_);
	nextTask_exec = NULL;
	currentJob = NULL;
	lastJob = NULL;
	lastJobInCaseOfRandomMovement = NULL;
	randomPOI = NULL;
	foundT1ForRPC = false;
	T5timingActive=false;
	leaveLeft = false;
}

JobHandler::~JobHandler() {
	if(grid != NULL)
		delete grid;
}
void JobHandler::startMachineTiming(){
	if(!T5timingActive){
		TimeStampT5_1 = ModelProvider::getInstance()->getGlobalGameTime();
		T5timingActive = true;
	}
	/*
	else if(!T4timingActive){
		TimeStampT4_1 = ModelProvider::getInstance()->getGlobalGameTime();
		T4timingActive = true;
	}
	else if(!T3timingActive){
		TimeStampT3_1 = ModelProvider::getInstance()->getGlobalGameTime();
		T3timingActive = true;
	}
	*/
}

void JobHandler::endMachineTiming(POIType::POIType type){
	TimeStampT5_2 = ModelProvider::getInstance()->getGlobalGameTime();
	WorldModelClientHandler::getInstance()->lock();
	if(type == POIType::T5){
		ModelProvider::getInstance()->getWorldModel()->T5prodTime = TimeStampT5_2.sec - TimeStampT5_1.sec;
		FileLog::log(log_Job,"T5 processing time is ",std::to_string(ModelProvider::getInstance()->getWorldModel()->T5prodTime),"s");
	}
/*	else if(type == POIType::T4){
		ModelProvider::getInstance()->getWorldModel()->T4prodTime = TimeStampT4_2.sec - TimeStampT4_1.sec;
	}
	else if(type == POIType::T3){
		ModelProvider::getInstance()->getWorldModel()->T3prodTime = TimeStampT3_2.sec - TimeStampT3_1.sec;
	}
*/
	WorldModelClientHandler::getInstance()->unlock();
}
//this function does not need an extra thread, since updating the lamp status should and is never done while anything critically happens
//if considered differently, events based on completion have to be defined as well
void JobHandler::updatePoiType(CameraLightState::CameraLightState lamp){
	WorldModelClientHandler::getInstance()->lock();
	Job* dJob = currentJob;
	updateProdMachineType(dJob,lamp);
	WorldModelClientHandler::getInstance()->unlock();
}


void JobHandler::updateProdMachineType(Job* dJob,CameraLightState::CameraLightState lamp){
	Time globalGameTime = ModelProvider::getInstance()->getGlobalGameTime();
	POI* poi = dJob->POITo;
	FileLog::log(log_Job,"[JobHandler] CameraLightState: ",std::to_string(lamp));
	FileLog::log(log_Job,"[JobHandler] CarriedGood: ", std::to_string(dJob->carriedGood));
	poi->status = POIStatus::READY;
	//if(dJob->jobClass == JobClass::Explore)
	//TODO : move all this puck-logic to appropriate place (where the pucks are actually placed in the next job?)
	switch(lamp){
		case CameraLightState::YELLOW:
			switch(dJob->carriedGood)
			{
				case Puck::S0:
					// I have an S0 and now the machine is yellow, thus ...
					if(poi->type == POIType::T2){
						poi->right = Puck::NON_EXISTING;
					}
					poi->left = Puck::CONSUMED_IN_USE;
					break;
				case Puck::S1:
					if(poi->type == POIType::T2){
						poi->left = Puck::NON_EXISTING;
					}
					poi->right = Puck::CONSUMED_IN_USE;
					break;
				case Puck::S2:
					poi->status = POIStatus::OFFLINE;
					break;
				default:
					cerr << "JobHandler::updateProdMachineType - YELLOW update error" << endl;
					break;
			}
			break;
		case CameraLightState::GREEN:
			switch(dJob->carriedGood){
				case Puck::S0:
					if(poi->type == POIType::T1)
					{
						poi->middle = Puck::S1;
					}
					else if(poi->type == POIType::T5){
						poi->middle = Puck::P3;
					}
					else if(poi->type == POIType::T2){
						poi->middle = Puck::S2;
					}
					break;
				case Puck::S1:
					poi->middle = Puck::S2;
					break;
				case Puck::S2:
					if(poi->type == POIType::T3)
						poi->middle = Puck::P1;
					else if(poi->type == POIType::T4)
						poi->middle = Puck::P2;
					break;
				//in case we want to deliver a P from T3,4,5 to gate, reset gate status in case it was set to processing
				case Puck::P1:
					dJob->POIFrom->status = POIStatus::READY;
					removePuckFromProductionMachine(dJob->POIFrom);
					break;
				case Puck::P2:
					dJob->POIFrom->status = POIStatus::READY;
					removePuckFromProductionMachine(dJob->POIFrom);
					break;
				case Puck::P3:
					dJob->POIFrom->status = POIStatus::READY;
					removePuckFromProductionMachine(dJob->POIFrom);
					break;

				case Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2:
					poi->middle=Puck::S0;
					break;
				case Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3:
					poi->middle=Puck::S0;
					break;
				case Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4:
					poi->middle=Puck::S0;
					break;
				default:
					cerr << "JobHandler::updateProdMachineType - GREEN update error" << endl;
					break;
			}
			break;
		case CameraLightState::YELLOW_FLASH:
			leaveLeft = true;
			poi->status = POIStatus::OFFLINE;
			poi->middle = Puck::NON_EXISTING;
			cerr << "Error! Yellow Flash detected!"<< endl;
			break;
		case CameraLightState::YELLOW_GREEN:
			//in case we bring an S2 to a T3 or T4, instead of waiting for a minute, we set the machine busy and do something else
			if(poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::CONSUMED_IN_USE && poi->type == POIType::T3)
			{
				poi->status = POIStatus::PROCESSING;
				//reset it after 30s, the maximum time it takes to produce P is 60s
				poi->timestamp.sec = globalGameTime.sec + 30;
				poi->timestamp.nsec = globalGameTime.nsec;
			}
			else if(poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::CONSUMED_IN_USE && poi->type == POIType::T4){
				poi->status = POIStatus::PROCESSING;
				//reset it after 30s, the maximum time it takes to produce P is 60s
				poi->timestamp.sec = globalGameTime.sec + 30;
				poi->timestamp.nsec = globalGameTime.nsec;
			}
			else if(poi->type == POIType::T5)
			{
				poi->status = POIStatus::PROCESSING;
				poi->timestamp.sec = globalGameTime.sec + ModelProvider::getInstance()->getWorldModel()->T5prodTime - 10; //TODO: avg. drivetopoifrom time
				poi->timestamp.nsec = globalGameTime.nsec;
			}
			break;
		case CameraLightState::RED:
			if(poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::CONSUMED_IN_USE)
			{
				//technically its not processing, but it will after it returns online
				poi->status = POIStatus::PROCESSING;
				poi->timestamp.sec = globalGameTime.sec + 50;
				poi->timestamp.nsec = globalGameTime.nsec;

			}
			break;
		//should never reach any of these 3 as these combinations are exploration-only, which is handled on its own
		case CameraLightState::RED_YELLOW:
			leaveLeft=true;
			break;
		case CameraLightState::RED_GREEN:
			leaveLeft=true;
			break;
		case CameraLightState::RED_YELLOW_GREEN:
			leaveLeft=true;
			break;
		case CameraLightState::OFFLINE:
			poi->status = POIStatus::OFFLINE;
			poi->timestamp.sec = globalGameTime.sec + 50;
			poi->timestamp.nsec = globalGameTime.nsec;
			leaveLeft=true;
			break;
		default:
			cerr << "JobHandler::updateProdMachineType - CameraLightState error" << endl;
			break;
	}
}

void JobHandler::updateRecMachineType(Job* rJob, CameraLightState::CameraLightState lamp){
	POI* poi = rJob->POITo;
	poi->status = POIStatus::READY;
	FileLog::log(log_Job,"[JobHandler] updateRecMachineType with lampstate: ",std::to_string(lamp));
	switch(lamp)
	{
		case CameraLightState::GREEN:
			poi->middle = Puck::S0;
			break;
		default:
			poi->status = POIStatus::OFFLINE;
			break;
	}
}

void JobHandler::handleOutOfOrder_impl(){
	Time globalGameTime = ModelProvider::getInstance()->getGlobalGameTime();
	FileLog::log(log_Job,"[JobHandler] handleOutOfOrder_impl called");
	WorldModelClientHandler::getInstance()->lock();
	if(currentJob->jobClass == JobClass::Process){
		Job* djob = currentJob;
		POI* oldPoiTo = djob->POITo;
		oldPoiTo->occupied =0;
		oldPoiTo->status = POIStatus::OUT_OF_ORD_t;
		oldPoiTo->timestamp.sec = globalGameTime.sec + 30;
		oldPoiTo->timestamp.nsec = globalGameTime.nsec;
		if(jobPlanner->redefineTarget(djob)){
			triggerEventLeavePoiWithPuck(oldPoiTo,POIAccessFrom::FRONT,djob->POITo,POIAccessFrom::FRONT);
			currentJob->status = JobProgressStatus::OUT_OF_MACHINE_1;
		}else{
			triggerEventContinueDeliveringPuck();
		}
	}
	else if(currentJob->jobClass == JobClass::Recycle){
		Job* rjob = ((Job*)currentJob);
		POI* oldPoiTo = rjob->POITo;
		oldPoiTo->occupied = 0;
		oldPoiTo->status = POIStatus::OUT_OF_ORD_t;
		oldPoiTo->timestamp.sec = globalGameTime.sec + 30;
		oldPoiTo->timestamp.nsec = globalGameTime.nsec;
		if(jobPlanner->redefineTarget(rjob)){
			triggerEventLeavePoiWithPuck(oldPoiTo,POIAccessFrom::FRONT,rjob->POITo,POIAccessFrom::FRONT);
			currentJob->status = JobProgressStatus::STARTING;
		}else{
			triggerEventContinueDeliveringPuck();
		}
	}
	WorldModelClientHandler::getInstance()->unlock();

}

void JobHandler::handleOutOfOrder(){
	nextTask_exec = new boost::thread(&JobHandler::handleOutOfOrder_impl,this);
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


void JobHandler::nextTask_impl(){
	//TODO: refactor this code!

	if(currentJob!=NULL&&currentJob->status==JobProgressStatus::FINISHED_JOB){

		// TODO: deletion of jobs good here?
		if(lastJob != NULL)
			delete lastJob;

		lastJob = currentJob;
		currentJob = NULL;
	}

	randomPOI = NULL;

	bool leaveLoop = false;

	while(randomPOI==NULL&&currentJob==NULL&&!leaveLoop){

		while(ModelProvider::getInstance()->gameStateIsPaused()){
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		}

		WorldModelClientHandler::getInstance()->lock();

		currentJob = jobPlanner->giveNewJob(lastJob);

		//TODO: KILL IT WITH FIRE!
		if(currentJob == NULL){
			if(ModelProvider::getInstance()->getLatestGameState().phase==Phase::PRODUCTION){
			// handle the case that we found no job

				//boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
				// if we had a last job, leave the machine first; else move a bit around
				if(lastJob != NULL)
				{
					if(lastJob->jobType == JobType::Explore)
					{
						WorldModelClientHandler::getInstance()->unlock();
						if(currentJob==NULL&&!leaveLoop){
							boost::this_thread::sleep(boost::posix_time::seconds(2));
						}
						continue;
					}

					if(lastJob->jobClass == JobClass::Deliver && lastJob->POITo->type == POIType::DELIVER)
					{
						randomPOI = pathfinder->moveABit();
						if(randomPOI != NULL)
							triggerEventDriveToPoi(randomPOI,NULL);
					}

					else
					{
						getOutOfMachine(lastJob);
						FileLog::log(log_Job,"no job, getting out of machine");
						lastJobInCaseOfRandomMovement = lastJob; // save lastjob for freeing PoiTo once we got out of the machine
						FileLog::log(log_Job,"lastJobInCaseOfRandomMovement = ",std::to_string(lastJobInCaseOfRandomMovement->jobType));
						lastJob = NULL;
						leaveLoop = true;
					}
				}
				else
				{
					FileLog::log(log_Job,"still no job, releasing old poito");
					FileLog::log(log_Job,"lastJobInCaseOfRandomMovement");

					// we got out of the machine and still have no job, starting random movement and releasing old PoiTo
					if(lastJobInCaseOfRandomMovement!=NULL && lastJobInCaseOfRandomMovement->POITo->occupied == ModelProvider::getInstance()->getID())
						lastJobInCaseOfRandomMovement->POITo->occupied = 0;

					lastJobInCaseOfRandomMovement = NULL;
					randomPOI = pathfinder->moveABit(); //TODO: do something more clever than random movement
					if(randomPOI != NULL)
						triggerEventDriveToPoi(randomPOI,NULL);
				}
			}
		}
		// special case for getting job directly after getting out of machine because of no job
		if(currentJob!= NULL)
		{
			if(lastJobInCaseOfRandomMovement!=NULL && lastJobInCaseOfRandomMovement->POITo->index != currentJob->POIFrom->index && lastJobInCaseOfRandomMovement->POITo->index != currentJob->POITo->index && lastJobInCaseOfRandomMovement->POITo->occupied == ModelProvider::getInstance()->getID())
				lastJobInCaseOfRandomMovement->POITo->occupied = 0;


			lastJobInCaseOfRandomMovement = NULL;
		}

		WorldModelClientHandler::getInstance()->unlock();

		if(currentJob==NULL&&!leaveLoop){
			boost::this_thread::sleep(boost::posix_time::seconds(2));
		}
	}

	if(currentJob == NULL){
		//trigger no job found (handle this here for the moment, TODO: prob. Statemachine should handle this (Pathfinder as well));
		//triggerEventNoJobFound();
	}
	else{
		WorldModelClientHandler::getInstance()->lock();
		handleJob(currentJob);
		WorldModelClientHandler::getInstance()->unlock();
	}
}

JobProgressStatus::JobProgressStatus JobHandler::prepareJob(Job* currentJob){

	if(lastJob!=NULL){
		//only if the last Job was a Job we have to consider the case that POIFrom = POITo
		// and handle different types of leaving the poi

		//post-exploration in case they are standing at the input zone already
		if(lastJob->POITo->type==POIType::INPUT && lastJob->jobClass == JobClass::Explore && ModelProvider::getInstance()->getLatestGameState().phase == Phase::PRODUCTION){
			triggerEventGrabPuckInputZone(currentJob->POIFrom,currentJob->POITo, POIAccessFrom::FRONT);
			return JobProgressStatus::OUT_OF_MACHINE_1;
		}
		else if(lastJob->jobClass == JobClass::Explore && ModelProvider::getInstance()->getLatestGameState().phase == Phase::PRODUCTION){
			triggerEventLeavePoiBackwards(lastJob->POITo, currentJob->POIFrom, POIAccessFrom::FRONT);
			return JobProgressStatus::STARTED;
		}
		Job* dLastJob = lastJob;
		return leaveProductionMachine(currentJob,dLastJob);
	}

	else{
		if(currentJob->jobClass == JobClass::Recycle)
			triggerEventDriveToPoi(currentJob->POIFrom,NULL, currentJob->accDir);
		else if(currentJob->jobClass == JobClass::Explore){
			if(lastJob==NULL && ((currentJob->POITo->type == POIType::INPUT))){
				triggerEventDriveToPoi(currentJob->POITo,NULL);
				return JobProgressStatus::FINISHED_JOB;
			}
			triggerEventDriveToPoi(currentJob->POITo,NULL);
			return JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_2;
		}
		else
			triggerEventDriveToPoi(currentJob->POIFrom,NULL);

		return JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_1;
	}
}

JobProgressStatus::JobProgressStatus JobHandler::leaveProductionMachine(Job* currentJob,Job* lastJob){
	//at the delivery gate it is assumed that we left the POI already
	if(lastJob->POITo->type== POIType::DELIVER){
		if(currentJob->jobClass == JobClass::Recycle)
			triggerEventDriveToPoi(currentJob->POIFrom,lastJob->POITo, currentJob->accDir);
		else
			triggerEventDriveToPoi(currentJob->POIFrom,lastJob->POITo);
		return JobProgressStatus::DRIVEN_TO_TARGET_MACHINE_1;
	}
	if(currentJob->jobType==JobType::Explore){
		triggerEventLeavePoiBackwards(currentJob->POIFrom,currentJob->POITo,currentJob->accDir);
		return JobProgressStatus::OUT_OF_MACHINE_1;
	}

	if(lastJob->POITo->index == currentJob->POIFrom->index && !(lastJob->jobClass == JobClass::Explore)){
		//in this case it is assumed that we have to need the current puck for the next task as well (TODO: check assumption)
		//TRIGGER EVENT TO GET OUT OF POI with PUCK

		removePuckFromProductionMachine(lastJob->POITo);

		triggerEventLeavePoiWithPuck(currentJob->POIFrom,POIAccessFrom::FRONT,currentJob->POITo,POIAccessFrom::FRONT);

		return JobProgressStatus::OUT_OF_MACHINE_1;
	}else{
		if(currentJob->jobClass == JobClass::Recycle)
			getOutOfMachine(lastJob, currentJob->POIFrom, currentJob->accDir);
		else
			getOutOfMachine(lastJob, currentJob->POIFrom, POIAccessFrom::FRONT);
		return  JobProgressStatus::STARTED;
	}
}

void JobHandler::getOutOfMachine(Job* dJob, POI* targetPOI, POIAccessFrom::POIAccessFrom targetAccDir){
	POI* toPoi;
	Puck::Puck carriedGood;

	toPoi = dJob->POITo;
	carriedGood = dJob->carriedGood;
	//in case we had yellow flash or other unknown error, leave puck left instead of middle
	if(leaveLeft){
		triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::LEFT, targetPOI, targetAccDir);
		leaveLeft = false;
		return;
	}
	switch(toPoi->type){
		case POIType::T1:
			triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		case POIType::T2:
			if(carriedGood == Puck::S0){
				if(toPoi->right == Puck::CONSUMED_IN_USE)
					triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
				else
					triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::LEFT, targetPOI, targetAccDir);
			}
			else if(carriedGood == Puck::S1)
			{
				if(toPoi->left == Puck::CONSUMED_IN_USE)
					triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
				else
					triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::RIGHT, targetPOI, targetAccDir);
			}
			else
				triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		case POIType::T3:
			if(carriedGood == Puck::S2){
				//Trigger event to leave poi backwards
				triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			}else if(carriedGood == Puck::S1){
				//Trigger event to push puck to the right
				triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::RIGHT, targetPOI, targetAccDir);
			}else if(carriedGood == Puck::S0){
				//Trigger event to push puck to the left
				triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::LEFT, targetPOI, targetAccDir);
			}
			else
				triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		case POIType::T4:
			if(carriedGood == Puck::S2){
				//Trigger event to leave poi backwards
				triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			}else if(carriedGood == Puck::S1){
				//Trigger event to push puck to the right
				triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::RIGHT, targetPOI, targetAccDir);
			}else if(carriedGood == Puck::S0){
				//Trigger event to push puck to the left
				triggerEventLeavePoiWithoutPuck(toPoi, POIAccessFrom::FRONT, LeaveDirection::LEFT, targetPOI, targetAccDir);
			}
			else
				triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		case POIType::T5:
			triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		case POIType::RECYCLE:
			triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		case POIType::UNKNOWN:
			triggerEventLeavePoiBackwards(toPoi, targetPOI, targetAccDir);
			break;
		default:
			FileLog::log(log_Job,"getOutOfMachine() crazy switch type");
			break;
		}
}

void JobHandler::removePuckFromProductionMachine(POI* poi){
	FileLog::log(log_Job,"removing puck ",std::to_string(poi->middle) ," from worldmodel at PoiIndex ",std::to_string(poi->index));
	poi->middle = Puck::NON_EXISTING;

	//poi->left = Puck::NON_EXISTING;
	//poi->right = Puck::NON_EXISTING;

	if(poi->type==POIType::T2)
	{
		if(poi->left!=Puck::NON_EXISTING)
			poi->left = Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2;
		if(poi->right!=Puck::NON_EXISTING)
			poi->right = Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2;
	}
	else if(poi->type==POIType::T3)
	{
		if(poi->left!=Puck::NON_EXISTING)
			poi->left = Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3;
		if(poi->right!=Puck::NON_EXISTING)
			poi->right = Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3;
	}
	else if(poi->type==POIType::T4)
	{
		if(poi->left!=Puck::NON_EXISTING)
			poi->left = Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4;
		if(poi->right!=Puck::NON_EXISTING)
			poi->right = Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4;
	}
}

void JobHandler::handleJob(Job* djob){
	switch(djob->status){
		case JobProgressStatus::STARTING:
			FileLog::log(log_Job,"[JobHandler] handleJob Status STARTING");
			currentJob->status = prepareJob(djob);
			break;
		case JobProgressStatus::STARTED:
			FileLog::log(log_Job,"[JobHandler] handleJob Status STARTED");
			// we moved out of the old PoiTo, release it
			if(lastJob!=NULL && lastJob->POITo->index != djob->POIFrom->index && ((Job*)lastJob)->POITo->index != djob->POITo->index && ((Job*)lastJob)->POITo->occupied == ModelProvider::getInstance()->getID())
				lastJob->POITo->occupied = 0;

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
			if(lastJob!=NULL && lastJob->POITo->index != djob->POIFrom->index && lastJob->POITo->index != djob->POITo->index && lastJob->POITo->occupied == ModelProvider::getInstance()->getID())
				lastJob->POITo->occupied = 0;

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
					removePuckFromProductionMachine(djob->POIFrom);

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
			if(djob->POIFrom!=NULL)
				if(!(djob->POIFrom->type == POIType::INPUT) && djob->POIFrom->occupied == ModelProvider::getInstance()->getID())
					djob->POIFrom->occupied = 0;

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
			if(currentJob->POIFrom!=NULL && currentJob->POIFrom->occupied == ModelProvider::getInstance()->getID())
				currentJob->POIFrom->occupied = 0;
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
