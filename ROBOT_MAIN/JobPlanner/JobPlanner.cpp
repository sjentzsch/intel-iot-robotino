/*
 * JobPlanner.cpp
 *
 *  Created on: 10.06.2011
 *      Author: root
 */

#include "JobPlanner.h"
#include "Jobs.h"
#include "BaseParameterProvider.h"

JobPlanner::JobPlanner(SensorServer *sensorSrv_):sensorSrv(sensorSrv_){
	grid = new Grid();
	worldModel = ModelProvider::getInstance()->getWorldModel();
	lastJob = NULL;
	lastPOITo = NULL;
}

JobPlanner::~JobPlanner() {}

Job* JobPlanner::giveNewJob(Job* lastJob)
{
	//Extract current grid node of odometry position
	vec2D pose;
	sensorSrv->getOdometry(pose);
	Node *node = grid->getNodeByCoord(pose);
	int gridX = node->getX();
	int gridY = node->getY();

	// set lastJob and lastPOITo
	this->lastJob = lastJob;
	if(lastJob != NULL)
		lastPOITo = lastJob->POITo;
	else
		lastPOITo = NULL;

	vector<Job*> vJobAll;
	vector<Job*> vJobAvailable;

	if(ModelProvider::getInstance()->getLatestGameState().phase==Phase::PRODUCTION && worldModel->updatedMachineTypes && worldModel->updatedTeamDistribution)
	{
		addJob(new Job(JobClass::Recycle, JobType::RECYCLE_PUCK_AT_T3_LEFT_PUCK, 1000, JobProgressStatus::STARTING, Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3, POIType::RECYCLE, POIRequiredNext::CONSUMED, POIAccessFrom::LEFT), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Recycle, JobType::RECYCLE_PUCK_AT_T3_RIGHT_PUCK, 1000, JobProgressStatus::STARTING, Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3, POIType::RECYCLE, POIRequiredNext::CONSUMED, POIAccessFrom::RIGHT), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Recycle, JobType::RECYCLE_PUCK_AT_T2_LEFT_PUCK, 1000, JobProgressStatus::STARTING, Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2, POIType::RECYCLE, POIRequiredNext::CONSUMED, POIAccessFrom::LEFT), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Recycle, JobType::RECYCLE_PUCK_AT_T2_RIGHT_PUCK, 1000, JobProgressStatus::STARTING, Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2, POIType::RECYCLE, POIRequiredNext::CONSUMED, POIAccessFrom::RIGHT), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Recycle, JobType::RECYCLE_PUCK_AT_T4_LEFT_PUCK, 1000, JobProgressStatus::STARTING, Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4, POIType::RECYCLE, POIRequiredNext::CONSUMED, POIAccessFrom::LEFT), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Recycle, JobType::RECYCLE_PUCK_AT_T4_RIGHT_PUCK, 1000, JobProgressStatus::STARTING, Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4, POIType::RECYCLE, POIRequiredNext::CONSUMED, POIAccessFrom::RIGHT), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S1_TO_T3, 900, JobProgressStatus::STARTING, Puck::S1, POIType::T3, POIRequiredNext::S1), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S1_TO_T4, 900, JobProgressStatus::STARTING, Puck::S1, POIType::T4, POIRequiredNext::S1), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S0_TO_T3, 700, JobProgressStatus::STARTING, Puck::S0, POIType::T3, POIRequiredNext::S0), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S0_TO_T4, 700, JobProgressStatus::STARTING, Puck::S0, POIType::T4, POIRequiredNext::S0), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S1_TO_T2, 500, JobProgressStatus::STARTING, Puck::S1, POIType::T2, POIRequiredNext::S1), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S0_TO_T1, 400, JobProgressStatus::STARTING, Puck::S0, POIType::T1, POIRequiredNext::S0), vJobAll, vJobAvailable, gridX, gridY);
		addJob(new Job(JobClass::Process, JobType::S0_TO_T2, 400, JobProgressStatus::STARTING, Puck::S0, POIType::T2, POIRequiredNext::S0), vJobAll, vJobAvailable, gridX, gridY);
	}
	printJobList(vJobAvailable);

	// choose the best job (i.e. highest priority) and store it in bestJob
	Job *bestJob = NULL;
	int bestPriority = numeric_limits<int>::min();
	for(unsigned int i=0; i<vJobAvailable.size(); i++)
	{
		if(vJobAvailable.at(i)->priority > bestPriority)
		{
			bestPriority = vJobAvailable.at(i)->priority;
			bestJob = vJobAvailable.at(i);
		}
	}

	// now copy the bestJob into resJob and set the according POIs to occupied
	Job *resJob = NULL;
	if(bestJob != NULL)
	{
		resJob = new Job(bestJob->jobClass, bestJob->jobType, bestJob->priority, bestJob->status, bestJob->carriedGood, bestJob->targetPOIType, bestJob->targetRequiredNext, bestJob->accDir);
		resJob->POIFrom = bestJob->POIFrom;
		resJob->POITo = bestJob->POITo;
	}

	// clean up
	for(unsigned int i=0; i<vJobAll.size(); i++)
		delete vJobAll.at(i);
	vJobAll.clear();
	vJobAvailable.clear();


	if(resJob == NULL){
		FileLog::log(log_Job,"[JobPlanner]: No job found");
		FileLog::log_NOTICE("[JobPlanner]: No job found");
	}
	else{
		FileLog::log(log_Job,"[JobPlanner]: New job ", JobType::cJobType[resJob->jobType], ", POI ", FileLog::integer(resJob->POIFrom->index) , " to POI ", FileLog::integer(resJob->POITo->index), ", Priority ", FileLog::integer(resJob->priority));
		FileLog::log_NOTICE("[JobPlanner]: New job ", JobType::cJobType[resJob->jobType], ", POI ", FileLog::integer(resJob->POIFrom->index) , " to POI ", FileLog::integer(resJob->POITo->index), ", Priority ", FileLog::integer(resJob->priority));
	}
	return resJob;
}

bool JobPlanner::addJob(Job *job, vector<Job*>& vJobAll, vector<Job*>& vJobAvailable, int gridX, int gridY)
{
	vJobAll.push_back(job);
	vJobAvailable.push_back(job);

	return true;
}

//Input: General job
//Output: detailed job (means e.g. the information to which exact machine the puck should be brought to will be examined)
//Also current location of the robot is considered here, for shortest possible paths
bool JobPlanner::defineDetailsOfJob(Job *job, int gridX, int gridY)
{
	// TODO

	return true;
}

void JobPlanner::printJobList(vector<Job *> vJob)
{
	FileLog::log(log_Job,"######## Available Job List ########");
	for(unsigned int i=0; i<vJob.size(); i++)
	{
		FileLog::log(log_Job,JobType::cJobType[(vJob.at(i))->jobType], ", Priority ", std::to_string(vJob.at(i)->priority));
	}

	FileLog::log(log_Job,"####################################");

	FileLog::log_NOTICE("######## Available Job List ########");
	for(unsigned int i=0; i<vJob.size(); i++)
	{
		FileLog::log_NOTICE(JobType::cJobType[(vJob.at(i))->jobType], ", Priority ", std::to_string(vJob.at(i)->priority));
	}

	FileLog::log_NOTICE("####################################");
}

void JobPlanner::printJob(Job *p)
{
	FileLog::log(log_Job,"Job:\t ------- Job - ", JobType::cJobType[p->jobType]);
	FileLog::log(log_Job,"Priority:\t ", std::to_string(p->priority));
	FileLog::log(log_Job,"From:\t ", std::to_string(p->POIFrom->index),  "\t Type: ", POIType::cPOIType[p->POIFrom->type]);
	FileLog::log(log_Job,"accDir:\t ", POIAccessFrom::cPOIAccessFrom[p->accDir]);
	FileLog::log(log_Job,"To:\t ", std::to_string(p->POITo->index),  "\t Type: ", POIType::cPOIType[p->POITo->type]);

	FileLog::log_NOTICE("Job:\t ------- Job - ", JobType::cJobType[p->jobType]);
	FileLog::log_NOTICE("Priority:\t ", std::to_string(p->priority));
	FileLog::log_NOTICE("From:\t ", std::to_string(p->POIFrom->index),  "\t Type: ", POIType::cPOIType[p->POIFrom->type]);
	FileLog::log_NOTICE("accDir:\t ", POIAccessFrom::cPOIAccessFrom[p->accDir]);
	FileLog::log_NOTICE("To:\t ", std::to_string(p->POITo->index),  "\t Type: ", POIType::cPOIType[p->POITo->type]);
}
