/*
 * JobPlanner.h
 *
 *  Created on: 10.06.2011
 *      Author: root
 */

#ifndef JOBPLANNER_H_
#define JOBPLANNER_H_
#include <boost/thread.hpp>
#include <limits>
#include "config.h"
#include "../StateMachineEvents.h"
#include "../StateBehaviorController.h"
#include "../pathfinder/Grid.h"
#include "JobPlannerEnums.h"
#include "Jobs.h"

class Job;
class SensorServer;

class JobPlanner {
private:
	SensorServer *sensorSrv;
	Grid *grid; //need this to convert odometry position into grid-Coordinates
	WorldModel *worldModel;

	Job *lastJob;
	POI *lastPOITo;

	bool defineDetailsOfJob(Job *job, int gridX, int gridY);

	bool addJob(Job *job, vector<Job*>& vJobAll, vector<Job*>& vJobAvailable, int gridX, int gridY);

public:
	JobPlanner(SensorServer *sensorSrv_);

	Job* giveNewJob(Job* lastJob);

	void printJobList(vector<Job *> vJob);
	void printJob(Job * job);
	virtual ~JobPlanner();
};

#endif /* JOBPLANNER_H_ */
