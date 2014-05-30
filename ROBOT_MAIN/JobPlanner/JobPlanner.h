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
class Job;

class JobPlanner {
private:
	SensorServer *sensorSrv;
	Grid *grid; //need this to convert odometry position into grid-Coordinates
	WorldModel *worldModel;

	Job *lastJob;
	POI *lastPOITo;

	int currentP1Orders;
	int currentP2Orders;
	int currentP3Orders;

	void checkOutOfOrder();
	void updateOrders();
	void updateTeamDistribution();
	void updateMachineTypes();

	//helper methods for JobPlanner
	bool checkAvailableJob(Job *job);
	bool checkLateOrder();
	bool checkAvailablePuck(int index, Puck::Puck p, POIAccessFrom::POIAccessFrom accDir=POIAccessFrom::FRONT);
	bool checkAvailablePuck(Puck::Puck p, POIAccessFrom::POIAccessFrom accDir=POIAccessFrom::FRONT);

	bool checkAvailablePOI(int index, POIType::POIType t, POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck);
	bool checkAvailablePOI(int index, POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck);
	bool checkAvailablePOI(POIType::POIType t, POIRequiredNext::POIRequiredNext req=POIRequiredNext::NON_EXISTING, POIStatus::POIStatus status = POIStatus::READY, Puck::Puck middlePuck = Puck::NON_EXISTING);

	bool defineDetailsOfJob(Job *job, int gridX, int gridY);

	bool addJob(Job *job, vector<Job*>& vJobAll, vector<Job*>& vJobAvailable, int gridX, int gridY);
	bool addExplorationJob(Job *job,vector<Job*>& vJobAll, vector<Job*>& vJobAvailable, int gridX, int gridY);

	vector<POI *> giveAllCandidatesPuck(Puck::Puck p, POIAccessFrom::POIAccessFrom accDir=POIAccessFrom::FRONT);
	vector<POI *> giveAllCandidatesPOI(POIType::POIType t, POIRequiredNext::POIRequiredNext req=POIRequiredNext::NON_EXISTING, POIStatus::POIStatus status = POIStatus::READY, Puck::Puck middlePuck = Puck::NON_EXISTING);
	vector<POI *> giveAllCandidatesPOI(POIRequiredNext::POIRequiredNext req=POIRequiredNext::NON_EXISTING, POIStatus::POIStatus status = POIStatus::READY, Puck::Puck middlePuck = Puck::NON_EXISTING);

	POI *giveNearestMachineIndex(vector<POI *> vPOIs, int gridX, int gridY, POIAccessFrom::POIAccessFrom accDir, int &finalAirTravelDistance);
	float calcDistance(int fromX, int fromY, int destX, int destY);
	bool isPOIRequired(POI *poi, POIRequiredNext::POIRequiredNext req);
	bool containsPOIType(std::vector<POI *> vCandidatesIndexesFrom, std::vector<POI *> &POISofType, POIType::POIType type);

public:
	JobPlanner(SensorServer *sensorSrv_);

	Job* giveNewJob(Job* lastJob);
	bool redefineTarget(Job *p);

	void printJobList(vector<Job *> vJob);
	void printJob(Job * job);
	virtual ~JobPlanner();
};

#endif /* JOBPLANNER_H_ */
