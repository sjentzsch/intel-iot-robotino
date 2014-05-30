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
	currentP1Orders = 0;
	currentP2Orders = 0;
	currentP3Orders = 0;
}

JobPlanner::~JobPlanner() {}

void JobPlanner::checkOutOfOrder()
{
	Time globalGameTime = ModelProvider::getInstance()->getGlobalGameTime();

	for(int i=0;i<NUMBER_OF_POIS;i++)
	{
		if(globalGameTime.isNewerThan(worldModel->poi[i].timestamp) && worldModel->poi[i].status == POIStatus::OUT_OF_ORD_t){
			worldModel->poi[i].status = POIStatus::READY;
		}
		//check for T3 processing, set good to finished after the processing time has finished
		if(globalGameTime.isNewerThan(worldModel->poi[i].timestamp) && (worldModel->poi[i].type==POIType::T3 || worldModel->poi[i].type==POIType::T4 || worldModel->poi[i].type==POIType::T5) && worldModel->poi[i].status == POIStatus::PROCESSING)
		{
			if(worldModel->poi[i].type==POIType::T3)
				worldModel->poi[i].middle = Puck::P1;
			else if(worldModel->poi[i].type==POIType::T4)
				worldModel->poi[i].middle = Puck::P2;
			else if(worldModel->poi[i].type==POIType::T5)
				worldModel->poi[i].middle = Puck::P3;

			worldModel->poi[i].timestamp.sec = ULLONG_MAX;
			worldModel->poi[i].timestamp.nsec = ULLONG_MAX;
		}
		if(globalGameTime.isNewerThan(worldModel->poi[i].timestamp) && (worldModel->poi[i].status == POIStatus::OFFLINE)){
			worldModel->poi[i].status = POIStatus::READY;
			worldModel->poi[i].timestamp.sec = ULLONG_MAX;
			worldModel->poi[i].timestamp.nsec = ULLONG_MAX;
		}
	}
}

void JobPlanner::updateTeamDistribution(){

	FileLog::log(log_Job,"[JobPlanner]: updating Team Distribution");

	//try to fetch information from machine Info
	if(ModelProvider::getInstance()->getGameData()->machineInfo.machines.size() > 0){
		//only the 24 unknown machines will be sent from the refbox, and each team has to set the "mine" flag for each machine (@TIM: 2. April)
		for(unsigned int i = 0; i < ModelProvider::getInstance()->getGameData()->machineInfo.machines.size(); i++){

			for(unsigned int poiIdx=0; poiIdx<NUMBER_OF_POIS; poiIdx++)
			{
				string poiTypeStr("M"+std::to_string(worldModel->poi[poiIdx].index));
				if(poiTypeStr.compare(ModelProvider::getInstance()->getGameData()->machineInfo.machines.at(i).name) == 0)
				{
					if(ModelProvider::getInstance()->getGameData()->machineInfo.machines.at(i).color == TeamColor::TeamColor(BaseParameterProvider::getInstance()->getParams()->team_number-1)){
						worldModel->poi[poiIdx].mine = true;
					} else {
						worldModel->poi[poiIdx].mine = false;
					}
				}

			}
		}
		//try to fetch information from exploration Info
	} else if (ModelProvider::getInstance()->getGameData()->explorationInfo.machines.size() > 0){
		//only the 24 unknown machines will be sent from the refbox, and each team has to set the "mine" flag for each machine (@TIM: 2. April)
		for(unsigned int i = 0; i < ModelProvider::getInstance()->getGameData()->explorationInfo.machines.size(); i++){

			for(unsigned int poiIdx=0; poiIdx<NUMBER_OF_POIS; poiIdx++)
			{
				string poiTypeStr("M"+std::to_string(worldModel->poi[poiIdx].index));
				if(poiTypeStr.compare(ModelProvider::getInstance()->getGameData()->explorationInfo.machines.at(i).name) == 0)
				{
					if(ModelProvider::getInstance()->getGameData()->explorationInfo.machines.at(i).color == TeamColor::TeamColor(BaseParameterProvider::getInstance()->getParams()->team_number-1)){
						worldModel->poi[poiIdx].mine = true;
					} else {
						worldModel->poi[poiIdx].mine = false;
					}
				}

			}
		}
	} else {
		cout << "Team distribution cannot be set! No exploration signal or machine info received yet!" << endl;
		return;
	}

	if(BaseParameterProvider::getInstance()->getParams()->team_number == 1){
		worldModel->inputyLeft = 3360;
		worldModel->inputyRight = 4480;
		worldModel->poi[12].mine = true;
		worldModel->poi[13].mine = true;
		worldModel->poi[14].mine = true;
		worldModel->poi[15].mine = true;
	}
	else{
		worldModel->inputyLeft = 6720;
		worldModel->inputyRight = 7840;
		worldModel->poi[28].mine = true;
		worldModel->poi[29].mine = true;
        worldModel->poi[30].mine = true;
        worldModel->poi[31].mine = true;
	}

	for(int i=0; i<NUMBER_OF_POIS; i++){
		if(!worldModel->poi[i].mine){
			worldModel->poi[i].status = POIStatus::OFFLINE;
			worldModel->poi[i].timestamp.sec = ULLONG_MAX;
			worldModel->poi[i].timestamp.nsec = ULLONG_MAX;
			worldModel->poi[i].occupied = 4;
		}
	}

	worldModel->updatedTeamDistribution = true;
}
void JobPlanner::updateMachineTypes()
{
	FileLog::log(log_Job,"[JobPlanner]: updating MachineTypes");
	MachineInfo machineInfo = ModelProvider::getInstance()->getLatestMachineInfo();
	if(machineInfo.machines.size() > 0)
	{
		for(unsigned int i=0; i<machineInfo.machines.size(); i++)
		{
			for(unsigned int u=0; u<NUMBER_OF_POIS; u++)
			{
				string poiTypeStr("M"+std::to_string(worldModel->poi[u].index));
				if(poiTypeStr.compare(machineInfo.machines.at(i).name) == 0)
				{
					if(machineInfo.machines.at(i).type.compare("T1")==0){
						worldModel->poi[u].type = POIType::T1;
					}
					else if(machineInfo.machines.at(i).type.compare("T2") == 0)
						worldModel->poi[u].type = POIType::T2;
					else if(machineInfo.machines.at(i).type.compare("T3") == 0)
						worldModel->poi[u].type = POIType::T3;
					else if(machineInfo.machines.at(i).type.compare("T4") == 0)
						worldModel->poi[u].type = POIType::T4;
					else if(machineInfo.machines.at(i).type.compare("T5") == 0)
						worldModel->poi[u].type = POIType::T5;
					else{
						FileLog::log(log_Job,"[JobPlanner]: updateMachineTypes() failed - unknown Machine/Type combination");
						exit(-1);
					}
					cout << "Assigned type " << worldModel->poi[u].type << " to machine "<< poiTypeStr << endl;
				}
			}
		}

		worldModel->updatedMachineTypes = true;
	}

//	exit(-1);
}

bool JobPlanner::checkLateOrder(){
	OrderInfo order = ModelProvider::getInstance()->getLatestOrderInfo();

	for(unsigned int i=0;i<order.orders.size();i++){ //TODO: check for deliverytime (currTime+xx s?)
		if(order.orders.at(i).delivery_period_end - order.orders.at(i).delivery_period_begin==120 && order.orders.at(i).delivery_period_end > ModelProvider::getInstance()->getLatestGameState().game_time.sec+20){
			FileLog::log(log_Job,"[JobPlanner]: checkLateOrder() - Late Order active!");
			return true;
		}
	}
	return false;
}
//update required product orders, only counts them if there is still time to deliver them
void JobPlanner::updateOrders(){ //TODO: update for 2014 season
	OrderInfo order = ModelProvider::getInstance()->getLatestOrderInfo();
	Time time = ModelProvider::getInstance()->getLatestGameState().game_time;
	for(unsigned int i=0;i<order.orders.size();i++){
		//only count them if the order is currently active
		/*if((order.orders.at(i).delivery_period_end > time.sec + 30) &&
				((order.orders.at(i).delivery_period_begin == 0 && time.sec >= 0 && time.sec < 300) ||
				(order.orders.at(i).delivery_period_begin == 300 && time.sec >= 300 && time.sec < 600) ||
				(order.orders.at(i).delivery_period_begin == 600 && time.sec >= 600 && time.sec < 900) ||
				(order.orders.at(i).delivery_period_begin == 900 && time.sec >= 900 && time.sec < 1200)))
		{*/
			if(order.orders.at(i).product == ProductType::P1){
				currentP1Orders = order.orders.at(i).quantity_requested - order.orders.at(i).quantity_delivered;
			}
			else if(order.orders.at(i).product == ProductType::P2){
				currentP2Orders = order.orders.at(i).quantity_requested - order.orders.at(i).quantity_delivered;
			}
			else if(order.orders.at(i).product == ProductType::P3){
				currentP3Orders = order.orders.at(i).quantity_requested - order.orders.at(i).quantity_delivered;
			}
		//}
	}
}

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

	checkOutOfOrder();
	//updatePOITypeLogic();

	vector<Job*> vJobAll;
	vector<Job*> vJobAvailable;

	if(!worldModel->updatedTeamDistribution){
		updateTeamDistribution();
	}

	if(ModelProvider::getInstance()->getLatestGameState().phase==Phase::PRODUCTION){
		if(!worldModel->updatedMachineTypes)
			updateMachineTypes();
	}
	if(ModelProvider::getInstance()->getLatestGameState().phase==Phase::PRODUCTION && worldModel->updatedMachineTypes && worldModel->updatedTeamDistribution){
		updateOrders();

		FileLog::log(log_Job,"[JobPlanner]: current Orders: P1: ",std::to_string(currentP1Orders)," P2: ",std::to_string(currentP2Orders)," P3: ", std::to_string(currentP3Orders));
		FileLog::log_NOTICE("[JobPlanner]: current Orders: P1: ",std::to_string(currentP1Orders)," P2: ",std::to_string(currentP2Orders)," P3: ", std::to_string(currentP3Orders));

		//if(currentP1Orders>0){
			addJob(new Job(JobClass::Deliver, JobType::P1_TO_DEL, 1200, JobProgressStatus::STARTING, Puck::P1, POIType::DELIVER, POIRequiredNext::P1), vJobAll, vJobAvailable, gridX, gridY);
			addJob(new Job(JobClass::Process, JobType::S2_TO_T3, 1100, JobProgressStatus::STARTING, Puck::S2, POIType::T3, POIRequiredNext::S2), vJobAll, vJobAvailable, gridX, gridY);
		//}
		//if(currentP2Orders>0){
			addJob(new Job(JobClass::Deliver, JobType::P2_TO_DEL, 1200, JobProgressStatus::STARTING, Puck::P2, POIType::DELIVER, POIRequiredNext::P2), vJobAll, vJobAvailable, gridX, gridY);
			addJob(new Job(JobClass::Process, JobType::S2_TO_T4, 1100, JobProgressStatus::STARTING, Puck::S2, POIType::T4, POIRequiredNext::S2), vJobAll, vJobAvailable, gridX, gridY);
		//}
		//if(currentP3Orders>0){
			addJob(new Job(JobClass::Deliver, JobType::P3_TO_DEL, 2000, JobProgressStatus::STARTING, Puck::P3, POIType::DELIVER, POIRequiredNext::P3), vJobAll, vJobAvailable, gridX, gridY);
			addJob(new Job(JobClass::Process, JobType::S0_TO_T5, 1200, JobProgressStatus::STARTING, Puck::S0, POIType::T5, POIRequiredNext::S0), vJobAll, vJobAvailable, gridX, gridY);
		//}
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
	else if((ModelProvider::getInstance()->getLatestGameState().phase==Phase::EXPLORATION && worldModel->updatedTeamDistribution)){
		addExplorationJob(new Job(JobClass::Explore, JobType::Explore, 1000, JobProgressStatus::STARTING, Puck::NON_EXISTING, POIType::UNKNOWN, POIRequiredNext::NON_EXISTING), vJobAll, vJobAvailable, gridX, gridY);
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
		if(!(resJob->jobClass == JobClass::Explore && (resJob->POIFrom->type == POIType::INPUT))){
			resJob->POIFrom->occupied = ModelProvider::getInstance()->getID();
		}
		if(resJob->POITo->type != POIType::DELIVER) // do not occupy the deliver gate -> leads to busy waiting for every robot that wants to deliver pucks to the gate
			resJob->POITo->occupied = ModelProvider::getInstance()->getID();
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

//adds an explorationJob to the list if available(either unknown machines left or a transitional poi for production phase
bool JobPlanner::addExplorationJob(Job *job,vector<Job*>& vJobAll, vector<Job*>& vJobAvailable, int gridX, int gridY){
	POI* iz1;
	POI* iz2;
	POI* loz;

	int airTravelDistancePOIFrom = 0;
	int airTravelDistancePOITo = 0;
	int unknowns = 0;


	vJobAll.push_back(job);

	for(int index =0;index<NUMBER_OF_POIS;index++){
		if(worldModel->poi[index].type==POIType::UNKNOWN && worldModel->poi[index].status!=POIStatus::OFFLINE && worldModel->poi[index].occupied == 0)
			unknowns++;
	}
	FileLog::log(log_Job, "[JobPlanner] number of unknown free machines: ", std::to_string(unknowns));
	//nothing left to explore!
	if(unknowns==0){
		if (BaseParameterProvider::getInstance()->getParams()->team_number == 1){
			iz1 = &(worldModel->poi[13]);
			iz2 = &(worldModel->poi[14]);
			loz = &(worldModel->poi[15]);
		}
		else{
			iz1 = &(worldModel->poi[29]);
			iz2 = &(worldModel->poi[30]);
			loz = &(worldModel->poi[31]);
		}

		if((iz1->occupied == (int)ModelProvider::getInstance()->getID())|| iz2->occupied == (int)ModelProvider::getInstance()->getID() || loz->occupied == (int)ModelProvider::getInstance()->getID()){
			return false;
		}

		//get into position at the input zones
		if(iz1->occupied==0)
			job->POITo = iz1;
		else if(iz2->occupied==0)
			job->POITo = iz2;
		//if both input zone nodes are already occupied, just get out of the way (currently: delivery poi)
		else if(loz->occupied==0){
			job->POITo = loz;
		}

	}
	if(lastJob != NULL){
		job->POIFrom = lastJob->POITo;
	}

	else{
		if (BaseParameterProvider::getInstance()->getParams()->team_number == 1){
			iz1 = &(worldModel->poi[13]);
			iz2 = &(worldModel->poi[14]);
			loz = &(worldModel->poi[15]);
		}
		else{
			iz1 = &(worldModel->poi[29]);
			iz2 = &(worldModel->poi[30]);
			loz = &(worldModel->poi[31]);
		}

		if(ModelProvider::getInstance()->getID()==ID::ROBO3){
			job->POIFrom = loz;
		}
		else if(ModelProvider::getInstance()->getID()==ID::ROBO2){
			job->POIFrom = iz2;

		}
		else if(ModelProvider::getInstance()->getID()==ID::ROBO1){
			job->POIFrom = iz1;
		}
	}
	if(job->POITo == NULL){
		std::vector<POI *> vCandidatesIndexesTo = giveAllCandidatesPOI(job->targetPOIType, job->targetRequiredNext);
		if(!vCandidatesIndexesTo.empty())
			job->POITo = giveNearestMachineIndex(vCandidatesIndexesTo, gridX, gridY, POIAccessFrom::FRONT, airTravelDistancePOITo);
		else
			return false;
	}
	if(lastJob != NULL && job->POITo->index == lastJob->POITo->index){
		return false;
	}
	vJobAvailable.push_back(job);
	return true;
}
bool JobPlanner::addJob(Job *job, vector<Job*>& vJobAll, vector<Job*>& vJobAvailable, int gridX, int gridY)
{
	vJobAll.push_back(job);
	if(checkAvailableJob(vJobAll.back()))
	{
		if(defineDetailsOfJob(vJobAll.back(), gridX, gridY))
		{
			vJobAvailable.push_back(job);
			return true;
		}
	}
	return false;
}

bool JobPlanner::checkAvailableJob(Job *job)
{
	//if(DEBUG_PRINTS_ENABLED_JOBPLANNER) cout << "-- Checking job\t " << JobType::cJobType[job->jobType] << endl;
	bool check1 = checkAvailablePuck(job->carriedGood, job->accDir);
	bool check2 = checkAvailablePOI(job->targetPOIType, job->targetRequiredNext);
	//cout << "checkAvailableJob check 1 was " << check1 << " (" << Puck::cPuck[job->carriedGood] << ") and check 2 was " << check2 << endl;
	return check1 && check2;
}

//Checks if any machine has the desired puck and is not occupied by a running job
bool JobPlanner::checkAvailablePuck(Puck::Puck p, POIAccessFrom::POIAccessFrom accDir)
{
	for(int i=0;i<NUMBER_OF_POIS;i++)
	{
		if(checkAvailablePuck(i, p, accDir)) return true;
	}
	return false;
}

bool JobPlanner::checkAvailablePuck(int index, Puck::Puck p, POIAccessFrom::POIAccessFrom accDir)
{
	bool puckAvailable = false;
	//Check if machine contains the desired puck in left, middle or right
	switch(accDir)
	{
	case POIAccessFrom::FRONT:
		puckAvailable = worldModel->poi[index].middle==p;
		break;
	case POIAccessFrom::LEFT:
		puckAvailable = worldModel->poi[index].left==p;
		break;
	case POIAccessFrom::RIGHT:
		puckAvailable = worldModel->poi[index].right==p;
		break;
	case POIAccessFrom::BACK:
		break;
	}

	bool machineAvailable;
	if(worldModel->poi[index].occupied == 0 || worldModel->poi[index].occupied == (int)ModelProvider::getInstance()->getID())
		machineAvailable = true;
	else
		machineAvailable = false;

	bool machineNotOffline;
	if(worldModel->poi[index].status != POIStatus::OFFLINE)
		machineNotOffline = true;
	else
		machineNotOffline = false;

	if(puckAvailable && machineAvailable && machineNotOffline) return true;
	else return false;
}

bool JobPlanner::checkAvailablePOI(POIType::POIType t, POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck)
{
	for(int i=0;i<NUMBER_OF_POIS;i++)
	{
		if(checkAvailablePOI(i, t, req, status, middlePuck)) return true;
	}
	return false;
}

bool JobPlanner::checkAvailablePOI(int index, POIType::POIType t, POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck)
{
	//check normal available + if required holds the desired value

	if(worldModel->poi[index].occupied != 0 && worldModel->poi[index].occupied != (int)ModelProvider::getInstance()->getID())
		return false;
	else
		return worldModel->poi[index].type == t && isPOIRequired(&worldModel->poi[index], req) && worldModel->poi[index].status == status && worldModel->poi[index].middle == middlePuck;
}

bool JobPlanner::checkAvailablePOI(int index, POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck)
{
	if(worldModel->poi[index].occupied != 0 && worldModel->poi[index].occupied != (int)ModelProvider::getInstance()->getID())
		return false;
	else
		return isPOIRequired(&worldModel->poi[index], req) && worldModel->poi[index].status == status && worldModel->poi[index].middle == middlePuck;
}

//Input: General job
//Output: detailed job (means e.g. the information to which exact machine the puck should be brought to will be examined)
//Also current location of the robot is considered here, for shortest possible paths
bool JobPlanner::defineDetailsOfJob(Job *job, int gridX, int gridY)
{
	int airTravelDistancePOIFrom = 0;
	int airTravelDistancePOITo = 0;
	POI* tempPoi;
	POI* tempPoi2;
	std::vector<POI *> vCandidatesIndexesFrom = giveAllCandidatesPuck(job->carriedGood, job->accDir);
	Time time = ModelProvider::getInstance()->getLatestGameState().game_time;
	if(!vCandidatesIndexesFrom.empty())
	{
		job->POIFrom = giveNearestMachineIndex(vCandidatesIndexesFrom, gridX, gridY, job->accDir, airTravelDistancePOIFrom);

		//S0s at Recycling Machines are preferred because else deadlock danger when all T2 and T3 want to recycle their pucks but cannot because of occupied Recycling Machines
		std::vector<POI *> POISOfType;
		if(job->carriedGood == Puck::S0 && containsPOIType(vCandidatesIndexesFrom, POISOfType, POIType::RECYCLE)){
			FileLog::log(log_Job,"[JobPlanner] PoiFrom candidate list contains recycling machine");
			int tmpDist;
			POI* tmpPOI = giveNearestMachineIndex(POISOfType, gridX, gridY, POIAccessFrom::FRONT, tmpDist);
			if(tmpPOI->index != job->POIFrom->index && tmpDist/2 < airTravelDistancePOIFrom){
				FileLog::log(log_Job,"[JobPlanner] changed POIFrom for grabbing S0 from recycling machine");
				job->POIFrom = tmpPOI;
			}
		}
	}
	else
		return false;

	if(job->POIFrom->type == POIType::INPUT && worldModel->pucksGrabbed >= 6)
	{	POI* iz1;
		POI* iz2;

		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1){
			iz1 = &(worldModel->poi[13]);
			iz2 = &(worldModel->poi[14]);
		}
		else{
			iz1 = &(worldModel->poi[29]);
			iz2 = &(worldModel->poi[30]);
		}

		if((iz1->occupied != 0 && iz1->occupied != (int)ModelProvider::getInstance()->getID()) || (iz2->occupied != 0 && iz2->occupied != (int)ModelProvider::getInstance()->getID()))
			return false;
	}

	if(job->jobClass == JobClass::Recycle)
	{
		//don't recycle if the product is still in the machine
		if(job->accDir == POIAccessFrom::FRONT && job->POIFrom->middle != Puck::NON_EXISTING)
			return false;

		// take the side machines into account and change the accDir if needed
		if(job->accDir == POIAccessFrom::LEFT && (job->POIFrom->x == 8 && job->POIFrom->dir == POIDirection::NORTH))
			job->accDir = POIAccessFrom::FRONT;
		else if(job->accDir == POIAccessFrom::RIGHT && (job->POIFrom->x == 8 && job->POIFrom->dir == POIDirection::SOUTH))
			job->accDir = POIAccessFrom::FRONT;
		//special case for the corner machines
		else if((job->accDir == POIAccessFrom::LEFT && job->POIFrom->index == 24) || (job->accDir == POIAccessFrom::RIGHT && job->POIFrom->index == 12)){
			job->accDir = POIAccessFrom::FRONT;
		}
	}

	std::vector<POI *> vCandidatesIndexesTo = giveAllCandidatesPOI(job->targetPOIType, job->targetRequiredNext);

	if(!vCandidatesIndexesTo.empty())
		job->POITo = giveNearestMachineIndex(vCandidatesIndexesTo, job->POIFrom->x, job->POIFrom->y, POIAccessFrom::FRONT, airTravelDistancePOITo);
	else
		return false;

	//logic optimizations, TODO: redo for 2014
	if(job->jobType == JobType::S0_TO_T2){
		for(int i=0;i<NUMBER_OF_POIS;i++){
			tempPoi = &(worldModel->poi[i]);
			if((tempPoi->type == POIType::T4 || tempPoi->type == POIType::T3) && (tempPoi->left == Puck::CONSUMED_IN_USE) && (tempPoi->right == Puck::CONSUMED_IN_USE)){
				for(unsigned int i=0;i<vCandidatesIndexesTo.size();i++){
					if(vCandidatesIndexesTo[i]->type == POIType::T2 && vCandidatesIndexesTo[i]->right == Puck::CONSUMED_IN_USE && !(vCandidatesIndexesTo[i]->middle == Puck::S2)){
						job->POITo = vCandidatesIndexesTo[i];
						job->priority+=200;
					}
				}
			}
		}
	}

	if(job->jobType == JobType::S1_TO_T2){
		for(int i=0;i<NUMBER_OF_POIS;i++){
			tempPoi = &(worldModel->poi[i]);
			if((tempPoi->type == POIType::T4 || tempPoi->type == POIType::T3) && (tempPoi->left == Puck::CONSUMED_IN_USE) && (tempPoi->right == Puck::CONSUMED_IN_USE)){
				for(unsigned int j=0;j<vCandidatesIndexesTo.size();j++){
					if(vCandidatesIndexesTo[j]->type == POIType::T2 && vCandidatesIndexesTo[j]->left == Puck::CONSUMED_IN_USE && !(vCandidatesIndexesTo[j]->middle == Puck::S2)){
						job->POITo = vCandidatesIndexesTo[j];
						job->priority+=200;
					}
				}
			}
		}
	}
	//increase priority of producing a S1 in case a T2 needs one to finish S2 which is needed at a T3/4
	if(job->jobType == JobType::S0_TO_T1){
		for(int i=0;i<NUMBER_OF_POIS;i++){
			tempPoi = &(worldModel->poi[i]);
			if((tempPoi->type == POIType::T4 || tempPoi->type == POIType::T3) && (tempPoi->left == Puck::CONSUMED_IN_USE) && (tempPoi->right == Puck::CONSUMED_IN_USE)){
				for(int j=0;j<NUMBER_OF_POIS;j++){
					tempPoi2 = &(worldModel->poi[j]);
					if(((tempPoi2->type == POIType::T2) && (tempPoi2->left == Puck::CONSUMED_IN_USE) && !(tempPoi2->middle == Puck::S2))){
						job->priority+=200;
					}
				}
			}
		}
	}
	//in the last few minutes, try to focus on one production machine
	if(time.sec > 700){
		for(int i=0;i<NUMBER_OF_POIS;i++){
			tempPoi = &(worldModel->poi[i]);
			//look for a production machine that is currently in use...
			if(((tempPoi->type == POIType::T4 || tempPoi->type == POIType::T3) && (tempPoi->left == Puck::CONSUMED_IN_USE)) || (tempPoi->right == Puck::CONSUMED_IN_USE)){
				for(int j=0;j<NUMBER_OF_POIS;j++){
					tempPoi2 = &(worldModel->poi[j]);
					//...and kill the other machine if there are no pucks at all
					if(((tempPoi2->type == POIType::T4 || tempPoi2->type == POIType::T3) && (tempPoi2->left == Puck::NON_EXISTING)&& (tempPoi2->right == Puck::NON_EXISTING)&& (tempPoi2->middle == Puck::NON_EXISTING))){
						tempPoi2->status = POIStatus::OFFLINE;
					}
				}
			}
		}
	}
	if((job->POITo->type == POIType::T3 || job->POITo->type == POIType::T4) && (job->POITo->index == 24 || job->POITo->index == 12))
		job->priority -= 50;

	job->priority -= 10*airTravelDistancePOIFrom;
	job->priority -= 10*airTravelDistancePOITo;
	if(lastJob != NULL && lastPOITo != job->POIFrom)
	{
		if(lastJob->jobClass == JobClass::Recycle)	// guarantee that after recycling, the S0 will be transported immediately
			job->priority = 0;
		else
			job->priority -= 100;
	}
	return true;
}

//Returns the indexes of the POIs from which the given Puck is available
vector<POI *> JobPlanner::giveAllCandidatesPuck(Puck::Puck p, POIAccessFrom::POIAccessFrom accDir)
{
	std::vector<POI *> vCandidatesIndexes;
	//List all station indexes with the desired puck
	for(int i=0;i<NUMBER_OF_POIS;i++)
	{
		if(checkAvailablePuck(i, p, accDir)) vCandidatesIndexes.push_back(&worldModel->poi[i]);
	}

	return vCandidatesIndexes;
}

vector<POI *> JobPlanner::giveAllCandidatesPOI(POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck)
{
	vector<POI *> vCandidatesIndexes;

	//List all station indexes with resp. Type and RequiredNext
	for(int i=0;i<NUMBER_OF_POIS;i++)
	{
		if(checkAvailablePOI(i, req, status, middlePuck)) vCandidatesIndexes.push_back(&worldModel->poi[i]);
	}

//	if(DEBUG_PRINTS_ENABLED_JOBPLANNER)
//	{
//		//cout << "Giving candidates for Type " << POIType::cPOIType[t] << " and requNext " << POIRequiredNext::cPOIRequiredNext[req] << endl;
//		//cout << "vCandidatesSize:\t " << vCandidatesIndexes.size() << endl;
//	}
	return vCandidatesIndexes;
}

vector<POI *> JobPlanner::giveAllCandidatesPOI(POIType::POIType t, POIRequiredNext::POIRequiredNext req, POIStatus::POIStatus status, Puck::Puck middlePuck)
{
	vector<POI *> vCandidatesIndexes;

	//List all station indexes with resp. Type and RequiredNext
	for(int i=0;i<NUMBER_OF_POIS;i++)
	{
		if(checkAvailablePOI(i, t, req, status, middlePuck)) vCandidatesIndexes.push_back(&worldModel->poi[i]);
	}

//	if(DEBUG_PRINTS_ENABLED_JOBPLANNER)
//	{
//		//cout << "Giving candidates for Type " << POIType::cPOIType[t] << " and requNext " << POIRequiredNext::cPOIRequiredNext[req] << endl;
//		//cout << "vCandidatesSize:\t " << vCandidatesIndexes.size() << endl;
//	}
	return vCandidatesIndexes;
}

//Returns the POI index in the array, which resp. POI is nearest to the given grid position
POI *JobPlanner::giveNearestMachineIndex(vector<POI *> vPOIs, int gridX, int gridY, POIAccessFrom::POIAccessFrom accDir, int &finalAirTravelDistance)
{
	if(vPOIs.empty()) return 0;

	POI *finalCandidate = vPOIs[0];
	float finalCandidateDistance = calcDistance(gridX, gridY, grid->getAccessNode(finalCandidate, accDir)->getX(), grid->getAccessNode(finalCandidate, accDir)->getY());

	for(unsigned int i=1; i<vPOIs.size(); i++)
	{
		POI *tempCandidate = vPOIs[i];
		float tempCandidateDistance = calcDistance(gridX, gridY, grid->getAccessNode(tempCandidate, accDir)->getX(), grid->getAccessNode(tempCandidate, accDir)->getY());
		if(tempCandidateDistance < finalCandidateDistance)
		{
			finalCandidate = tempCandidate;
			finalCandidateDistance = tempCandidateDistance;
		}
	}

	finalAirTravelDistance = abs(grid->getAccessNode(finalCandidate, accDir)->getX() - gridX) + abs(grid->getAccessNode(finalCandidate, accDir)->getY() - gridY);

	//if(DEBUG_PRINTS_ENABLED_JOBPLANNER) cout << "FinalIndex:\t " << finalCandidate->index << endl;
	return finalCandidate;
}

float JobPlanner::calcDistance(int fromX, int fromY, int destX, int destY)
{
	float dx = (float)destX - fromX;
	float dy = (float)destY - fromY;
	return sqrt(dx*dx + dy*dy);
}

bool JobPlanner::redefineTarget(Job *p)
{
	checkOutOfOrder();

	if(p->jobClass == JobClass::Deliver || p->jobClass == JobClass::Process)
	{
		Job* djob = (Job*)p;
		if(djob->carriedGood != Puck::S0 && djob->carriedGood != Puck::S1 && djob->carriedGood != Puck::S2)
		{
			cerr << "JobPlanner::redefineTarget: djob->carriedGood is strange.";
			return false;
		}

		// look if a machine is available, which requires the carriedGood
		std::vector<POI *> vCandidatesIndexes = giveAllCandidatesPOI(djob->targetRequiredNext);
		if(!vCandidatesIndexes.empty())
		{
			int airTravelDistancePOITo = 0;
			djob->POITo = giveNearestMachineIndex(vCandidatesIndexes, djob->POITo->x, djob->POITo->y, POIAccessFrom::FRONT, airTravelDistancePOITo);
			djob->POITo->occupied = (int)ModelProvider::getInstance()->getID();
			djob->targetPOIType = djob->POITo->type;

			//Set the job type according to the new target POI type
			switch(djob->targetPOIType)
			{
			case POIType::T5:
				if(djob->carriedGood == Puck::S0)
					djob->jobType = JobType::S0_TO_T5;
				else
				{
					cerr << "Something is wrong. --> carriedPuck type _TO_T5" << endl;
					return false;
				}
				break;
			case POIType::T4:
				if(djob->carriedGood == Puck::S2)
					djob->jobType = JobType::S2_TO_T4;
				else if(djob->carriedGood == Puck::S1)
					djob->jobType = JobType::S1_TO_T4;
				else if(djob->carriedGood == Puck::S0)
					djob->jobType = JobType::S0_TO_T4;
				else
				{
					cerr << "Something is wrong. --> carriedPuck type _TO_T4" << endl;
					return false;
				}
				break;
			case POIType::T3:
				if(djob->carriedGood == Puck::S2)
					djob->jobType = JobType::S2_TO_T3;
				else if(djob->carriedGood == Puck::S1)
					djob->jobType = JobType::S1_TO_T3;
				else if(djob->carriedGood == Puck::S0)
					djob->jobType = JobType::S0_TO_T3;
				else
				{
					cerr << "Something is wrong. --> carriedPuck type _TO_T3" << endl;
					return false;
				}
				break;
			case POIType::T2:
				if(djob->carriedGood == Puck::S0)
					djob->jobType = JobType::S0_TO_T2;
				else if(djob->carriedGood == Puck::S1)
					djob->jobType = JobType::S1_TO_T2;
				else
				{
					cerr << "Something is wrong. --> carriedPuck type _TO_T2" << endl;
					return false;
				}
				break;
			case POIType::T1:
				if(djob->carriedGood == Puck::S0)
					djob->jobType = JobType::S0_TO_T1;
				else
				{
					cerr << "Something is wrong. --> carriedPuck type _TO_T1" << endl;
					return false;
				}
				break;
			default:
				cerr << "Something is wrong. --> targetPOIType" << endl;
				return false;
			}
			return true;
		}
		else
		{
			// no alternative machine found
			return false;
		}
	}
	else if(p->jobClass == JobClass::Recycle)
	{
		return false;
	}
	return false;
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

bool JobPlanner::isPOIRequired(POI *poi, POIRequiredNext::POIRequiredNext req)
{
	switch(poi->type)
	{
	case POIType::DELIVER:
		return req == POIRequiredNext::P1 || req == POIRequiredNext::P2 || req == POIRequiredNext::P3;
	case POIType::INPUT:
		return req == POIRequiredNext::NON_EXISTING;
	case POIType::T1:
	{
		if(poi->middle == Puck::NON_EXISTING)
			return req == POIRequiredNext::S0;
		else
			return req == POIRequiredNext::NON_EXISTING;
	}
	case POIType::T2:
	{
		if(poi->middle == Puck::NON_EXISTING && poi->left == Puck::NON_EXISTING && poi->right == Puck::NON_EXISTING) // machine is emtpy
			return req == POIRequiredNext::S0 || req == POIRequiredNext::S1;
		else if(poi->middle == Puck::NON_EXISTING && poi->left == Puck::CONSUMED_IN_USE) // one puck has been consumed
			return req == POIRequiredNext::S1;
		else if(poi->middle == Puck::NON_EXISTING && poi->right == Puck::CONSUMED_IN_USE) // one puck has been consumed
			return req == POIRequiredNext::S0;
		else if(poi->left == Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2 || poi->right == Puck::CONSUMED_FREE_TO_RECYCLE_AT_T2) // one puck is free to be recycled
			return req == POIRequiredNext::RECYCLE;
		else
			return req == POIRequiredNext::NON_EXISTING;
	}
	case POIType::T3:
	{
		if(poi->middle == Puck::NON_EXISTING && poi->left == Puck::NON_EXISTING && poi->right == Puck::NON_EXISTING) // machine is emtpy
			return req == POIRequiredNext::S0 || req == POIRequiredNext::S1;
		else if(poi->middle == Puck::NON_EXISTING && (poi->left == Puck::NON_EXISTING && poi->right == Puck::CONSUMED_IN_USE))
			return req == POIRequiredNext::S0;
		else if(poi->middle == Puck::NON_EXISTING && (poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::NON_EXISTING))
			return req == POIRequiredNext::S1;
		else if(poi->middle == Puck::NON_EXISTING && (poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::CONSUMED_IN_USE)) // two pucks have been consumed
			return req == POIRequiredNext::S2;
		else if(poi->left == Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3 || poi->right == Puck::CONSUMED_FREE_TO_RECYCLE_AT_T3) // one puck is free to be recycled
			return req == POIRequiredNext::RECYCLE;
		else
			return req == POIRequiredNext::NON_EXISTING;
	}
	case POIType::T4:
	{
		if(poi->middle == Puck::NON_EXISTING && poi->left == Puck::NON_EXISTING && poi->right == Puck::NON_EXISTING) // machine is emtpy
			return req == POIRequiredNext::S0 || req == POIRequiredNext::S1;
		else if(poi->middle == Puck::NON_EXISTING && (poi->left == Puck::NON_EXISTING && poi->right == Puck::CONSUMED_IN_USE))
			return req == POIRequiredNext::S0;
		else if(poi->middle == Puck::NON_EXISTING && (poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::NON_EXISTING))
			return req == POIRequiredNext::S1;
		else if(poi->middle == Puck::NON_EXISTING && (poi->left == Puck::CONSUMED_IN_USE && poi->right == Puck::CONSUMED_IN_USE)) // two pucks have been consumed
			return req == POIRequiredNext::S2;
		else if(poi->left == Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4 || poi->right == Puck::CONSUMED_FREE_TO_RECYCLE_AT_T4) // one puck is free to be recycled
			return req == POIRequiredNext::RECYCLE;
		else
			return req == POIRequiredNext::NON_EXISTING;
	}
	case POIType::T5:
	{
		if(poi->middle == Puck::NON_EXISTING)
			return req == POIRequiredNext::S0;
		else
			return req == POIRequiredNext::NON_EXISTING;
	}
	case POIType::RECYCLE:
	{
		if(poi->middle == Puck::NON_EXISTING)
			return req == POIRequiredNext::CONSUMED;
		else
			return req == POIRequiredNext::NON_EXISTING;
	}
	default:
		return req == POIRequiredNext::NON_EXISTING;
	}
}

bool JobPlanner::containsPOIType(std::vector<POI *> vCandidatesIndexesFrom, std::vector<POI *> &POISofType, POIType::POIType type){

	bool result = false;
	for(unsigned int i = 0; i < vCandidatesIndexesFrom.size(); i++){
		if(vCandidatesIndexesFrom[i]->type == type){
			POISofType.push_back(vCandidatesIndexesFrom[i]);
			result = true;
		}
	}
	return result;
}
