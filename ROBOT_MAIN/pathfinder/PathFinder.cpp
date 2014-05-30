/*
 * PathFinder.cpp
 *
 *  Created on: Jun 4, 2011
 *      Author: root
 */

#include "PathFinder.h"
#include "communication/Communication.h"
#include "communication/WorldModelClientHandler.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "BaseParameterProvider.h"

PathFinder::PathFinder(AsyncStateMachine* asyncStateMachine, SensorServer* sensorServer) {
	this->asyncStateMachine = asyncStateMachine;
	this->sensorServer = sensorServer;
	grid = new Grid();
	findPathThread = NULL;
	randomPOI = new POI();
}

PathFinder::~PathFinder()
{
	if(grid != NULL)
		delete grid;
	if(randomPOI != NULL)
		delete randomPOI;
}

bool PathFinder::isNextRelevantInSameNode(int gridX, int gridY){
	float x = sensorServer->getX();
	float y = sensorServer->getY();
	Node* node = grid->getNodeByCoord(x,y);

	if(node->getX() == gridX && node->getY()==gridY){
		return true;
	}else{
		return false;
	}

}

// input position in mm
bool PathFinder::isNextRelevantInSameNodeMM(float gridXPos, float gridYPos){
	float x = sensorServer->getX();
	float y = sensorServer->getY();
	Node* node = grid->getNodeByCoord(x,y);
	Node* node2 = grid->getNodeByCoord(gridXPos, gridYPos);

	if(node->getX() == node2->getX() && node->getY()== node2->getY()){
		return true;
	}else{
		return false;
	}

}

// input position in mm
bool PathFinder::isNextRelevantInSameNodeMM(float gridXPos, float gridYPos, float gridPhi){
	float x,y,phi;
	sensorServer->getOdometry(x,y,phi);
	Node* node = grid->getNodeByCoord(x,y);
	Node* node2 = grid->getNodeByCoord(gridXPos, gridYPos);

	if(node->getX() == node2->getX() && node->getY()== node2->getY()){
		if(getDirectionFromAngle(phi) == getDirectionFromAngle(gridPhi)){
			return true;
		}
		else{
			return false;
		}
	}else{
		return false;
	}

}

POIDirection::POIDirection PathFinder::getDirection(float phi)
{

	if(phi<=-45 && phi>-135)
		return POIDirection::WEST;
	else if(phi<=135 && phi>45)
		return POIDirection::EAST;
	else if(phi<=45 && phi>-45)
		return POIDirection::SOUTH;
	else
		return POIDirection::NORTH;
}

float PathFinder::getPhi(POIDirection::POIDirection dir){
	switch(dir){
	case POIDirection::NORTH: return 180;
	case POIDirection::SOUTH: return 0;
	case POIDirection::EAST: return 90;
	case POIDirection::WEST: return -90;
	}
	return 0;
}

POI* PathFinder::moveABit(){
	float x = sensorServer->getX();
	float y = sensorServer->getY();
	float rotation = sensorServer->getPhi();
	Node* startNode = grid->getNodeByCoord(x,y);
	Node* endNode;
	POIDirection::POIDirection direction = getDirection(rotation);
	srand ( time(NULL) );

	//POIDirection::POIDirection randPOIDir = (POIDirection::POIDirection)(rand() % 4);

	int randomDirection = rand() % 4;

	for(int i=0;i<4;i++){
		randomDirection = (randomDirection + 1) % 4;

		if(startNode->getNeighbor((POIDirection::POIDirection)(randomDirection)) !=NULL){
			endNode = startNode->getNeighbor((POIDirection::POIDirection)(randomDirection));

			//if(!(endNode->getX()==1 && (endNode->getY()>=2 && endNode->getY()<=3))){
			if(!(endNode->getX()==0 || endNode->getX()==8)){
				if((endNode->getNodeRaw()->occupied==0 || endNode->getNodeRaw()->occupied==(int)ModelProvider::getInstance()->getID()) && !endNode->isBlocked()){
					//path = grid->getPath(startNode,direction,endNode,(POIDirection::POIDirection)(randomDirection)); // direction
					randomPOI->type = POIType::RECYCLE; //TODO: was POIType::SCANNER; before, okay now?
					randomPOI->dir = (POIDirection::POIDirection)((randomDirection+2)%4); //(POIDirection::POIDirection)((((int)direction)+2)%4);
					randomPOI->x = endNode->getX();
					randomPOI->y = endNode->getY();
					return randomPOI;
				}
			}
		}

	}

	return NULL;

}

void PathFinder::findPathExec(POI* poi, POIAccessFrom::POIAccessFrom accDir, Node* startNode, POIDirection::POIDirection startPOIdir, POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, const vector<Node*> blacklist){
	list<ExpandedNode>* path = NULL;

	while(path==NULL){

		if(ModelProvider::getInstance()->gameStateIsPaused()){
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			continue;
		}

		WorldModelClientHandler::getInstance()->lock();


		Node* endNode = grid->getAccessNode(poi,accDir);
		POIDirection::POIDirection accessDirection = grid->getAccessDirection(poi,accDir);

		grid->clear(startNode);
		//Block the access node
		if(poiFrom != NULL)
		{
			Node* accFromNode = grid->getAccessNode(poiFrom,accDirFrom);
			accFromNode->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
			// TODO: set getNodeRaw()->timestamp ?!
		}
		path = grid->getPath(startNode,startPOIdir,endNode,accessDirection, blacklist);
//		if(path==NULL){
//			boost::shared_ptr<EvNoPathFound> ev(new EvNoPathFound());
//			FileLog::log_NOTICE("[PathFinder] EvNoPathFound");
//			asyncStateMachine->queueEvent(ev);
//		}else
		if(path!=NULL){
			if(path->size()>1){
				ExpandedNode nextRelevantNode = grid->getNextRelevantNode(path);
				vector<vec3D> vias = convertExpandedPathToViaPoints(path);
				event=boost::shared_ptr<EvPathFound>(new EvPathFound(nextRelevantNode.getNode()->getXPos(),nextRelevantNode.getNode()->getYPos(),getPhi(nextRelevantNode.getDirection()),vias));
				FileLog::log_NOTICE("[PathFinder] EvPathFound");
			} else{
				event= boost::shared_ptr<EvPathDriven>(new EvPathDriven()); //boost::shared_ptr<EvPathDriven> e
				FileLog::log_NOTICE("[PathFinder] EvPathDriven");

			}
		}



		WorldModelClientHandler::getInstance()->unlock();


		if(path==NULL){
			boost::this_thread::sleep(boost::posix_time::seconds(2));
		}else{
			asyncStateMachine->queueEvent(event);
		}
	}

}

// TODO: only used once ?! replace with findPathTo?!
void PathFinder::findPathFromProductionMachineExec(POI* poiFrom,POIAccessFrom::POIAccessFrom accDirFrom,POI* poiTo,POIAccessFrom::POIAccessFrom accDirTo){
	list<ExpandedNode>* leftPath= NULL;
	list<ExpandedNode>* rightPath = NULL;

	while(leftPath==NULL && rightPath == NULL){

		if(ModelProvider::getInstance()->gameStateIsPaused()){
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			continue;
		}

		WorldModelClientHandler::getInstance()->lock();

		grid->clearAll();
		Node* startNode = grid->getNode(poiFrom->x,poiFrom->y);
		Node* accessNode = grid->getAccessNode(poiFrom,accDirFrom);
		POIDirection::POIDirection accessDirectionFrom = grid->getAccessDirection(poiFrom,accDirFrom);

		Node* endNode = grid->getAccessNode(poiTo,accDirTo);
		POIDirection::POIDirection accessDirectionTo = grid->getAccessDirection(poiTo,accDirTo);

		POIDirection::POIDirection leftNeighborDir = (POIDirection::POIDirection)((accessDirectionFrom+3)%4);
		bool leftIsRegular;
		Node* leftNeighbor =   startNode->getNeighbor(leftNeighborDir, leftIsRegular);
		Node* leftAccessNeighbor =   accessNode->getNeighbor(leftNeighborDir);

		POIDirection::POIDirection rightNeighborDir = (POIDirection::POIDirection)((accessDirectionFrom+1)%4);
		bool rightIsRegular;
		Node* rightNeighbor =   startNode->getNeighbor(rightNeighborDir, rightIsRegular);
		Node* rightAccessNeighbor =   accessNode->getNeighbor(rightNeighborDir);



		//Find right and left path if existing
		if(leftIsRegular && leftNeighbor->getNodeRaw()->occupied==0 && leftAccessNeighbor->getNodeRaw()->occupied==0){
			leftAccessNeighbor->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
			leftPath = grid->getPath(leftNeighbor,leftNeighborDir,endNode,accessDirectionTo);
		}
		grid->clearAll();
		if(rightIsRegular && rightNeighbor->getNodeRaw()->occupied==0 && rightAccessNeighbor->getNodeRaw()->occupied==0){
			rightAccessNeighbor->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
			rightPath = grid->getPath(rightNeighbor,rightNeighborDir,endNode,accessDirectionTo);
		}

		int lengthLeftPath = 1000;
		int lengthRightPath = 1000; //initialization is a maximum (longer paths will surely not exist)

		if(leftPath != NULL){
			lengthLeftPath = leftPath->size();
		}

		if(rightPath != NULL){
			lengthRightPath = rightPath->size();
		}

		grid->clearAll();
		//Block the access node
		accessNode->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
		if(leftPath != NULL||rightPath != NULL){


			if(lengthLeftPath<lengthRightPath){
				leftAccessNeighbor->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
				leftPath = grid->getPath(leftNeighbor,leftNeighborDir,endNode,accessDirectionTo);
				ExpandedNode nextRelevantNode = leftPath->front();
				event = boost::shared_ptr<EvPathFoundLeavePoi>(new EvPathFoundLeavePoi(nextRelevantNode.getNode()->getXPos(),nextRelevantNode.getNode()->getYPos(),getPhi(nextRelevantNode.getDirection()),LeaveDirection::LEFT));
				FileLog::log_NOTICE("[PathFinder] EvPathFound LEFT");

			}else{
				rightAccessNeighbor->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
				rightPath = grid->getPath(rightNeighbor,rightNeighborDir,endNode,accessDirectionTo);

				ExpandedNode nextRelevantNode = rightPath->front();
				event = boost::shared_ptr<EvPathFoundLeavePoi>(new EvPathFoundLeavePoi(nextRelevantNode.getNode()->getXPos(),nextRelevantNode.getNode()->getYPos(),getPhi(nextRelevantNode.getDirection()),LeaveDirection::RIGHT));
				FileLog::log_NOTICE("[PathFinder] EvPathFound RIGHT");
			}
		}

		WorldModelClientHandler::getInstance()->unlock();
		if(leftPath == NULL && rightPath == NULL){
			boost::this_thread::sleep(boost::posix_time::seconds(2));
		}else{
			asyncStateMachine->queueEvent(event);
		}
	}

}


void PathFinder::getAccessToDeliveryZoneExec()
{
	bool gotAccess = false;

	while(!gotAccess)
	{
		if(ModelProvider::getInstance()->gameStateIsPaused())
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			continue;
		}

		WorldModelClientHandler::getInstance()->lock();

		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
		{
			POI* deliveryZone = &ModelProvider::getInstance()->getWorldModel()->poi[15];
			int startx = deliveryZone->x-1;
			int starty = deliveryZone->y;
			gotAccess = reserveZone(startx,starty,ModelProvider::getInstance()->getID(),3,2);
		}
		else
		{
			POI* deliveryZone = &ModelProvider::getInstance()->getWorldModel()->poi[31];
			int startx = deliveryZone->x-1;
			int starty = deliveryZone->y-1;
			gotAccess = reserveZone(startx,starty,ModelProvider::getInstance()->getID(),3,2);
		}

		WorldModelClientHandler::getInstance()->unlock();

		if(gotAccess==false){
			boost::this_thread::sleep(boost::posix_time::seconds(2));
		}else{
			boost::shared_ptr<EvAccessZoneReserved> ev(new EvAccessZoneReserved());
			FileLog::log_NOTICE("[PathFinder] EvAccessZoneReserved");
			asyncStateMachine->queueEvent(ev);
		}
	}
}

/**
 * trys to block a zone 3*2 starting from startx, starty from left to right and up to down
 * and returns whether it was successful
 */
bool PathFinder::reserveZone(int startx, int starty, ID::ID id_,int fields_x,int fields_y){

	int id = (int)id_;
	for(int x =startx;x<startx+fields_x;x++){
		for(int y=starty;y<starty+fields_y;y++){

			if(grid->getNode(x,y)->getNodeRaw()->occupied != 0 && grid->getNode(x,y)->getNodeRaw()->occupied != id){
				return false;
			}else{
				grid->getNode(x,y)->getNodeRaw()->occupied = id;
			}
		}
	}

	return true;
}

void PathFinder::getAccessBackOfPOIExec(POI* poiFrom){

	bool gotAccess = false;

	while(!gotAccess){
			if(ModelProvider::getInstance()->gameStateIsPaused()){
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				continue;
			}

			Node *nodeBackPOI = grid->getAccessNode(poiFrom, POIAccessFrom::BACK);

			WorldModelClientHandler::getInstance()->lock();

			if(nodeBackPOI->getNodeRaw()->occupied == 0 || nodeBackPOI->getNodeRaw()->occupied == ModelProvider::getInstance()->getID())
			{
				gotAccess = true;
				nodeBackPOI->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();

				boost::shared_ptr<EvAccessTripletReserved> ev(new EvAccessTripletReserved());
				FileLog::log_NOTICE("[PathFinder] EvAccessTripletReserved");
				asyncStateMachine->queueEvent(ev);
			}

			WorldModelClientHandler::getInstance()->unlock();

			if(!gotAccess)
				boost::this_thread::sleep(boost::posix_time::seconds(2));
	}
}

void PathFinder::getAccessTripletExec(POI* poiFrom,POIAccessFrom::POIAccessFrom accDirFrom,LeaveDirection::LeaveDirection leaveDir){

	bool gotAccess = false;

	while(!gotAccess){
			if(ModelProvider::getInstance()->gameStateIsPaused()){
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				continue;
			}

			WorldModelClientHandler::getInstance()->lock();
			Node* startNode = grid->getNode(poiFrom->x,poiFrom->y);

			grid->clear(startNode);
			Node* accessNode = grid->getAccessNode(poiFrom,accDirFrom);
			POIDirection::POIDirection accessDirectionFrom = grid->getAccessDirection(poiFrom,accDirFrom);

			POIDirection::POIDirection neighborDir;


			if(leaveDir==LeaveDirection::LEFT){
				neighborDir = (POIDirection::POIDirection)((accessDirectionFrom+3)%4);
			}else{
				neighborDir = (POIDirection::POIDirection)((accessDirectionFrom+1)%4);
			}


			Node* neighbor =  startNode->getNeighbor(neighborDir);
			Node* accessNeighbor =   accessNode->getNeighbor(neighborDir);
			cout<<"X: "<<startNode->getX()<<" Y: "<<startNode->getY()<<" AD: "<<(int)accessDirectionFrom<<" ND: "<<(int)neighborDir<<endl;
			cout<<accessNeighbor<<endl;

			accessNode->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
			if((neighbor->getNodeRaw()->occupied == 0 || neighbor->getNodeRaw()->occupied == ModelProvider::getInstance()->getID()) &&
			   (accessNeighbor->getNodeRaw()->occupied == 0 || accessNeighbor->getNodeRaw()->occupied == ModelProvider::getInstance()->getID()))
			{
				neighbor->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
				accessNeighbor->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
				gotAccess = true;

				boost::shared_ptr<EvAccessTripletReserved> ev(new EvAccessTripletReserved());
				FileLog::log_NOTICE("[PathFinder] EvAccessTripletReserved");
				asyncStateMachine->queueEvent(ev);

			}

			WorldModelClientHandler::getInstance()->unlock();

			if(gotAccess==false){
				boost::this_thread::sleep(boost::posix_time::seconds(2));
			}
	}
}

void PathFinder::getAccessBackOfPOI(POI* poiFrom){

	findPathThread = new boost::thread(&PathFinder::getAccessBackOfPOIExec,this,poiFrom);
}

void PathFinder::getAccessTriplet(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, LeaveDirection::LeaveDirection leaveDir){

	findPathThread = new boost::thread(&PathFinder::getAccessTripletExec,this,poiFrom,accDirFrom,leaveDir);
}

void PathFinder::getAccessToDeliveryZone(){

	findPathThread = new boost::thread(&PathFinder::getAccessToDeliveryZoneExec,this);
}

void PathFinder::findPathFromProductionMachine(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom,POI* poiTo,POIAccessFrom::POIAccessFrom accDirTo){

	findPathThread = new boost::thread(&PathFinder::findPathFromProductionMachineExec,this,poiFrom,accDirFrom,poiTo,accDirTo);
}

void PathFinder::findPathTo(Node* startNode, POIDirection::POIDirection startPOIdir, POI* poi, POIAccessFrom::POIAccessFrom accDir, POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, const vector<Node*> blacklist){
	//TODO: ensure that only a thread is started if the other is terminated (consult Sebastian)
	findPathThread = new boost::thread(&PathFinder::findPathExec,this,poi,accDir,startNode,startPOIdir,poiFrom,accDirFrom, blacklist);
}

void PathFinder::findPathTo(POI* poi,POIAccessFrom::POIAccessFrom accDir, POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, const vector<Node*> blacklist){
	//TODO: ensure that only a thread is started if the other is terminated (consult Sebastian)
	findPathThread = new boost::thread(&PathFinder::findPathExec,this,poi,accDir,grid->getNodeByCoord(sensorServer->getX(),sensorServer->getY()),getDirection(sensorServer->getPhi()), poiFrom, accDirFrom, blacklist);
}

void PathFinder::setInputZoneYLeftCoordinate(float y){
	WorldModelClientHandler::getInstance()->lock();
	ModelProvider::getInstance()->getWorldModel()->inputyLeft = y;
	WorldModelClientHandler::getInstance()->unlock();
}

void PathFinder::setInputZoneYRightCoordinate(float y){
	WorldModelClientHandler::getInstance()->lock();
	ModelProvider::getInstance()->getWorldModel()->inputyRight = y;
	WorldModelClientHandler::getInstance()->unlock();
}

void PathFinder::increaseInputZonePucksGrabbed(){
	WorldModelClientHandler::getInstance()->lock();
	ModelProvider::getInstance()->getWorldModel()->pucksGrabbed++;
	WorldModelClientHandler::getInstance()->unlock();
}

float PathFinder::getInputZoneYLeftCoordinate(){
	return ModelProvider::getInstance()->getWorldModel()->inputyLeft;
}

float PathFinder::getInputZoneYRightCoordinate(){
	return ModelProvider::getInstance()->getWorldModel()->inputyRight;
}

int PathFinder::getInputZonePucksGrabbed() {
	return ModelProvider::getInstance()->getWorldModel()->pucksGrabbed;
}

vector<vec3D> PathFinder::convertExpandedPathToViaPoints(list<ExpandedNode> *expandedPath)
{
	list<vec3D> vias_list;
	vector<vec3D> vias;

	FileLog::log_NOTICE("[Pathfinder] Converting path to vias:");
	FileLog::log_NOTICE("[Pathfinder] Input: ", printPath(expandedPath));

	/*
	 * First step: make sure that expanded nodes with the same position but different orientation are collapsed, take the last node of this position
	 */
	{
		if(expandedPath->size() == 0)
		{
			return vias;
		}
		else if(expandedPath->size() == 1)
		{
			vias_list.push_back(vec3D(expandedPath->begin()->getNode()->getXPos(), expandedPath->begin()->getNode()->getYPos(), expandedPath->begin()->getPhi()));
		}
		else
		{
			list<ExpandedNode>::iterator it;
			it = expandedPath->begin();
			ExpandedNode *nodeInQuestion;
			nodeInQuestion = &*it;
			it++;
			for (; it != expandedPath->end(); it++ )
			{
				if(it->getNode()->getXPos() == nodeInQuestion->getNode()->getXPos() && it->getNode()->getYPos() == nodeInQuestion->getNode()->getYPos())
				{
					// current node has the same coordinates as nodeInQuestion --> update and try next
					//vias.push_back(nodeInQuestion.getXPos(), nodeInQuestion.getYPos(), getPhi(nodeInQuestion.getDirection()));
					nodeInQuestion = &*it;
				}
				else
				{
					vias_list.push_back(vec3D(nodeInQuestion->getNode()->getXPos(), nodeInQuestion->getNode()->getYPos(), getPhi(nodeInQuestion->getDirection())));
					nodeInQuestion = &*it;
				}
			}
			// always need to push the very last node, as no one left to trigger pushing the
			vias_list.push_back(vec3D(nodeInQuestion->getNode()->getXPos(), nodeInQuestion->getNode()->getYPos(), getPhi(nodeInQuestion->getDirection())));
		}
	}

	FileLog::log_NOTICE("[Pathfinder] After first step: ", printVec3D(vias_list));

	/*
	 * Second step: eliminate all nodes on a line, except the last node in the line
	 * (which also points into the new direction) and the node before the
	 * last node so that on longer straight segments rotation starts between this node and the corner node
	 */
	{
		if(vias_list.size() == 0)
		{
			return vias; // shouldn't/doesn't happen
		}
		else if(vias_list.size() == 1)
		{
			// do nothing, which means take this one via point as
		}
		else
		{
			list<vec3D> temp;
			list<vec3D>::iterator it;
			it = vias_list.begin();
			vec3D nodeInQuestion;
			nodeInQuestion = *it;
			it++;
			for(; it != --(vias_list.end()); it++)
			{
				if(it->phi != nodeInQuestion.phi)
				{
					if(temp.size() == 0)
					{
						// this is the node directly in front of the corner, which get's pushed because it isn't pushed, list is empty
						temp.push_back(nodeInQuestion);
					}
					else if(temp.back().x != nodeInQuestion.x || temp.back().y != nodeInQuestion.y || temp.back().phi != nodeInQuestion.phi)
					{
						temp.push_back(nodeInQuestion); // this is the node directly in front of the corner, which get's pushed if not already pushed
					}
					temp.push_back(*it); // this is the corner, always push it;
				}
				nodeInQuestion = *it;
			}
			// last node special case
			if(it->phi != nodeInQuestion.phi)
			{
				if(temp.size() == 0)
				{
					temp.push_back(nodeInQuestion); // this is the node directly in front of the last corner, which get's pushed because it isn't pushed, list is empty
				}
				else if(temp.back().x != nodeInQuestion.x || temp.back().y != nodeInQuestion.y || temp.back().phi != nodeInQuestion.phi)
				{
					temp.push_back(nodeInQuestion); // this is the node directly in front of the last corner, which get's pushed if not already pushed
				}
			}
			temp.push_back(*it); // this is the last, always push it;
			vias_list = temp;
		}
	}

	FileLog::log_NOTICE("[Pathfinder] After second step: ", printVec3D(vias_list));

	list<vec3D>::iterator it;
	it = vias_list.begin();
	for(; it != vias_list.end(); it++)
	{
		vias.push_back(*it);
	}
	return vias;
}

vector<Node*> PathFinder::convertViaPointsToNodes(vector<vec3D> vias)
{
	vector<Node*> nodes;
	Node* targetNode;
	Node* currNode;
	Node* lastNode;
	POIDirection::POIDirection lastNodeDir;

	if(vias.size() < 2)
	{
		FileLog::log_NOTICE("[Pathfinder] ERROR?! -> Via-list contains less than 2 elements?!");
		if(vias.size() == 1)
			nodes.push_back(grid->getNodeByCoord(vias.at(0).x, vias.at(0).y));
		return nodes;
	}

	lastNode = grid->getNodeByCoord(vias.at(0).x, vias.at(0).y);
	lastNodeDir = getDirection(vias.at(0).phi);

	for(unsigned int i=1; i<vias.size(); i++)
	{
		targetNode = grid->getNodeByCoord(vias.at(i).x, vias.at(i).y);

		currNode = lastNode;
		while(currNode != targetNode)
		{
			nodes.push_back(currNode);
			currNode = currNode->getNeighbor(lastNodeDir);
		}

		lastNode = targetNode;
		lastNodeDir = getDirection(vias.at(i).phi);
	}
	nodes.push_back(currNode);

	return nodes;
}

string PathFinder::printPath(list<ExpandedNode> *path)
{
	stringstream buf;
	list<ExpandedNode>::iterator it;
	buf << "list<ExpandedNode>: ";
	for(it=path->begin(); it!=path->end(); it++)
	{
		buf << it->toString();
	}
	buf << endl;
	return buf.str();
}

//void PathFinder::communicationTestExec(){
//
//	int counterToHell = 0;
//
//	while(true){
//
//		if(ModelProvider::getInstance()->gameStateIsPaused()){
//			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//			continue;
//		}
//
//		srand(time(NULL));
//
//		if(ModelProvider::getInstance()->getID() == ID::ROBO3)
//			counterToHell++;
//
//		cout << "START locking (" << counterToHell << ")" << endl;
//
//
//
//		WorldModelClientHandler::getInstance()->lock();
//
//		if(counterToHell == 10)
//		{
//			Node *node = NULL;
//			// ...
//		}
//
//
//		cout << "Process start" << endl;
//
//		ModelProvider::getInstance()->getWorldModel()->node[rand()%9][rand()%9].occupied = ModelProvider::getInstance()->getID();
//
//		boost::this_thread::sleep(boost::posix_time::milliseconds(rand()%1000+200));
//
//		cout << "Process end" << endl;
//
//
//		WorldModelClientHandler::getInstance()->unlock();
//
//		boost::this_thread::sleep(boost::posix_time::milliseconds(rand()%1000));
//	}
//
//}
//
//void PathFinder::communicationTest(){
//	commThread = new boost::thread(&PathFinder::communicationTestExec,this);
//}

