/*
 * PathFinder.h
 *
 *  Created on: Jun 4, 2011
 *      Author: root
 */

#ifndef PATHFINDER_H_
#define PATHFINDER_H_

#include <boost/thread.hpp>
#include "config.h"
#include "model/WorldModel.h"
#include "Grid.h"
#include "../StateMachineEvents.h"
#include "../AsyncStateMachine.h"
#include "../SensorServer.h"
#include <vector>
#include "../LSPBTrajectory/pose.h"
#include <string>
#include <sstream>


using namespace std;

class PathFinder {
private:
	Grid* grid;
	void findPathExec(POI* poi,POIAccessFrom::POIAccessFrom accDir, Node* startNode, POIDirection::POIDirection startPOIdir, POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom, const vector<Node*> blacklist = vector<Node*>());
	void findPathFromProductionMachineExec(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom,POI* poiTo,POIAccessFrom::POIAccessFrom accDirTo);
	void getAccessBackOfPOIExec(POI* poiFrom);
	void getAccessTripletExec(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom,LeaveDirection::LeaveDirection leaveDir);

	void getAccessToDeliveryZoneExec();

	boost::shared_ptr<boost::statechart::event_base> event;

	POI* randomPOI;

	boost::thread *commThread;
	boost::thread *findPathThread; //thread for finding the path
	AsyncStateMachine* asyncStateMachine;
	SensorServer* sensorServer;
	POIDirection::POIDirection getDirection(float rotation);

	bool reserveZone(int startx, int starty, ID::ID id_, int field_x, int field_y);
	vector<vec3D> convertExpandedPathToViaPoints(list<ExpandedNode> *expandedPath);

	void communicationTestExec();

public:
	PathFinder(AsyncStateMachine* asyncStateMachine,SensorServer* sensorServer);
	~PathFinder();

	POI* moveABit();
	float getPhi(POIDirection::POIDirection);

	vector<Node*> convertViaPointsToNodes(vector<vec3D> vias);

	//tries to find a path from the given coordinates to the given POI and triggers eventually an EvPathFound or EvNoPathFound
	void findPathTo(Node* startNode, POIDirection::POIDirection startPOIdir, POI* poi,POIAccessFrom::POIAccessFrom accDir=POIAccessFrom::FRONT, POI* poiFrom=NULL, POIAccessFrom::POIAccessFrom accDirFrom=POIAccessFrom::FRONT, const vector<Node*> blacklist = vector<Node*>());
	void findPathTo(POI* poi,POIAccessFrom::POIAccessFrom accDir=POIAccessFrom::FRONT, POI* poiFrom=NULL, POIAccessFrom::POIAccessFrom accDirFrom=POIAccessFrom::FRONT, const vector<Node*> blacklist = vector<Node*>());

	void communicationTest();

	void findPathFromProductionMachine(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom,POI* poiTo,POIAccessFrom::POIAccessFrom accDirTo);

	void getAccessBackOfPOI(POI* poiFrom);
	void getAccessTriplet(POI* poiFrom, POIAccessFrom::POIAccessFrom accDirFrom,LeaveDirection::LeaveDirection leaveDir);

	bool isNextRelevantInSameNode(int x, int y);
	bool isNextRelevantInSameNodeMM(float gridXPos, float gridYPos);
	bool isNextRelevantInSameNodeMM(float gridXPos, float gridYPos, float gridPhi);

	void getAccessToDeliveryZone();

	void setInputZoneYLeftCoordinate(float y);
	float getInputZoneYLeftCoordinate();

	void setInputZoneYRightCoordinate(float y);
	float getInputZoneYRightCoordinate();

	void increaseInputZonePucksGrabbed();
	int getInputZonePucksGrabbed();

	string printPath(list<ExpandedNode> *path);
};

#endif /* PATHFINDER_H_ */
