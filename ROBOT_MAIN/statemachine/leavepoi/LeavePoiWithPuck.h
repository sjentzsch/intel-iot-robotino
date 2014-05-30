/*
 * LeavePoiBackwards.h
 *
 *  Created on: Jun 12, 2011
 *      Author: root
 */

#ifndef LEAVEPOIWITHPUCK_H_
#define LEAVEPOIWITHPUCK_H_

#include "../StateMachine.h"

//States
struct lpwpInit;
struct lpwpChoosingDirection;
struct lpwpWaitingForPath;
struct lpwpLeavingWithPuck;
struct lpwpLeftPoi;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    LeavePoiWithPuck
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct LeavePoiWithPuck : sc::state<LeavePoiWithPuck,StateMachine1, lpwpInit>
{
	LeavePoiWithPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("LeavePoiWithPuck");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		poiFrom = context<StateMachine1>().poiFrom;
		accessDirectionFrom = context<StateMachine1>().accessDirectionFrom;
		poiTo = context<StateMachine1>().poiTo;
		accessDirectionTo = context<StateMachine1>().accessDirectionTo;
	} // entry

	~LeavePoiWithPuck() {
	} // exit

	void init(const EvInit&){
		stateBehavCtrl->getPathFinder()->findPathFromProductionMachine(poiFrom,accessDirectionFrom,poiTo,accessDirectionTo);
	}

	void pathFound(const EvPathFoundLeavePoi& event) {
		leaveDir = event.leaveDir;
		Grid* grid = context<StateMachine1>().getGrid();

		if(poiTo->type != POIType::RECYCLE){
					if(leaveDir == LeaveDirection::LEFT)
						stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::LEFT), grid->getAccessDirection(poiFrom, POIAccessFrom::LEFT), poiTo, accessDirectionTo, poiFrom, accessDirectionFrom);
					else
						stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::RIGHT), grid->getAccessDirection(poiFrom, POIAccessFrom::RIGHT), poiTo, accessDirectionTo, poiFrom, accessDirectionFrom);
				}
		else{
			if(accessDirectionFrom == POIAccessFrom::LEFT)
			{
				if(leaveDir == LeaveDirection::LEFT)
					stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::BACK), grid->getAccessDirection(poiFrom, POIAccessFrom::FRONT), poiTo, accessDirectionTo, poiFrom, accessDirectionFrom);
				else
					stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::FRONT), grid->getAccessDirection(poiFrom, POIAccessFrom::FRONT), poiTo, accessDirectionTo, poiFrom, accessDirectionFrom);
			}
			else
			{
				if(leaveDir == LeaveDirection::LEFT)
					stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::FRONT), grid->getAccessDirection(poiFrom, POIAccessFrom::FRONT), poiTo, accessDirectionTo, poiFrom, accessDirectionFrom);
				else
					stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::BACK), grid->getAccessDirection(poiFrom, POIAccessFrom::FRONT), poiTo, accessDirectionTo, poiFrom, accessDirectionFrom);
			}
		}
	}

	void leaveWithPuck(const EvPathFound& event) {

		context<StateMachine1>().pathToPOI->x = event.x;
		context<StateMachine1>().pathToPOI->y = event.y;
		context<StateMachine1>().pathToPOI->rotation = event.rotation;
		context<StateMachine1>().pathToPOI->vias = event.vias;
		context<StateMachine1>().havePathToPOI = true;

		Grid *grid = context<StateMachine1>().getGrid();
		Node *nodePOI = grid->getNode(poiFrom->x, poiFrom->y);
		POIDirection::POIDirection directionPOI = poiFrom->dir;

		vector<vec3D> vPoints;

		if(leaveDir == LeaveDirection::LEFT)
		{
			vPoints.push_back(stateBehavCtrl->getMotorCtrl()->relToAbs(vec3D(0, 260, 0)));
			vPoints.push_back(vec3D(event.x,event.y,event.rotation));
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(vPoints,250,40,false,ForceRotationDirection::LEFT);
		}
		else
		{
			vPoints.push_back(stateBehavCtrl->getMotorCtrl()->relToAbs(vec3D(0, -260, 0)));
			vPoints.push_back(vec3D(event.x,event.y,event.rotation));
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(vPoints,250,40,false,ForceRotationDirection::RIGHT);
		}
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController *stateBehavCtrl;
	POI* poiFrom;
	POIAccessFrom::POIAccessFrom accessDirectionFrom;
	POI* poiTo;
	POIAccessFrom::POIAccessFrom accessDirectionTo;
	LeaveDirection::LeaveDirection leaveDir;
};


struct lpwpInit : sc::state< lpwpInit, LeavePoiWithPuck>
{
	lpwpInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwpInit");
		post_event(EvInit());
	} // entry

	~lpwpInit() {
	} // exit

	//Reactions
	typedef mpl::list<
			sc::transition<EvInit, lpwpChoosingDirection, LeavePoiWithPuck, &LeavePoiWithPuck::init>
	> reactions;
};


struct lpwpChoosingDirection : sc::state< lpwpChoosingDirection, LeavePoiWithPuck>
{
	lpwpChoosingDirection(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwpChoosingDirection");
	} // entry

	~lpwpChoosingDirection() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvPathFoundLeavePoi, lpwpWaitingForPath,LeavePoiWithPuck, &LeavePoiWithPuck::pathFound>
	> reactions;
};

struct lpwpWaitingForPath : sc::state< lpwpWaitingForPath, LeavePoiWithPuck>
{
	lpwpWaitingForPath(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwpWaitingForPath");
	} // entry

	~lpwpWaitingForPath() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvPathFound, lpwpLeavingWithPuck,LeavePoiWithPuck, &LeavePoiWithPuck::leaveWithPuck>
		//sc::transition<EvPathDriven,lpbLeft>	// needed?
	> reactions;
};

struct lpwpLeavingWithPuck : sc::state< lpwpLeavingWithPuck, LeavePoiWithPuck>
{
	lpwpLeavingWithPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwpLeavingWithPuck");
	} // entry

	~lpwpLeavingWithPuck() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, lpwpLeftPoi>
	> reactions;
};


struct lpwpLeftPoi : sc::state< lpwpLeftPoi, LeavePoiWithPuck>
{
	lpwpLeftPoi(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwpLeftPoi");
		post_event(EvSuccess());
	} // entry

	~lpwpLeftPoi() {
	} // exit

};


#endif /* LEAVEPOIWITHPUCK_H_ */
