/*
 * LeavePoiBackwards.h
 *
 *  Created on: Jun 12, 2011
 *      Author: root
 */

#ifndef LEAVEPOIBACKWARDS_H_
#define LEAVEPOIBACKWARDS_H_

#include "../StateMachine.h"

//States
struct lpbInit; //entry state
struct lpbWaitingPath;
struct lpbDrivingToNode;
struct lpbLeft; //final state

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    LeavePoiBackwards
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct LeavePoiBackwards : sc::state<LeavePoiBackwards,StateMachine1, lpbInit>
{
	LeavePoiBackwards(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("LeavePoiBackwards");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		poiFrom = context<StateMachine1>().poiFrom;
		grid = context<StateMachine1>().getGrid();
	} // entry

	~LeavePoiBackwards() {
	} // exit

	void init(const EvInit&)
	{
		//stateBehavCtrl->getMotorCtrl()->terminate();

		// only use this method if we have a job
		if(context<StateMachine1>().poiTo != NULL)
			stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::FRONT), grid->getAccessDirection(poiFrom, POIAccessFrom::FRONT), context<StateMachine1>().poiTo, context<StateMachine1>().accessDirectionTo, context<StateMachine1>().poiFrom, POIAccessFrom::FRONT);
		else
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(-150,0,0,200);
	}

	void driveToNode(const EvPathFound &ev)
	{
		context<StateMachine1>().pathToPOI->x = ev.x;
		context<StateMachine1>().pathToPOI->y = ev.y;
		context<StateMachine1>().pathToPOI->rotation = ev.rotation;
		context<StateMachine1>().pathToPOI->vias = ev.vias;
		context<StateMachine1>().havePathToPOI = false;		// true -> false because we have no puck, thus we can try to find a more efficient path after leaving

		vector<vec3D> vPoints;
		Node *nodePOI = grid->getNode(context<StateMachine1>().poiFrom->x, context<StateMachine1>().poiFrom->y);
		POIDirection::POIDirection directionPOI = context<StateMachine1>().poiFrom->dir;
		vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, -100, 0));
		vPoints.push_back(vec3D(ev.x,ev.y,ev.rotation));
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(vPoints,600.0f,60.0f);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController *stateBehavCtrl;
	POI* poiFrom;
	Grid* grid;
};


struct lpbInit : sc::state< lpbInit, LeavePoiBackwards>
{
	lpbInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpbInit");
		post_event(EvInit());
	} // entry

	~lpbInit() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvInit,lpbWaitingPath,LeavePoiBackwards,&LeavePoiBackwards::init>
	> reactions;
};

struct lpbWaitingPath : sc::state< lpbWaitingPath, LeavePoiBackwards>
{
	lpbWaitingPath(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpbWaitingPath");
	} // entry

	~lpbWaitingPath() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvPathFound,lpbDrivingToNode, LeavePoiBackwards, &LeavePoiBackwards::driveToNode>,
		sc::transition<EvPathDriven,lpbLeft>,
		sc::transition<EvMotorCtrlReady,lpbLeft>
	> reactions;
};

struct lpbDrivingToNode : sc::state< lpbDrivingToNode, LeavePoiBackwards>
{
	lpbDrivingToNode(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpbDrivingToNode");
	} // entry

	~lpbDrivingToNode() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady,lpbLeft>
	> reactions;
};

struct lpbLeft : sc::state< lpbLeft, LeavePoiBackwards>
{
	lpbLeft(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpbLeft");
		post_event(EvSuccess());
	} // entry

	~lpbLeft() {
	} // exit
};


#endif /* LEAVEPOIBACKWARDS_H_ */
