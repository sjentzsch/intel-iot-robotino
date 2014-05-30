/*
 * LeavePoiBackwards.h
 *
 *  Created on: Jun 12, 2011
 *      Author: root
 */

#ifndef LEAVEPOIWITHOUTPUCK_H_
#define LEAVEPOIWITHOUTPUCK_H_

#include "../StateMachine.h"

//States
struct lpwopInit; //entry state
struct lpwopWaitingForAccessTriplet;
struct lpwopPreparePath;
struct lpwopWaitingForPath;
struct lpwopMovingSideForwards;
struct lpwopLeftPoi; //final state

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    LeavePoiWithouPuck
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct LeavePoiWithoutPuck : sc::state<LeavePoiWithoutPuck,StateMachine1, lpwopInit>
{
	LeavePoiWithoutPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("LeavePoiWithoutPuck");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		poiFrom = context<StateMachine1>().poiFrom;
		accessDirectionFrom = context<StateMachine1>().accessDirectionFrom;
		leaveDir = context<StateMachine1>().leaveDir;
		speed = 250;

		if(poiFrom->x == X_GRID_WIDTH){
			if((leaveDir == LeaveDirection::LEFT && poiFrom->dir==POIDirection::SOUTH) ||
			   (leaveDir == LeaveDirection::RIGHT && poiFrom->dir==POIDirection::NORTH))
				sideIsWall = true;
			else
				sideIsWall = false;
		}
	} // entry

	~LeavePoiWithoutPuck() {
	} // exit


	void reserveAccessTriplet(){
		stateBehavCtrl->getPathFinder()->getAccessTriplet(poiFrom,accessDirectionFrom,leaveDir);
	}

	void findPath(){
		Grid* grid = context<StateMachine1>().getGrid();
		if(leaveDir == LeaveDirection::LEFT)
			stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::LEFT), grid->getAccessDirection(poiFrom, POIAccessFrom::LEFT), context<StateMachine1>().poiTo, context<StateMachine1>().accessDirectionTo, context<StateMachine1>().poiFrom, context<StateMachine1>().accessDirectionFrom);
		else
			stateBehavCtrl->getPathFinder()->findPathTo(grid->getAccessNode(poiFrom, POIAccessFrom::RIGHT), grid->getAccessDirection(poiFrom, POIAccessFrom::RIGHT), context<StateMachine1>().poiTo, context<StateMachine1>().accessDirectionTo, context<StateMachine1>().poiFrom, context<StateMachine1>().accessDirectionFrom);
	}

	void moveSideForwards(){

		vector<vec3D> vPoints;

		Grid *grid = context<StateMachine1>().getGrid();
		Node *nodePOI = grid->getNode(context<StateMachine1>().poiFrom->x, context<StateMachine1>().poiFrom->y);
		POIDirection::POIDirection directionPOI = context<StateMachine1>().poiFrom->dir;

		if(leaveDir == LeaveDirection::LEFT)
		{
			vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, 0, 170));
			vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, 80, 170));
			vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, -20, 170));
		}
		else
		{
			vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, 0, -170));
			vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, 80, -170));
			vPoints.push_back(stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, -20, -170));
		}

		vPoints.push_back(leavePoint);

		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(vPoints,speed,60.0f);
	}

	LeaveDirection::LeaveDirection leaveDir;
	POI* poiFrom;
	vec3D leavePoint;
	bool sideIsWall;
	StateBehaviorController *stateBehavCtrl;

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	POIAccessFrom::POIAccessFrom accessDirectionFrom;
	float speed;
};


struct lpwopInit : sc::state< lpwopInit, LeavePoiWithoutPuck>
{
	lpwopInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwopInit");
		post_event(EvInit());
	} // entry

	~lpwopInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		if(!context<LeavePoiWithoutPuck>().sideIsWall)
		{
			context<LeavePoiWithoutPuck>().reserveAccessTriplet();
			return transit<lpwopWaitingForAccessTriplet>();
		}
		else
		{
			return transit<lpwopPreparePath>();
		}
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct lpwopWaitingForAccessTriplet : sc::state< lpwopWaitingForAccessTriplet, LeavePoiWithoutPuck>
{
	lpwopWaitingForAccessTriplet(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwopWaitingForAccessTriplet");
	} // entry

	~lpwopWaitingForAccessTriplet() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvAccessTripletReserved, lpwopPreparePath>
	> reactions;
};

struct lpwopPreparePath : sc::state< lpwopPreparePath, LeavePoiWithoutPuck>
{
	lpwopPreparePath(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwopPreparePath");
		post_event(EvInit());
	} // entry

	~lpwopPreparePath() {
	} // exit

	sc::result react(const EvInit&)
	{
		Grid *grid = context<StateMachine1>().getGrid();

		if(context<StateMachine1>().poiTo == NULL)
		{
			Node *nodeBackPOI = grid->getAccessNode(context<LeavePoiWithoutPuck>().poiFrom, POIAccessFrom::FRONT);
			context<LeavePoiWithoutPuck>().leavePoint.set(nodeBackPOI->getXPos(), nodeBackPOI->getYPos(), context<LeavePoiWithoutPuck>().stateBehavCtrl->getPathFinder()->getPhi(grid->getAccessDirection(context<LeavePoiWithoutPuck>().poiFrom, POIAccessFrom::FRONT)));

			context<LeavePoiWithoutPuck>().moveSideForwards();
			return transit<lpwopMovingSideForwards>();
		}
		else if(context<LeavePoiWithoutPuck>().sideIsWall)
		{
			Node *nodeBackPOI = grid->getAccessNode(context<LeavePoiWithoutPuck>().poiFrom, POIAccessFrom::FRONT);
			float rotation = 0.0f;
			if(context<LeavePoiWithoutPuck>().poiFrom->y == 8)
				rotation = context<LeavePoiWithoutPuck>().stateBehavCtrl->getPathFinder()->getPhi(POIDirection::WEST);
			else if(context<LeavePoiWithoutPuck>().poiFrom->y == 0)
				rotation = context<LeavePoiWithoutPuck>().stateBehavCtrl->getPathFinder()->getPhi(POIDirection::EAST);
			context<LeavePoiWithoutPuck>().leavePoint.set(nodeBackPOI->getXPos(), nodeBackPOI->getYPos(), rotation);

			context<LeavePoiWithoutPuck>().moveSideForwards();
			return transit<lpwopMovingSideForwards>();
		}
		else
		{
			context<LeavePoiWithoutPuck>().findPath();
			return transit<lpwopWaitingForPath>();
		}
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct lpwopWaitingForPath : sc::state< lpwopWaitingForPath, LeavePoiWithoutPuck>
{
	lpwopWaitingForPath(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwopWaitingForPath");
	} // entry

	~lpwopWaitingForPath() {
	} // exit

	sc::result react(const EvPathFound &ev)
	{
		context<StateMachine1>().pathToPOI->x = ev.x;
		context<StateMachine1>().pathToPOI->y = ev.y;
		context<StateMachine1>().pathToPOI->rotation = ev.rotation;
		context<StateMachine1>().pathToPOI->vias = ev.vias;
		context<StateMachine1>().havePathToPOI = false;	// true -> false because we have no puck, thus we can try to find a more efficient path after leaving

		context<LeavePoiWithoutPuck>().leavePoint.set(ev.x, ev.y, ev.rotation);
		context<LeavePoiWithoutPuck>().moveSideForwards();
		return transit<lpwopMovingSideForwards>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvPathFound>
		//sc::transition<EvPathDriven,lpbLeft>	// needed?
	> reactions;
};

struct lpwopMovingSideForwards : sc::state< lpwopMovingSideForwards, LeavePoiWithoutPuck>
{
	lpwopMovingSideForwards(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwopMovingSideForwards");
	} // entry

	~lpwopMovingSideForwards() {
	} // exit

	/*sc::result react(const EvMotorCtrlReady&)
	{
		if((context<LeavePoiWithoutPuck>().leaveDir == LeaveDirection::LEFT && ((context<LeavePoiWithoutPuck>().poiFrom->dir == POIDirection::NORTH && context<LeavePoiWithoutPuck>().poiFrom->y==6) || (context<LeavePoiWithoutPuck>().poiFrom->dir == POIDirection::SOUTH && context<LeavePoiWithoutPuck>().poiFrom->y==2))) ||
		   (context<LeavePoiWithoutPuck>().leaveDir == LeaveDirection::RIGHT && ((context<LeavePoiWithoutPuck>().poiFrom->dir == POIDirection::SOUTH && context<LeavePoiWithoutPuck>().poiFrom->y==6) || (context<LeavePoiWithoutPuck>().poiFrom->dir == POIDirection::NORTH && context<LeavePoiWithoutPuck>().poiFrom->y==2))))
		{
			context<LeavePoiWithoutPuck>().moveBackwardsSidewards();
			return transit<MovingBackwardsSidewards>();
		}
		else
		{
			context<LeavePoiWithoutPuck>().moveSidewardsWithoutPuck();
			return transit<MovingSidewardsWithoutPuck>();
		}
	}*/

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, lpwopLeftPoi>
	> reactions;
};


struct lpwopLeftPoi : sc::state< lpwopLeftPoi, LeavePoiWithoutPuck>
{
	lpwopLeftPoi(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpwopLeftPoi");
		post_event(EvSuccess());
	} // entry

	~lpwopLeftPoi() {
	} // exit

};


#endif /* LEAVEPOIWITHPUCK_H_ */
