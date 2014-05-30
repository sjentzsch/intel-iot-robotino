/*
 * LeavePoiWithPuckRotatingOut.h
 *
 *  Created on: May 22, 2012
 *      Author: root
 */

#ifndef LeavePoiWithPuckRotatingOut_H_
#define LeavePoiWithPuckRotatingOut_H_

#include "../StateMachine.h"

//States
struct lpproInit; //entry state
struct lpproWaitingForAccess;
struct lpproRotating;
struct lpproMovingForwardOut;
struct lpproLeftPOI; //final state

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    LeavePoiWithPuckRotatingOut, used for retrieving recycle-pucks from machines near wall
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct LeavePoiWithPuckRotatingOut : sc::state<LeavePoiWithPuckRotatingOut,StateMachine1, lpproInit>
{
	LeavePoiWithPuckRotatingOut(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("LeavePoiWithPuckRotatingOut");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		poiFrom = context<StateMachine1>().poiFrom;
		grid = context<StateMachine1>().getGrid();
	} // entry

	~LeavePoiWithPuckRotatingOut() {
	} // exit

	void getAccess(const EvInit&){
		grid->getAccessNode(poiFrom,POIAccessFrom::FRONT);
	}

	void rotate(const EvAccessTripletReserved&){
		 //are we left or right of the machine?, TODO: hack, do properly in relation to position
		if((poiFrom->index==24 || poiFrom->index==8))
			stateBehavCtrl->getMotorCtrl()->rotateToRelAngle(180,ForceRotationDirection::LEFT,90);
		else if(poiFrom->index == 20 || poiFrom->index == 12)
			stateBehavCtrl->getMotorCtrl()->rotateToRelAngle(180,ForceRotationDirection::RIGHT,90);
	}

	void forwardOut(const EvMotorCtrlReady&){
		if((poiFrom->index==24 || poiFrom->index==8))
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(150,150,0,200);
		else if(poiFrom->index == 20 || poiFrom->index == 12)
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(150,-150,0,200);
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


struct lpproInit : sc::state< lpproInit, LeavePoiWithPuckRotatingOut>
{
	lpproInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpproInit");
		post_event(EvInit());
	} // entry

	~lpproInit() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvInit,lpproWaitingForAccess,LeavePoiWithPuckRotatingOut,&LeavePoiWithPuckRotatingOut::getAccess>
	> reactions;
};

struct lpproWaitingForAccess : sc::state< lpproWaitingForAccess, LeavePoiWithPuckRotatingOut>
{
	lpproWaitingForAccess(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpproWaitingForAccess");
	} // entry

	~lpproWaitingForAccess() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvAccessTripletReserved,lpproRotating,LeavePoiWithPuckRotatingOut,&LeavePoiWithPuckRotatingOut::rotate>
	> reactions;
};

struct lpproRotating : sc::state< lpproRotating, LeavePoiWithPuckRotatingOut>
{
	lpproRotating(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpproRotating");
	} // entry

	~lpproRotating() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady,lpproMovingForwardOut,LeavePoiWithPuckRotatingOut,&LeavePoiWithPuckRotatingOut::forwardOut>
	> reactions;
};

struct lpproMovingForwardOut : sc::state< lpproMovingForwardOut, LeavePoiWithPuckRotatingOut>
{
	lpproMovingForwardOut(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpproMovingForwardOut");
	} // entry

	~lpproMovingForwardOut() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady,lpproLeftPOI>
	> reactions;
};

struct lpproLeftPOI : sc::state< lpproLeftPOI, LeavePoiWithPuckRotatingOut>
{
	lpproLeftPOI(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("lpproLeftPOI");
		post_event(EvSuccess());
	} // entry

	~lpproLeftPOI() {
	} // exit

};

#endif /* LeavePoiWithPuckRotatingOut_H_ */
