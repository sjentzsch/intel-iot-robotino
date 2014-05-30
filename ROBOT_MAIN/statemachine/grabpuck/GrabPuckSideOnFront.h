/*
 * GrabPuckSideOnFront.h
 *
 *  Created on: May 22, 2012
 *      Author: root
 */

#ifndef GRABPUCKSIDEONFRONT_H_
#define GRABPUCKSIDEONFRONT_H_

#include "../StateMachine.h"

//States
struct gbsofInit;
struct gbsofDrivingToLine;
struct gbsofDrivingForwardToMachine;
struct gbsofDrivingLeftToMachine;
struct gbsofDrivingRightToMachine;
struct gbsofDrivingSidewardsPuck;
struct gbsofDrivingForwardsPuck;
struct gbsofGotPuck;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    GrabPuckSideOnFront
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct GrabPuckSideOnFront : sc::state<GrabPuckSideOnFront,StateMachine1, gbsofInit>
{
	GrabPuckSideOnFront(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("GrabPuckSideOnFront");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		leftObstacleSeen = false;
		rightObstacleSeen = false;
		grid = context<StateMachine1>().getGrid();
		poiFrom = context<StateMachine1>().poiFrom;
		leaveDir = context<StateMachine1>().leaveDir;
	} // entry

	~GrabPuckSideOnFront() {
	} // exit

	void moveForward(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(40,0,0,120);
	}

	void moveForwardToLamp(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(400,0,0,120);
	}

	void driveToLine(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(230,0,0,120);
	}

	void driveToLamp(const EvCameraLampDetected& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.lampXPos, ev.lampYPos, stateBehavCtrl->getPathFinder()->getPhi(poiFrom->dir)-180, 100.0);
	}

	void driveSidewardsPuck() {
		float y = 200;

		if(leaveDir == LeaveDirection::LEFT)
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,y,0,150.0f);
		else
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,-y,0,150.0f);
	}

	void driveForwardsPuck(const EvMotorCtrlReady&){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(150,0,0,150.0f);
	}

	void moveRight(){
		stateBehavCtrl->getMotorCtrl()->terminate();
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,-250,0,70);
	}

	void moveLeft(){
		stateBehavCtrl->getMotorCtrl()->terminate();
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,250,0,70);
	}

	bool leftObstacleSeen;
	bool rightObstacleSeen;

	void calibrate()
	{
		Node *nodePOI = grid->getNode(poiFrom->x, poiFrom->y);
		stateBehavCtrl->getSensorControl()->calibrateOnMachineFront(nodePOI, poiFrom->dir);
		stateBehavCtrl->getSensorControl()->calibrateOnMachineSide(nodePOI, poiFrom->dir);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController *stateBehavCtrl;
	Grid *grid;
	POI* poiFrom;
	LeaveDirection::LeaveDirection leaveDir;
};

struct gbsofInit : sc::state< gbsofInit, GrabPuckSideOnFront>
{
	gbsofInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofInit");
		post_event(EvInit());
	} // entry

	~gbsofInit() {
	} // exit

	//Reactions

	sc::result react(const EvInit&)
	{
		context<GrabPuckSideOnFront>().driveToLine();
		return transit<gbsofDrivingToLine>();
	}

	//sc::transition<EvInit,gbsofDrivingToLine,GrabPuckSideOnFront,&GrabPuckSideOnFront::driveToLine>,
	typedef mpl::list<
		sc::custom_reaction<EvInit>,
		sc::deferral<EvAngleCalibration>
	> reactions;
};

struct gbsofDrivingToLine : sc::state< gbsofDrivingToLine, GrabPuckSideOnFront>
{
	gbsofDrivingToLine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofDrivingToLine");
	} // entry

	~gbsofDrivingToLine() {
	} // exit

	sc::result react(const EvAngleCalibration &ev)
	{
		if(ev.diffAngle != 0)
			context<StateMachine1>().getStateBehavController()->getSensorControl()->calibrateAngle(ev.lineAngle+ev.diffAngle);
		FileLog::log_NOTICE("###############################################");
		FileLog::log_NOTICE("--------------------> Angle Calibrated: diff Angle: ", FileLog::real(ev.diffAngle), ", corrected angle: ", FileLog::real(ev.lineAngle+ev.diffAngle));
		FileLog::log_NOTICE("###############################################");

		context<GrabPuckSideOnFront>().moveForwardToLamp();
		return transit<gbsofDrivingForwardToMachine>();
	}

	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<GrabPuckSideOnFront>().rightObstacleSeen){
			context<GrabPuckSideOnFront>().driveSidewardsPuck();
			return transit<gbsofDrivingSidewardsPuck>();
		}else{
			context<GrabPuckSideOnFront>().leftObstacleSeen = true;
			context<GrabPuckSideOnFront>().moveLeft();
			return transit<gbsofDrivingLeftToMachine>();
		}
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<GrabPuckSideOnFront>().leftObstacleSeen){
			context<GrabPuckSideOnFront>().driveSidewardsPuck();
			return transit<gbsofDrivingSidewardsPuck>();
		}else{
			context<GrabPuckSideOnFront>().rightObstacleSeen = true;
			context<GrabPuckSideOnFront>().moveRight();
			return transit<gbsofDrivingRightToMachine>();
		}
	}

	sc::result react(const EvMotorCtrlReady &ev)
	{
		context<GrabPuckSideOnFront>().moveForwardToLamp();
		return transit<gbsofDrivingForwardToMachine>();
		// TODO: change if align on lamp
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::custom_reaction<EvAngleCalibration>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>
	> reactions;
};

struct gbsofDrivingForwardToMachine : sc::state< gbsofDrivingForwardToMachine, GrabPuckSideOnFront>
{
	gbsofDrivingForwardToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofDrivingForwardToMachine");
	} // entry

	~gbsofDrivingForwardToMachine() {
	} // exit


	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<GrabPuckSideOnFront>().rightObstacleSeen){
			context<GrabPuckSideOnFront>().driveSidewardsPuck();
			return transit<gbsofDrivingSidewardsPuck>();
		}else{
			context<GrabPuckSideOnFront>().leftObstacleSeen = true;
			context<GrabPuckSideOnFront>().moveLeft();
			return transit<gbsofDrivingLeftToMachine>();
		}
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<GrabPuckSideOnFront>().leftObstacleSeen){
			context<GrabPuckSideOnFront>().driveSidewardsPuck();
			return transit<gbsofDrivingSidewardsPuck>();
		}else{
			context<GrabPuckSideOnFront>().rightObstacleSeen = true;
			context<GrabPuckSideOnFront>().moveRight();
			return transit<gbsofDrivingRightToMachine>();
		}
	}

	sc::result react(const EvMotorCtrlReady&)
	{
		context<GrabPuckSideOnFront>().driveSidewardsPuck();
		return transit<gbsofDrivingSidewardsPuck>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>,
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct gbsofDrivingLeftToMachine : sc::state< gbsofDrivingLeftToMachine, GrabPuckSideOnFront>
{
	gbsofDrivingLeftToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofDrivingLeftToMachine");
	} // entry

	~gbsofDrivingLeftToMachine() {
	} // exit


	sc::result react(const EvSensorFrontLeftIsFree&)
	{
		//context<GrabPuckSideOnFront>().leftObstacleSeen = false;
		context<GrabPuckSideOnFront>().leftObstacleSeen = true;
		context<GrabPuckSideOnFront>().moveRight();
		return transit<gbsofDrivingRightToMachine>();
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<GrabPuckSideOnFront>().leftObstacleSeen){
			context<GrabPuckSideOnFront>().driveSidewardsPuck();
			return transit<gbsofDrivingSidewardsPuck>();
		}else{
			context<GrabPuckSideOnFront>().rightObstacleSeen = true;
			context<GrabPuckSideOnFront>().moveForward();
			return transit<gbsofDrivingForwardToMachine>();
		}
	}


	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftIsFree>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>
	> reactions;

};

struct gbsofDrivingRightToMachine : sc::state< gbsofDrivingRightToMachine, GrabPuckSideOnFront>
{
	gbsofDrivingRightToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofDrivingRightToMachine");
	} // entry

	~gbsofDrivingRightToMachine() {
	} // exit


	sc::result react(const EvSensorFrontRightIsFree&)
	{
		//context<GrabPuckSideOnFront>().rightObstacleSeen = false;
		context<GrabPuckSideOnFront>().rightObstacleSeen = true;
		context<GrabPuckSideOnFront>().moveLeft();
		return transit<gbsofDrivingLeftToMachine>();
	}

	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<GrabPuckSideOnFront>().rightObstacleSeen){
			context<GrabPuckSideOnFront>().driveSidewardsPuck();
			return transit<gbsofDrivingSidewardsPuck>();
		}else{
			context<GrabPuckSideOnFront>().leftObstacleSeen = true;
			context<GrabPuckSideOnFront>().moveForward();
			return transit<gbsofDrivingForwardToMachine>();
		}
	}


	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontRightIsFree>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>
	> reactions;
};

struct gbsofDrivingSidewardsPuck : sc::state< gbsofDrivingSidewardsPuck, GrabPuckSideOnFront>
{
	gbsofDrivingSidewardsPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofDrivingSidewardsPuck");
	} // entry

	~gbsofDrivingSidewardsPuck() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, gbsofDrivingForwardsPuck, GrabPuckSideOnFront, &GrabPuckSideOnFront::driveForwardsPuck>
	> reactions;
};
//TODO: use camera to locate and grab the puck instead ?
struct gbsofDrivingForwardsPuck : sc::state< gbsofDrivingForwardsPuck, GrabPuckSideOnFront>
{
	gbsofDrivingForwardsPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofDrivingForwardsPuck");
	} // entry

	~gbsofDrivingForwardsPuck() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, gbsofGotPuck>
	> reactions;
};

struct gbsofGotPuck : sc::state< gbsofGotPuck, GrabPuckSideOnFront>
{
	gbsofGotPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gbsofGotPuck");
		post_event(EvSuccess());
	} // entry

	~gbsofGotPuck() {
	} // exit

};

#endif /* GRABPUCKSIDEONFRONT_H_ */
