/*
 * DeliverPuckToGate
 *
 *  Created on: Jun 11, 2011
 *      Author: root
 */

#ifndef DELIVERPUCKTOGATE_H_
#define DELIVERPUCKTOGATE_H_

#include "../StateMachine.h"
#include "BaseParameterProvider.h"

//States
struct dptgInit;
struct dptgWaitingForAccess;
struct dptgDrivingToMid;
struct dptgDrivingToLeft;
struct dptgDrivingToRight;
struct dptgDrivingToLamp;
struct dptgDrivingForwardToMachine;
struct dptgDrivingLeftToMachine;
struct dptgDrivingRightToMachine;
struct dptgDrivingBack;
struct dptgDrivingToExitLocation;
struct dptgDelivered;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    DeliverPuckToGate
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct DeliverPuckToGate : sc::state<DeliverPuckToGate,StateMachine1, dptgInit>
{
	DeliverPuckToGate(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("DeliverPuckToGate");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		leftObstacleSeen = false;
		rightObstacleSeen = false;
		accessNode = context<StateMachine1>().accessNode;
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setHavingPuck(true);
		stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::SEARCH_GATE);
	} // entry

	~DeliverPuckToGate() {
		stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	void calibrate() {
		context<DeliverPuckToGate>().stop();
		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
			stateBehavCtrl->getSensorControl()->calibrateOnLineY(580);
		else
			stateBehavCtrl->getSensorControl()->calibrateOnLineY(10620);
	}

	void getAccess(const EvInit&){
		//stateBehavCtrl->getPathFinder()->getAccessToDeliveryZone();
	}

	void driveToMid() {
		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos(), accessNode->getYPos()-200, -90, 100.0);
		else
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos(), accessNode->getYPos()+200, 90, 100.0);
	}

	void driveToLeft() {
		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
			stateBehavCtrl->getMotorCtrl()->moveToAbsXPosRelYPos(accessNode->getXPos()+450, -100, -90, 200.0);
		else
			stateBehavCtrl->getMotorCtrl()->moveToAbsXPosRelYPos(accessNode->getXPos()-450, 100, 90, 200.0);
	}

	void driveToRight() {
		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
			stateBehavCtrl->getMotorCtrl()->moveToAbsXPosRelYPos(accessNode->getXPos()-450, -100, -90, 200.0);
		else
			stateBehavCtrl->getMotorCtrl()->moveToAbsXPosRelYPos(accessNode->getXPos()+450, 100, 90, 200.0);
	}

	void moveForward(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(40,0,0,150);
	}

	void driveForwardToMachine() {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(300,0,0,150);
	}

	void driveToLamp(const EvCameraLampDetected& ev) {
		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.lampXPos, ev.lampYPos, -90, 300.0);
		else
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.lampXPos, ev.lampYPos, 90, 300.0);

		//if((ev.lampXPos-70) < )
		//{
			/*float myY = stateBehavCtrl->getSensorControl()->getRobotY();
			if(ev.lampYPos > myY)
				stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.lampXPos-60, ev.lampYPos-30, 0, 200.0);
			else
				stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.lampXPos-60, ev.lampYPos+30, 0, 200.0);*/
		//}
	}

	void moveRight(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,-250,0,120);
	}

	void moveLeft(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,250,0,120);
	}

	void stop(){
		stateBehavCtrl->getMotorCtrl()->terminate();
	}

	void driveBack(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(-150,0,0,200);
	}

	void driveToExitLocation(const EvMotorCtrlReady&){
		if(BaseParameterProvider::getInstance()->getParams()->team_number == 1)
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos(), accessNode->getYPos(), 90, 400, 120); // rot was default (=40)
		else
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos(), accessNode->getYPos(), -90, 400, 120); // rot was default (=40)
	}


	bool leftObstacleSeen;
	bool rightObstacleSeen;

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController *stateBehavCtrl;
	Node* accessNode;
};

struct dptgInit : sc::state< dptgInit, DeliverPuckToGate>
{
	dptgInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgInit");
		post_event(EvInit());
	} // entry

	~dptgInit() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::deferral<EvCameraLampDetected>,
		sc::transition<EvInit, dptgWaitingForAccess, DeliverPuckToGate, &DeliverPuckToGate::getAccess>
	> reactions;
};

struct dptgWaitingForAccess : sc::state< dptgWaitingForAccess, DeliverPuckToGate>
{
	dptgWaitingForAccess(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgWaitingForAccess");
	} // entry

	~dptgWaitingForAccess() {
	} // exit

	sc::result react(const EvAccessZoneReserved&)
	{
		context<DeliverPuckToGate>().driveToMid();
		return transit<dptgDrivingToMid>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvAccessZoneReserved>,
		sc::deferral<EvCameraLampDetected>
	> reactions;
};

struct dptgDrivingToMid : sc::state<dptgDrivingToMid, DeliverPuckToGate>
{
	dptgDrivingToMid(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToMid");
	} // entry

	~dptgDrivingToMid() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<DeliverPuckToGate>().driveToLeft();
		return transit<dptgDrivingToLeft>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::transition<EvCameraLampDetected, dptgDrivingToLamp, DeliverPuckToGate, &DeliverPuckToGate::driveToLamp>
	> reactions;
};

struct dptgDrivingToLeftForwarder : sc::state<dptgDrivingToLeftForwarder, DeliverPuckToGate>
{
	dptgDrivingToLeftForwarder(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToLeftForwarder");
		post_event(EvForward());
	} // entry

	~dptgDrivingToLeftForwarder() {
	} // exit

	sc::result react(const EvForward&)
	{
		context<DeliverPuckToGate>().driveToLeft();
		return transit<dptgDrivingToLeft>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvForward>
	> reactions;
};

struct dptgDrivingToLeft : sc::state<dptgDrivingToLeft, DeliverPuckToGate>
{
	dptgDrivingToLeft(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToLeft");
	} // entry

	~dptgDrivingToLeft() {
	} // exit

	sc::result react(const EvRobotLostPuck &)
	{
		context<StateMachine1>().regainPuckReturn = RegainPuckReturn::dptgDrivingToLeftForwarder;
		return transit<RegainPuck>();
	}

	sc::result react(const EvMotorCtrlReady&)
	{
		context<DeliverPuckToGate>().driveToRight();
		return transit<dptgDrivingToRight>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvRobotLostPuck>,
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::transition<EvCameraLampDetected, dptgDrivingToLamp, DeliverPuckToGate, &DeliverPuckToGate::driveToLamp>
	> reactions;
};

struct dptgDrivingToRightForwarder : sc::state<dptgDrivingToRightForwarder, DeliverPuckToGate>
{
	dptgDrivingToRightForwarder(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToRightForwarder");
		post_event(EvForward());
	} // entry

	~dptgDrivingToRightForwarder() {
	} // exit

	sc::result react(const EvForward&)
	{
		context<DeliverPuckToGate>().driveToRight();
		return transit<dptgDrivingToRight>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvForward>
	> reactions;
};

struct dptgDrivingToRight : sc::state<dptgDrivingToRight, DeliverPuckToGate>
{
	dptgDrivingToRight(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToRight");
	} // entry

	~dptgDrivingToRight() {
	} // exit

	sc::result react(const EvRobotLostPuck &)
	{
		context<StateMachine1>().regainPuckReturn = RegainPuckReturn::dptgDrivingToRightForwarder;
		return transit<RegainPuck>();
	}

	sc::result react(const EvMotorCtrlReady&)
	{
		context<DeliverPuckToGate>().driveToLeft();
		return transit<dptgDrivingToLeft>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvRobotLostPuck>,
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::transition<EvCameraLampDetected, dptgDrivingToLamp, DeliverPuckToGate, &DeliverPuckToGate::driveToLamp>
	> reactions;
};

struct dptgDrivingToLamp : sc::state< dptgDrivingToLamp, DeliverPuckToGate>
{
	dptgDrivingToLamp(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToLamp");
	} // entry

	~dptgDrivingToLamp() {
	} // exit

//	sc::result react(const EvSensorFloorLeftIsBlack&)
//	{
//		context<DeliverPuckToGate>().driveForwardToMachine();
//		return transit<dptgDrivingForwardToMachine>();
//	}
//
//	sc::result react(const EvSensorFloorRightIsBlack&)
//	{
//		context<DeliverPuckToGate>().driveForwardToMachine();
//		return transit<dptgDrivingForwardToMachine>();
//	}

	sc::result react(const EvMotorCtrlReady&)
	{
		context<DeliverPuckToGate>().driveForwardToMachine();
		return transit<dptgDrivingForwardToMachine>();
	}

	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		post_event(EvSensorFrontLeftFoundObstacle());
		return transit<dptgDrivingForwardToMachine>();
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		post_event(EvSensorFrontRightFoundObstacle());
		return transit<dptgDrivingForwardToMachine>();
	}

	//Reactions
	// TODO: implement the same way as driveToLamp (no self-transition)
	typedef mpl::list<
		sc::transition<EvCameraLampDetected, dptgDrivingToLamp, DeliverPuckToGate, &DeliverPuckToGate::driveToLamp>,
//		sc::custom_reaction<EvSensorFloorLeftIsBlack>,
//		sc::custom_reaction<EvSensorFloorRightIsBlack>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>,
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;

};

struct dptgDrivingForwardToMachine : sc::state< dptgDrivingForwardToMachine, DeliverPuckToGate>
{
	dptgDrivingForwardToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingForwardToMachine");
	} // entry

	~dptgDrivingForwardToMachine() {
	} // exit


	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<DeliverPuckToGate>().rightObstacleSeen){
			context<DeliverPuckToGate>().calibrate();
			context<DeliverPuckToGate>().driveBack();
			return transit<dptgDrivingBack>();
		}else{
			context<DeliverPuckToGate>().leftObstacleSeen = true;
			context<DeliverPuckToGate>().moveLeft();
			return transit<dptgDrivingLeftToMachine>();
		}
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<DeliverPuckToGate>().leftObstacleSeen){
			context<DeliverPuckToGate>().calibrate();
			context<DeliverPuckToGate>().driveBack();
			return transit<dptgDrivingBack>();
		}else{
			context<DeliverPuckToGate>().rightObstacleSeen = true;
			context<DeliverPuckToGate>().moveRight();
			return transit<dptgDrivingRightToMachine>();
		}
	}

	sc::result react(const EvMotorCtrlReady&)
	{
		context<DeliverPuckToGate>().calibrate();
		context<DeliverPuckToGate>().driveBack();
		return transit<dptgDrivingBack>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>,
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;

};

struct dptgDrivingLeftToMachine : sc::state< dptgDrivingLeftToMachine, DeliverPuckToGate>
{
	dptgDrivingLeftToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingLeftToMachine");
	} // entry

	~dptgDrivingLeftToMachine() {
	} // exit


	sc::result react(const EvSensorFrontLeftIsFree&)
	{
		context<DeliverPuckToGate>().leftObstacleSeen = true;
		//context<DeliverPuckToGate>().leftObstacleSeen = false;
		context<DeliverPuckToGate>().moveRight();
		return transit<dptgDrivingRightToMachine>();
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<DeliverPuckToGate>().leftObstacleSeen){
			context<DeliverPuckToGate>().calibrate();
			context<DeliverPuckToGate>().driveBack();
			return transit<dptgDrivingBack>();
		}else{
			context<DeliverPuckToGate>().rightObstacleSeen = true;
			context<DeliverPuckToGate>().moveForward();
			return transit<dptgDrivingForwardToMachine>();
		}
	}


	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftIsFree>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>
	> reactions;

};

struct dptgDrivingRightToMachine : sc::state< dptgDrivingRightToMachine, DeliverPuckToGate>
{
	dptgDrivingRightToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingRightToMachine");
	} // entry

	~dptgDrivingRightToMachine() {
	} // exit


	sc::result react(const EvSensorFrontRightIsFree&)
	{
		context<DeliverPuckToGate>().rightObstacleSeen = true;
		//context<DeliverPuckToGate>().rightObstacleSeen = false;
		context<DeliverPuckToGate>().moveLeft();
		return transit<dptgDrivingLeftToMachine>();
	}

	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<DeliverPuckToGate>().rightObstacleSeen){
			context<DeliverPuckToGate>().calibrate();
			context<DeliverPuckToGate>().driveBack();
			return transit<dptgDrivingBack>();
		}else{
			context<DeliverPuckToGate>().leftObstacleSeen = true;
			context<DeliverPuckToGate>().moveForward();
			return transit<dptgDrivingForwardToMachine>();
		}
	}


	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontRightIsFree>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>
	> reactions;
};

struct dptgDrivingBack : sc::state< dptgDrivingBack, DeliverPuckToGate>
{
	dptgDrivingBack(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingBack");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setHavingPuck(false);
	} // entry

	~dptgDrivingBack() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, dptgDrivingToExitLocation, DeliverPuckToGate, &DeliverPuckToGate::driveToExitLocation>
	> reactions;
};

struct dptgDrivingToExitLocation : sc::state< dptgDrivingToExitLocation, DeliverPuckToGate>
{
	dptgDrivingToExitLocation(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDrivingToExitLocation");
	} // entry

	~dptgDrivingToExitLocation() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, dptgDelivered>
	> reactions;
};

struct dptgDelivered : sc::state< dptgDelivered, DeliverPuckToGate>
{
	dptgDelivered(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptgDelivered");
		post_event(EvSuccess());
	} // entry

	~dptgDelivered() {
	} // exit

};

#endif /* DeliverPuckToGate_H_ */
