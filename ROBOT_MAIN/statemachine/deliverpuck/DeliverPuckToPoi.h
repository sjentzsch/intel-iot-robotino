/*
 * DeliverPuckToPoi
 *
 *  Created on: Jun 11, 2011
 *      Author: root
 */

#ifndef DELIVERPUCKTOPOI_H_
#define DELIVERPUCKTOPOI_H_

#include "../StateMachine.h"

//States
struct dptpInit;
struct dptpDrivingToLine;
struct dptpCheckOutOfOrder;
struct dptpDrivingFixedDistanceToLamp;
struct dptpDrivingForwardToMachine;
struct dptpDrivingLeftToMachine;
struct dptpDrivingRightToMachine;
struct dptpWaitingDelivered;
struct dptpDeliveredGreen;
struct dptpDeliveredYellow;
struct dptpDeliveredYellowFlash;
struct dptpDeliveredYellowGreen;
struct dptpDeliveredRed;
struct dptpDeliveredRedGreen;
struct dptpDeliveredRedYellow;
struct dptpDeliveredRedYellowGreen;
struct dptpDeliveredOffline;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    DeliverPuckToPoi
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct DeliverPuckToPoi : sc::state<DeliverPuckToPoi,StateMachine1, dptpInit>
{
	DeliverPuckToPoi(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("DeliverPuckToPoi");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		leftObstacleSeen = false;
		rightObstacleSeen = false;
		grid = context<StateMachine1>().getGrid();
	} // entry

	~DeliverPuckToPoi() {
	} // exit

	void moveForward(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(40,0,0,120);
	}

	void moveForwardToLamp(){
		Node *nodePOI = grid->getNode(context<StateMachine1>().poiTo->x, context<StateMachine1>().poiTo->y);
		POIDirection::POIDirection directionPOI = context<StateMachine1>().poiTo->dir;

		vec3D absTarget = stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI,directionPOI,0,0);
		if(directionPOI == POIDirection::SOUTH){
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(stateBehavCtrl->getSensorControl()->getRobotX() - 300,absTarget.y,absTarget.phi,150);
		} else if(directionPOI == POIDirection::EAST) {
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(absTarget.x,stateBehavCtrl->getSensorControl()->getRobotY() - 300,absTarget.phi,150);
		} else if(directionPOI == POIDirection::NORTH) {
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(stateBehavCtrl->getSensorControl()->getRobotX() + 300,absTarget.y,absTarget.phi,150);
		} else {
			stateBehavCtrl->getMotorCtrl()->moveToAbsPos(absTarget.x,stateBehavCtrl->getSensorControl()->getRobotY() + 300,absTarget.phi,150);
		}
	}

	void moveForwardToLampSlow(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(500,0,0,120);
	}

	void driveToLine(){
		moveForwardToLamp();
		//stateBehavCtrl->getMotorCtrl()->moveToRelPos(250,0,0,120);
	}

	void driveToLamp(const EvCameraLampDetected& ev) {
		Node *nodePOI = grid->getNode(context<StateMachine1>().poiTo->x, context<StateMachine1>().poiTo->y);
		POIDirection::POIDirection directionPOI = context<StateMachine1>().poiTo->dir;
		vec3D vPOI = stateBehavCtrl->getSensorControl()->getPosOnMachineFront(nodePOI, directionPOI, 0, ev.lampYPos+10);
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(vPOI.x, vPOI.y, vPOI.phi, 250.0f);
	}

	void moveRight(){
		stateBehavCtrl->getMotorCtrl()->terminate();
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,-250,0,70);
	}

	void moveLeft(){
		stateBehavCtrl->getMotorCtrl()->terminate();
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0,250,0,70);
	}

	void stop(){
		stateBehavCtrl->getMotorCtrl()->terminate();
	}

	void turnOffCameraDetection(){
		stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	}

	StateBehaviorController* getStateBehavCtrl(){
		return stateBehavCtrl;
	}

	bool leftObstacleSeen;
	bool rightObstacleSeen;

	void calibrate()
	{
		Node *nodePOI = grid->getNode(context<StateMachine1>().poiTo->x, context<StateMachine1>().poiTo->y);
		POIDirection::POIDirection directionPOI = context<StateMachine1>().poiTo->dir;
		stateBehavCtrl->getSensorControl()->calibrateOnMachineFront(nodePOI, directionPOI);
		stateBehavCtrl->getSensorControl()->calibrateOnMachineSide(nodePOI, directionPOI);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController *stateBehavCtrl;
	Grid *grid;
};

struct dptpInit : sc::state< dptpInit, DeliverPuckToPoi>
{
	dptpInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpInit");
		post_event(EvInit());
	} // entry

	~dptpInit() {
	} // exit

	//Reactions

	sc::result react(const EvInit&)
	{
		if(context<StateMachine1>().isContinueDelivering)
		{
			context<DeliverPuckToPoi>().moveForwardToLamp();
			context<StateMachine1>().isContinueDelivering = false;
			return transit<dptpDrivingFixedDistanceToLamp>();
		}
		else
		{
			context<DeliverPuckToPoi>().driveToLine();
			return transit<dptpDrivingToLine>();
		}
	}
	typedef mpl::list<
		sc::custom_reaction<EvInit>,
		sc::deferral<EvAngleCalibration>
	> reactions;
};

struct dptpDrivingToLine : sc::state< dptpDrivingToLine, DeliverPuckToPoi>
{
	dptpDrivingToLine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDrivingToLine");
	} // entry

	~dptpDrivingToLine() {
	} // exit

	sc::result react(const EvAngleCalibration &ev)
	{
		context<DeliverPuckToPoi>().getStateBehavCtrl()->getMotorCtrl()->terminate(); //TODO: check for "drive-backwards" bug
		if(ev.diffAngle != 0)
			context<StateMachine1>().getStateBehavController()->getSensorControl()->calibrateAngle(ev.lineAngle+ev.diffAngle);
		FileLog::log_NOTICE("###############################################");
		FileLog::log_NOTICE("--------------------> Angle Calibrated: diff Angle: ", FileLog::real(ev.diffAngle), ", corrected angle: ", FileLog::real(ev.lineAngle+ev.diffAngle));
		FileLog::log_NOTICE("###############################################");
		//return discard_event();
		//only do out of order check in production, when no machines are unknown
		if(context<StateMachine1>().poiTo->type != POIType::UNKNOWN)
			return transit<dptpCheckOutOfOrder>();	// TODO: change if align on lamp
		else{
			context<DeliverPuckToPoi>().moveForwardToLamp();
			return transit<dptpDrivingFixedDistanceToLamp>();
		}

	}

	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<DeliverPuckToPoi>().rightObstacleSeen){
			context<DeliverPuckToPoi>().stop();
			return transit<dptpWaitingDelivered>();
		}else{
			context<DeliverPuckToPoi>().leftObstacleSeen = true;
			context<DeliverPuckToPoi>().moveLeft();
			return transit<dptpDrivingLeftToMachine>();
		}
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<DeliverPuckToPoi>().leftObstacleSeen){
			context<DeliverPuckToPoi>().stop();
			return transit<dptpWaitingDelivered>();
		}else{
			context<DeliverPuckToPoi>().rightObstacleSeen = true;
			context<DeliverPuckToPoi>().moveRight();
			return transit<dptpDrivingRightToMachine>();
		}
	}

	sc::result react(const EvMotorCtrlReady &ev)
	{
		if(context<StateMachine1>().poiTo->type != POIType::UNKNOWN)
			return transit<dptpCheckOutOfOrder>();	// TODO: change if align on lamp
		else{
			context<DeliverPuckToPoi>().moveForwardToLamp();
			return transit<dptpDrivingFixedDistanceToLamp>();
			}
	}
	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::custom_reaction<EvAngleCalibration>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>
	> reactions;
};

struct dptpCheckOutOfOrder : sc::state< dptpCheckOutOfOrder, DeliverPuckToPoi>
{
	dptpCheckOutOfOrder(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpCheckOutOfOrder");
		context<DeliverPuckToPoi>().stop();
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::FAR);
	} // entry

	~dptpCheckOutOfOrder() {
	} // exit


	sc::result react(const EvCameraRedLightDetected& evt)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		return transit<Dispatcher>(&StateMachine1::handleOutOfOrder,evt);
	}

	sc::result react(const EvCameraRedGreenLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}

	sc::result react(const EvCameraYellowFlashLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}
	sc::result react(const EvCameraRedYellowGreenLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}

	sc::result react(const EvCameraOfflineLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}

	sc::result react(const EvCameraGreenLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}

	sc::result react(const EvCameraYellowLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();

	}

	sc::result react(const EvCameraRedYellowLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}

	sc::result react(const EvCameraYellowGreenLightDetected&)
	{
		context<DeliverPuckToPoi>().turnOffCameraDetection();
		context<DeliverPuckToPoi>().moveForwardToLamp();
		//return transit<dptpDrivingForwardToMachine>();
		return transit<dptpDrivingFixedDistanceToLamp>();
	}

	typedef mpl::list<
		sc::custom_reaction<EvCameraRedGreenLightDetected>,
		sc::custom_reaction<EvCameraYellowGreenLightDetected>,
		sc::custom_reaction<EvCameraRedYellowLightDetected>,
		sc::custom_reaction<EvCameraYellowFlashLightDetected>,
		sc::custom_reaction<EvCameraRedYellowGreenLightDetected>,
		sc::custom_reaction<EvCameraOfflineLightDetected>,
		sc::custom_reaction<EvCameraGreenLightDetected>,
		sc::custom_reaction<EvCameraYellowLightDetected>,
		sc::custom_reaction<EvCameraRedLightDetected>
	> reactions;

};

struct dptpDrivingFixedDistanceToLamp : sc::state< dptpDrivingFixedDistanceToLamp, DeliverPuckToPoi>
{
	dptpDrivingFixedDistanceToLamp(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDrivingFixedDistanceToLamp");
	} // entry

	~dptpDrivingFixedDistanceToLamp() {
	} // exit


	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		context<DeliverPuckToPoi>().leftObstacleSeen = true;
		context<DeliverPuckToPoi>().moveLeft();
		return transit<dptpDrivingLeftToMachine>();
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		context<DeliverPuckToPoi>().rightObstacleSeen = true;
		context<DeliverPuckToPoi>().moveRight();
		return transit<dptpDrivingRightToMachine>();
	}

	sc::result react(const EvMotorCtrlReady&)
	{
		context<DeliverPuckToPoi>().moveForwardToLampSlow();
		return transit<dptpDrivingForwardToMachine>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>,
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};



struct dptpDrivingForwardToMachine : sc::state< dptpDrivingForwardToMachine, DeliverPuckToPoi>
{
	dptpDrivingForwardToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDrivingForwardToMachine");
	} // entry

	~dptpDrivingForwardToMachine() {
	} // exit


	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<DeliverPuckToPoi>().rightObstacleSeen){
			context<DeliverPuckToPoi>().stop();
			return transit<dptpWaitingDelivered>();
		}else{
			context<DeliverPuckToPoi>().leftObstacleSeen = true;
			context<DeliverPuckToPoi>().moveLeft();
			return transit<dptpDrivingLeftToMachine>();
		}
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<DeliverPuckToPoi>().leftObstacleSeen){
			context<DeliverPuckToPoi>().stop();
			return transit<dptpWaitingDelivered>();
		}else{
			context<DeliverPuckToPoi>().rightObstacleSeen = true;
			context<DeliverPuckToPoi>().moveRight();
			return transit<dptpDrivingRightToMachine>();
		}
	}

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<dptpWaitingDelivered>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>,
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};


struct dptpDrivingLeftToMachine : sc::state< dptpDrivingLeftToMachine, DeliverPuckToPoi>
{
	dptpDrivingLeftToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDrivingLeftToMachine");
	} // entry

	~dptpDrivingLeftToMachine() {
	} // exit


	sc::result react(const EvSensorFrontLeftIsFree&)
	{
		//context<DeliverPuckToPoi>().leftObstacleSeen = false;
		context<DeliverPuckToPoi>().leftObstacleSeen = true;
		context<DeliverPuckToPoi>().moveRight();
		return transit<dptpDrivingRightToMachine>();
	}

	sc::result react(const EvSensorFrontRightFoundObstacle&)
	{
		if(context<DeliverPuckToPoi>().leftObstacleSeen){
			context<DeliverPuckToPoi>().stop();
			return transit<dptpWaitingDelivered>();
		}else{
			context<DeliverPuckToPoi>().rightObstacleSeen = true;
			context<DeliverPuckToPoi>().moveForward();
			return transit<dptpDrivingForwardToMachine>();
		}
	}


	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontLeftIsFree>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>
	> reactions;

};

struct dptpDrivingRightToMachine : sc::state< dptpDrivingRightToMachine, DeliverPuckToPoi>
{
	dptpDrivingRightToMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDrivingRightToMachine");
	} // entry

	~dptpDrivingRightToMachine() {
	} // exit


	sc::result react(const EvSensorFrontRightIsFree&)
	{
		//context<DeliverPuckToPoi>().rightObstacleSeen = false;
		context<DeliverPuckToPoi>().rightObstacleSeen = true;
		context<DeliverPuckToPoi>().moveLeft();
		return transit<dptpDrivingLeftToMachine>();
	}

	sc::result react(const EvSensorFrontLeftFoundObstacle&)
	{
		if(context<DeliverPuckToPoi>().rightObstacleSeen){
			context<DeliverPuckToPoi>().stop();
			return transit<dptpWaitingDelivered>();
		}else{
			context<DeliverPuckToPoi>().leftObstacleSeen = true;
			context<DeliverPuckToPoi>().moveForward();
			return transit<dptpDrivingForwardToMachine>();
		}
	}


	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorFrontRightIsFree>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>
	> reactions;
};

struct dptpWaitingDelivered : sc::state< dptpWaitingDelivered, DeliverPuckToPoi>
{
	dptpWaitingDelivered(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpWaitingDelivered");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::NEAR);
		/*TEST-CODE
		int i;
		cin>>i;
		if(i==1){
			context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::GREEN);
		}else{
			context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::YELLOW);
		}
		boost::shared_ptr<EvSuccess> ev(new EvSuccess());
		context<StateMachine1>().getStateBehavController()->getAsyncStateMachine()->queueEvent(ev);
		*/
	} // entry

	~dptpWaitingDelivered() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	sc::result react(const EvCameraGreenLightDetected &)
	{
		/*
		if(context<StateMachine1>().poiTo->type == POIType::T5 && ModelProvider::getInstance()->getWorldModel()->T5prodTime == 0){
			context<StateMachine1>().getStateBehavController()->getJobHandler()->endMachineTiming(POIType::T5);
		}
		*/
		/*
		else if(context<StateMachine1>().poiTo->type == POIType::T4 && ModelProvider::getInstance()->getWorldModel()->T4procTime == 0){
			context<StateMachine1>().getStateBehavController()->getJobHandler()->endMachineTiming(POIType::T4);
		}
		else if(context<StateMachine1>().poiTo->type == POIType::T3 && ModelProvider::getInstance()->getWorldModel()->T3procTime == 0){
			context<StateMachine1>().getStateBehavController()->getJobHandler()->endMachineTiming(POIType::T3);
		}
		*/
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredGreen>();
	}

	sc::result react(const EvCameraYellowLightDetected &)
	{
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredYellow>();
	}

	sc::result react(const EvCameraYellowFlashLightDetected &)
	{
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredYellowFlash>();
	}

	/**
	 * Yellow AND Green detected => leave the T3, T4 or T5 machine to produce on their own, ignore event for all other machines (wait)
	 * also do NOT wait when in exploration phase, as green-yellow can be a type indicator
	 */
	sc::result react(const EvCameraYellowGreenLightDetected &)
	{
		if((context<StateMachine1>().poiTo->type == POIType::UNKNOWN) ||
			(/*((context<StateMachine1>().poiTo->type == POIType::T5) && (ModelProvider::getInstance()->getWorldModel()->T5prodTime > 25)) ||*/
			((context<StateMachine1>().poiTo->type == POIType::T4) && context<StateMachine1>().poiFrom->type == POIType::T2)||
			((context<StateMachine1>().poiTo->type == POIType::T3) && context<StateMachine1>().poiFrom->type == POIType::T2))){
			context<DeliverPuckToPoi>().calibrate();
			return transit<dptpDeliveredYellowGreen>();
		}
		else if(context<StateMachine1>().poiTo->type == POIType::T5 && ModelProvider::getInstance()->getWorldModel()->T5prodTime == 0){
			context<StateMachine1>().getStateBehavController()->getJobHandler()->startMachineTiming();
			return discard_event();
		}
		else return discard_event();
	}

	sc::result react(const EvCameraRedLightDetected &)
	{
		if((context<StateMachine1>().poiTo->type == POIType::T5) && !(context<StateMachine1>().poiTo->type == POIType::UNKNOWN)){
			return discard_event();
		}
		if((context<StateMachine1>().poiTo->type == POIType::UNKNOWN) || ((((context<StateMachine1>().poiTo->type == POIType::T3) || (context<StateMachine1>().poiTo->type == POIType::T4) )&& context<StateMachine1>().poiFrom->type == POIType::T2))){
			context<DeliverPuckToPoi>().calibrate();
			return transit<dptpDeliveredRed>();
		}
		else return discard_event();
	}

	sc::result react(const EvCameraRedGreenLightDetected &)
	{
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredRedGreen>();
	}

	sc::result react(const EvCameraRedYellowLightDetected &)
	{
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredRedYellow>();
	}

	sc::result react(const EvCameraRedYellowGreenLightDetected &)
	{
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredRedYellowGreen>();
	}

	sc::result react(const EvCameraOfflineLightDetected &)
	{
		context<DeliverPuckToPoi>().calibrate();
		return transit<dptpDeliveredOffline>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvCameraGreenLightDetected>,
		sc::custom_reaction<EvCameraYellowLightDetected>,
		sc::custom_reaction<EvCameraYellowFlashLightDetected>,
		sc::custom_reaction<EvCameraYellowGreenLightDetected>,
		sc::custom_reaction<EvCameraRedLightDetected>,
		sc::custom_reaction<EvCameraRedGreenLightDetected>,
		sc::custom_reaction<EvCameraRedYellowLightDetected>,
		sc::custom_reaction<EvCameraRedYellowGreenLightDetected>,
		sc::custom_reaction<EvCameraOfflineLightDetected>
	> reactions;
};

struct dptpDeliveredGreen : sc::state< dptpDeliveredGreen, DeliverPuckToPoi>
{
	dptpDeliveredGreen(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredGreen");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::GREEN);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredGreen() {
	} // exit
};

struct dptpDeliveredYellow : sc::state< dptpDeliveredYellow, DeliverPuckToPoi>
{
	dptpDeliveredYellow(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredYellow");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::YELLOW);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredYellow() {
	} // exit
};

struct dptpDeliveredYellowFlash : sc::state< dptpDeliveredYellowFlash, DeliverPuckToPoi>
{
	dptpDeliveredYellowFlash(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredYellowFlash");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::YELLOW_FLASH);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredYellowFlash() {
	} // exit
};

struct dptpDeliveredYellowGreen : sc::state< dptpDeliveredYellowGreen, DeliverPuckToPoi>
{
	dptpDeliveredYellowGreen(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredYellowGreen");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::YELLOW_GREEN);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredYellowGreen() {
	} // exit
};

struct dptpDeliveredRed : sc::state< dptpDeliveredRed, DeliverPuckToPoi>
{
	dptpDeliveredRed(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredRed");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::RED);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredRed() {
	} // exit
};

struct dptpDeliveredRedGreen : sc::state< dptpDeliveredRedGreen, DeliverPuckToPoi>
{
	dptpDeliveredRedGreen(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredRedGreen");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::RED_GREEN);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredRedGreen() {
	} // exit
};


struct dptpDeliveredRedYellow : sc::state< dptpDeliveredRedYellow, DeliverPuckToPoi>
{
	dptpDeliveredRedYellow(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredRedYellow");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::RED_YELLOW);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredRedYellow() {
	} // exit
};


struct dptpDeliveredRedYellowGreen : sc::state< dptpDeliveredRedYellowGreen, DeliverPuckToPoi>
{
	dptpDeliveredRedYellowGreen(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredRedYellowGreen");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::RED_YELLOW_GREEN);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredRedYellowGreen() {
	} // exit
};

struct dptpDeliveredOffline : sc::state< dptpDeliveredOffline, DeliverPuckToPoi>
{
	dptpDeliveredOffline(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dptpDeliveredOffline");
		context<StateMachine1>().getStateBehavController()->getJobHandler()->updatePoiType(CameraLightState::OFFLINE);
		post_event(EvSuccess());
	} // entry

	~dptpDeliveredOffline() {
	} // exit
};

#endif /* DELIVERPUCKTOPOI_H_ */
