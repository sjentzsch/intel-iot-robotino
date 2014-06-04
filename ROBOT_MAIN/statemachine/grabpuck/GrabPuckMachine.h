/*
 * GrabPuckMachine.h
 *
 *  Created on: 22 Jun 2011
 *      Author: root
 */

#ifndef GRABPUCKMACHINE_H_
#define GRABPUCKMACHINE_H_
#include "../StateMachine.h"

//States
struct gpmInit;
struct gpmDrivingForward;
struct gpmWrongRecyclingPuckBack;
struct gpmCheckingForGreenLightNoPuck;
struct gpmCheckingForGreenLightWithPuck;
struct gpmDrivingToPuck;
struct gpmFinishedForwarder;
struct gpmFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    GrabPuckMachine
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct GrabPuckMachine : sc::state<GrabPuckMachine, StateMachine1, gpmInit>
{
	GrabPuckMachine(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("GrabPuckMachine");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		if(context<StateMachine1>().gpmSidewards){
			FileLog::log_NOTICE("Switching to CameraPuckDetection::RECYCLING_FROM_MACHINE Puck Detection-Mode");
			stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::RECYCLING_FROM_MACHINE, CameraLightDetection::OFF);
		} else {
			stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::ALL_MACHINE, CameraLightDetection::OFF);
		}

	} // entry

	~GrabPuckMachine() {
		stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	void driveForward() {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(200, 0, 0, 80.0);
	}

	void driveToPuck(const EvCameraPuckDetected& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPosOnly(ev.puckXPos, ev.puckYPos, 150.0);

		// 35 cm Puck vom Mittelpunkt Robotino entfernt
		// seitl: 20 cm
		// Tiefe: ca. 7.5 cm
	}

	void driveToLine(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(250,0,0,70);
	}

	void driveBack(){
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(-300,0,0,70);
	}

	void prepareRegainPuck() {
		context<StateMachine1>().regainPuckReturn = RegainPuckReturn::gpmFinishedForwarder;
	}

	void calibrate(const EvForward& ev)
	{
		if(context<StateMachine1>().poiFrom!=NULL && !context<StateMachine1>().gpmSidewards){
			Grid *grid = context<StateMachine1>().getGrid();
			Node *nodePOI = grid->getNode(context<StateMachine1>().poiFrom->x, context<StateMachine1>().poiFrom->y);
			POIDirection::POIDirection directionPOI = context<StateMachine1>().poiFrom->dir;
			stateBehavCtrl->getSensorControl()->calibrateOnMachineFront(nodePOI, directionPOI);
			stateBehavCtrl->getSensorControl()->calibrateOnMachineSide(nodePOI, directionPOI);
		}
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;
};

struct gpmInit : sc::state<gpmInit, GrabPuckMachine>
{
	gpmInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpmInit");
		post_event(EvInit());
	} // entry

	~gpmInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<GrabPuckMachine>().driveForward();
		return transit<gpmDrivingForward>();

	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>,
		sc::deferral<EvAngleCalibration>
	> reactions;
};


struct gpmDrivingForward : sc::state<gpmDrivingForward, GrabPuckMachine>
{
	gpmDrivingForward(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpmDrivingForward");
	} // entry

	~gpmDrivingForward() {
	} // exit

	sc::result react(const EvCameraInitialNoPuck &)
	{
		//in case we are fetching a P from an T3, check whether the processing has actually finished (green light)
		if(context<StateMachine1>().gpmSidewards){
			context<GrabPuckMachine>().driveBack();
			return transit<gpmDrivingForward>();
		} else {
			return discard_event();
		}

	}

	//Reactions
	typedef mpl::list<
		// TODO: on EvMotorCtrlReady, trigger EvFailure
		sc::transition<EvCameraPuckDetected, gpmDrivingToPuck, GrabPuckMachine, &GrabPuckMachine::driveToPuck>,
		sc::custom_reaction<EvCameraInitialNoPuck>
	> reactions;
};

struct gpmWrongRecyclingPuckBack : sc::state<gpmWrongRecyclingPuckBack, GrabPuckMachine>
{
	gpmWrongRecyclingPuckBack(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpmWrongRecyclingPuckBack");
	} // entry

	~gpmWrongRecyclingPuckBack() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::RECYCLING_FROM_MACHINE, CameraLightDetection::OFF);
	} // exit

	sc::result react(const EvMotorCtrlReady &)
	{
		context<GrabPuckMachine>().driveForward();
		return transit<gpmDrivingForward>();
	}

	//Reactions
	typedef mpl::list<
		// TODO: on EvMotorCtrlReady, trigger EvFailure
		//sc::transition<EvMotorCtrlReady, gpmDrivingForward, GrabPuckMachine, &GrabPuckMachine::driveForward>
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct gpmDrivingToPuck : sc::state<gpmDrivingToPuck, GrabPuckMachine>
{
	gpmDrivingToPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpmDrivingToPuck");
	} // entry

	~gpmDrivingToPuck() {
	} // exit

	sc::result react(const EvMotorCtrlReady &)
	{
		//in case we are fetching a P from an T3, check whether the processing has actually finished (green light)
		if((context<StateMachine1>().poiFrom->type == POIType::T3 || context<StateMachine1>().poiFrom->type == POIType::T4 || context<StateMachine1>().poiFrom->type == POIType::T5) && context<StateMachine1>().poiFrom->status == POIStatus::PROCESSING ){
			return transit<gpmCheckingForGreenLightNoPuck>();
		}
		else{
			context<GrabPuckMachine>().prepareRegainPuck();
			return transit<RegainPuck>();
		}
	}

	sc::result react(const EvSensorHasPuck &)
	{
		//in case we are fetching a P from an T3, check whether the processing has actually finished (green light)
		if((context<StateMachine1>().poiFrom->type == POIType::T3 || context<StateMachine1>().poiFrom->type == POIType::T4 || context<StateMachine1>().poiFrom->type == POIType::T5) && context<StateMachine1>().poiFrom->status == POIStatus::PROCESSING ){
			return transit<gpmCheckingForGreenLightWithPuck>();
		}
		else{
			return transit<gpmFinishedForwarder>();
		}
	}

	sc::result react(const EvSensorFloorLeftIsBlack &)
	{
		if(context<StateMachine1>().gpmSidewards){
			context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
			context<GrabPuckMachine>().driveBack();
			return transit<gpmWrongRecyclingPuckBack>();
		} else {
			return discard_event();
		}
	}

	sc::result react(const EvSensorFloorRightIsBlack &)
	{
		if(context<StateMachine1>().gpmSidewards){
			context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
			context<GrabPuckMachine>().driveBack();
			return transit<gpmWrongRecyclingPuckBack>();
		} else {
			return discard_event();
		}
	}

	//Reactions
	typedef mpl::list<
		// TODO: check if it can lead to failure during the last seconds, when the robot might only see a puck != the one he just grabbed
		sc::transition<EvCameraPuckDetected, gpmDrivingToPuck, GrabPuckMachine, &GrabPuckMachine::driveToPuck>,
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::custom_reaction<EvSensorHasPuck>,
		sc::custom_reaction<EvSensorFloorLeftIsBlack>,
		sc::custom_reaction<EvSensorFloorRightIsBlack>
	> reactions;
};

struct gpmFinishedForwarder : sc::state<gpmFinishedForwarder, GrabPuckMachine>
{
	gpmFinishedForwarder(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpmFinishedForwarder");
		post_event(EvForward());
	} // entry

	~gpmFinishedForwarder() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvForward, gpmFinished, GrabPuckMachine, &GrabPuckMachine::calibrate>
	> reactions;
};

/**
 * Wait for green light at the machine before taking the puck
 */
struct gpmCheckingForGreenLightNoPuck : sc::state<gpmCheckingForGreenLightNoPuck, GrabPuckMachine>
{
	gpmCheckingForGreenLightNoPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::FAR);
		context<StateMachine1>().logAndDisplayStateName("gpmCheckingForGreenLightNoPuck");
	} // entry

	~gpmCheckingForGreenLightNoPuck() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	sc::result react(const EvCameraGreenLightDetected &)
	{
		context<GrabPuckMachine>().prepareRegainPuck();
		return transit<RegainPuck>();
	}

	typedef mpl::list<
			sc::custom_reaction<EvCameraGreenLightDetected>
		> reactions;
};

struct gpmCheckingForGreenLightWithPuck : sc::state<gpmCheckingForGreenLightWithPuck, GrabPuckMachine>
{
	gpmCheckingForGreenLightWithPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::FAR);
		context<StateMachine1>().logAndDisplayStateName("gpmCheckingForGreenLightWithPuck");
	} // entry

	~gpmCheckingForGreenLightWithPuck() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	sc::result react(const EvCameraGreenLightDetected &)
	{
		return transit<gpmFinishedForwarder>();
	}

	typedef mpl::list<
			sc::custom_reaction<EvCameraGreenLightDetected>
		> reactions;
};


struct gpmFinished : sc::state<gpmFinished, GrabPuckMachine>
{
	gpmFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpmFinished");
		post_event(EvSuccess());
	} // entry

	~gpmFinished() {
	} // exit
};

#endif /* GRABPUCKMACHINE_H_ */
