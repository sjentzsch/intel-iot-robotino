/*
 * GrabPuckInputZone.h
 *
 *  Created on: Jun 5, 2012
 *      Author: root
 */

#ifndef GRABPUCKINPUTZONE_H_
#define GRABPUCKINPUTZONE_H_
#include "../StateMachine.h"

//States
struct gpizInit;
struct gpizDrivingFrontRight;
struct gpizSearchingPuck;
struct gpizDrivingToPuck;
//struct gpizSecuringPuck;
struct gpizRotatingToZeroForwarder;
struct gpizRotatingToZero;
struct gpizWaitingForPath;
struct gpizFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    GrabPuckInputZone
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct GrabPuckInputZone : sc::state<GrabPuckInputZone, StateMachine1, gpizInit>
{
	GrabPuckInputZone(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("GrabPuckInputZone");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		poiFrom = context<StateMachine1>().poiFrom;
		grid = context<StateMachine1>().getGrid();
		accessNode = grid->getAccessNode(poiFrom, POIAccessFrom::FRONT);
		grabLeft = (poiFrom->index == 27 || poiFrom->index == 29);
		//YPosWasCalibrated = false;
	} // entry

	~GrabPuckInputZone() {
		stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setHavingPuck(false);
	} // exit

	//bool YPosWasCalibrated;
	float startYPos;
	POI* poiFrom;
	Node* accessNode;
	Grid* grid;
	bool grabLeft;

	void driveFrontRight() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos()-370, startYPos, 180, 300.0);
	}

	void searchPuckLeft() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos()-370, startYPos-1000, 180, 100.0);
	}

	void searchPuckRight() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(accessNode->getXPos()-370, startYPos+1000, 180, 100.0);
	}

	void calibrateX() {
		// also change the constant in RegainPuck
		stateBehavCtrl->getSensorControl()->calibrateOnLineX(580);
	}

	void securePuck() {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(80, 0, 0, 100.0);
	}

	void driveToPuck(const EvCameraPuckDetected& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.puckXPos, ev.puckYPos, 180, 150.0);
	}

	void findPath() {
		//Grid* grid = context<StateMachine1>().getGrid();
		//stateBehavCtrl->getPathFinder()->findPathTo(context<StateMachine1>().poiTo, context<StateMachine1>().accessDirectionTo, context<StateMachine1>().poiFrom, POIAccessFrom::FRONT);
	}

	void rotateToZero(const EvForward&) {
		if(grabLeft)
			stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(0, ForceRotationDirection::LEFT, 50.0f);
		else
			stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(0, ForceRotationDirection::RIGHT, 50.0f);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;
};

struct gpizInit : sc::state<gpizInit, GrabPuckInputZone>
{
	gpizInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizInit");
		post_event(EvInit());
	} // entry

	~gpizInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<GrabPuckInputZone>().driveFrontRight();
		return transit<gpizDrivingFrontRight>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct gpizDrivingFrontRight : sc::state<gpizDrivingFrontRight, GrabPuckInputZone>
{
	gpizDrivingFrontRight(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizDrivingFrontRight");
	} // entry

	~gpizDrivingFrontRight() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		//context<GrabPuckInputZone>().searchPuck();
		return transit<gpizSearchingPuck>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::deferral<EvAngleCalibration>
	> reactions;
};

struct gpizSearchingPuck : sc::state<gpizSearchingPuck, GrabPuckInputZone>
{
	gpizSearchingPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizSearchingPuck");
		if(context<GrabPuckInputZone>().grabLeft)
			context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::ALL_INPUT_LEFT, CameraLightDetection::OFF);
		else
			context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::ALL_INPUT_RIGHT, CameraLightDetection::OFF);
	} // entry

	~gpizSearchingPuck() {
	} // exit

	sc::result react(const EvCameraInitialNoPuck &)
	{
		if(context<GrabPuckInputZone>().grabLeft)
			context<GrabPuckInputZone>().searchPuckRight();
		else
			context<GrabPuckInputZone>().searchPuckLeft();
		return discard_event();
	}

	sc::result react(const EvCameraPuckDetected & ev)
	{
		context<GrabPuckInputZone>().driveToPuck(ev);
		return transit<gpizDrivingToPuck>();
	}

	sc::result react(const EvMotorCtrlReady & ev)
	{
		// TODO
		cout << "ERROR: NO PUCKS IN INPUT ZONE FOUND" << endl;
		return discard_event();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvCameraInitialNoPuck>,
		sc::custom_reaction<EvCameraPuckDetected>,
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::deferral<EvAngleCalibration>
	> reactions;
};

struct gpizDrivingToPuck : sc::state<gpizDrivingToPuck, GrabPuckInputZone>
{
	gpizDrivingToPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizDrivingToPuck");
	} // entry

	~gpizDrivingToPuck() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	sc::result react(const EvAngleCalibration &ev)
	{
		if(ev.diffAngle != 0)
			context<StateMachine1>().getStateBehavController()->getSensorControl()->calibrateAngle(ev.lineAngle+ev.diffAngle);
		context<GrabPuckInputZone>().calibrateX();
		FileLog::log_NOTICE("###############################################");
		FileLog::log_NOTICE("--------------------> Angle Calibrated: diff Angle: ", FileLog::real(ev.diffAngle), ", corrected angle: ", FileLog::real(ev.lineAngle+ev.diffAngle));
		FileLog::log_NOTICE("###############################################");
		return discard_event();
	}

	sc::result react(const EvMotorCtrlReady &)
	{
		context<StateMachine1>().regainPuckReturn = RegainPuckReturn::gpizRotatingToZeroForwarder;
		return transit<RegainPuck>();
	}

	sc::result react(const EvCameraLostPuckVision &)
	{
		// TODO
		//context<GrabPuckInputZone>().driveForward();
		return transit<gpizDrivingToPuck>();
	}

	sc::result react(const EvCameraPuckDetected & ev)
	{
		context<GrabPuckInputZone>().driveToPuck(ev);
		return transit<gpizDrivingToPuck>();
	}

	//Reactions
	typedef mpl::list<
		//sc::custom_reaction<EvCameraLostPuckVision>,
		sc::custom_reaction<EvAngleCalibration>,
		sc::custom_reaction<EvMotorCtrlReady>
		//sc::custom_reaction<EvCameraPuckDetected>
	> reactions;
};


struct gpizSecuringPuck : sc::state<gpizSecuringPuck, GrabPuckInputZone>
{
	gpizSecuringPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizSecuringPuck");
	} // entry

	~gpizSecuringPuck() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, gpizRotatingToZeroForwarder>
	> reactions;
};

struct gpizRotatingToZeroForwarder : sc::state<gpizRotatingToZeroForwarder, GrabPuckInputZone>
{
	gpizRotatingToZeroForwarder(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizRotatingToZeroForwarder");
		post_event(EvForward());
	} // entry

	~gpizRotatingToZeroForwarder() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvForward, gpizRotatingToZero, GrabPuckInputZone, &GrabPuckInputZone::rotateToZero>
	> reactions;
};

struct gpizRotatingToZero : sc::state<gpizRotatingToZero, GrabPuckInputZone>
{
	gpizRotatingToZero(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizRotatingToZero");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setHavingPuck(true);
	} // entry

	~gpizRotatingToZero() {
	} // exit

	sc::result react(const EvSensorLostPuck &)
	{
		context<StateMachine1>().regainPuckReturn = RegainPuckReturn::gpizRotatingToZeroForwarder;
		return transit<RegainPuck>();
	}

	sc::result react(const EvMotorCtrlReady &)
	{
		//context<StateMachine1>().getStateBehavController()->getPathFinder()->clearGrid();
		//context<StateMachine1>().getStateBehavController()->getPathFinder()->increaseInputZonePucksGrabbed();
		if(context<GrabPuckInputZone>().grabLeft)
		{
			float newStartYPos = context<StateMachine1>().getStateBehavController()->getSensorControl()->getRobotY() - 100;
		}
		else
		{
			float newStartYPos = context<StateMachine1>().getStateBehavController()->getSensorControl()->getRobotY() + 100;
		}
		context<GrabPuckInputZone>().findPath();
		return transit<gpizWaitingForPath>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorLostPuck>,
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct gpizWaitingForPath : sc::state< gpizWaitingForPath, GrabPuckInputZone>
{
	gpizWaitingForPath(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizWaitingForPath");
	} // entry

	~gpizWaitingForPath() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvPathFound, gpizFinished>
		//sc::transition<EvPathDriven,lpbLeft>	// needed?
	> reactions;
};

struct gpizFinished : sc::state<gpizFinished, GrabPuckInputZone>
{
	gpizFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("gpizFinished");
		post_event(EvSuccess());
	} // entry

	~gpizFinished() {
	} // exit
};

#endif /* GRABPUCKINPUTZONE_H_ */
