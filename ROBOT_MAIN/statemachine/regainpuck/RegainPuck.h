/*
 * RegainPuck.h
 *
 *  Created on: Jun 11, 2011
 *      Author: root
 */

#ifndef REGAINPUCK_H
#define REGAINPUCK_H
#include "../StateMachine.h"
#include "BaseParameterProvider.h"

//States
struct rpDriveSidewards;
struct rpCatchPuck;
struct rpSecurePuck;
struct rpFailureRepositioning;
struct rpFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    RegainPuck
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct RegainPuck : sc::state<RegainPuck, StateMachine1, rpDriveSidewards>
{
	RegainPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("RegainPuck");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		stateBehavCtrl->getMotorCtrl()->terminate();
		stateBehavCtrl->getSensorControl()->setCameraDetection(CameraPuckDetection::SEARCH_CATCH_STRATEGY, CameraLightDetection::OFF);

		if(BaseParameterProvider::getInstance()->getParams()->simulation_mode)
		{
			post_event(EvSuccess());
		}

	} // entry

	~RegainPuck() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	void driveLeft(const EvCameraCatchPuckLeft&) {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0, 200, 0, 50.0);
	}

	void driveRight(const EvCameraCatchPuckRight&) {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(0, -200, 0, 50.0);
	}

	void catchPuck() {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(160, 0, 0, 100.0);
	}

	void reposition() {
		stateBehavCtrl->getMotorCtrl()->moveToRelPos(-30, -40, 0, 100.0);
	}

	void securePuck() {
		if(context<StateMachine1>().regainPuckReturn != RegainPuckReturn::gpmFinishedForwarder)
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(55, 0, 0, 100.0);
		else
			stateBehavCtrl->getMotorCtrl()->moveToRelPos(45, 0, 0, 100.0);
	}

	sc::result react (const EvSuccess & evt)
	{
		switch(context<StateMachine1>().regainPuckReturn)
		{
		case RegainPuckReturn::dtpInit:
			return transit<dtpInit>();
		case RegainPuckReturn::gpizRotatingToZeroForwarder:
			return transit<gpizRotatingToZeroForwarder>();
		case RegainPuckReturn::gpmFinishedForwarder:
			return transit<gpmFinishedForwarder>();
		case RegainPuckReturn::dptgDrivingToLeftForwarder:
			return transit<dptgDrivingToLeftForwarder>();
		case RegainPuckReturn::dptgDrivingToRightForwarder:
			return transit<dptgDrivingToRightForwarder>();

		default:
			// TODO: transition to failure state, this transition should never happen
			return transit<Dispatcher>(&StateMachine1::taskSuccess, evt);
		}
	}

	sc::result react (const EvAngleCalibration &ev)
	{
		if(context<StateMachine1>().regainPuckReturn == RegainPuckReturn::gpizRotatingToZeroForwarder)
		{
			if(ev.diffAngle != 0)
				context<StateMachine1>().getStateBehavController()->getSensorControl()->calibrateAngle(ev.lineAngle+ev.diffAngle);
			context<StateMachine1>().getStateBehavController()->getSensorControl()->calibrateOnLineX(580);
			FileLog::log_NOTICE("###############################################");
			FileLog::log_NOTICE("--------------------> Angle Calibrated: diff Angle: ", FileLog::real(ev.diffAngle), ", corrected angle: ", FileLog::real(ev.lineAngle+ev.diffAngle));
			FileLog::log_NOTICE("###############################################");
			return discard_event();
		}
		else
			return defer_event();
	}

	typedef mpl::list<
		sc::custom_reaction<EvSuccess>,
		sc::custom_reaction<EvAngleCalibration>
		//sc::deferral<EvAngleCalibration>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;
};

struct rpDriveSidewards : sc::state<rpDriveSidewards, RegainPuck>
{
	rpDriveSidewards(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("rpDriveSidewards");
	} // entry

	~rpDriveSidewards() {
	} // exit

	sc::result react (const EvCameraInitialNoPuck&)
	{
		context<RegainPuck>().catchPuck();
		return transit<rpCatchPuck>();
	}

	sc::result react (const EvCameraLostPuckVision&)
	{
		context<RegainPuck>().catchPuck();
		return transit<rpCatchPuck>();
	}

	sc::result react (const EvCameraCatchPuckStraight&)
	{
		context<RegainPuck>().catchPuck();
		return transit<rpCatchPuck>();
	}

	sc::result react (const EvMotorCtrlReady&)
	{
		/*if(context<StateMachine1>().regainPuckReturn == RegainPuckReturn::gpizRotatingToZeroForwarder)
		{
			return transit<gpizCleanBackRightForwarder>();
		}
		else
		{*/
			context<RegainPuck>().reposition();
			return discard_event();
		//}
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::custom_reaction<EvCameraInitialNoPuck>,
		sc::custom_reaction<EvCameraLostPuckVision>,
		sc::transition<EvCameraCatchPuckLeft, rpDriveSidewards, RegainPuck, &RegainPuck::driveLeft>,
		sc::transition<EvCameraCatchPuckRight, rpDriveSidewards, RegainPuck, &RegainPuck::driveRight>,
		sc::custom_reaction<EvCameraCatchPuckStraight>,
		sc::deferral<EvSensorHasPuck>
	> reactions;
};

struct rpCatchPuck : sc::state<rpCatchPuck, RegainPuck>
{
	rpCatchPuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("rpCatchPuck");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // entry

	~rpCatchPuck() {
	} // exit

	sc::result react (const EvSensorHasPuck & evt)
	{
		if(context<StateMachine1>().gpmSidewards || context<StateMachine1>().regainPuckReturn != RegainPuckReturn::gpmFinishedForwarder)
		{
			context<RegainPuck>().securePuck();
			return transit<rpSecurePuck>();
		}
		else
			return discard_event();
	}

	sc::result react (const EvMotorCtrlReady & evt)
	{
		if(context<StateMachine1>().gpmSidewards || context<StateMachine1>().regainPuckReturn != RegainPuckReturn::gpmFinishedForwarder)
		{
			context<RegainPuck>().reposition();
			return transit<rpFailureRepositioning>();
			//return transit<rpDriveSidewards>();
		}
		else
		{
			context<RegainPuck>().securePuck();
			return transit<rpCatchPuck>();
		}
	}

	sc::result react (const EvSensorFrontLeftFoundObstacle&)
	{
		if(!context<StateMachine1>().gpmSidewards && context<StateMachine1>().regainPuckReturn == RegainPuckReturn::gpmFinishedForwarder)
		{
			context<RegainPuck>().securePuck();
			return transit<rpSecurePuck>();
		}
		else
			return discard_event();
	}

	sc::result react (const EvSensorFrontRightFoundObstacle&)
	{
		if(!context<StateMachine1>().gpmSidewards && context<StateMachine1>().regainPuckReturn == RegainPuckReturn::gpmFinishedForwarder)
		{
			context<RegainPuck>().securePuck();
			return transit<rpSecurePuck>();
		}
		else
			return discard_event();
	}

	//Reactions
	typedef mpl::list<
		//sc::transition<EvCameraCatchPuckLeft, rpDriveSidewards, RegainPuck, &RegainPuck::driveLeft>,
		//sc::transition<EvCameraCatchPuckRight, rpDriveSidewards, RegainPuck, &RegainPuck::driveRight>,
		sc::custom_reaction<EvSensorFrontLeftFoundObstacle>,
		sc::custom_reaction<EvSensorFrontRightFoundObstacle>,
		sc::custom_reaction<EvSensorHasPuck>,
		sc::custom_reaction<EvMotorCtrlReady>
		//sc::deferral<EvCameraCatchPuckLeft>,
		//sc::deferral<EvCameraCatchPuckRight>,
		//sc::deferral<EvCameraCatchPuckStraight>
	> reactions;
};

struct rpFailureRepositioning : sc::state<rpFailureRepositioning, RegainPuck>
{
	rpFailureRepositioning(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("rpFailureRepositioning");
	} // entry

	~rpFailureRepositioning() {
	} // exit

	sc::result react (const EvMotorCtrlReady&)
	{
		//context<RegainPuck>().catchPuck();
		//return transit<rpCatchPuck>();
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::SEARCH_CATCH_STRATEGY, CameraLightDetection::OFF);
		return transit<rpDriveSidewards>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>,
		sc::deferral<EvSensorHasPuck>
	> reactions;
};

struct rpSecurePuck : sc::state<rpSecurePuck, RegainPuck>
{
	rpSecurePuck(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("rpSecurePuck");
	} // entry

	~rpSecurePuck() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, rpFinished>
	> reactions;
};

struct rpFinished : sc::state<rpFinished, RegainPuck>
{
	rpFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("rpFinished");
		post_event(EvSuccess());
	} // entry

	~rpFinished() {
	} // exit
};

#endif /* REGAINPUCK_H */
