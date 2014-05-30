/*
 * StateMachine.h
 *
 *  Created on: 24.04.2011
 *      Author: root
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_
#include <iostream>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deferral.hpp>
#include <boost/mpl/list.hpp>

#include "../MotorController.h"
#include "../StateBehaviorController.h"
#include "../StateMachineEvents.h"
#include "utils/FileLogger.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

//States
struct CameraInit;
struct CameraDone;
struct DriveForward;
struct DriveInit;
struct Drive1;
struct Drive2;
struct Drive3;
struct Drive4;
struct Stop;
struct DriveOverBlackLine;
struct FirstLineCrossed;
struct DriveToLamp;



/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    StateMachine1
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct StateMachine1 : sc::state_machine<StateMachine1, CameraInit>
{
	StateMachine1(StateBehaviorController *stateBehavCtrl_, Grid *grid_):stateBehavCtrl(stateBehavCtrl_){}
	~StateMachine1(){}
	StateBehaviorController *getStateBehavController(){ return stateBehavCtrl;}

	float odometryX;
	float odometryY;
	float odometryPhi;

	float targetX;
	float targetY;

	void driveToLamp(const EvCameraLampDetected& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(ev.lampXPos, ev.lampYPos, 0, 100.0);
	}

	void saveTargetPos(const EvCameraLampDetected& ev){
		targetX = ev.lampXPos;
		targetY = ev.lampYPos;
	}

	float getOdometryX(){
		stateBehavCtrl->getSensorControl()->getRobotX();
	}
	
	void drive1(const EvInit& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(300, 300, 0, 200.0);
	}

	void drive2(const EvMotorCtrlReady& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(500, 300, 90, 200.0);
	}

	void drive3(const EvMotorCtrlReady& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(300, 0, 90, 200.0);
	}

	void drive4(const EvMotorCtrlReady& ev) {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(0, 0, 0, 200.0);
	}

private:
	StateBehaviorController *stateBehavCtrl;
};

struct CameraInit : sc::state< CameraInit, StateMachine1>
{
	CameraInit(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] CameraInit");
		//context<StateMachine1>().getStateBehavController()->getSensorControl()->setOdometry(0.0f, 0.0f, 0.0f);
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::NEAR);
	} // entry

	~CameraInit() {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
	} // exit

	sc::result react(const EvCameraLampDetected& ev)
	{
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(ev.lampXPos, ev.lampYPos, 0, 350.0);
		//return transit<CameraInit>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvCameraLampDetected>
		//sc::transition<EvInit, Drive1, StateMachine1, &StateMachine1::drive1>
	> reactions;
};

struct CameraDone : sc::state< CameraDone, StateMachine1>
{
	CameraDone(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] CameraDone");
	} // entry

	~CameraDone() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<CameraInit>();
	}

	//Reactions
	typedef mpl::list<
		//sc::custom_reaction<EvMotorCtrlReady>
		//sc::transition<EvInit, Drive1, StateMachine1, &StateMachine1::drive1>
	> reactions;
};

struct DriveForward : sc::state< DriveForward, StateMachine1>
{
	DriveForward(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveForward'");
		//context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);

		context<StateMachine1>().getStateBehavController()->getSensorControl()->setOdometry(0, 0, 0);
		vector<vec3D> vPoints;
		vPoints.push_back(vec3D(500, 0, 0));
		vPoints.push_back(vec3D(500, 500, 0));
		vPoints.push_back(vec3D(0, 500, 0));
		vPoints.push_back(vec3D(0, 0, 0));
		/*vPoints.push_back(vec3D(300, 300, 0));
		vPoints.push_back(vec3D(500, 300, 90));
		vPoints.push_back(vec3D(300, 0, 90));
		vPoints.push_back(vec3D(0, 0, 0));*/
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(vPoints,150.0f);
	} // entry

	~DriveForward() {
	} // exit

	//Reactions
	typedef mpl::list<
		//sc::transition<EvCameraLampDetected, DriveForward, StateMachine1, &StateMachine1::driveToLamp>
		sc::transition<EvCameraLampDetected, DriveToLamp, StateMachine1, &StateMachine1::saveTargetPos>
		//sc::transition<EvMotorCtrlReady, Stop>
		//sc::transition<EvSensorFrontLeftFoundObstacle, DriveLeft>,
		//sc::transition<EvSensorFrontRightFoundObstacle, DriveRight>
	> reactions;
};

struct DriveOverBlackLine: sc::state<DriveOverBlackLine, StateMachine1> {
	DriveOverBlackLine(my_context ctx) :
		my_base(ctx) {
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
		context<StateMachine1>().odometryX = context<StateMachine1>().getOdometryX();
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveOverBlackLine'");
		cout << "####################### Lamp Event Successfull#########" << endl;
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToRelPos(1000,0,0,100);
	} // entry

	~DriveOverBlackLine() {
	} // exit

	//Reactions
		typedef mpl::list<
			sc::transition<EvMotorCtrlReady, Stop>,
			sc::transition<EvSensorFrontLeftFoundObstacle, Stop>,
			sc::transition<EvSensorFrontRightFoundObstacle, Stop>,
			sc::transition<EvSensorFloorLeftIsBlack, FirstLineCrossed>,
			sc::transition<EvSensorFloorRightIsBlack, FirstLineCrossed>
		> reactions;


};

struct FirstLineCrossed: sc::state<FirstLineCrossed, StateMachine1> {

	FirstLineCrossed(my_context ctx) :
		my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered 'FirstLineCrossed'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToRelPos(1000,0,0,100);
	} // entry

	~FirstLineCrossed() {
	} // exit

	//Reactions
		typedef mpl::list<
			sc::transition<EvMotorCtrlReady, Stop>,
			sc::transition<EvSensorFrontLeftFoundObstacle, Stop>,
			sc::transition<EvSensorFrontRightFoundObstacle, Stop>,
			sc::transition<EvSensorFloorLeftIsBlack, DriveToLamp>,
			sc::transition<EvSensorFloorRightIsBlack, DriveToLamp>
		> reactions;

};

struct DriveToLamp: sc::state<DriveToLamp, StateMachine1> {

	DriveToLamp(my_context ctx) :
		my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveToLamp'");
		float currentX = context<StateMachine1>().getOdometryX();
		float diff = context<StateMachine1>().odometryX - currentX;
		cout << "context<StateMachine1>().targetX" << context<StateMachine1>().targetX << endl;
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToRelPos(140, context<StateMachine1>().targetY+10, 0, 200);
	} // entry

	~DriveToLamp() {
	} // exit

	//Reactions
		typedef mpl::list<
			sc::transition<EvMotorCtrlReady, Stop>
			//sc::transition<EvSensorFrontLeftFoundObstacle, Stop>,
			//sc::transition<EvSensorFrontRightFoundObstacle, Stop>
			//sc::transition<EvSensorFloorLeftIsBlack, Stop>,
			//sc::transition<EvSensorFloorRightIsBlack, Stop>
		> reactions;

};


struct DriveInit : sc::state< DriveInit, StateMachine1>
{
	DriveInit(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered DriveInit");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setOdometry(0, 0, 0);
		post_event(EvInit());
		//context<StateMachine1>().getStateBehavController()->getMotorCtrl()->move(0,-1000,0,130.0f);
	} // entry

	~DriveInit() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvInit, Drive1, StateMachine1, &StateMachine1::drive1>
	> reactions;
};

struct Drive1 : sc::state< Drive1, StateMachine1>
{
	Drive1(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered 'Drive1'");
		//context<StateMachine1>().getStateBehavController()->getMotorCtrl()->move(0,-1000,0,130.0f);
	} // entry

	~Drive1() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, Drive2, StateMachine1, &StateMachine1::drive2>
	> reactions;
};

struct Drive2 : sc::state< Drive2, StateMachine1>
{
	Drive2(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered Drive2");
		//context<StateMachine1>().getStateBehavController()->getMotorCtrl()->move(0,1000,0,130.0f);
	} // entry

	~Drive2() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, Drive3, StateMachine1, &StateMachine1::drive3>
	> reactions;
};

struct Drive3 : sc::state< Drive3, StateMachine1>
{
	Drive3(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered Drive3");
		//context<StateMachine1>().getStateBehavController()->getMotorCtrl()->move(0,1000,0,130.0f);
	} // entry

	~Drive3() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, Drive4, StateMachine1, &StateMachine1::drive4>
	> reactions;
};

struct Drive4 : sc::state< Drive4, StateMachine1>
{
	Drive4(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered Drive4");
		//context<StateMachine1>().getStateBehavController()->getMotorCtrl()->move(0,1000,0,130.0f);
	} // entry

	~Drive4() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, Stop>
	> reactions;
};

struct Stop : sc::state< Stop, StateMachine1>
{
	Stop(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'Stop'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->terminate();
	} // entry

	~Stop() {
	} // exit

	//Reactions
	typedef mpl::list<
	> reactions;
};

#endif /* STATEMACHINE_H_ */
