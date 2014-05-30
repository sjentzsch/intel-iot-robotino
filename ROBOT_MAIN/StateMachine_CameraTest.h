/*
 * StateMachine.h
 *
 *  Created on: 24.04.2011
 *      Author: root
 */

#ifndef STATEMACHINECAMERATEST_H_
#define STATEMACHINECAMERATEST_H_
#include <iostream>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include "RobotCamera.h"
#include "MotorController.h"
#include "StateBehaviorController.h"
#include "StateMachineEvents.h"
#include "FileLogger.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

//States
struct DriveToA; //Initial state
struct DriveToB;
struct DriveToC;
struct DriveToD;
struct DriveToE;


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    StateMachine1
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct StateMachine1 : sc::state_machine<StateMachine1, DriveToA>
{
	StateMachine1(StateBehaviorController *stateBehavCtrl_):stateBehavCtrl(stateBehavCtrl_){}
	~StateMachine1(){}
	StateBehaviorController *getStateBehavController(){ return stateBehavCtrl;}
private:
	StateBehaviorController *stateBehavCtrl;
};

struct DriveToA : sc::state< DriveToA, StateMachine1>
{
	DriveToA(my_context ctx) : my_base(ctx) {
		//cout << "Entered state DriveToA!\n";
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveToA'");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setCameraDetection(CameraPuckDetection::ALL, CameraLightDetection::OFF);
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(700,560,180);
	} // entry

	~DriveToA() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, DriveToB>
	> reactions;
};

struct DriveToB : sc::state< DriveToB, StateMachine1>
{
	DriveToB(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveToB'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(700,2600,180);
	} // entry

	~DriveToB() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, DriveToC>
	> reactions;
};

struct DriveToC : sc::state< DriveToC, StateMachine1>
{
	DriveToC(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveToC'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(700,1600,0);
	} // entry

	~DriveToC() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, DriveToD>
	> reactions;
};

struct DriveToD : sc::state< DriveToD, StateMachine1>
{
	DriveToD(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveToD'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(700,500,0);
	} // entry

	~DriveToD() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, DriveToE>
	> reactions;
};

struct DriveToE : sc::state< DriveToE, StateMachine1>
{
	DriveToE(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'DriveToE'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->moveToAbsPos(0,0,0);
	} // entry

	~DriveToE() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvMotorCtrlReady, DriveToA>
	> reactions;
};

#endif /* STATEMACHINECAMERATEST_H_ */
