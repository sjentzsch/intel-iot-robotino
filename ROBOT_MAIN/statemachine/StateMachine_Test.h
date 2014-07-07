//
// StateMachine_Test.h
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Sören Jentzsch
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
struct testInit;
struct testDrivingForward;
struct Stop;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    StateMachine1
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct StateMachine1 : sc::state_machine<StateMachine1, testInit>
{
	StateMachine1(StateBehaviorController *stateBehavCtrl_):stateBehavCtrl(stateBehavCtrl_){}
	virtual ~StateMachine1(){}
	StateBehaviorController *getStateBehavController(){ return stateBehavCtrl;}

	float odometryX;
	float odometryY;
	float odometryPhi;

	float targetX;
	float targetY;

	void driveForwardCF() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPosOnlyCF(3000, 0);
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

struct testInit : sc::state<testInit, StateMachine1>
{
	testInit(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'testInit'");
		context<StateMachine1>().getStateBehavController()->getSensorControl()->setOdometry(0, 0, 0);
		context<StateMachine1>().getStateBehavController()->getTaskManager()->startSendBeacon();
		post_event(EvInit());
	} // entry

	virtual ~testInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<StateMachine1>().driveForwardCF();
		return transit<testDrivingForward>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct testDrivingForward : sc::state<testDrivingForward, StateMachine1>
{
	testDrivingForward(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'testDrivingForward'");
	} // entry

	virtual ~testDrivingForward() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<testDrivingForward>();
	}

	//Reactions
	typedef mpl::list<
		//sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct Stop : sc::state< Stop, StateMachine1>
{
	Stop(my_context ctx) : my_base(ctx) {
		FileLog::log_NOTICE("[StateMachine] Entered 'Stop'");
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->terminate();
	} // entry

	virtual ~Stop() {
	} // exit

	//Reactions
	typedef mpl::list<
	> reactions;
};

#endif /* STATEMACHINE_H_ */
