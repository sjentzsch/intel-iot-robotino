//
// StartupCalibration.h
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

#ifndef STARTUPCALIBRATION_H_
#define STARTUPCALIBRATION_H_

#include "StateMachine.h"

// States
struct startupInit;
struct startupCalibratingOnBaseFront;
struct startupCalibratingOnBaseSide;
struct startupDrivingToPointA;
struct startupDrivingToPointB;
struct startupFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    StartupCalibration
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct StartupCalibration : sc::state<StartupCalibration, StateMachine1, startupInit>
{
	StartupCalibration(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("StartupCalibration");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		msgEnvironment = new MsgEnvironment(DataProvider::getInstance()->getLatestMsgEnvironment());
	} // entry

	virtual ~StartupCalibration() {
		delete msgEnvironment;
	} // exit

	void calibrateOnBaseFront() {
		stateBehavCtrl->getSensorControl()->calibrateOnBaseFront();
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment->x_base_robot_start*1000, msgEnvironment->y_base_robot_start*1000, msgEnvironment->phi_base);
	}

	void calibrateOnBaseSide() {
		stateBehavCtrl->getSensorControl()->calibrateOnBaseSide();
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment->x_base_robot_start*1000, msgEnvironment->y_base_robot_start*1000, msgEnvironment->phi_base);
	}

	void driveToPointA() {
		// TODO: depends on base_dir!
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment->x_base_robot_start*1000-150, msgEnvironment->y_base_robot_start*1000+200, msgEnvironment->phi_base+180);
	}

	void driveToPointB() {
		// TODO: depends on base_dir!
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment->x_base_robot_start*1000+150, msgEnvironment->y_base_robot_start*1000+200, msgEnvironment->phi_base+180);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::nextTask>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;
	MsgEnvironment* msgEnvironment;
};

struct startupInit : sc::state<startupInit, StartupCalibration>
{
	startupInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("startupInit");
		post_event(EvInit());
	} // entry

	virtual ~startupInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<StartupCalibration>().calibrateOnBaseFront();
		return transit<startupCalibratingOnBaseFront>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct startupCalibratingOnBaseFront : sc::state<startupCalibratingOnBaseFront, StartupCalibration>
{
	startupCalibratingOnBaseFront(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("startupCalibratingOnBaseFront");
	} // entry

	virtual ~startupCalibratingOnBaseFront() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<StartupCalibration>().calibrateOnBaseSide();
		return transit<startupCalibratingOnBaseSide>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct startupCalibratingOnBaseSide : sc::state<startupCalibratingOnBaseSide, StartupCalibration>
{
	startupCalibratingOnBaseSide(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("startupCalibratingOnBaseSide");
	} // entry

	virtual ~startupCalibratingOnBaseSide() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<StartupCalibration>().driveToPointA();
		return transit<startupDrivingToPointA>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct startupDrivingToPointA : sc::state<startupDrivingToPointA, StartupCalibration>
{
	startupDrivingToPointA(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("startupDrivingToPointA");
	} // entry

	virtual ~startupDrivingToPointA() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<StartupCalibration>().driveToPointB();
		return transit<startupDrivingToPointB>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct startupDrivingToPointB : sc::state<startupDrivingToPointB, StartupCalibration>
{
	startupDrivingToPointB(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("startupDrivingToPointB");
	} // entry

	virtual ~startupDrivingToPointB() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<startupFinished>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct startupFinished : sc::state<startupFinished, StartupCalibration>
{
	startupFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("startupFinished");
		DataProvider::getInstance()->setRunning(false);
		post_event(EvSuccess());
	} // entry

	virtual ~startupFinished() {
	} // exit
};

#endif /* STARTUPCALIBRATION_H_ */
