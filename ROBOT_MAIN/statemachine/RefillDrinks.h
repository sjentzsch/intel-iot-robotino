/*
 * RefillDrinks.h
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#ifndef REFILLDRINKS_H_
#define REFILLDRINKS_H_

#include "StateMachine.h"

// States
struct refillInit;
struct refillRotatingToBaseStart;
struct refillDrivingToBaseStart;
struct refillRotatingToBase;
struct refillCalibratingOnBaseFront;
struct refillCalibratingOnBaseSide;
struct refillWaitForRefill;
struct refillFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    RefillDrinks
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct RefillDrinks : sc::state<RefillDrinks, StateMachine1, refillInit>
{
	RefillDrinks(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("RefillDrinks");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		msgEnvironment = new MsgEnvironment(DataProvider::getInstance()->getLatestMsgEnvironment());
	} // entry

	virtual ~RefillDrinks() {
		delete msgEnvironment;
	} // exit

	void rotateToBaseStart() {
		stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(RADTODEG(atan2(msgEnvironment->y_base_start*1000 - stateBehavCtrl->getSensorControl()->getRobotY(), msgEnvironment->x_base_start*1000 - stateBehavCtrl->getSensorControl()->getRobotX())));
	}

	void driveToBaseStart() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPosOnly(msgEnvironment->x_base_start*1000, msgEnvironment->y_base_start*1000);
	}

	void rotateToBase() {
		stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(msgEnvironment->phi_base);
	}

	void calibrateOnBaseFront() {
		stateBehavCtrl->getSensorControl()->calibrateOnBaseFront();
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment->x_base_start*1000, msgEnvironment->y_base_start*1000, msgEnvironment->phi_base);
	}

	void calibrateOnBaseSide() {
		stateBehavCtrl->getSensorControl()->calibrateOnBaseSide();
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment->x_base_start*1000, msgEnvironment->y_base_start*1000, msgEnvironment->phi_base);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::nextTask>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;
	MsgEnvironment* msgEnvironment;
};

struct refillInit : sc::state<refillInit, RefillDrinks>
{
	refillInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillInit");

		// TODO: TEST CODE
		cout << "wait for key pressed ..." << endl;
		cin.get(); cin.clear();

		post_event(EvInit());
	} // entry

	virtual ~refillInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<RefillDrinks>().rotateToBaseStart();
		return transit<refillRotatingToBaseStart>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct refillRotatingToBaseStart : sc::state<refillRotatingToBaseStart, RefillDrinks>
{
	refillRotatingToBaseStart(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillRotatingToBaseStart");
	} // entry

	virtual ~refillRotatingToBaseStart() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<RefillDrinks>().driveToBaseStart();
		return transit<refillDrivingToBaseStart>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct refillDrivingToBaseStart : sc::state<refillDrivingToBaseStart, RefillDrinks>
{
	refillDrivingToBaseStart(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillDrivingToBaseStart");
	} // entry

	virtual ~refillDrivingToBaseStart() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<RefillDrinks>().rotateToBase();
		return transit<refillRotatingToBase>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct refillRotatingToBase : sc::state<refillRotatingToBase, RefillDrinks>
{
	refillRotatingToBase(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillRotatingToBase");
	} // entry

	virtual ~refillRotatingToBase() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<RefillDrinks>().calibrateOnBaseFront();
		return transit<refillCalibratingOnBaseFront>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct refillCalibratingOnBaseFront : sc::state<refillCalibratingOnBaseFront, RefillDrinks>
{
	refillCalibratingOnBaseFront(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillCalibratingOnBaseFront");
	} // entry

	virtual ~refillCalibratingOnBaseFront() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<RefillDrinks>().calibrateOnBaseSide();
		return transit<refillCalibratingOnBaseSide>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct refillCalibratingOnBaseSide : sc::state<refillCalibratingOnBaseSide, RefillDrinks>
{
	refillCalibratingOnBaseSide(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillCalibratingOnBaseSide");
	} // entry

	virtual ~refillCalibratingOnBaseSide() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<refillWaitForRefill>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct refillWaitForRefill : sc::state<refillWaitForRefill, RefillDrinks>
{
	refillWaitForRefill(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillWaitForRefill");
	} // entry

	virtual ~refillWaitForRefill() {
	} // exit

	sc::result react(const EvSensorAllDrinksRefilled&)
	{
		return transit<refillFinished>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorAllDrinksRefilled>
	> reactions;
};

struct refillFinished : sc::state<refillFinished, RefillDrinks>
{
	refillFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillFinished");
		// wait for 2 seconds for a more natural behavior, not driving away immediately ...
		//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		post_event(EvSuccess());
	} // entry

	virtual ~refillFinished() {
	} // exit
};

#endif /* REFILLDRINKS_H_ */
