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
struct refillDrivingToBaseStart;
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
	} // entry

	virtual ~RefillDrinks() {
	} // exit

	void driveToBaseStart() {
		MsgEnvironment msgEnvironment = DataProvider::getInstance()->getLatestMsgEnvironment();
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(msgEnvironment.x_base*1000, msgEnvironment.y_base*1000, msgEnvironment.phi_base, 100.0);
	}

	void refillDrinks() {
		stateBehavCtrl->getTaskManager()->refillDrinks();
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::nextTask>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;
};

struct refillInit : sc::state<refillInit, RefillDrinks>
{
	refillInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillInit");
		post_event(EvInit());
	} // entry

	virtual ~refillInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<RefillDrinks>().driveToBaseStart();
		return transit<refillDrivingToBaseStart>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
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
		context<RefillDrinks>().refillDrinks();
		return transit<refillFinished>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct refillFinished : sc::state<refillFinished, RefillDrinks>
{
	refillFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillFinished");
		post_event(EvSuccess());
	} // entry

	virtual ~refillFinished() {
	} // exit
};

#endif /* REFILLDRINKS_H_ */
