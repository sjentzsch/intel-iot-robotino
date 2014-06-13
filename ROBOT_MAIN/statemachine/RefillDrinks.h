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
struct refillDriving;
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

	void driveToRandPos() {
		// stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(0, ForceRotationDirection::LEFT, 50.0f);
		stateBehavCtrl->getMotorCtrl()->moveToAbsPos(/*accessNode->getXPos()-*/370, /*startYPos*/0, 180, 300.0);
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
		context<RefillDrinks>().driveToRandPos();
		return transit<refillDriving>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct refillDriving : sc::state<refillDriving, RefillDrinks>
{
	refillDriving(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("refillDriving");
	} // entry

	virtual ~refillDriving() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
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
