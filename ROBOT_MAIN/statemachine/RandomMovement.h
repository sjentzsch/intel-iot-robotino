/*
 * RandomMovement.h
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#ifndef RANDOMMOVEMENT_H_
#define RANDOMMOVEMENT_H_

#include "StateMachine.h"

// States
struct randInit;
struct randDriving;
struct randFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    RandomMovement
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct RandomMovement : sc::state<RandomMovement, StateMachine1, randInit>
{
	RandomMovement(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("RandomMovement");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
	} // entry

	virtual ~RandomMovement() {
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

struct randInit : sc::state<randInit, RandomMovement>
{
	randInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("randInit");
		post_event(EvInit());
	} // entry

	virtual ~randInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<RandomMovement>().driveToRandPos();
		return transit<randDriving>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct randDriving : sc::state<randDriving, RandomMovement>
{
	randDriving(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("randDriving");
	} // entry

	virtual ~randDriving() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<randFinished>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct randFinished : sc::state<randFinished, RandomMovement>
{
	randFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("randFinished");
		post_event(EvSuccess());
	} // entry

	virtual ~randFinished() {
	} // exit
};

#endif /* RANDOMMOVEMENT_H_ */