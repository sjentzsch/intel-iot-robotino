/*
 * ServeCustomer.h
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#ifndef SERVECUSTOMER_H_
#define SERVECUSTOMER_H_

#include "StateMachine.h"

// States
struct serveInit;
struct serveDriving;
struct serveFinished;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    ServeCustomer
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct ServeCustomer : sc::state<ServeCustomer, StateMachine1, serveInit>
{
	ServeCustomer(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("ServeCustomer");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
	} // entry

	virtual ~ServeCustomer() {
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

struct serveInit : sc::state<serveInit, ServeCustomer>
{
	serveInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("serveInit");
		post_event(EvInit());
	} // entry

	virtual ~serveInit() {
	} // exit

	sc::result react(const EvInit&)
	{
		context<ServeCustomer>().driveToRandPos();
		return transit<serveDriving>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct serveDriving : sc::state<serveDriving, ServeCustomer>
{
	serveDriving(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("serveDriving");
	} // entry

	virtual ~serveDriving() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<serveFinished>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct serveFinished : sc::state<serveFinished, ServeCustomer>
{
	serveFinished(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("serveFinished");
		post_event(EvSuccess());
	} // entry

	virtual ~serveFinished() {
	} // exit
};

#endif /* SERVECUSTOMER_H_ */
