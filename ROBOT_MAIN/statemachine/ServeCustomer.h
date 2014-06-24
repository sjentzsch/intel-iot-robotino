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
struct serveRotatingToCustomer;
struct serveDrivingToCustomer;
struct serveWaitForPickup;
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

		curr_customer_id = stateBehavCtrl->getTaskManager()->getCurrCustomerOrder().customer_id;
		vector< MsgCustomerPos > vecMsgCustomerPoses = DataProvider::getInstance()->getMsgCustomerPoses();
		for(unsigned int i=0; i<vecMsgCustomerPoses.size(); i++)
		{
			if(curr_customer_id == vecMsgCustomerPoses.at(i).customer_id)
			{
				msgCustomerPos = new MsgCustomerPos(vecMsgCustomerPoses.at(i));
				break;
			}
		}
	} // entry

	virtual ~ServeCustomer() {
	} // exit

	void rotateToCustomer() {
		stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(RADTODEG(atan2(msgCustomerPos->y*1000 - stateBehavCtrl->getSensorControl()->getRobotY(), msgCustomerPos->x*1000 - stateBehavCtrl->getSensorControl()->getRobotX())));
	}

	void driveToCustomer() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPosOnly(msgCustomerPos->x*1000, msgCustomerPos->y*1000);
	}

	void serveDrink() {
		stateBehavCtrl->getTaskManager()->serveDrink();
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::nextTask>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;

	unsigned long curr_customer_id;
	MsgCustomerPos* msgCustomerPos;
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
		context<ServeCustomer>().rotateToCustomer();
		return transit<serveRotatingToCustomer>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct serveRotatingToCustomer : sc::state<serveRotatingToCustomer, ServeCustomer>
{
	serveRotatingToCustomer(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("serveRotatingToCustomer");
	} // entry

	virtual ~serveRotatingToCustomer() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<ServeCustomer>().driveToCustomer();
		return transit<serveDrivingToCustomer>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct serveDrivingToCustomer : sc::state<serveDrivingToCustomer, ServeCustomer>
{
	serveDrivingToCustomer(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("serveDrivingToCustomer");
	} // entry

	virtual ~serveDrivingToCustomer() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		return transit<serveWaitForPickup>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct serveWaitForPickup : sc::state<serveWaitForPickup, ServeCustomer>
{
	serveWaitForPickup(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("serveWaitForPickup");
	} // entry

	virtual ~serveWaitForPickup() {
	} // exit

	sc::result react(const EvSensorDrinkTaken &ev)
	{
		context<ServeCustomer>().serveDrink();
		return transit<serveFinished>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorDrinkTaken>
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
