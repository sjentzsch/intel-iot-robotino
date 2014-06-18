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
struct serveDrivingToCustomer;
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

	void driveToCustomer() {
		unsigned long curr_customer_id = stateBehavCtrl->getTaskManager()->getCurrCustomerOrder().customer_id;
		vector< MsgCustomerPos > vecMsgCustomerPoses = DataProvider::getInstance()->getMsgCustomerPoses();
		for(unsigned int i=0; i<vecMsgCustomerPoses.size(); i++)
		{
			if(curr_customer_id == vecMsgCustomerPoses.at(i).customer_id)
			{
				stateBehavCtrl->getMotorCtrl()->moveToAbsPos(vecMsgCustomerPoses.at(i).x*1000, vecMsgCustomerPoses.at(i).y*1000, 0, 300.0);
				break;
			}
		}
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
		context<ServeCustomer>().driveToCustomer();
		return transit<serveDrivingToCustomer>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
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
		context<ServeCustomer>().serveDrink();
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
