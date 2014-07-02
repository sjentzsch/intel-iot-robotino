//
// ServeCustomer.h
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
		// wait for 2 seconds for a more natural behavior, not driving away immediately ...
		//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		post_event(EvSuccess());
	} // entry

	virtual ~serveFinished() {
	} // exit
};

#endif /* SERVECUSTOMER_H_ */
