//
// StateMachine.h
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

#include "config.h"
#include "../MotorController.h"
#include "../StateBehaviorController.h"
#include "../StateMachineEvents.h"
#include "utils/FileLogger.h"
#include "../StateBehaviorController.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

//States
struct Init;
struct Dispatcher;

//Sub-State Machines
struct RandomMovement;
struct RefillDrinks;
struct ServeCustomer;
struct StartupCalibration;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    StateMachine1
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct StateMachine1 : sc::state_machine<StateMachine1, Init>
{
	StateMachine1(StateBehaviorController *stateBehavCtrl_):stateBehavCtrl(stateBehavCtrl_) {}

	~StateMachine1() {}

	StateBehaviorController* getStateBehavController() {return stateBehavCtrl;}

	void logAndDisplayStateName(std::string newState){
		stateBehavCtrl->getTaskManager()->setCurrState(newState);
		FileLog::log_NOTICE("[StateMachine] Entered '", newState, "'");
	}

	void nextTask(const EvInit&){
		MsgEnvironment msgEnvironment = DataProvider::getInstance()->getLatestMsgEnvironment();
		stateBehavCtrl->getSensorControl()->setOdometry(msgEnvironment.x_base_robot_start*1000, msgEnvironment.y_base_robot_start*1000, msgEnvironment.phi_base);

		/*while(true)
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));*/

		stateBehavCtrl->getTaskManager()->startSendBeacon();
		post_event(EvStartupCalibration());
	}

	void nextTask(const EvSuccess&){
		stateBehavCtrl->getTaskManager()->nextTask();
	}

private:
	StateBehaviorController *stateBehavCtrl;
};

struct Init : sc::state< Init, StateMachine1>
{
	Init(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("Init");
		post_event(EvInit());
	} // entry

	virtual ~Init() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvInit, Dispatcher, StateMachine1, &StateMachine1::nextTask>
	> reactions;
};

struct Dispatcher : sc::state< Dispatcher, StateMachine1>
{
	Dispatcher(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("Dispatcher");
	} // entry

	virtual ~Dispatcher() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvRandomMovement,RandomMovement>,
		sc::transition<EvRefillDrinks,RefillDrinks>,
		sc::transition<EvServeCustomer,ServeCustomer>,
		sc::transition<EvStartupCalibration,StartupCalibration>
		> reactions;
};

//finally include the substate machines
#include "RandomMovement.h"
#include "RefillDrinks.h"
#include "ServeCustomer.h"
#include "StartupCalibration.h"

#endif /* STATEMACHINE_H_ */
