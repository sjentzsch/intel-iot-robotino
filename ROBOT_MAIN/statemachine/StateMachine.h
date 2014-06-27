/*
 * StateMachine.h
 *
 *  Created on: 24.04.2011
 *      Author: root
 */

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
#include "model/WorldModel.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

//States
struct Init;
struct Dispatcher;

//Sub-State Machines
struct RandomMovement;
struct RefillDrinks;
struct ServeCustomer;

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
		stateBehavCtrl->getSensorControl()->setOdometry(msgEnvironment.x_robot*1000, msgEnvironment.y_robot*1000, msgEnvironment.phi_robot);

		/*while(true)
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));*/

		stateBehavCtrl->getTaskManager()->startSendBeacon();
		stateBehavCtrl->getTaskManager()->nextTask();
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
		sc::transition<EvServeCustomer,ServeCustomer>
		> reactions;
};

//finally include the substate machines
#include "RandomMovement.h"
#include "RefillDrinks.h"
#include "ServeCustomer.h"

#endif /* STATEMACHINE_H_ */
