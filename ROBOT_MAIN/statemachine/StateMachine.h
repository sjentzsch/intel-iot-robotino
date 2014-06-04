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

//Substate-Machines
//

//States
struct Init;
struct Dispatcher;

//Sub-State Machines
struct RegainPuck;
struct DriveToPoi;
struct GrabPuckInputZone;
struct GrabPuckMachine;
struct GrabPuckSideOnFront;
struct DeliverPuckToPoi;
struct DeliverPuckToGate;

struct LeavePoiBackwards;
struct LeavePoiWithPuck;
struct LeavePoiWithPuckRotatingOut;
struct LeavePoiWithoutPuck;

struct WaitForRPC;
struct DriveToRPC;

struct MoveToAndCheckRPC;

namespace RegainPuckReturn
{
	enum RegainPuckReturn{dtpInit, gpizRotatingToZeroForwarder, gpmFinishedForwarder, wfrInitMountRPC, dptgDrivingToLeftForwarder, dptgDrivingToRightForwarder, lrpcwpLeavingForwarder};
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    StateMachine1
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct StateMachine1 : sc::state_machine<StateMachine1, Init>
{
	StateMachine1(StateBehaviorController *stateBehavCtrl_, Grid *grid_):stateBehavCtrl(stateBehavCtrl_),grid(grid_){
		poiTo = NULL;
		poiFrom = NULL;
		havePathToPOI = false;
		pathToPOI = new EvPathFound(0, 0, 0, vector<vec3D>());
		isContinueDelivering = false;
		gpmSidewards = false;
		gpmRec = false;
	}
	~StateMachine1()
	{
		if(grid != NULL)
			delete grid;
	}

	StateBehaviorController* getStateBehavController() {return stateBehavCtrl;}
	Grid* getGrid() {return grid;}

	bool gpmSidewards;
	bool gpmRec;
	bool isContinueDelivering;	// true after red light but continue driving to the machine
	bool driveWithPuck;
	POI* poiFrom;
	POIAccessFrom::POIAccessFrom accessDirectionFrom;
	POI* poiTo;
	POIAccessFrom::POIAccessFrom accessDirectionTo;

	bool havePathToPOI;
	EvPathFound* pathToPOI;

	LeaveDirection::LeaveDirection leaveDir;

	Node* accessNode;

	RegainPuckReturn::RegainPuckReturn regainPuckReturn;

	// TODO: need to add '\0' to the end or not?
	void logAndDisplayStateName(const char* description){
		//strcpy(&DynDataProvider::getInstance()->getStateInformation(ModelProvider::getInstance()->getID())->description[0], description);
		//communication::sendStateInformation();
		FileLog::log_NOTICE("[StateMachine] Entered '", description, "'");
	}

	void nextTask(){
		stateBehavCtrl->getJobHandler()->nextTask();
	}

	void continueDelivering(const EvContinueDeliveringPuck&){
		isContinueDelivering = true;
	}

	//if a task has successfully been completed
	void taskSuccess(const EvSuccess&){
		cout << "taskSuccess triggered nextTask" << endl;
		nextTask();
	}

	void puckDetectedRPC(const EvCameraPuckDetected&){
		cout << "puckDetectedRPC triggered nextTask" << endl;
		nextTask();
	}

	void findTaskInit(const EvInitJobPlanner&){
		cout << "findTaskInit triggered nextTask" << endl;
		nextTask();
	}

	void findTaskWaiting(const EvNoJobFound&){
		cout << "findTaskWaiting triggered nextTask" << endl;
		nextTask();
	}

	void driveToPoi(const EvDriveToPoi& ev){
		poiTo = ev.poiTo;
		poiFrom = ev.poiFrom;
		driveWithPuck = ev.driveWithPuck;
		accessDirectionTo = ev.accDir;
	}

	void grabPuckInputZone(const EvGrabPuckInputZone& ev){
		poiFrom = ev.poiFrom;
		poiTo = ev.poiTo;
		accessDirectionTo = ev.accDirTo;
	}

	void deliverPuckToGate(const EvDeliverPuckToGate& ev){
		accessNode = ev.node;
	}

	void grabPuckMachine(const EvGrabPuckMachine& ev){
		poiFrom = ev.poiFrom;
		poiTo = ev.poiTo;
		gpmSidewards = ev.gpmSidewards;
		gpmRec = ev.gpmRec;
	}

	void grabPuckSideOnFront(const EvGrabPuckSideOnFront& ev){
		poiFrom = ev.poiFrom;
		leaveDir = ev.leaveDir;
	}

	void leavePoiWithPuck(const EvLeavePoiWithPuck& ev){
		poiFrom = ev.poiFrom;
		accessDirectionFrom = ev.accDirFrom;

		poiTo = ev.poiTo;
		accessDirectionTo = ev.accDirTo;
	}

	void LeavePoiWithPuckRotatingOut(const EvLeavePoiWithPuckRotatingOut& ev){
		poiFrom = ev.poiFrom;
	}

	void leavePoiBackwards(const EvLeavePoiBackwards& ev){
		poiFrom = ev.poiFrom;

		poiTo = ev.poiTo;
		accessDirectionTo = ev.accDirTo;
	}

	void leavePoiWithoutPuck(const EvLeavePoiWithoutPuck& ev){
		poiFrom = ev.poiFrom;
		accessDirectionFrom = ev.accDirFrom;
		leaveDir = ev.leaveDir;

		poiTo = ev.poiTo;
		accessDirectionTo = ev.accDirTo;
	}


private:
	StateBehaviorController *stateBehavCtrl;
	Grid *grid;
};

struct Init : sc::state< Init, StateMachine1>
{
	Init(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("Init");
		boost::shared_ptr<EvInitJobPlanner> ev(new EvInitJobPlanner());
		context<StateMachine1>().getStateBehavController()->getAsyncStateMachine()->queueEvent(ev);

	} // entry

	~Init() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvInitJobPlanner, Dispatcher, StateMachine1, &StateMachine1::findTaskInit>
	> reactions;
};

struct Dispatcher : sc::state< Dispatcher, StateMachine1>
{
	Dispatcher(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("Dispatcher");
	} // entry

	~Dispatcher() {
	} // exit

	//Reactions
	typedef mpl::list<
		sc::transition<EvNoJobFound,Dispatcher,StateMachine1, &StateMachine1::findTaskWaiting>,
		sc::transition<EvDriveToPoi,DriveToPoi,StateMachine1, &StateMachine1::driveToPoi>,
		sc::transition<EvGrabPuckInputZone,GrabPuckInputZone,StateMachine1, &StateMachine1::grabPuckInputZone>,
		sc::transition<EvGrabPuckMachine,GrabPuckMachine,StateMachine1, &StateMachine1::grabPuckMachine>,
		sc::transition<EvGrabPuckSideOnFront,GrabPuckSideOnFront,StateMachine1, &StateMachine1::grabPuckSideOnFront>,
		sc::transition<EvDeliverPuckToPoi,DeliverPuckToPoi>,
		sc::transition<EvDeliverPuckToGate,DeliverPuckToGate,StateMachine1, &StateMachine1::deliverPuckToGate>,
		sc::transition<EvLeavePoiBackwards,LeavePoiBackwards,StateMachine1, &StateMachine1::leavePoiBackwards>,
		sc::transition<EvLeavePoiWithPuck,LeavePoiWithPuck,StateMachine1, &StateMachine1::leavePoiWithPuck>,
		sc::transition<EvLeavePoiWithoutPuck,LeavePoiWithoutPuck,StateMachine1, &StateMachine1::leavePoiWithoutPuck>,
		sc::transition<EvLeavePoiWithPuckRotatingOut,LeavePoiWithPuckRotatingOut,StateMachine1, &StateMachine1::LeavePoiWithPuckRotatingOut>,

		// if JobHandler decided to continue driving to the machine if red light was seen
		sc::transition<EvContinueDeliveringPuck,DeliverPuckToPoi,StateMachine1,&StateMachine1::continueDelivering>
		> reactions;
};

//finally include the substate machines
#include "drivetopoi/DriveToPoi.h"
#include "grabpuck/GrabPuckInputZone.h"
#include "grabpuck/GrabPuckMachine.h"
#include "grabpuck/GrabPuckSideOnFront.h"
#include "deliverpuck/DeliverPuckToPoi.h"
#include "deliverpuck/DeliverPuckToGate.h"
#include "leavepoi/LeavePoiBackwards.h"
#include "leavepoi/LeavePoiWithPuck.h"
#include "leavepoi/LeavePoiWithPuckRotatingOut.h"
#include "leavepoi/LeavePoiWithoutPuck.h"

#include "regainpuck/RegainPuck.h"
#endif /* STATEMACHINE_H_ */
