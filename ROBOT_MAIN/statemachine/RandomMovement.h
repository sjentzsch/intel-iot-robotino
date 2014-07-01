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
struct randRotatingToRandPos;
struct randDrivingToRandPos;
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

		// TODO: not nice here ...
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

		msgEnvironment = new MsgEnvironment(DataProvider::getInstance()->getLatestMsgEnvironment());

		float xMin = msgEnvironment->x_max / 3;
		float xMax = msgEnvironment->x_max - xMin;
		randX = xMin + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(xMax-xMin)));

		float yMin = msgEnvironment->y_max / 3;
		float yMax = msgEnvironment->y_max - yMin;
		randY = yMin + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(yMax-yMin)));

		FileLog::log_NOTICE("[StateMachine | RandomMovement] Random Point: ", std::to_string(randX), ", ", std::to_string(randY));

	} // entry

	virtual ~RandomMovement() {
	} // exit

	void rotateToRandPos() {
		stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(RADTODEG(atan2(randY*1000 - stateBehavCtrl->getSensorControl()->getRobotY(), randX*1000 - stateBehavCtrl->getSensorControl()->getRobotX())));
	}

	void driveToRandPos() {
		stateBehavCtrl->getMotorCtrl()->moveToAbsPosOnly(randX*1000, randY*1000);
	}

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::nextTask>
	> reactions;

private:
	StateBehaviorController* stateBehavCtrl;

	MsgEnvironment* msgEnvironment;
	float randX;
	float randY;
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
		context<RandomMovement>().rotateToRandPos();
		return transit<randRotatingToRandPos>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInit>
	> reactions;
};

struct randRotatingToRandPos : sc::state<randRotatingToRandPos, RandomMovement>
{
	randRotatingToRandPos(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("randRotatingToRandPos");
	} // entry

	virtual ~randRotatingToRandPos() {
	} // exit

	sc::result react(const EvMotorCtrlReady&)
	{
		context<RandomMovement>().driveToRandPos();
		return transit<randDrivingToRandPos>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvMotorCtrlReady>
	> reactions;
};

struct randDrivingToRandPos : sc::state<randDrivingToRandPos, RandomMovement>
{
	randDrivingToRandPos(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("randDrivingToRandPos");
	} // entry

	virtual ~randDrivingToRandPos() {
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
		// wait for 2 seconds for a more natural behavior, not driving away immediately ...
		//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		post_event(EvSuccess());
	} // entry

	virtual ~randFinished() {
	} // exit
};

#endif /* RANDOMMOVEMENT_H_ */
