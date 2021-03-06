//
// RandomMovement.h
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
		stateBehavCtrl->getMotorCtrl()->moveToAbsPosOnlyCF(randX*1000, randY*1000);
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
