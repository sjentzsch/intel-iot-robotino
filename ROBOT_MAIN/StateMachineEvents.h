//
// StateMachineEvents.h
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

#ifndef STATEMACHINEEVENTS_H_
#define STATEMACHINEEVENTS_H_

#include <boost/statechart/event.hpp>
#include "utils/pose.h"

namespace sc = boost::statechart;

// General purpose events
struct EvInit : sc::event<EvInit> {}; //Dummy Event for all state-machines
struct EvForward : sc::event<EvForward> {}; //Dummy Event for all state-machines
struct EvSuccess : sc::event<EvSuccess> {};
struct EvFailure: sc::event<EvFailure> {};

// PathFinder Events
struct EvPathFound: sc::event<EvPathFound> {
	float x,y,rotation;
	vector<struct vec3D> vias;
	EvPathFound(float x_,float y_,float rotation_, vector<struct vec3D> vias_):x(x_),y(y_),rotation(rotation_),vias(vias_){};
	~EvPathFound(){};
};
struct EvFinishedTrajSegment : sc::event<EvFinishedTrajSegment>{
	vector<struct vec3D> upComingVias;
	EvFinishedTrajSegment(vector<struct vec3D> upComingVias_):upComingVias(upComingVias_){};
	~EvFinishedTrajSegment(){};
};
struct EvNoPathFound : sc::event<EvNoPathFound> {};
struct EvPathDriven : sc::event<EvPathDriven> {};

// Tasks Events
struct EvRandomMovement : sc::event<EvRandomMovement> {};
struct EvRefillDrinks : sc::event<EvRefillDrinks> {};
struct EvServeCustomer : sc::event<EvServeCustomer> {};
struct EvStartupCalibration : sc::event<EvStartupCalibration> {};

// Motor Events
struct EvMotorCtrlReady : sc::event<EvMotorCtrlReady> {};

// Sensor Events
struct EvSensorFrontLeftFoundObstacle : sc::event<EvSensorFrontLeftFoundObstacle> {};
struct EvSensorFrontLeftIsFree : sc::event<EvSensorFrontLeftIsFree> {};
struct EvSensorFrontRightFoundObstacle : sc::event<EvSensorFrontRightFoundObstacle> {};
struct EvSensorFrontRightIsFree : sc::event<EvSensorFrontRightIsFree> {};

struct EvSensorFloorLeftIsBlack : sc::event<EvSensorFloorLeftIsBlack> {};
struct EvSensorFloorRightIsBlack : sc::event<EvSensorFloorRightIsBlack> {};

struct EvSensorDrinkTaken : sc::event<EvSensorDrinkTaken> {
	unsigned int number;
	EvSensorDrinkTaken(unsigned int number_):number(number_){};
	~EvSensorDrinkTaken(){};
};
struct EvSensorDrinkRefilled : sc::event<EvSensorDrinkRefilled> {
	unsigned int number;
	EvSensorDrinkRefilled(unsigned int number_):number(number_){};
	~EvSensorDrinkRefilled(){};
};
struct EvSensorAllDrinksRefilled : sc::event<EvSensorAllDrinksRefilled> {};

struct EvSensorDistance1Blocked : sc::event<EvSensorDistance1Blocked> {};
struct EvSensorDistance1Free : sc::event<EvSensorDistance1Free> {};
struct EvSensorDistance2Blocked : sc::event<EvSensorDistance2Blocked> {};
struct EvSensorDistance2Free : sc::event<EvSensorDistance2Free> {};
struct EvSensorDistance3Blocked : sc::event<EvSensorDistance3Blocked> {};
struct EvSensorDistance3Free : sc::event<EvSensorDistance3Free> {};
struct EvSensorDistance4Blocked : sc::event<EvSensorDistance4Blocked> {};
struct EvSensorDistance4Free : sc::event<EvSensorDistance4Free> {};
struct EvSensorDistance5Blocked : sc::event<EvSensorDistance5Blocked> {};
struct EvSensorDistance5Free : sc::event<EvSensorDistance5Free> {};
struct EvSensorDistance6Blocked : sc::event<EvSensorDistance6Blocked> {};
struct EvSensorDistance6Free : sc::event<EvSensorDistance6Free> {};
struct EvSensorDistance7Blocked : sc::event<EvSensorDistance7Blocked> {};
struct EvSensorDistance7Free : sc::event<EvSensorDistance7Free> {};
struct EvSensorDistance8Blocked : sc::event<EvSensorDistance8Blocked> {};
struct EvSensorDistance8Free : sc::event<EvSensorDistance8Free> {};
struct EvSensorDistance9Blocked : sc::event<EvSensorDistance9Blocked> {};
struct EvSensorDistance9Free : sc::event<EvSensorDistance9Free> {};

// Combined (intelligent) Events
struct EvAngleCalibration : sc::event<EvAngleCalibration> {
	float lineAngle;
	float diffAngle;
	EvAngleCalibration(float lineAngle_, float diffAngle_):lineAngle(lineAngle_), diffAngle(diffAngle_){};
	~EvAngleCalibration(){};
};

struct EvObstacleIsClose : sc::event<EvObstacleIsClose> {
	float obsX;
	float obsY;
	EvObstacleIsClose(float obsX_, float obsY_):obsX(obsX_), obsY(obsY_){};
	~EvObstacleIsClose(){};
};

#endif /* STATEMACHINEEVENTS_H_ */
