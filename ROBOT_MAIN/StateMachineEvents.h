/*
 * StateMachineEvents.h
 *
 *  Created on: 27.05.2011
 *      Author: root
 */

#ifndef STATEMACHINEEVENTS_H_
#define STATEMACHINEEVENTS_H_

#include <boost/statechart/event.hpp>
#include "model/WorldModel.h"
#include "pathfinder/Grid.h"

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

// Motor Events
struct EvMotorCtrlReady : sc::event<EvMotorCtrlReady> {};

// Sensor Events
struct EvSensorFrontLeftFoundObstacle : sc::event<EvSensorFrontLeftFoundObstacle> {};
struct EvSensorFrontLeftIsFree : sc::event<EvSensorFrontLeftIsFree> {};
struct EvSensorFrontRightFoundObstacle : sc::event<EvSensorFrontRightFoundObstacle> {};
struct EvSensorFrontRightIsFree : sc::event<EvSensorFrontRightIsFree> {};

struct EvSensorFloorLeftIsBlack : sc::event<EvSensorFloorLeftIsBlack> {};
struct EvSensorFloorRightIsBlack : sc::event<EvSensorFloorRightIsBlack> {};

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

// Laserscanner and Obstacle Events
struct EvObstacleMap : sc::event<EvObstacleMap>{
	vector<Node*> obstacleNodes;
	EvObstacleMap(vector<Node*> obstacleNodes_):obstacleNodes(obstacleNodes_){};
	~EvObstacleMap(){};
};

struct EvObstacleIsClose : sc::event<EvObstacleIsClose> {
	float obsX;
	float obsY;
	EvObstacleIsClose(float obsX_, float obsY_):obsX(obsX_), obsY(obsY_){};
	~EvObstacleIsClose(){};
};

#endif /* STATEMACHINEEVENTS_H_ */
