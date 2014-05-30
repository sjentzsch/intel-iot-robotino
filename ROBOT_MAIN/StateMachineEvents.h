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


namespace LeaveDirection
{
	enum LeaveDirection {LEFT=0, RIGHT=1};
	static const char *cLeaveDirection[2] = {"LEFT", "RIGHT"};
}

//general purpose event for initializing a state
struct EvInit : sc::event<EvInit> {}; //Dummy Event for all state-machines
struct EvForward : sc::event<EvForward> {}; //Dummy Event for all state-machines

//PathFinder Events
struct EvInitPathFinder : sc::event<EvInitPathFinder> {}; //Dummy Event for pathfinder
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

struct EvPathFoundLeavePoi: sc::event<EvPathFoundLeavePoi> {
	float x,y,rotation;
	LeaveDirection::LeaveDirection leaveDir;
	EvPathFoundLeavePoi(float x_,float y_,float rotation_,LeaveDirection::LeaveDirection leaveDir_):x(x_),y(y_),rotation(rotation_),leaveDir(leaveDir_){};
	~EvPathFoundLeavePoi(){};
};

struct EvAccessTripletReserved : sc::event<EvAccessTripletReserved> {};
struct EvAccessZoneReserved : sc::event<EvAccessZoneReserved> {};
//General events of a task
struct EvSuccess : sc::event<EvSuccess> {};
struct EvFailure: sc::event<EvFailure> {};

//JobPlanner Events
struct EvInitJobPlanner : sc::event<EvInitJobPlanner> {}; //Dummy Event for JobPlanner

struct EvWaitForRPC : sc::event<EvWaitForRPC> {};
struct EvDriveToRPC : sc::event<EvDriveToRPC> {};
struct EvContinueDeliveringPuck : sc::event<EvContinueDeliveringPuck> {};

struct EvDriveToPoi : sc::event<EvDriveToPoi> {
	POI* poiTo;
	POI* poiFrom;
	bool driveWithPuck;
	POIAccessFrom::POIAccessFrom accDir;
	EvDriveToPoi(POI* poiTo_,POI* poiFrom_,bool driveWithPuck_,POIAccessFrom::POIAccessFrom accDir_):poiTo(poiTo_),poiFrom(poiFrom_),driveWithPuck(driveWithPuck_),accDir(accDir_){};
	~EvDriveToPoi(){};
};

struct EvLeavePoiBackwards : sc::event<EvLeavePoiBackwards> {
	POI* poiFrom;
	POI* poiTo;
	POIAccessFrom::POIAccessFrom accDirTo;
	EvLeavePoiBackwards(POI* poiFrom_, POI* targetPOI_, POIAccessFrom::POIAccessFrom targetAccDir_):poiFrom(poiFrom_),poiTo(targetPOI_),accDirTo(targetAccDir_){};
	EvLeavePoiBackwards(){};
};

struct EvLeavePoiWithoutPuck : sc::event<EvLeavePoiWithoutPuck> {
	POI* poiFrom;
	POIAccessFrom::POIAccessFrom accDirFrom;
	LeaveDirection::LeaveDirection leaveDir;
	POI* poiTo;
	POIAccessFrom::POIAccessFrom accDirTo;
	EvLeavePoiWithoutPuck(POI* poiFrom_,POIAccessFrom::POIAccessFrom accDirFrom_,LeaveDirection::LeaveDirection leaveDir_,POI* poiTo_,POIAccessFrom::POIAccessFrom accDirTo_):poiFrom(poiFrom_),accDirFrom(accDirFrom_),leaveDir(leaveDir_),poiTo(poiTo_),accDirTo(accDirTo_){};
	EvLeavePoiWithoutPuck(){};
};

struct EvLeavePoiWithPuckRotatingOut : sc::event<EvLeavePoiWithPuckRotatingOut> {
	POI* poiFrom;
	EvLeavePoiWithPuckRotatingOut(POI* poiFrom_):poiFrom(poiFrom_){};
	EvLeavePoiWithPuckRotatingOut(){};
};

struct EvLeavePoiWithPuck : sc::event<EvLeavePoiWithPuck> {
	POI* poiFrom;
	POI* poiTo;
	POIAccessFrom::POIAccessFrom accDirFrom;
	POIAccessFrom::POIAccessFrom accDirTo;
	EvLeavePoiWithPuck(POI* poiFrom_,POIAccessFrom::POIAccessFrom accDirFrom_,POI* poiTo_,POIAccessFrom::POIAccessFrom accDirTo_):poiFrom(poiFrom_),poiTo(poiTo_),accDirFrom(accDirFrom_),accDirTo(accDirTo_){};
	EvLeavePoiWithPuck(){};
};

struct EvLeaveRPCVM : sc::event<EvLeaveRPCVM> {};

struct EvLeaveRPCWithPuck : sc::event<EvLeaveRPCWithPuck> {};

struct EvGrabPuckMachine  : sc::event<EvGrabPuckMachine> {
	POI* poiFrom;
	POI* poiTo;
	bool gpmSidewards;
	bool gpmRec;
	EvGrabPuckMachine(POI* poiFrom_, POI* poiTo_, bool gpmSidewards_, bool gpmRec_):poiFrom(poiFrom_),poiTo(poiTo_),gpmSidewards(gpmSidewards_),gpmRec(gpmRec_){};
};

struct EvGrabPuckSideOnFront  : sc::event<EvGrabPuckSideOnFront> {
	POI* poiFrom;
	LeaveDirection::LeaveDirection leaveDir;
	EvGrabPuckSideOnFront(POI* poiFrom_, LeaveDirection::LeaveDirection leaveDir_):poiFrom(poiFrom_),leaveDir(leaveDir_){};
};

struct EvGrabPuckInputZone  : sc::event<EvGrabPuckInputZone> {
	POI* poiFrom;
	POI* poiTo;
	POIAccessFrom::POIAccessFrom accDirTo;
	EvGrabPuckInputZone(POI* poiFrom_, POI* poiTo_, POIAccessFrom::POIAccessFrom accDirTo_):poiFrom(poiFrom_), poiTo(poiTo_), accDirTo(accDirTo_){};
	EvGrabPuckInputZone(){};
};

struct EvDeliverPuckToPoi : sc::event<EvDeliverPuckToPoi> {};
struct EvDeliverPuckToGate  : sc::event<EvDeliverPuckToGate> {
	Node* node;
	EvDeliverPuckToGate(Node* node_):node(node_){};
	EvDeliverPuckToGate(){};
};

struct EvMoveToAndCheckRPC : sc::event<EvMoveToAndCheckRPC> {};

struct EvNoJobFound : sc::event<EvNoJobFound> {};

//struct EvJobFound : sc::event<EvJobFound> {
//	Job * job;
//	EvJobFound(Job *job_):job(job_){};
//	~EvJobFound(){};
//};
//struct EvNoJobFound : sc::event<EvNoJobFound> {};


//Controller Events
/* Not needed anymore
struct EvControllerStart : sc::event<EvControllerStart> {};
struct EvControllerPause : sc::event<EvControllerPause> {};
*/

// Motor Events
struct EvMotorCtrlReady : sc::event<EvMotorCtrlReady> {};

// Sensor Events
struct EvSensorFrontLeftFoundObstacle : sc::event<EvSensorFrontLeftFoundObstacle> {};
struct EvSensorFrontLeftIsFree : sc::event<EvSensorFrontLeftIsFree> {};
struct EvSensorFrontRightFoundObstacle : sc::event<EvSensorFrontRightFoundObstacle> {};
struct EvSensorFrontRightIsFree : sc::event<EvSensorFrontRightIsFree> {};

struct EvSensorFloorLeftIsBlack : sc::event<EvSensorFloorLeftIsBlack> {};
struct EvSensorFloorRightIsBlack : sc::event<EvSensorFloorRightIsBlack> {};

struct EvSensorHasPuck : sc::event<EvSensorHasPuck> {};
struct EvSensorLostPuck : sc::event<EvSensorLostPuck> {};

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

// Camera Events
struct EvCameraPuckDetected : sc::event<EvCameraPuckDetected> {
	float puckXPos;
	float puckYPos;
	EvCameraPuckDetected(float puckXPos_, float puckYPos_):puckXPos(puckXPos_),puckYPos(puckYPos_){};
	~EvCameraPuckDetected(){};
};
struct EvCameraLostPuckVision : sc::event<EvCameraLostPuckVision> {};
struct EvCameraInitialNoPuck : sc::event<EvCameraInitialNoPuck> {};
struct EvCameraCatchPuckLeft : sc::event<EvCameraCatchPuckLeft> {};
struct EvCameraCatchPuckRight : sc::event<EvCameraCatchPuckRight> {};
struct EvCameraCatchPuckStraight : sc::event<EvCameraCatchPuckStraight> {};

struct EvCameraLampDetected : sc::event<EvCameraLampDetected> {
	float lampXPos;
	float lampYPos;
	EvCameraLampDetected(float lampXPos_, float lampYPos_):lampXPos(lampXPos_),lampYPos(lampYPos_){};
	~EvCameraLampDetected(){};
};
struct EvCameraLampDetectedAlign : sc::event<EvCameraLampDetectedAlign> {
	float driveYPos;
	EvCameraLampDetectedAlign(float driveYPos_):driveYPos(driveYPos_){};
	~EvCameraLampDetectedAlign(){};
};
struct EvCameraLostLampVision : sc::event<EvCameraLostLampVision> {};
//obsolete: struct EvCameraLampOnLeftSide : sc::event<EvCameraLampOnLeftSide> {};
//obsolete: struct EvCameraLampOnRightSide : sc::event<EvCameraLampOnRightSide> {};

struct EvCameraUnknownLightDetected : sc::event<EvCameraUnknownLightDetected> {};
struct EvCameraOfflineLightDetected : sc::event<EvCameraOfflineLightDetected> {};
struct EvCameraRedLightDetected : sc::event<EvCameraRedLightDetected> {};
struct EvCameraYellowLightDetected : sc::event<EvCameraYellowLightDetected> {};
struct EvCameraYellowFlashLightDetected : sc::event<EvCameraYellowFlashLightDetected> {};
struct EvCameraGreenLightDetected : sc::event<EvCameraGreenLightDetected> {};
struct EvCameraRedYellowLightDetected : sc::event<EvCameraRedYellowLightDetected> {};
struct EvCameraRedGreenLightDetected : sc::event<EvCameraRedGreenLightDetected> {};
struct EvCameraYellowGreenLightDetected : sc::event<EvCameraYellowGreenLightDetected> {};
struct EvCameraRedYellowGreenLightDetected : sc::event<EvCameraRedYellowGreenLightDetected> {};

//struct EvCameraBlueDivisionLeft : sc::event<EvCameraBlueDivisionLeft> {};

// Combined (intelligent) Events
struct EvRobotLostPuck : sc::event<EvRobotLostPuck> {};
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
