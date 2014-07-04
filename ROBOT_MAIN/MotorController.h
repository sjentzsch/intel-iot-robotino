//
// MotorController.h
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//   Sebastian Riedel <riedels@cs.tum.edu>
//
// Copyright (c) 2014 Sören Jentzsch, Sebastian Riedel
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

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <cmath>
#include <iostream>
#include "utils/Timer.h"
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include "SensorServer.h"
#include "utils/FileLogger.h"

class StateBehaviorController;

using namespace std;

namespace MotorCtrlerSignal{
	enum MotorCtrlerSignal {RUN, PAUSE, TERMINATE};
	//static const char * cMtrCtrlSignal[3] = {"RUN", "PAUSE", "TERMINATE"};
}

namespace ForceRotationDirection{
	enum ForceRotationDirection {SHORTEST, LEFT, RIGHT};
}

class MotorController {
private:
	static const unsigned int MIN_DISTANCE;
	static const unsigned int MIN_DEGREE_DISTANCE;
	static const unsigned int SLOW_DEGREE_DISTANCE;
	static const unsigned int SLOW_START_DURATION;
	static const unsigned int SLOW_DISTANCE;
	static const float MIN_SPEED;
	static const float MIN_ROT_SPEED;
	static const float PHI_CONTROL_PROPORTIONAL_CONSTANT;
	static const unsigned int DEST_TO_TARGET_BEGIN_ROTATE; // original value 600
	static const float MAX_ROT_SPEED;
	static const float MAX_SPEED;

	Timer slowStartTimer;

	// OmniDrive drive;
	SensorServer *sensorSrv;
	StateBehaviorController *stateCtrl;

	// Variables for the motor control thread
	boost::thread *execThread; //thread of execution, controls motors
	MotorCtrlerSignal::MotorCtrlerSignal signal; //represents the current control signal
	boost::mutex signal_mutex; //mutex for access control
	boost::condition_variable signal_cond; //condition variable, signals changes to control signal

	int dtToLastImageInMs;
	int fps;
	int coutCounter;

	bool checkSignalStatus();
	void sendReadyEvent();
	void setVelocity(float vx, float vy, float vphi);
	void moveToAbsPos_impl(vector<vec3D> vPoints, float myMaxSpeed, float myMaxRotSpeed, bool onlyRotate, ForceRotationDirection::ForceRotationDirection forcedDir);
	void moveToAbsPosCF_impl(float destX, float destY, float myMaxSpeed);

public:
	MotorController(SensorServer *sensorSrv_);
	virtual ~MotorController();
	void pause(); // pauses the current movement if any
	void run(); // starts the current movement if any
	void terminate(); //terminates the current movement if any, blocks until thread is terminated
	void setStateBehaviorController(StateBehaviorController *stateCtrl_);
	float getPositiveDegree(float phi) const;
	float getDistDegree(float phi1, float phi2) const;
	vec3D relToAbs(vec3D vPoint);

	void moveToAbsPos(vector<vec3D> vPoints, float myMaxSpeed=MAX_SPEED, float myMaxRotSpeed=MAX_ROT_SPEED, bool onlyRotate=false, ForceRotationDirection::ForceRotationDirection forcedDir=ForceRotationDirection::SHORTEST);
	void moveToAbsPos(float destX, float destY, float destPhi, float myMaxSpeed=MAX_SPEED, float myMaxRotSpeed=MAX_ROT_SPEED, bool onlyRotate=false, ForceRotationDirection::ForceRotationDirection forcedDir=ForceRotationDirection::SHORTEST); // orignal default values: float myMaxSpeed=300.0f, float myMaxRotSpeed=20.0f
	void moveToAbsPosOnly(float destX, float destY, float myMaxSpeed=MAX_SPEED);
	void moveToRelPos(float destX, float destY, float destPhi, float myMaxSpeed=MAX_SPEED, float myMaxRotSpeed=MAX_ROT_SPEED, bool onlyRotate=false, ForceRotationDirection::ForceRotationDirection forcedDir=ForceRotationDirection::SHORTEST);
	void moveToRelXPosAbsYPos(float destX, float destY, float destPhi, float myMaxSpeed=MAX_SPEED);
	void moveToAbsXPosRelYPos(float destX, float destY, float destPhi, float myMaxSpeed=MAX_SPEED);
	void rotateToAbsAngle(float destPhi, ForceRotationDirection::ForceRotationDirection forcedDir=ForceRotationDirection::SHORTEST, float myMaxRotSpeed=MAX_ROT_SPEED);
	void rotateToRelAngle(float destPhi, ForceRotationDirection::ForceRotationDirection forcedDir=ForceRotationDirection::SHORTEST, float myMaxRotSpeed=MAX_ROT_SPEED);

	void moveToAbsPosOnlyCF(float destX, float destY, float myMaxSpeed=MAX_SPEED);
};

#endif /* MOTORCONTROLLER_H_ */
