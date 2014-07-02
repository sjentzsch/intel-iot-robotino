//
// SensorEventGenerator.h
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

#ifndef SENSOREVENTGENERATOR_H_
#define SENSOREVENTGENERATOR_H_

#include <iostream>
#include "config.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "utils/Timer.h"
#include "utils/FileLogger.h"
#include "StateMachineEvents.h"
#include "SensorEventGeneratorBuffer.hpp"

class SensorServer;
class StateBehaviorController;

using boost::asio::ip::tcp;
using namespace std;

namespace SensorEventGenSignal
{
	enum SensorEventGenSignal {RUN, PAUSE};
	//static const char * cMtrCtrlSignal[3] = {"RUN", "PAUSE", "TERMINATE"};
}

namespace SensorPuckBiasConstants
{
	const float LOST(1.9f);
	const float BLOCKED(1.3f);
}

namespace FloorSensor{
	enum FloorSensor {INIT,LEFT,RIGHT};
}

class SensorEventGenerator {
public:
	SensorEventGenerator(SensorServer *sensorSrv_);
	virtual ~SensorEventGenerator();

	void pause(); // pauses the event generation
	void run(); // (re)starts the event generation
	void initiate(); // initiates the event generation thread
	void setStateBehaviorController(StateBehaviorController *stateCtrl_);
private:
	static const float FLOOR_SENSOR_DISTANCE;

	SensorServer *sensorSrv;
	StateBehaviorController *stateCtrl;

	// Variables for the event generator thread
	boost::thread *execThread; //thread of execution, checks for new events
	SensorEventGenSignal::SensorEventGenSignal signal; //represents the current control signal
	boost::mutex signal_mutex; //mutex for access control
	boost::condition_variable signal_cond; //condition variable, signals changes to control signal

	SensorEventGeneratorBuffer *oldSensorState;
	SensorEventGeneratorBuffer *newSensorState;

	bool evObstacleIsCloseThrown;

	void checkSignalStatus();
	void monitorSensors();
	void getNewSensorValues();
	void handleLineCross();
	float getPositiveDegree(float phi) const;
	float getDistDegree(float phi1, float phi2) const;

	float lastLineCrossX;
	float lastLineCrossY;
	float lastLineCrossPhi;
	FloorSensor::FloorSensor lastLineCrossSensor;
	Timer lastLineCrossTimer;

	int dtToLastImageInMs;
	int fps;
	Timer timer;
	int coutCounter;
};

#endif /* SENSOREVENTGENERATOR_H_ */
