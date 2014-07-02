//
// LaserScanner.h
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

#ifndef LASERSCANNER_H_
#define LASERSCANNER_H_

#include "config.h"

namespace LaserScannerSignal{
	enum LaserScanner {RUN, PAUSE};
}

#include <cmath>
#include <boost/thread.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "LaserScannerReadings.h"
#include "ObstacleBuffer.h"
#include "utils/FileLogger.h"
#include "ILaserScannerDriver.h"

using namespace cv;
using namespace std;

class SensorServer;

class LaserScanner {

public:
	LaserScanner(SensorServer* sensorServer);
	~LaserScanner();

	void startThread();
	void loop();

	bool checkSignalStatus();
	void pause();
	void run();

	LaserScannerReadings readings() const;

	ObstacleBuffer getLatestObstacleBuffer();
	LaserScannerReadings getLatestScan();

	// helper functions
	static void angle2vec2d(const float angle, float& vec_x, float& vec_y)
	{
		vec_x = cos(angle);
		vec_y = sin(angle);
	}

	static void range2vec2d(const float range, const float angle, float& vec_x, float &vec_y)
	{
		float dir_x = cos(angle);
		float dir_y = sin(angle);

		vec_x = dir_x * range;
		vec_y = dir_y * range;
	}

	static void vec2d2range(const float vec_x, const float vec_y, float& range, float& angle)
	{
		range = sqrt(vec_x * vec_x + vec_y * vec_y);
		angle = atan2(vec_y, vec_x);
	}

	void rangeInBaseFrame(const float range, const float angle, float& base_range, float& base_angle) const;

	bool hasValidData() {return this->validData;}

	boost::thread *execThread;
	LaserScannerSignal::LaserScanner signal; //represents the current control signal
	boost::mutex signal_mutex; //mutex for access control
	boost::condition_variable signal_cond; //condition variable, signals changes to control signal

private:
	bool validData;

	// physical calibration offset of laserrange finder
	float T_laser2base[9];

	LaserScannerReadings latestScan;
	boost::mutex m_mutex_scan;

	ObstacleBuffer latestObstacleBuffer;

	// mutex for communication with the SensorEventGenerator-Thread
	boost::mutex m_mutex;

	SensorServer* sensorServer;

	ILaserScannerDriver *scannerDriver;
};

#endif /* LASERSCANNER_H_ */
