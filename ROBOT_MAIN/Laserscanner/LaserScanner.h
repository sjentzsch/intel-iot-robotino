/*
 * LaserScanner.h
 *
 *  Created on: Mar 27, 2014
 *      Author: root
 */

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

	boost::thread *execThread;
	LaserScannerSignal::LaserScanner signal; //represents the current control signal
	boost::mutex signal_mutex; //mutex for access control
	boost::condition_variable signal_cond; //condition variable, signals changes to control signal

private:
	// physical calibration  offset of laserrange finder
	float T_laser2base[9];

	ObstacleBuffer latestObstacleBuffer;

	// mutex for communication with the SensorEventGenerator-Thread
	boost::mutex m_mutex;

	SensorServer* sensorServer;

	Mat laserscannerMap;

	ILaserScannerDriver *scannerDriver;
};

#endif /* LASERSCANNER_H_ */
