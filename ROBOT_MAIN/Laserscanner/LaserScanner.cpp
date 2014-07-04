//
// LaserScanner.cpp
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

#include "LaserScanner.h"
#include "../SensorServer.h"
#include <cmath>
#include "LaserScannerApi.h"

LaserScanner::LaserScanner(SensorServer* sensorServer): execThread(NULL), signal(LaserScannerSignal::RUN)
{
	validData = false;

	// setting up physical laser calibration
	float offset_x = 0.104;
	float offset_y = 0.0;
	float theta = 0.0;

	T_laser2base[0] = cos(theta);  	T_laser2base[1] = -sin(theta);		T_laser2base[2] = offset_x;
	T_laser2base[3] = sin(theta);	T_laser2base[4] = cos(theta);		T_laser2base[5] = offset_y;
	T_laser2base[6] = 0.0;			T_laser2base[7] = 0.0;				T_laser2base[8] = 1.0;

	scannerDriver = new LaserScannerApi();

	this->sensorServer = sensorServer;

	const int WALL_THICKNESS = 20;	// in cm
	const int MACHINE_RADIUS = 25;	// in cm
}

LaserScanner::~LaserScanner()
{
	delete scannerDriver;
}

void LaserScanner::startThread()
{
	execThread = new boost::thread(&LaserScanner::loop, this);
}

void LaserScanner::loop()
{
	while(true)
	{
		checkSignalStatus();

		LaserScannerReadings scan = this->readings();

		//cout << "*****************" << endl;
		/*cout << "angle_min: " << RADTODEG(scan.api_readings.angle_min) << endl;
		cout << "angle_max: " << RADTODEG(scan.api_readings.angle_max) << endl;
		cout << "angle_increment: " << RADTODEG(scan.api_readings.angle_increment) << endl;
		cout << "time_increment: " << scan.api_readings.time_increment << endl;
		cout << "scan_time: " << scan.api_readings.scan_time << endl;
		cout << "range_min: " << scan.api_readings.range_min << endl;
		cout << "range_max: " << scan.api_readings.range_max << endl;
		for(unsigned int i=0; i<scan.positions.size(); i+=50)
		{
			cout << i << ": " << scan.ranges[i] << endl;
		}
		if(scan.positions.size() > 0)
		{
			cout << "middle: " << scan.ranges[scan.positions.size()/2] << endl;
		}*/

		float my_x, my_y, my_phi;
		this->sensorServer->getOdometry(my_x, my_y, my_phi);
		for(unsigned int i=0; i<scan.positions.size(); i++)
			this->sensorServer->transformBase2World(scan.positions.at(i)[0], scan.positions.at(i)[1], my_x, my_y, my_phi, scan.positionsGlob.at(i)[0], scan.positionsGlob.at(i)[1]);

		//if(scan.positions.size() > 0)
		//	cout << "middleGlob: " << scan.positionsGlob.at(scan.positions.size()/2).at(0) << ", " << scan.positionsGlob.at(scan.positions.size()/2).at(1) << endl;

		// set latestObstacleBuffer
		{
			boost::mutex::scoped_lock l(m_mutex);
			this->latestObstacleBuffer.my_x = my_x;
			this->latestObstacleBuffer.my_y = my_y;
			this->latestObstacleBuffer.my_phi = my_phi;
			//this->latestObstacleBuffer.obstacles = clustersMid;

			if(scan.positions.size() > 0)
				this->validData = true;
			else
				this->validData = false;
		}

		{
			boost::mutex::scoped_lock l(m_mutex_scan);
			this->latestScan = scan;
		}
	}
}

bool LaserScanner::checkSignalStatus()
{
	bool wasPaused=false;
	boost::unique_lock<boost::mutex> lock(signal_mutex);
	while(signal == LaserScannerSignal::PAUSE) //wait if signal is PAUSE
	{
		wasPaused=true;
		//cout << "LaserScanner received signal PAUSED." << endl;
		FileLog::log_NOTICE("[LaserScanner] PAUSED");
		signal_cond.wait(lock); //waits for the notify, handles mutex locking/unlocking
	}

	if(wasPaused)
	{
		FileLog::log_NOTICE("[LaserScanner] RUN");
	}
	return true;
}

void LaserScanner::pause()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log_NOTICE("LaserScanner is signaled PAUSED.");
		signal=LaserScannerSignal::PAUSE;
	}
}

void LaserScanner::run()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log_NOTICE("LaserScanner is signaled RUN.");
		signal=LaserScannerSignal::RUN;
	}
	signal_cond.notify_all();
}

LaserScannerReadings LaserScanner::readings() const
{
	LaserScannerReadings readings;

	rec::robotino::api2::LaserRangeFinderReadings api_readings = scannerDriver->readings();
	readings.api_readings = api_readings;

	// compute angles, ranges and positions in local robotino frame
	unsigned int rangesSize = 0;
	const float* ranges;
	api_readings.ranges(&ranges, &rangesSize);

	readings.angles.resize(rangesSize);
	readings.ranges.resize(rangesSize);
	readings.inRange.resize(rangesSize);
	readings.positions.resize(rangesSize);
	readings.positionsGlob.resize(rangesSize);

	float cur_angle = api_readings.angle_min - api_readings.angle_increment;
	for(unsigned int i=0; i<rangesSize; ++i)
	{
		cur_angle += api_readings.angle_increment;

		// convert range and angle into point in laser coordinate system
		float x_laser, y_laser;
		range2vec2d(ranges[i], cur_angle, x_laser, y_laser);

		// transform to base coordinate system using calibration matrix
		float x_base, y_base;
		x_base = T_laser2base[0] * x_laser + T_laser2base[1] * y_laser + T_laser2base[2];
		y_base = T_laser2base[3] * x_laser + T_laser2base[4] * y_laser + T_laser2base[5];

		// convert point back to
		float range_base, angle_base;
		vec2d2range(x_base, y_base, range_base, angle_base);

		readings.angles[i] = angle_base;
		readings.ranges[i] = range_base;
		readings.inRange[i] = (ranges[i] > api_readings.range_min) && (ranges[i] < api_readings.range_max);
		readings.positions[i].resize(2);
		readings.positions[i][0] = x_base;
		readings.positions[i][1] = y_base;
		readings.positionsGlob[i].resize(2);
		readings.positionsGlob[i][0] = 0;
		readings.positionsGlob[i][1] = 0;

		if(x_base >= 0.1 && x_base <= 2.0 && y_base >= -0.3 && y_base <= 0.3)
			readings.indicesInFrontPos.push_back(i);

		//cout << "range: " << i << ", cur_angle: " << RADTODEG(cur_angle) <<  ", angle: " << RADTODEG(readings.angles[i]) << endl;
	}

	return readings;
}

void LaserScanner::rangeInBaseFrame(const float range, const float angle, float& base_range, float& base_angle) const
{
	// convert range and angle into point in laser coordinate system
	float x_laser, y_laser;
	range2vec2d(range, angle, x_laser, y_laser);

	// transform to base coordinate system using calibration matrix
	float x_base, y_base;
	x_base = T_laser2base[0] * x_laser + T_laser2base[1] * y_laser + T_laser2base[2];
	y_base = T_laser2base[3] * x_laser + T_laser2base[4] * y_laser + T_laser2base[5];

	// convert point back to
	vec2d2range(x_base, y_base, base_range, base_angle);
}

ObstacleBuffer LaserScanner::getLatestObstacleBuffer()
{
	boost::mutex::scoped_lock l(m_mutex);
	return this->latestObstacleBuffer;
}

LaserScannerReadings LaserScanner::getLatestScan()
{
	boost::mutex::scoped_lock l(m_mutex_scan);
	return this->latestScan;
}
