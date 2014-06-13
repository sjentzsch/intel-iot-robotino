/*
 * Odometry.cpp
 *
 *  Created on: Apr 18, 2014
 *      Author: Stefan Profanter
 */

#include "OdometrySimulation.h"

#include <iostream>
using namespace std;

OdometrySimulation::OdometrySimulation() : x(0),y(0),phi(0),vx(0),vy(0),vphi(0), lastUpdateTime(boost::posix_time::microsec_clock::local_time()){
#if USE_ODOMETRY_THREAD
	startThread();
#endif /* USE_ODOMETRY_THREAD */
}

OdometrySimulation::~OdometrySimulation() {
#if USE_ODOMETRY_THREAD
	stopThread();
#endif /* USE_ODOMETRY_THREAD*/
}

void OdometrySimulation::updateOdometry() {
	boost::mutex::scoped_lock l(mutexOdometryValues);

	// calculate time difference between last update and now
	boost::posix_time::ptime currTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration diff = currTime - lastUpdateTime;
	lastUpdateTime = currTime;
	float elapsedSeconds = diff.total_microseconds()/1000000.0;

	// now update the odometry

	// we need to project the velocity values from robot base to world base
	float vecX = this->vx;
	float vecY = this->vy;

	//rotate vector by current robot angle
	float cs = cos(this->phi);
	float sn = sin(this->phi);
	float tx = vecX * cs - vecY * sn;
	float ty = vecX * sn + vecY * cs;

	// now update robot's position
	this->x += tx* elapsedSeconds;
	this->y += ty* elapsedSeconds;
	this->phi += this->vphi* elapsedSeconds;
	if (this->phi < -M_PI)
		this->phi = 2*M_PI - this->phi;
	else if (this->phi > M_PI)
		this->phi = -2*M_PI + this->phi;
}


void OdometrySimulation::readings(double* x, double* y, double* phi) {
#if USE_ODOMETRY_THREAD == 0
	updateOdometry();
#endif /* USE_ODOMETRY_THREAD*/
	{
		boost::mutex::scoped_lock l(mutexOdometryValues);
		*x = this->x;
		*y = this->y;
		*phi = this->phi;
	}
}

void OdometrySimulation::set(double x, double y, double phi) {
#if USE_ODOMETRY_THREAD == 0
	updateOdometry();
#endif /* USE_ODOMETRY_THREAD*/
	{
		boost::mutex::scoped_lock l(mutexOdometryValues);
		this->x = x;
		this->y = y;
		this->phi = phi;
		this->lastUpdateTime = boost::posix_time::microsec_clock::local_time();
	}
}


void OdometrySimulation::setVelocity(double vx, double vy, double vphi) {
#if USE_ODOMETRY_THREAD == 0
	updateOdometry();
#endif /* USE_ODOMETRY_THREAD*/
	{
		boost::mutex::scoped_lock l(mutexOdometryValues);
		this->vx = vx;
		this->vy = vy;
		this->vphi = vphi;
	}
}

ostream& operator<< (ostream &out, OdometrySimulation &os)
{
    double x, y, phi;
    os.readings(&x, &y, &phi);
    out << "(" << x << "m, " <<
        y << "m, " <<
        phi << " rad)";
    return out;
}

#if USE_ODOMETRY_THREAD
void OdometrySimulation::startThread() {
	thread = boost::thread(&OdometrySimulation::updateOdometryLoop, this);
}

void OdometrySimulation::stopThread() {
	thread.interrupt();
	thread.join();
}

void OdometrySimulation::updateOdometryLoop() {
	while (true) {
		updateOdometry();
		try
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		}
		catch(boost::thread_interrupted&)
		{
			return;
		}
	}
}
#endif /* USE_ODOMETRY_THREAD */

void OdometrySimulation::test() {
	cout << "#### OdometrySimulation Test START ###" << endl;

	OdometrySimulation os;

	cout << "Initial: " << os << endl;

	cout << "Set speed to (2m/s, 0.5m/s, 0) and wait 2 seconds" << endl;
	os.setVelocity(2, 0.5, 0);
	sleep(2);
	cout << "2s: " << os << " expected: (4,1,0)" << endl;

	cout << "Stopping" << endl;
	os.setVelocity(0, 0, 0);
	sleep(2);
	cout << "4s: " << os << " expected: (4,1,0)" << endl;

	cout << "Set speed to (0m/s, -0.1m/s, PI/8) and wait 2 seconds" << endl;
	os.setVelocity(0, -0.1, M_PI/8);
	sleep(4);
	cout << "8s: " << os << " expected: (4,0.6,0.78)" << endl;

	cout << "Stopping" << endl;
	os.setVelocity(0, 0, 0);
	sleep(2);
	cout << "10s: " << os << " expected: (4,0.6,1.57)" << endl;


	cout << "#### OdometrySimulation Test END ###" << endl;
}
