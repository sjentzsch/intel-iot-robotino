/*
 * OdometrySimulation.h
 *
 *  Created on: Apr 18, 2014
 *      Author: Stefan Profanter
 */

#ifndef ODOMETRYSIMULATION_H_
#define ODOMETRYSIMULATION_H_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define USE_ODOMETRY_THREAD 0

using namespace std;

class OdometrySimulation {

private:

    /**
     * current X coordinate in meters
     */
    double x;
    /**
     * current y coordinate in meters
     */
    double y;
    /**
     * current angle in radiant
     */
    double phi;

    /**
     * current velocity in x direction in meters/second
     */
    double vx;

    /**
     * current velocity in y direction in meters/second
     */
    double vy;

    /**
     * current rotation velocity in radiant/second
     */
    double vphi;

    /**
     * Mutex for odometry values (set, get, update)
     */
    boost::mutex mutexOdometryValues;

    /**
     * Time of last update call
     */
    boost::posix_time::ptime lastUpdateTime;
    /**
     * Update the odometry based on current velocity and elapsed time
     */
    void updateOdometry();

#if USE_ODOMETRY_THREAD
    /**
     * Odometry update thread
     */
    boost::thread    thread;


    /**
     * Thread worker updating the odometry within an infinite loop
     */
    void updateOdometryLoop();

    /**
     * Start the update thread
     */
    void startThread();

    /**
     * Stop the update thread
     */
    void stopThread();
#endif /* USE_ODOMETRY_THREAD */

public:
	OdometrySimulation();
	virtual ~OdometrySimulation();

	/**
	 * get current odometry values
	 * @param x X coordinate in meters
	 * @param y Y coordinate in meters
	 * @param phi current angle in radiants
	 */
	void readings(double *x, double *y, double *phi);

	/**
	 * set current odometry values
	 * @param x X coordinate in meters
	 * @param y Y coordinate in meters
	 * @param phi current angle in radiants
	 */
	void set(double x, double y, double phi);

	/**
	 * set omnidrive velocities
	 * @param vx velocity on x axis in meters/second
	 * @param vy velocity on y axis in meters/second
	 * @param vphi rotation velocity radiants/second
	 */
	void setVelocity(double vx, double vy, double vphi);

	friend ostream& operator<< (ostream &out, OdometrySimulation &os);

	/**
	 * Tests the functionality of this class and outputs the results
	 */
	static void test();
};

#endif /* ODOMETRYSIMULATION_H_ */
