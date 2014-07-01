/*
 * SensorServer.h
 *
 *  Created on: 25.04.2011
 *      Author: root
 */

#ifndef SENSORSERVER_H_
#define SENSORSERVER_H_

#include <iostream>
#include "config.h"
#include "ISensorControl.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "Api2Com.h"
#include "Simulation/OdometrySimulation.h"
#include "Laserscanner/LaserScanner.h"

//#include <rec/robotino/api2/all.h>
#include <rec/robotino/api2/Com.h>
#include <rec/robotino/api2/AnalogInput.h>
#include <rec/robotino/api2/DigitalInput.h>
#include <rec/robotino/api2/DistanceSensorArray.h>
#include <rec/robotino/api2/Odometry.h>
#include <rec/robotino/api2/OmniDrive.h>
#include <rec/robotino/api2/utils.h>

#include "SensorEventGeneratorBuffer.hpp"
#include "utils/pose.h"


using namespace std;

class SensorServer : public ISensorControl
{
private:
	static const unsigned int brightSensorFrontLeft;
	static const unsigned int brightSensorFrontRight;
	static const unsigned int brightSensorFloorLeft;
	static const unsigned int brightSensorFloorRight;
	static const unsigned int brightSensorPuck;

	static const unsigned int analogSensorDrink1;
	static const unsigned int analogSensorDrink2;
	static const unsigned int analogSensorDrink3;

	rec::robotino::api2::DigitalInput brightSensorFrontLeftInput;
	rec::robotino::api2::DigitalInput brightSensorFrontRightInput;
	rec::robotino::api2::DigitalInput brightSensorFloorLeftInput;
	rec::robotino::api2::DigitalInput brightSensorFloorRightInput;
	rec::robotino::api2::DigitalInput brightSensorPuckInput;

	rec::robotino::api2::AnalogInput analogSensorDrinkInput1;
	rec::robotino::api2::AnalogInput analogSensorDrinkInput2;
	rec::robotino::api2::AnalogInput analogSensorDrinkInput3;

	OdometrySimulation odometry_sim;
	rec::robotino::api2::Odometry odometry;
	rec::robotino::api2::OmniDrive omniDrive;
	rec::robotino::api2::DistanceSensorArray distanceSensorArray;
	LaserScanner *laserScanner;

	bool havingPuck;



	void relToAbsCoord(float* xWorldPos, float* yWorldPos, float myX, float myY, float myPhi) const;

	void getOdometryReadings(double &x, double &y, double &phi)
	{
#if SIMULATION_MODE == 1
		odometry_sim.readings(&x, &y, &phi);
#else
		odometry.readings(&x, &y, &phi);
#endif
	}

public:
	SensorServer();
	virtual ~SensorServer();

	bool frontRightBlack() {
		return brightSensorFrontRightInput.value();
	}
	bool frontLeftBlack() {
		return brightSensorFrontLeftInput.value();
	}
	bool floorRightBlack() {
		return !brightSensorFloorRightInput.value();
	}
	bool floorLeftBlack() {
		return !brightSensorFloorLeftInput.value();
	}
	bool puckBlack() {
		return brightSensorPuckInput.value();
	}

	float valueDrink1() {
		return analogSensorDrinkInput1.value();
	}
	bool hasDrink1() {
		return analogSensorDrinkInput1.value() > 4.0f;
	}

	float valueDrink2() {
		return analogSensorDrinkInput2.value();
	}
	bool hasDrink2() {
		return analogSensorDrinkInput2.value() > 4.0f;
	}

	float valueDrink3() {
		return analogSensorDrinkInput3.value();
	}
	bool hasDrink3() {
		return analogSensorDrinkInput3.value() > 4.0f;
	}

	unsigned long numDrinks() {
		unsigned long i = 0;
		if(hasDrink1())
			i++;
		if(hasDrink2())
			i++;
		if(hasDrink3())
			i++;
		return i;
	}

	float getX() {
		double x,y,phi;
		getOdometryReadings(x,y,phi);
		return x*1000.0;
	}
	float getY() {
		double x,y,phi;
		getOdometryReadings(x,y,phi);
		return y*1000.0;
	}
	float getPhi() {
		double x,y,phi;
		getOdometryReadings(x,y,phi);
		return RADTODEG(phi);
	}

	void getOdometry(float &x, float& y, float &phi)
	{
		double x_,y_,phi_;
		getOdometryReadings(x_,y_,phi_);
		x = x_*1000.0;
		y = y_*1000.0;
		phi = RADTODEG(phi_);
	}

	void getOdometry(double &x, double& y, double &phi)
	{
		double x_,y_,phi_;
		getOdometryReadings(x_,y_,phi_);
		x = x_*1000.0;
		y = y_*1000.0;
		phi = RADTODEG(phi_);
	}

	void getOdometry(vec3D& pose)
	{
		double x_,y_,phi_;
		getOdometryReadings(x_,y_,phi_);
		pose.x = x_*1000.0;
		pose.y = y_*1000.0;
		pose.phi = RADTODEG(phi_);
	}

	void getOdometry(vec2D& pose)
	{
		double x_,y_,phi_;
		getOdometryReadings(x_,y_,phi_);
		pose.x = x_*1000.0;
		pose.y = y_*1000.0;
	}

	// convert coordinates in robotino base frame to world frame using current odometry position
	void transformBase2World(const float in_x, const float in_y, float& out_x, float& out_y);
	// odo_x, odo_y in [mm], odo_phi in [rad]
	void transformBase2World(const float in_x, const float in_y, const float odo_x, const float odo_y, const float odo_phi, float& out_x, float& out_y);


	ObstacleBuffer getLatestObstacleBuffer() const {return laserScanner->getLatestObstacleBuffer();}

	void getNewSensorValues(SensorEventGeneratorBuffer* buf,SensorEventGeneratorBuffer* oldSensorState);


	virtual void setOdometry(float x, float y, float phi);

	virtual void calibrateOnLineX(float x);
	virtual void calibrateOnLineY(float y);

	virtual void calibrateAngle(float newAngle);

	virtual void calibrateOnBaseFront();
	virtual void calibrateOnBaseSide();

	virtual void setVelocity(float vx, float vy, float vphi);

	virtual float getRobotX();
	virtual float getRobotY();
	virtual float getRobotPhi();
};

#endif /* SENSORSERVER_H_ */
