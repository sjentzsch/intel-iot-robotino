/*
 * LaserScannerSim.h
 *
 *  Created on: May 2, 2014
 *      Author: sprofanter
 */

#ifndef LASERSCANNERSIM_H_
#define LASERSCANNERSIM_H_

#include "ILaserScannerDriver.h"
#include "../SensorServer.h"
#include "Obstacle/IObstacle.h"

class LaserScannerSim : public ILaserScannerDriver{
private:
	// Needed to get Odometry
	SensorServer *sensorServer;
	std::vector<IObstacle*> obstacleStatic;
	std::vector<IObstacle*> obstacleDynamic;

	//physical offset of laser scanner to base in meters and radiants
	float laserOffsetX;
	float laserOffsetY;
	float laserOffsetPhi;

	rec::robotino::api2::LaserRangeFinderReadings data;

	Line2D* rayInit;
	int rayCount;
	Line2D* rays;
	float* ranges;

	Circle2D *robots[3];

	/**
	 * Determines the current global laser scanner position based on the current odometry values of the robot.
	 * @param x laser scanner x position
	 * @param y laser scanner y position
	 * @param phi laser scanner orientation
	 */
	void getLaserScannerPos(float &x, float &y, float &phi) const;

	/**
	 * Updates the array of all the laser scanner rays needed for obstacle intersection checking to determine the laser scanner readings.
	 */
	void updateLaserScannerRays(float posX, float posY, float posPhi);

	/**
	 * Updates the position of the dynamic obstacles
	 */
	void updateDynamicObstacles();
public:
	LaserScannerSim(SensorServer *sensorServer, float laserOffsetX, float laserOffsetY, float laserOffsetPhi);
	virtual ~LaserScannerSim();
	rec::robotino::api2::LaserRangeFinderReadings readings();
	bool coordInsideField(float x, float y);
};

#endif /* LASERSCANNERSIM_H_ */
