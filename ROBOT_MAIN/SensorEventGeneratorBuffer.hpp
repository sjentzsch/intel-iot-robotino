/*
 * SensorEventGeneratorBuffer.hpp
 *
 *  Created on: 11.06.2012
 *      Author: root
 */

#ifndef SENSOREVENTGENERATORBUFFER_HPP_
#define SENSOREVENTGENERATORBUFFER_HPP_

#include "Laserscanner/ObstacleBuffer.h"
#include "utils/pose.h"

// contains Variables for sensor monitoring
class SensorEventGeneratorBuffer
{
public:
	SensorEventGeneratorBuffer() {}
	~SensorEventGeneratorBuffer() {
	}

	bool sensorFrontLeftObstacle;
	bool sensorFrontRightObstacle;
	bool sensorFloorLeftBlack;
	bool sensorFloorRightBlack;
	bool sensorPuckBlack;

	bool sensorHasDrink1;
	bool sensorHasDrink2;
	bool sensorHasDrink3;

	float sensorDistance[9];

	ObstacleBuffer obstacleBuffer;
};


#endif /* SENSOREVENTGENERATORBUFFER_HPP_ */
