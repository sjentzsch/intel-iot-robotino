/*
 * SensorEventGeneratorBuffer.hpp
 *
 *  Created on: 11.06.2012
 *      Author: root
 */

#ifndef SENSOREVENTGENERATORBUFFER_HPP_
#define SENSOREVENTGENERATORBUFFER_HPP_

#include "Laserscanner/ObstacleBuffer.h"

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
	//float sensorPuckBias;

	float sensorDistance[9];

	CameraPuckState::CameraPuckState cameraPuckState;
	bool gotCameraPuckPos;
	vec3D cameraPuckPos;
	bool havingPuck;

	CameraLampState::CameraLampState cameraLampState;
	bool gotCameraLampPos;
	vec3D cameraLampPos;

	CameraLightState::CameraLightState cameraLightState;

	ObstacleBuffer obstacleBuffer;
};


#endif /* SENSOREVENTGENERATORBUFFER_HPP_ */
