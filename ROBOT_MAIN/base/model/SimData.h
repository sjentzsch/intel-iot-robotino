/*
 * Simulation.h
 *
 *  Created on: May 30, 2013
 *      Author: root
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "RobotCameraEnums.h"

namespace SimSensorTransit
{
	enum SimSensorTransit {NONE=0, WHITE_TO_BLACK, BLACK_TO_WHITE};
	static const char * cSimSensorTransit[3] = {"NONE", "0_TO_1", "1_TO_0"};
}

struct SimPosition
{
	float absX;
	float absY;

	SimPosition()
	{
		absX = 0;
		absY = 0;
	}

	SimPosition(float x, float y)
	{
		absX = x;
		absY = y;
	}
};

struct SimData
{
	SimSensorTransit::SimSensorTransit brightnessSensorRight;
	SimSensorTransit::SimSensorTransit brightnessSensorLeft;
	SimSensorTransit::SimSensorTransit distanceSensorRight;
	SimSensorTransit::SimSensorTransit distanceSensorLeft;

	CameraPuckState::CameraPuckState cameraPuckState;
	float sensorPuckBias;
	SimPosition puckPosition;

	CameraLightState::CameraLightState cameraLightState;

	CameraLampState::CameraLampState cameraLampState;
	SimPosition deliveryGatePosition;
};


#endif /* SIMULATION_H_ */
