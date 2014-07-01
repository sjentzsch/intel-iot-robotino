/*
 * ISensorControl.h
 *
 *  Created on: 25.04.2011
 *      Author: root
 */

#ifndef ISENSORCONTROL_H_
#define ISENSORCONTROL_H_

#include "Camera/RobotCameraEnums.h"
#include "utils/pose.h"

class ISensorControl
{
public:
	virtual ~ISensorControl() {}
	virtual void setOdometry(float x, float y, float phi) = 0;
	virtual void setCameraDetection(CameraPuckDetection::CameraPuckDetection puckDetection, CameraLightDetection::CameraLightDetection lightDetection) = 0;
	virtual void setHavingPuck(bool havingPuck) = 0;
	virtual void calibrateOnLineX(float x) = 0;
	virtual void calibrateOnLineY(float y) = 0;
	virtual void calibrateAngle(float newAngle) = 0;
	virtual void calibrateOnBaseFront() = 0;
	virtual void calibrateOnBaseSide() = 0;
	virtual bool setCameraSettingsforLight() = 0;
	virtual bool setCameraSettingsforPuck() = 0;

	virtual void setVelocity(float vx, float vy, float vphi) = 0;

	virtual float getRobotX() = 0;
	virtual float getRobotY() = 0;
	virtual float getRobotPhi() = 0;
};

#endif /* ISENSORCONTROL_H_ */
