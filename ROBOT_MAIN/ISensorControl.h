/*
 * ISensorControl.h
 *
 *  Created on: 25.04.2011
 *      Author: root
 */

#ifndef ISENSORCONTROL_H_
#define ISENSORCONTROL_H_

#include "utils/pose.h"

class ISensorControl
{
public:
	virtual ~ISensorControl() {}
	virtual void setOdometry(float x, float y, float phi) = 0;
	virtual void calibrateOnLineX(float x) = 0;
	virtual void calibrateOnLineY(float y) = 0;
	virtual void calibrateAngle(float newAngle) = 0;
	virtual void calibrateOnBaseFront() = 0;
	virtual void calibrateOnBaseSide() = 0;

	virtual void setVelocity(float vx, float vy, float vphi) = 0;

	virtual float getRobotX() = 0;
	virtual float getRobotY() = 0;
	virtual float getRobotPhi() = 0;
};

#endif /* ISENSORCONTROL_H_ */
