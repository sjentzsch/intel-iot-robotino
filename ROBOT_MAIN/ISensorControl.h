//
// ISensorControl.h
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
