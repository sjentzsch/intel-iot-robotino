//
// config.h
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

#ifndef CONFIG_H_
#define CONFIG_H_

#define SIMULATION_MODE 0
#define USE_CAMERA 0

const float OBS_X_START = 0.1;
const float OBS_X_STILL_MAX = 0.35;
const float OBS_X_SAFETY_BACK = 0.30;
const float OBS_X_END = 1.5;
const float OBS_Y_END = 0.3;
const float IR_SENSOR_THRESHOLD = 0.12;

/* SOUND FUNCTIONS */
#define PLAYSOUND(x) system("nohup play -v 0.05 /home/robotino/" + x + " > /dev/null 2>&1 &");

/* USEFUL PREPROCESSOR DIRECTIVES */
#define PRINT(x) std::cout << x << std::endl
#define SQUARE(x) ((x) * (x))
#define DEGTORAD(x) ((x) * 0.01745329251f)
#define RADTODEG(x) ((x) * 57.2957795131f)

#endif /* CONFIG_H_ */
