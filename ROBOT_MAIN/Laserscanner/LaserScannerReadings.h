//
// LaserScannerReadings.h
//
// Authors:
//   Stefan Profanter <stefan.profanter@gmail.com>
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Stefan Profanter, Sören Jentzsch
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

#ifndef LASERSCANNERREADINGS_H_
#define LASERSCANNERREADINGS_H_

#include <rec/robotino/api2/LaserRangeFinderReadings.h>

class LaserScannerReadings{
public:
	LaserScannerReadings();
	virtual ~LaserScannerReadings();

	std::vector<float> angles;	// in rad
	std::vector<float> ranges;	// in m
	std::vector<bool> inRange;	// true if range value is between range_min and range_max
	std::vector<std::vector<float> > positions;		// vector of x,y in m (robotino frame)
	std::vector<std::vector<float> > positionsGlob;	// vector of x,y in m (global frame)
	std::vector<unsigned int> indicesInFrontPos;	// vector of indices of positions in front

	rec::robotino::api2::LaserRangeFinderReadings api_readings;
};

#endif /* LASERSCANNERREADINGS_H_ */
