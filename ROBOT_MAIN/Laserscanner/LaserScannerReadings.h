/*
 * LaserScannerReadings.h
 *
 *  Created on: Mar 30, 2014
 *      Author: root
 */

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

	rec::robotino::api2::LaserRangeFinderReadings api_readings;
};

#endif /* LASERSCANNERREADINGS_H_ */
