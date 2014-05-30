/*
 * LaserScannerApi.h
 *
 *  Created on: May 2, 2014
 *      Author: sprofanter
 */

#ifndef LASERSCANNERAPI_H_
#define LASERSCANNERAPI_H_

#include "ILaserScannerDriver.h"

#include <rec/robotino/api2/LaserRangeFinder.h>

class LaserScannerApi: public rec::robotino::api2::LaserRangeFinder, public ILaserScannerDriver {
public:
	LaserScannerApi();
	virtual ~LaserScannerApi();
	rec::robotino::api2::LaserRangeFinderReadings readings();
	bool coordInsideField(float x, float y);

};

#endif /* LASERSCANNERAPI_H_ */
