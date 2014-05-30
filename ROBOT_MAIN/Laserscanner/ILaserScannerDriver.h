/*
 * ILaserScannerDriver.h
 *
 *  Created on: May 2, 2014
 *      Author: sprofanter
 */

#ifndef ILASERSCANNERDRIVER_H_
#define ILASERSCANNERDRIVER_H_

#include <rec/robotino/api2/LaserRangeFinderReadings.h>

class ILaserScannerDriver
{
    public:
        virtual ~ILaserScannerDriver() {}
        virtual rec::robotino::api2::LaserRangeFinderReadings readings() = 0;
        virtual bool coordInsideField(float x, float y) = 0;
};

#endif /* ILASERSCANNERDRIVER_H_ */
