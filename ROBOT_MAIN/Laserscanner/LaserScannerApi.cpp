/*
 * LaserScannerApi.cpp
 *
 *  Created on: May 2, 2014
 *      Author: sprofanter
 */

#include "LaserScannerApi.h"

LaserScannerApi::LaserScannerApi() {

}

LaserScannerApi::~LaserScannerApi() {
}

rec::robotino::api2::LaserRangeFinderReadings LaserScannerApi::readings() {
	return rec::robotino::api2::LaserRangeFinder::readings();
}


bool LaserScannerApi::coordInsideField(float x, float y) {
	//TODO FIX ME IMMEDIATELY!!!
	return x >= 0 && x <= 5.6 && y >= 0 && y <= 11.2;
}

