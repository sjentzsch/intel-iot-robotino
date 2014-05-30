/*
 * LaserScannerData.h
 *
 *  Created on: Mar 31, 2014
 *      Author: root
 */

#ifndef LASERSCANNERDATA_H_
#define LASERSCANNERDATA_H_

const unsigned int LASERSCANNER_MAX_POINTS = 1080;			// max #points
const unsigned int LASERSCANNER_MAX_DYN_OBSTACLES = 30;

struct LaserScannerData {
	int id;
	float my_x, my_y, my_phi;	// odometry values when collecting / transforming this data
	int laserScannerData[LASERSCANNER_MAX_POINTS][2];
	int dynObstacles[LASERSCANNER_MAX_DYN_OBSTACLES][2];
	bool isDynObstacle[LASERSCANNER_MAX_POINTS];
	bool isValid[LASERSCANNER_MAX_POINTS];	// true if inside game field and within valid range of laserscanner
	LaserScannerData(){};
	LaserScannerData(int id_):id(id_){};
};

#endif /* LASERSCANNERDATA_H_ */
