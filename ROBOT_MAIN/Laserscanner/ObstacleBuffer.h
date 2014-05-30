/*
 * ObstacleBuffer.h
 *
 *  Created on: Mar 30, 2014
 *      Author: root
 */

#ifndef OBSTACLEBUFFER_H_
#define OBSTACLEBUFFER_H_

#include "config.h"
#include <vector>

class ObstacleBuffer{
public:
	ObstacleBuffer();
	virtual ~ObstacleBuffer();

	float my_x, my_y, my_phi;	// odometry (in mm and deg) values when creating this data
	std::vector<std::vector<float> > obstacles;	// vector of x,y (in mm) in global frame
};

#endif /* OBSTACLEBUFFER_H_ */
