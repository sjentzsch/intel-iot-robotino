/*
 * LaserScannerSim.cpp
 *
 *  Created on: May 2, 2014
 *      Author: sprofanter
 */

#include "LaserScannerSim.h"
#include "model/ModelProvider.h"
#include "model/WorldModel.h"
#include "Obstacle/Geometry.h"

LaserScannerSim::LaserScannerSim(SensorServer *_sensorServer, float _laserOffsetX, float _laserOffsetY, float _laserOffsetPhi) : sensorServer(_sensorServer), laserOffsetX(_laserOffsetX), laserOffsetY(_laserOffsetY), laserOffsetPhi(_laserOffsetPhi) {
	WorldModel *worldModel = ModelProvider::getInstance()->getWorldModel();

	// add the walls to the obstacle map
	if(BaseParameterProvider::getInstance()->getParams()->garching_environment)
	{
		//TODO define walls for GHB
	} else {

		// left
		obstacleStatic.push_back(new Line2D(Point2D(0,0), Point2D(5.60,0)));
		// right
		obstacleStatic.push_back(new Line2D(Point2D(0,11.20), Point2D(5.60,11.20)));
		// top
		obstacleStatic.push_back(new Line2D(Point2D(0,0), Point2D(0,11.20)));
		// bottom
		obstacleStatic.push_back(new Line2D(Point2D(5.60,0), Point2D(5.60,11.20)));
	}

	// in meters
	const float MACHINE_RADIUS = 0.15;

	for (unsigned int i=0; i< NUMBER_OF_POIS; i++) {
		if (!worldModel->poi[i].isVisibleForLaser)
			continue;

		obstacleStatic.push_back(new Circle2D(Point2D((worldModel->poi[i].x+1)*X_GRID_WIDTH/1000.0, (worldModel->poi[i].y+1)*Y_GRID_WIDTH/1000.0), MACHINE_RADIUS));
	}

	for (unsigned int i=0; i<3; i++) {
		if ((int)(ModelProvider::getInstance()->getHWID())!=i+1)
		{
			robots[i] = new Circle2D();
			robots[i]->radius = 0.23;
			obstacleDynamic.push_back(robots[i]);
		} else
			robots[i] = NULL;
	}

	data = rec::robotino::api2::LaserRangeFinderReadings();

	// TODO
	data.angle_min = -M_PI /2.0;
	data.angle_max = M_PI/2.0;
	data.angle_increment = 0.5*M_PI/180.0;
	// in meters
	data.range_min = 0;
	data.range_max = 5;

	// create all the rays, these will be transformed to the current robot position
	float currAngle = data.angle_min;
	rayCount = (int)floor((data.angle_max-data.angle_min)/data.angle_increment);
	rayInit = new Line2D[rayCount];

	rays = new Line2D[rayCount];

	ranges = new float[rayCount];


	for (int i=0; i<rayCount; i++) {
		float currAngle = data.angle_min + i*data.angle_increment + M_PI;

		Point2D p1 = Point2D(-data.range_min,0);
		p1.rotateByAngle(currAngle);
		Point2D p2 = Point2D(-data.range_max,0);
		p2.rotateByAngle(currAngle);
		rayInit[i].p1 = p1;
		rayInit[i].p2 = p2;
	}

}

bool LaserScannerSim::coordInsideField(float x, float y)
{
	if(BaseParameterProvider::getInstance()->getParams()->garching_environment)
	{
		//TODO define walls for GHB
		return true;
	} else {
		return x >= 0 && x <= 5.6 && y >= 0 && y <= 11.2;
	}
}

void LaserScannerSim::getLaserScannerPos(float &x, float &y, float &phi) const {
	float odoX, odoY, odoPhi;
	sensorServer->getOdometry(odoX, odoY, odoPhi);
	//to meters
	odoX /= 1000.0;
	odoY /= 1000.0;
	odoPhi = DEGTORAD(odoPhi);


	float cs = cos(odoPhi);
	float sn = sin(odoPhi);

	x = odoX + laserOffsetX * cs - laserOffsetY * sn;
	y = odoY + laserOffsetX * sn + laserOffsetY * cs;
	phi = odoPhi + laserOffsetPhi;
}

LaserScannerSim::~LaserScannerSim() {
	for(unsigned int i = 0; i < obstacleStatic.size(); ++i)
	   delete obstacleStatic[i];
	for(unsigned int i = 0; i < obstacleDynamic.size(); ++i)
	   delete obstacleDynamic[i];
	delete[] rayInit;
	delete[] rays;
	delete[] ranges;
}

void LaserScannerSim::updateLaserScannerRays(float posX, float posY, float posPhi) {

	//transform the initial rays to match current position

	float T_initToPos[9] = {
			cos(posPhi),	-sin(posPhi),	posX,
			sin(posPhi),	cos(posPhi),	posY,
			0.0,			0.0,			1.0
	};

	for (int i=0; i<rayCount; i++) {
		rays[i].p1.x = T_initToPos[0] * rayInit[i].p1.x + T_initToPos[1] * rayInit[i].p1.y + T_initToPos[2];
		rays[i].p1.y = T_initToPos[3] * rayInit[i].p1.x + T_initToPos[4] * rayInit[i].p1.y + T_initToPos[5];

		rays[i].p2.x = T_initToPos[0] * rayInit[i].p2.x + T_initToPos[1] * rayInit[i].p2.y + T_initToPos[2];
		rays[i].p2.y = T_initToPos[3] * rayInit[i].p2.x + T_initToPos[4] * rayInit[i].p2.y + T_initToPos[5];
	}


}

float getShortestRange(Line2D &ray, Point2D &laserPos, float maxRange, std::vector<IObstacle*> &obstacleArr) {
	float minRange = pow(maxRange,2);
	for (unsigned int o=0; o<obstacleArr.size(); o++){
		std::vector<Point2D>* intersect = obstacleArr[o]->getIntersection(&ray);
		if (intersect != NULL) {
			for (unsigned int x=0; x<intersect->size(); x++) {
				minRange = min(laserPos.distToSqrt((*intersect)[x]), minRange);
			}
			delete intersect;
		}
	}
	return sqrt(minRange);
}

void LaserScannerSim::updateDynamicObstacles() {

	for (int i=0; i<3; i++) {
		if (robots[i] == NULL)
			continue;
		// refbox is idx 0, thus +1
		robots[i]->center.x = ModelProvider::getInstance()->getGameData()->communicationInfo.stations[i+1].peer_odometry.x;
		robots[i]->center.y = ModelProvider::getInstance()->getGameData()->communicationInfo.stations[i+1].peer_odometry.y;
		//cout << "Pos " << i << " " << robots[i]->center << endl;
	}
}

rec::robotino::api2::LaserRangeFinderReadings LaserScannerSim::readings() {

	// get current laser scanner position and angle
	float posX, posY, posPhi;
	this->getLaserScannerPos(posX, posY, posPhi);
	updateLaserScannerRays(posX, posY, posPhi);
	updateDynamicObstacles();

	Point2D laserPos(posX, posY);

	for (int i=0; i<rayCount; i++) {
		ranges[i] = min(getShortestRange(rays[i], laserPos, data.range_max, obstacleStatic),getShortestRange(rays[i], laserPos, data.range_max, obstacleDynamic));
	}

	data.setRanges(ranges, rayCount);
	return data;
}
