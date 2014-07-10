//
// SensorServer.cpp
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

#include "SensorServer.h"
#include "utils/FileLogger.h"
#include "DataProvider.h"
#include <cmath>

const unsigned int SensorServer::brightSensorFrontLeft(0);
const unsigned int SensorServer::brightSensorFrontRight(1);
const unsigned int SensorServer::brightSensorFloorLeft(2);
const unsigned int SensorServer::brightSensorFloorRight(3);
const unsigned int SensorServer::brightSensorPuck(4);
const unsigned int SensorServer::analogSensorDrink1(1);
const unsigned int SensorServer::analogSensorDrink2(2);
const unsigned int SensorServer::analogSensorDrink3(3);

SensorServer::SensorServer() :havingPuck(false)
{
	brightSensorFrontLeftInput.setInputNumber(brightSensorFrontLeft);
	brightSensorFrontRightInput.setInputNumber(brightSensorFrontRight);
	brightSensorFloorLeftInput.setInputNumber(brightSensorFloorLeft);
	brightSensorFloorRightInput.setInputNumber(brightSensorFloorRight);
	brightSensorPuckInput.setInputNumber(brightSensorPuck);
	analogSensorDrinkInput1.setInputNumber(analogSensorDrink1);
	analogSensorDrinkInput2.setInputNumber(analogSensorDrink2);
	analogSensorDrinkInput3.setInputNumber(analogSensorDrink3);

//	for(int i=0; i<9; i++)
//	{
//		distSensor[i].setComId(com_->id());
//		distSensor[i].setSensorNumber(i);
//	}

//	brightSensorFrontRight.setComId(com_->id());
//	brightSensorFrontRight.setInputNumber(1);
//	brightSensorFrontLeft.setComId(com_->id());
//	brightSensorFrontLeft.setInputNumber(0);


	// JUST A TRY TO USE THE CAMERA WITH OPENCV ... WITHOUT SUCCESS ... [segmentation fault]
	// initialize the camera with id=0 (standard-camera)
	/*VideoCapture capture(0);
	if(!capture.isOpened())
	{
		cout << "ERROR: VideoCapture wasn't opened" << endl;
		//return -1;
	}
	capture.set(CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);

	cout << "WIDTH: " << capture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
	cout << "HEIGHT: " << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

	if(!capture.isOpened())
		cout << "it was not opened!" << endl;
	Mat mRGBcam;//(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
	cout << "capturing ..." << endl;
	capture >> mRGBcam;
	cout << "CAPTURE DONE!" << endl;*/



//	robotCamera = new RobotCamera();
//
//	// care which thread creates the SensorServer, that one will wait for the first picture being received by the camera, might be crucial
//	// TODO: uncomment following lines when issues is fixed, that images may not be received in the simulator
//	robotCamera->setComId(com_->id());
////	while(!robotCamera->istre()){
////		cout << "...";
////		com_->waitForUpdate();
////	}
//
//	while(!robotCamera->getIsStreaming())
//	{
//		//cout << "...";
//		com_->waitForUpdate();
//	}

	laserScanner = new LaserScanner(this);
	laserScanner->startThread();
#if SIMULATION_MODE == 0
	while(!laserScanner->hasValidData())
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
#endif
}

SensorServer::~SensorServer()
{
	if(laserScanner != NULL)
		delete laserScanner;
}

void SensorServer::relToAbsCoord(float* xWorldPos, float* yWorldPos, float myX, float myY, float myPhi) const
{
	double destVectorLength = sqrt(SQUARE(*xWorldPos)+SQUARE(*yWorldPos));
	double destVectorRelAngle = 0;

	if(destVectorLength != 0)
	{
		destVectorRelAngle = acos(*xWorldPos/destVectorLength);
		if(*yWorldPos < 0)
			destVectorRelAngle = -destVectorRelAngle;
	}

	double sensPhiRad = DEGTORAD(myPhi);

	*xWorldPos = cos(destVectorRelAngle+sensPhiRad)*destVectorLength + myX;
	*yWorldPos = sin(destVectorRelAngle+sensPhiRad)*destVectorLength + myY;
}

void SensorServer::transformBase2World(const float in_x, const float in_y, float& out_x, float& out_y)
{
	float x,y,phi;
	getOdometry(x,y,phi);
	transformBase2World(in_x, in_y, x, y, phi, out_x, out_y);
}

void SensorServer::transformBase2World(const float in_x, const float in_y, const float odo_x, const float odo_y, const float odo_phi, float& out_x, float& out_y)
{
	// build transformation matrix
	float T_base2world[6];
	float theta = DEGTORAD(odo_phi);
	T_base2world[0] = cos(theta);	T_base2world[1] = -sin(theta);	T_base2world[2] = odo_x / 1000.0;
	T_base2world[3] = sin(theta);	T_base2world[4] = cos(theta);	T_base2world[5] = odo_y / 1000.0;

	out_x = T_base2world[0] * in_x + T_base2world[1] * in_y + T_base2world[2];
	out_y = T_base2world[3] * in_x + T_base2world[4] * in_y + T_base2world[5];
}

/*
 * x in mm
 * y in mm
 * phi in degree
 */
void SensorServer::setOdometry(float x, float y, float phi)
{
#if SIMULATION_MODE == 1
	odometry_sim.set(x/1000.0, y/1000.0, DEGTORAD(phi));
#else
	odometry.set(x/1000.0, y/1000.0, DEGTORAD(phi));
#endif


	vec3D newPose;
	getOdometry(newPose);

	int counter = 0;

	while(counter < 50 && (abs(newPose.x-x) > 1.0 || abs(newPose.y-y) > 1.0 || abs(fmod((double)newPose.phi-phi, 360.0)) > 0.5))
	{
		cout << "set odometry wait ... " << newPose.x << " to " << x << ", " << newPose.y << " to " << y << ", " << newPose.phi << " to " << phi << endl;
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));
		getOdometry(newPose);
		counter++;
	}
	if(counter >= 50)
		cout << "set odometry give up ..." << endl;

		//boost::this_thread::yield();

	//cout << "SensorServer: set odometry to: " << x << ", " << y << ", " << phi << endl;
}

void SensorServer::calibrateOnLineX(float x){

	 double x_,y_,phi_;
	 getOdometry(x_, y_, phi_);
	 setOdometry(x,y_,phi_);
}

void SensorServer::calibrateOnLineY(float y){
	 double x_,y_,phi_;
	 getOdometry(x_, y_, phi_);
	 setOdometry(x_,y,phi_);
}

void SensorServer::calibrateAngle(float newAngle)
{
	double x,y,phi;
	getOdometry(x, y, phi);
	setOdometry(x,y,(float)newAngle);
}

void SensorServer::calibrateOnBaseFront()
{
#if SIMULATION_MODE == 1
	FileLog::log_NOTICE("No calibration available in Simulation mode. Skipping.");
	return;
#endif

	// wait for latest odometry values to arrive ....
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	MsgEnvironment msgEnvironment = DataProvider::getInstance()->getLatestMsgEnvironment();
	Direction::DIRECTION dirBase = DataProvider::getInstance()->getLatestBaseDir();
	cout << "calibrateOnBaseFront: " << Direction::cDIRECTION[dirBase] << endl;

	LaserScannerReadings scan = this->laserScanner->getLatestScan();
	float midPosGlobX = scan.positionsGlob.at(scan.positions.size()/2).at(0);
	float midPosGlobY = scan.positionsGlob.at(scan.positions.size()/2).at(1);

	float xCurr,yCurr,phiCurr;
	getOdometry(xCurr, yCurr, phiCurr);
	cout << "calibrateOnBaseFront: xCurr: " << xCurr << ", " << "yCurr: " << yCurr << ", " << "phiCurr: " << phiCurr << endl;

	float xDiff, yDiff;
	switch(dirBase)
	{
	case Direction::DIRECTION::NORTH:

		break;
	case Direction::DIRECTION::EAST:
		yDiff = (msgEnvironment.y_base_left_corner - midPosGlobY)*1000;
		cout << "calibrateOnBaseFront: yDiff (desired - actual): " << yDiff << " (in mm)" << endl;
		setOdometry(xCurr, yCurr + yDiff, phiCurr);
		break;
	case Direction::DIRECTION::SOUTH:

		break;
	case Direction::DIRECTION::WEST:

		break;
	default:
		break;
	}

	/*cout << "wait for key pressed ..." << endl;
	cin.get(); cin.clear();*/
}

void SensorServer::calibrateOnBaseSide()
{
#if SIMULATION_MODE == 1
	FileLog::log_NOTICE("No calibration available in Simulation mode. Skipping.");
	return;
#endif

	const float LINE_DIFF_THRESHOLD = 0.05;	// in m

	// wait for latest odometry values to arrive ....
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	MsgEnvironment msgEnvironment = DataProvider::getInstance()->getLatestMsgEnvironment();
	Direction::DIRECTION dirBase = DataProvider::getInstance()->getLatestBaseDir();
	cout << "calibrateOnBaseSide: " << Direction::cDIRECTION[dirBase] << endl;

	LaserScannerReadings scan = this->laserScanner->getLatestScan();

	float xCurr,yCurr,phiCurr;
	getOdometry(xCurr, yCurr, phiCurr);
	cout << "calibrateOnBaseSide: xCurr: " << xCurr << ", " << "yCurr: " << yCurr << ", " << "phiCurr: " << phiCurr << endl;

	float xDiff, yDiff;
	switch(dirBase)
	{
	case Direction::DIRECTION::NORTH:
	{

		break;
	}
	case Direction::DIRECTION::EAST:
	{
		unsigned int i = scan.positions.size()/2;
		cout << "start at ray " << i << endl;
		vec2D lastPosGlob, currPosGlob;
		do
		{
			lastPosGlob = vec2D(scan.positionsGlob.at(i).at(0), scan.positionsGlob.at(i).at(1));
			i++;
			currPosGlob = vec2D(scan.positionsGlob.at(i).at(0), scan.positionsGlob.at(i).at(1));
			//cout << "scan diffs between scan curr " << i << " (" << currPosGlob.x << ", " << currPosGlob.y << ", angle: " << RADTODEG(scan.angles.at(i)) << ") and scan last " << (i-1) << " (" << lastPosGlob.x << ", " << lastPosGlob.y << ", angle: " << RADTODEG(scan.angles.at(i-1)) << ")" << endl;
		}while(i < scan.positions.size()-1 && abs(lastPosGlob.y - currPosGlob.y) < LINE_DIFF_THRESHOLD);

		cout << "=> scan diffs between scan curr " << i << " (" << currPosGlob.x << ", " << currPosGlob.y << ", angle: " << RADTODEG(scan.angles.at(i)) << ") and scan last " << (i-1) << " (" << lastPosGlob.x << ", " << lastPosGlob.y << ", angle: " << RADTODEG(scan.angles.at(i-1)) << ")" << endl;

		xDiff = (msgEnvironment.x_base_left_corner - lastPosGlob.x)*1000;
		cout << "calibrateOnBaseSide: xDiff (desired - actual): " << xDiff << " (in mm)" << endl;
		setOdometry(xCurr + xDiff, yCurr, phiCurr);
		break;
	}
	case Direction::DIRECTION::SOUTH:
	{

		break;
	}
	case Direction::DIRECTION::WEST:
	{

		break;
	}
	default:
		break;
	}

	/*cout << "wait for key pressed ..." << endl;
	cin.get(); cin.clear();*/
}

float SensorServer::getRobotX()
{
	return this->getX();
}

float SensorServer::getRobotY()
{
	return this->getY();
}

float SensorServer::getRobotPhi()
{
	return this->getPhi();
}

/*
 * vx in mm/s
 * vy in mm/s
 * vphi in degree/s
 */
void SensorServer::setVelocity(float vx, float vy, float vphi)
{
#if SIMULATION_MODE == 1
	odometry_sim.setVelocity(vx/1000.0,vy/1000.0, DEGTORAD(vphi));
#else
	omniDrive.setVelocity(vx/1000.0,vy/1000.0, DEGTORAD(vphi));
#endif
}

void SensorServer::getNewSensorValues(SensorEventGeneratorBuffer* buf, SensorEventGeneratorBuffer* oldSensorState)
{
	double myX, myY, myPhi;

	buf->sensorFrontLeftObstacle = frontLeftBlack();
	buf->sensorFrontRightObstacle = frontRightBlack();
	buf->sensorFloorLeftBlack = floorLeftBlack();
	buf->sensorFloorRightBlack = floorRightBlack();
	buf->sensorPuckBlack = puckBlack();

	buf->sensorHasDrink1 = hasDrink1();
	buf->sensorHasDrink2 = hasDrink2();
	buf->sensorHasDrink3 = hasDrink3();

	distanceSensorArray.distances(buf->sensorDistance);

	//buf->sensorPuckBias = buf->sensorDistance[0]; old IR-Sensor
	getOdometry(myX, myY, myPhi);

	if(oldSensorState != NULL)
	{
//		if(buf->sensorPuckBias == 0)
//			buf->sensorPuckBias = oldSensorState->sensorPuckBias;

		for(unsigned int i=0; i<9; i++)
		{
			if(buf->sensorDistance[i] == 0)
				buf->sensorDistance[i] = oldSensorState->sensorDistance[i];
		}
	}

	buf->obstacleBuffer = this->getLatestObstacleBuffer();
}
