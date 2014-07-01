/*
 * SensorServer.cpp
 *
 *  Created on: 25.04.2011
 *      Author: root
 */

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
	robotCamera = new V4LRobotCamera();
#if USE_CAMERA == 1
	robotCamera->startThread();
#endif

	laserScanner = new LaserScanner(this);
	laserScanner->startThread();
#if SIMULATION_MODE == 0
	while(!laserScanner->hasValidData())
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
#endif
}

SensorServer::~SensorServer()
{
	if(robotCamera != NULL)
		delete robotCamera;

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

CameraPuckState::CameraPuckState SensorServer::calcPuckInfos(bool &gotCameraPuckPos, float* xWorldPos, float* yWorldPos, float* distWorld, float myX, float myY, float myPhi)
{
	CameraPuckDetection::CameraPuckDetection actualPuckDetection = robotCamera->getPuckDetection();
	vector<puckContainer> vPucks = robotCamera->getPucksFinalCloned();
	CameraPuckState::CameraPuckState currPuckState = robotCamera->getPuckState();

	//cout << "entered calcPuckInfos with actualPuckDetection = " << CameraPuckDetection::cCameraPuckDetection[actualPuckDetection] << endl;

	puckContainer nearestPuck;
	bool nearestPuckOnceSet = false;

	unsigned int nearestDistance = numeric_limits<unsigned int>::max();
	unsigned int actualNearestDistance = nearestDistance;

	for(unsigned int i=0; i<vPucks.size(); i++)
	{
		vPucks.at(i).worldAbsX = vPucks.at(i).worldRelX;
		vPucks.at(i).worldAbsY = vPucks.at(i).worldRelY;
		relToAbsCoord(&(vPucks.at(i).worldAbsX), &(vPucks.at(i).worldAbsY), myX, myY, myPhi);

		// if CameraPuckDetection is ALL, the distance is just the relative x-image-distance
		if(actualPuckDetection == CameraPuckDetection::ALL_INPUT_LEFT)
		{
			actualNearestDistance = (unsigned int)((int)(vPucks.at(i).imgX) + 3*(int(CAM_HEIGHT-vPucks.at(i).imgY)));
			//actualNearestDistance = (unsigned int)((int)(CAM_WIDTH-vPucks[i][0]) + 3*(int(CAM_HEIGHT-vPucks[i][1])));
			//actualNearestDistance = (unsigned int)(int(CAM_HEIGHT-vPucks[i][1]));
		}
		else if(actualPuckDetection == CameraPuckDetection::ALL_INPUT_RIGHT)
		{
			actualNearestDistance = (unsigned int)((int)(CAM_WIDTH-vPucks.at(i).imgX) + 3*(int(CAM_HEIGHT-vPucks.at(i).imgY)));
		}
		else if(actualPuckDetection == CameraPuckDetection::ALL_MACHINE)
		{
			actualNearestDistance = (unsigned int)(int(CAM_HEIGHT-vPucks.at(i).imgY));
		}
		else
			actualNearestDistance = (unsigned int)sqrt((float)SQUARE((int)(robotCamera->GRABBER_MIDPOINT-vPucks.at(i).imgX)) + (float)SQUARE(int(CAM_HEIGHT-vPucks.at(i).imgY)));

		//cout << "puck " << i << ": " << vPucks.at(i).imgX << " - " << vPucks.at(i).imgY << " - " << vPucks.at(i).worldRelX << " - " << vPucks.at(i).worldRelY << " - " << vPucks.at(i).worldAbsX << " - " << vPucks.at(i).worldAbsY << " - " << actualNearestDistance << endl;

		if(actualNearestDistance < nearestDistance)
		{
			nearestPuck.imgX = vPucks.at(i).imgX;
			nearestPuck.imgY = vPucks.at(i).imgY;
			nearestPuck.worldRelX = vPucks.at(i).worldRelX;
			nearestPuck.worldRelY = vPucks.at(i).worldRelY;
			nearestPuck.worldAbsX = vPucks.at(i).worldAbsX;
			nearestPuck.worldAbsY = vPucks.at(i).worldAbsY;
			nearestPuckOnceSet = true;
			nearestDistance = actualNearestDistance;
		}
	}

	if(!nearestPuckOnceSet)
	{
		//cout << "=> no nearestPuck" << endl;

		gotCameraPuckPos = false;
		if(actualPuckDetection == CameraPuckDetection::OFF)
			return CameraPuckState::OFF;
		else
			return CameraPuckState::NO_PUCK;
	}

	//cout << "=> nearestPuck: " << nearestPuck.imgX << " - " << nearestPuck.imgY << " - " << nearestPuck.worldRelX << " - " << nearestPuck.worldRelY << " - " << nearestPuck.worldAbsX << " - " << nearestPuck.worldAbsY << endl;

	if(actualPuckDetection == CameraPuckDetection::SEARCH_CATCH_STRATEGY)
	{
		unsigned int catch_bottom_left_bias = robotCamera->GRABBER_MIDPOINT - ((robotCamera->GRABBER_MIDPOINT-robotCamera->CATCH_BOTTOM_LEFT) * (unsigned int)nearestPuck.imgY / (CAM_HEIGHT));

		//cout << "SEARCH_CATCH_STRATEGY: " << catch_bottom_left_bias << " - " << nearestPuck.imgX << " - " << (2*robotCamera->GRABBER_MIDPOINT - catch_bottom_left_bias) << endl;

		if((unsigned int)nearestPuck.imgX < catch_bottom_left_bias)
			currPuckState = CameraPuckState::CATCH_PUCK_LEFT;
		else if((unsigned int)nearestPuck.imgX > 2*robotCamera->GRABBER_MIDPOINT - catch_bottom_left_bias)
			currPuckState = CameraPuckState::CATCH_PUCK_RIGHT;
		else
			currPuckState = CameraPuckState::CATCH_PUCK_STRAIGHT;
	}
	else
		currPuckState = CameraPuckState::PUCK_IN_SIGHT;


	*xWorldPos = nearestPuck.worldAbsX;
	*yWorldPos = nearestPuck.worldAbsY;
	*distWorld = sqrt(SQUARE(*xWorldPos) + SQUARE(*yWorldPos));

	gotCameraPuckPos = true;

	// do not see pucks near the wall during the process of puck-grabbing from the insert zone
	// TODO: needed anymore ?! CHECK!
	/*if(*xWorldPos < 145)
		return false;*/

	return currPuckState;
}

/*bool SensorServer::getPuckAbsPos(float* xWorldPos, float* yWorldPos, float myX, float myY, float myPhi) const
{
	if(!robotCamera->getPuckRelPos(xWorldPos, yWorldPos))
		return false;

	relToAbsCoord(xWorldPos, yWorldPos, myX, myY, myPhi);

	// do not see pucks near the wall during the process of puck-grabbing from the insert zone
	// TODO: needed anymore ?! CHECK!
	if(*xWorldPos < 145)
		return false;

	return true;
}*/

bool SensorServer::getLampAbsPos(float* xWorldPos, float* yWorldPos, float myX, float myY, float myPhi) const
{
	if(!robotCamera->getLampRelPos(xWorldPos, yWorldPos))
		return false;

	// in case the lamp-entry-point is behind the robot, do not apply the new lampPos
	if(*xWorldPos < 0)
		return false;

	relToAbsCoord(xWorldPos, yWorldPos, myX, myY, myPhi);	// TODO: DELETE for kinect stuff

	return true;
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

void SensorServer::setCameraDetection(CameraPuckDetection::CameraPuckDetection puckDetection, CameraLightDetection::CameraLightDetection lightDetection)
{
	robotCamera->setPuckDetection(puckDetection);
	robotCamera->setLightDetection(lightDetection);
}

void SensorServer::setHavingPuck(bool havingPuck)
{
	if(havingPuck)
	{
		setCameraDetection(CameraPuckDetection::NEARBY, CameraLightDetection::OFF);
		this->havingPuck = true;
	}
	else
	{
		setCameraDetection(CameraPuckDetection::OFF, CameraLightDetection::OFF);
		this->havingPuck = false;
	}
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
		yDiff = (msgEnvironment.y_base_end - midPosGlobY)*1000;
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

		//cout << "=> scan diffs between scan curr " << i << " (" << currPosGlob.x << ", " << currPosGlob.y << ", angle: " << RADTODEG(scan.angles.at(i)) << ") and scan last " << (i-1) << " (" << lastPosGlob.x << ", " << lastPosGlob.y << ", angle: " << RADTODEG(scan.angles.at(i-1)) << ")" << endl;

		xDiff = (msgEnvironment.x_base_end - lastPosGlob.x)*1000;
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

bool SensorServer::setCameraSettingsforPuck()
{
	return robotCamera->switchToPuckDetection();
}

bool SensorServer::setCameraSettingsforLight()
{
	return robotCamera->switchToLightDetection();
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

	buf->cameraPuckPos = vec3D();
	buf->gotCameraPuckPos = false;
	if(robotCamera->getVPucksFinalProcessed())
	{
		buf->cameraPuckState = calcPuckInfos(buf->gotCameraPuckPos, &(buf->cameraPuckPos.x), &(buf->cameraPuckPos.y), &(buf->cameraPuckPos.phi), myX, myY, myPhi);
		if(buf->cameraPuckState != CameraPuckState::OFF && !buf->gotCameraPuckPos)
			buf->cameraPuckState = CameraPuckState::NO_PUCK;
	}
	/*else if(oldSensorState != NULL)
		buf->cameraPuckState = oldSensorState->cameraPuckState;*/
	else
		buf->cameraPuckState = CameraPuckState::OFF;

	//cout << "puckState 1/2: " << CameraPuckState::cCameraPuckState[buf->cameraPuckState] << endl;

	if(buf->cameraPuckState != CameraPuckState::OFF && !buf->gotCameraPuckPos)
		buf->cameraPuckState = CameraPuckState::NO_PUCK;
	buf->havingPuck = havingPuck;

	//cout << "puckState 2/2: " << CameraPuckState::cCameraPuckState[buf->cameraPuckState] << endl;;

	//cout << "buf->gotCameraPuckPos: " << buf->gotCameraPuckPos << " and " << buf->cameraPuckPos.x << "," << buf->cameraPuckPos.y << "," << buf->cameraPuckPos.phi << endl;

	/*buf->cameraPuckState = robotCamera->getPuckState();
	buf->cameraPuckPos = new float[3];
	buf->gotCameraPuckPos = getPuckAbsPos(&buf->cameraPuckPos[0], &buf->cameraPuckPos[1], myX, myY, myPhi);
	buf->cameraPuckPos[2] = sqrt(SQUARE(buf->cameraPuckPos[0]) + SQUARE(buf->cameraPuckPos[1]));
	if(buf->cameraPuckState != CameraPuckState::OFF && !buf->gotCameraPuckPos)
		buf->cameraPuckState = CameraPuckState::NO_PUCK;
	buf->havingPuck = havingPuck;*/

	buf->cameraLampState = robotCamera->getLampState();
	buf->cameraLampPos = vec3D();
	buf->gotCameraLampPos = getLampAbsPos(&(buf->cameraLampPos.x), &(buf->cameraLampPos.y), myX, myY, myPhi);
	buf->cameraLampPos.phi = sqrt(SQUARE(buf->cameraLampPos.x) + SQUARE(buf->cameraLampPos.y));

	buf->cameraLightState = robotCamera->getLightState();

	buf->obstacleBuffer = this->getLatestObstacleBuffer();
}
