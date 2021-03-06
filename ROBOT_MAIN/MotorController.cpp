//
// MotorController.cpp
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//   Sebastian Riedel <riedels@cs.tum.edu>
//
// Copyright (c) 2014 Sören Jentzsch, Sebastian Riedel
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

#include "MotorController.h"
#include "StateBehaviorController.h"
#include "StateMachineEvents.h"

const float MotorController::MIN_SPEED(60.0f); // original value 40.0f (03.06.11)
const float MotorController::MIN_ROT_SPEED(8.0f); // original value 5.0f (03.06.11), increased to 7.0f (29.0
const unsigned int MotorController::MIN_DISTANCE(15);
const unsigned int MotorController::MIN_DEGREE_DISTANCE(2);
const unsigned int MotorController::SLOW_DEGREE_DISTANCE(45);
const unsigned int MotorController::SLOW_START_DURATION(2000);
const unsigned int MotorController::SLOW_DISTANCE(250);
const float MotorController::PHI_CONTROL_PROPORTIONAL_CONSTANT(2.5);
const unsigned int MotorController::DEST_TO_TARGET_BEGIN_ROTATE = 1100; // original value 600
const float MotorController::MAX_ROT_SPEED(30.0f); // original value 30.0f (02.07.11)
const float MotorController::MAX_SPEED(400.0f);


MotorController::MotorController(SensorServer *sensorSrv_):sensorSrv(sensorSrv_){
//	paused = true;
//	terminate = false;
//	drive.setComId(comId_);
	signal = MotorCtrlerSignal::PAUSE;
	slowStartTimer.reset();
	execThread = NULL;
	stateCtrl = NULL;

//	state = mem->getData();
//	state->isDirectControl = true;
//	encoder = new Encoder(state);

	coutCounter = 0;
	fps = 0;
}

MotorController::~MotorController() {

}

vec3D MotorController::relToAbs(vec3D vPoint)
{
	double destVectorLength = sqrt(vPoint.x*vPoint.x+vPoint.y*vPoint.y); //pythagoras
	double destVectorRelAngle;

	if(destVectorLength == 0) // avoid 0/0 division
	{
		destVectorRelAngle = 0;
	}
	else
	{
		destVectorRelAngle = acos(vPoint.x/destVectorLength);
		if(vPoint.y < 0)
			destVectorRelAngle = -destVectorRelAngle;
	}

	vec3D curPose;
	sensorSrv->getOdometry(curPose);
	double sensPhi = curPose.phi;
	double sensPhiRad = DEGTORAD(curPose.phi);

	return vec3D(cos(destVectorRelAngle+sensPhiRad)*destVectorLength + curPose.x, sin(destVectorRelAngle+sensPhiRad)*destVectorLength + curPose.y, sensPhi+vPoint.phi);
}

void MotorController::moveToRelPos(float destX, float destY, float destPhi, float myMaxSpeed, float myMaxRotSpeed, bool onlyRotate, ForceRotationDirection::ForceRotationDirection forcedDir)
{
	double destVectorLength = sqrt(destX*destX+destY*destY); //pythagoras
	double destVectorRelAngle;

	if(destVectorLength == 0) // avoid 0/0 division
	{
		destVectorRelAngle = 0;
	}
	else
	{
		destVectorRelAngle = acos(destX/destVectorLength);
		if(destY < 0)
			destVectorRelAngle = -destVectorRelAngle;
	}

	vec3D pose;
	sensorSrv->getOdometry(pose);
	double sensPhi = pose.phi;
	double sensPhiRad = DEGTORAD(sensPhi);

	if(destPhi != 0)
		moveToAbsPos(cos(destVectorRelAngle+sensPhiRad)*destVectorLength + pose.x, sin(destVectorRelAngle+sensPhiRad)*destVectorLength + pose.y, sensPhi+destPhi, myMaxSpeed, myMaxRotSpeed, onlyRotate, forcedDir);
	else
		moveToAbsPos(cos(destVectorRelAngle+sensPhiRad)*destVectorLength + pose.x, sin(destVectorRelAngle+sensPhiRad)*destVectorLength + pose.y, sensPhi+destPhi, myMaxSpeed, 5.0, onlyRotate, forcedDir);
}

void MotorController::moveToAbsPos(vector<vec3D> vPoints, float myMaxSpeed, float myMaxRotSpeed, bool onlyRotate, ForceRotationDirection::ForceRotationDirection forcedDir)
{
	if(execThread != NULL)
	{
		//Terminate current thread
		terminate(); //Blocks until execThread is terminated
	}

	pause(); //prevent dangling TERMINATE signals by actively signaling PAUSE

	string msg = "";
	for(unsigned int i=0; i<vPoints.size(); i++)
	{
		msg += "<"+boost::lexical_cast<string>(vPoints.at(i).x)+","+boost::lexical_cast<string>(vPoints.at(i).y)+","+boost::lexical_cast<string>(vPoints.at(i).phi)+">";
		if(i < vPoints.size()-1)
			msg += ", ";
	}

	FileLog::log(log_MotorController, "Path of ", FileLog::integer(vPoints.size())," points: "+msg);
	execThread = new boost::thread(&MotorController::moveToAbsPos_impl,this,vPoints,myMaxSpeed,myMaxRotSpeed, onlyRotate, forcedDir);
	run(); //start the motor thread
}

void MotorController::moveToAbsPos(float destX, float destY, float destPhi, float myMaxSpeed, float myMaxRotSpeed, bool onlyRotate, ForceRotationDirection::ForceRotationDirection forcedDir)
{
	vector<vec3D> vPoints(1);
	vPoints.at(0) = vec3D(destX, destY, destPhi);

	moveToAbsPos(vPoints, myMaxSpeed, myMaxRotSpeed, onlyRotate, forcedDir);
}

void MotorController::moveToAbsPos_impl(vector<vec3D> vPoints, float myMaxSpeed, float myMaxRotSpeed, bool onlyRotate, ForceRotationDirection::ForceRotationDirection forcedDir)
{
	/* if the distance is closer than this threshold, begin to rotate */
	bool wasTerminated = false;
	static int counter = 0;

	/* rotate: declare variables */
	float destPhiNow;
	float nowPhi, distRotate, diffPhi, dirRotate, speedRotate;

	/* drive: declare variables */
	float rotX, rotY, distDrive, speedDrive;

	/* timer to ensure slow start */
	slowStartTimer.start();

	float destX;
	float destY;
	float destPhi;

	float currX;
	float currY;
	float currPhi;

	/*float currPathDistance = 0.0f;
	for(unsigned int i=0; i<vPoints.size(); i++)
	{
		if(i == 0)
			pathDistance += sqrt(SQUARE(vPoints.at(i).x-sensorSrv->getX()) + SQUARE(vPoints.at(i).y-sensorSrv->getY()));
		else
			pathDistance += sqrt(SQUARE(vPoints.at(i+1).x-vPoints.at(i).x) + SQUARE(vPoints.at(i+1).y-vPoints.at(i).y));
	}*/

	unsigned int MIN_DISTANCE_curr;
	unsigned int MIN_DEGREE_DISTANCE_curr;

	for(unsigned int i=0; i<vPoints.size(); i++)
	{
		FileLog::log(log_MotorController, "Driving ", FileLog::integer(i), "/", FileLog::integer(vPoints.size())," point");

		destX = vPoints.at(i).x;
		destY = vPoints.at(i).y;
		destPhi = vPoints.at(i).phi;

		//FileLog::log(log_MotorController, "CP 1");
		destPhiNow = getPositiveDegree(sensorSrv->getPhi());
		//FileLog::log(log_MotorController, "CP 2");

		if(i == vPoints.size()-1)
		{
			MIN_DISTANCE_curr = MIN_DISTANCE;
			MIN_DEGREE_DISTANCE_curr = MIN_DEGREE_DISTANCE;
		}
		else
		{
			// TODO: check
			MIN_DISTANCE_curr = 20;
			MIN_DEGREE_DISTANCE_curr = 3;
		}

		cout << "moveToAbsPos_impl: driving from curr (" << sensorSrv->getX() << ", " << sensorSrv->getY() << ", " << sensorSrv->getPhi() << ") to goal (" << destX << ", " << destY << ", " << destPhi << ")" << endl;

		do
		{
			//cout << "[MotorController] Checkpoint 1" << endl;
			if(!checkSignalStatus()) //received TERMINATE signal
			{
				wasTerminated = true;
				break;
			}
			//cout << "[MotorController] Checkpoint 2" << endl;

			// GOT PAUSE SIGNAL? Wait!
			while(!DataProvider::getInstance()->isRunning())
			{
				setVelocity(0, 0, 0);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				slowStartTimer.reset();
				slowStartTimer.start();
			}

			//FileLog::log(log_MotorController, "CP 3");

			vec3D pose;
			sensorSrv->getOdometry(pose);
			currX = pose.x;
			currY = pose.y;
			currPhi = pose.phi;

			//FileLog::log(log_MotorController, "CP 4");

			//Make FPS calculation
	//		dtToLastImageInMs = (int)timer.msecsElapsed();
	//		if(dtToLastImageInMs != 0) fps = 1000 / dtToLastImageInMs;
	//		timer.reset();
	//		timer.start();
	//		if(coutCounter % 100 == 0)
	//			FileLog::log_DEBUG("[MotorController] Current FPS: ", FileLog::integer(fps));


			//received RUN signal
			//cout << "[MotorController] Checkpoint 5" << endl;
			rotX = cos(-DEGTORAD(currPhi))*(destX-currX) - sin(-DEGTORAD(currPhi))*(destY-currY);
			rotY = sin(-DEGTORAD(currPhi))*(destX-currX) + cos(-DEGTORAD(currPhi))*(destY-currY);

			distDrive = sqrt(SQUARE(rotX) + SQUARE(rotY));

			if(distDrive < DEST_TO_TARGET_BEGIN_ROTATE)
			{
				destPhiNow = getPositiveDegree(destPhi);
			}
			//cout << "[MotorController] Checkpoint 6" << endl;
			nowPhi = getPositiveDegree(currPhi);
			distRotate = getDistDegree(destPhiNow, nowPhi);
			diffPhi = destPhiNow - nowPhi;
			//cout << "[MotorController] Checkpoint 7" << endl;
			if(distRotate > 15 && forcedDir==ForceRotationDirection::LEFT)
				dirRotate = 1;
			else if(distRotate > 15 && forcedDir == ForceRotationDirection::RIGHT)
				dirRotate = -1;
			else if((diffPhi < -180) || (diffPhi > 0 && diffPhi < 180)) // ForceRotationDirection == SHORTEST
				dirRotate = 1;	// rotate anti-clockwise (= to the left)
			else
				dirRotate = -1;	// rotate clockwise (= to the right)

			speedDrive = 0;
			speedRotate = 0;

			//cout << "x: " << sensorSrv->getX() << "\t y: " << sensorSrv->getY() << "\t p: " << sensorSrv->getPhi() << "\t distDrive: " << distDrive << "\t distRotate: " << distRotate << endl;
			//cout << "[MotorController] Checkpoint 8" << endl;
			if(distDrive > MIN_DISTANCE_curr)
			{
				if(slowStartTimer.msecsElapsed() < SLOW_START_DURATION && (distDrive > SLOW_DISTANCE || i < vPoints.size()-1))
					speedDrive = ((float)slowStartTimer.msecsElapsed())/SLOW_START_DURATION*(myMaxSpeed-MIN_SPEED) + MIN_SPEED;
				else if(i == vPoints.size()-1 && distDrive <= SLOW_DISTANCE)
					speedDrive = (myMaxSpeed-MIN_SPEED)/(SLOW_DISTANCE-MIN_DISTANCE_curr) * (distDrive-MIN_DISTANCE_curr) + MIN_SPEED;
				else
					speedDrive = myMaxSpeed;
			}
			//cout << "[MotorController] Checkpoint 9" << endl;
			if(distRotate > MIN_DEGREE_DISTANCE_curr)
			{
				if(slowStartTimer.msecsElapsed() < SLOW_START_DURATION)
					speedRotate = dirRotate*(((float)slowStartTimer.msecsElapsed())/SLOW_START_DURATION*(myMaxRotSpeed-MIN_ROT_SPEED) + MIN_ROT_SPEED);

				/*else if(distRotate > SLOW_DEGREE_DISTANCE)
					speedRotate = dirRotate*(myMaxRotSpeed);
				else
					speedRotate = dirRotate*((myMaxRotSpeed-MIN_ROT_SPEED)/(SLOW_DEGREE_DISTANCE-MIN_DEGREE_DISTANCE_curr) * (distRotate-MIN_DEGREE_DISTANCE_curr) + MIN_ROT_SPEED);*/
				speedRotate = abs(distRotate)*PHI_CONTROL_PROPORTIONAL_CONSTANT; // make rotation speed proportional to difference in between target and current phi
				if(speedRotate < MIN_ROT_SPEED)
					speedRotate = MIN_ROT_SPEED;
				if(speedRotate > myMaxRotSpeed)
					speedRotate = myMaxRotSpeed;

				speedRotate *= dirRotate; // give the speed the correct direction


					//cout << "------------ myMaxRotationSpeed: " << myMaxRotSpeed << endl;
					//cout << "------------ Rotationspeed to high: " << speedRotate << endl;
			}
			else
			{
				speedRotate = dirRotate*abs(distRotate)*PHI_CONTROL_PROPORTIONAL_CONSTANT; // make rotation speed proportional to difference in between target and current phi
			}

			//FileLog::log(log_MotorController, "CP 5");

			//cout << "[MotorController] Checkpoint 10" << endl;
			if(distDrive == 0 || onlyRotate)
				setVelocity(0, 0, speedRotate);
			else
				setVelocity(speedDrive*rotX/distDrive, speedDrive*rotY/distDrive, speedRotate);

			//FileLog::log(log_MotorController, "CP 6");

			//cout << "[MotorController] Checkpoint 11" << endl;
			//boost::this_thread::yield(); //make room for the other threads
			if(counter%500==0)
			{
				FileLog::log_NOTICE("[MotorController] running: distDrive ", FileLog::real(distDrive), " distRotate: ", FileLog::real(distRotate), " speedRotate:", FileLog::real(speedRotate), " speedDrive:", FileLog::real(speedDrive));
			}
			counter++;
			//cout << "[MotorController] Checkpoint 12" << endl;
			boost::this_thread::sleep(boost::posix_time::milliseconds(5));
			//cout << "[MotorController] Checkpoint 13" << endl;

		} while ((distDrive > MIN_DISTANCE_curr && !onlyRotate) || distRotate > MIN_DEGREE_DISTANCE_curr);
	}

	//FileLog::log(log_MotorController, "CP 7");

	slowStartTimer.reset();
	setVelocity(0, 0, 0);

	//FileLog::log(log_MotorController, "CP 8");

	if(!wasTerminated)
	{
		sendReadyEvent();
	}
}

void MotorController::moveToAbsPosCF_impl(float destX, float destY, float myMaxSpeed, bool allowRedefineTarget)
{
	const unsigned int SLOW_DISTANCE_CF = 800;
	const unsigned int MIN_DISTANCE_CF = allowRedefineTarget ? 100 : 20;
	const unsigned int MAX_DIST_TO_ORIG_GOAL = 600;

	enum DRIVE_DIR {BOTH, ONLY_LEFT, ONLY_RIGHT};
	DRIVE_DIR currDriveDir = DRIVE_DIR::BOTH;

	bool wasTerminated = false;
	unsigned int counter = 0;

	vec3D pose;
	sensorSrv->getOdometry(pose);

	if(this->msgEnvironment != NULL)
		delete this->msgEnvironment;
	this->msgEnvironment = new MsgEnvironment(DataProvider::getInstance()->getLatestMsgEnvironment());

	/* rotate: declare variables */
	float destPhiNow = getPositiveDegree(RADTODEG(atan2(destY - pose.y, destX - pose.x)));
	float nowPhi, distRotate, diffPhi, dirRotate, speedRotate;

	/* drive: declare variables */
	float distToOrigGoal = sqrt(SQUARE(destX-pose.x) + SQUARE(destY-pose.y));
	float distToGoal = distToOrigGoal;
	float currBaseVel = MIN_SPEED;

	/* variables used for local-minima-avoidance routine */
	/*float lastDistToGoal = distToGoal;
	bool avoidToLeft = false;
	bool avoidToRight = false;*/
	//bool allowDriveToRight = true;

	/* variables used for redefining target */
	float currDestX = destX;
	float currDestY = destY;

	/* variables used for storing latest control direction -> for bumper routine etc. */
	float currVelX = 0.0f;
	float currVelY = 0.0f;

	/* timer to ensure slow start */
	slowStartTimer.start();

	cout << "moveToAbsPosCF_impl: driving from curr (" << pose.x << ", " << pose.y << ", " << pose.phi << ") to goal (" << currDestX << ", " << currDestY << ", " << destPhiNow << ")" << endl;

	do
	{
		if(!checkSignalStatus()) //received TERMINATE signal
		{
			wasTerminated = true;
			break;
		}

		// GOT PAUSE SIGNAL? Wait!
		while(!DataProvider::getInstance()->isRunning())
		{
			setVelocity(0, 0, 0);
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			slowStartTimer.reset();
			slowStartTimer.start();
		}


		// get current pose/odometry, laser scan, and ir sensors
		sensorSrv->getOdometry(pose);
		LaserScannerReadings scan = this->sensorSrv->getLatestScan();
		this->sensorSrv->getIRSensors(readings);



		// get the position of the nearest obstacle relative to the Robotino frame
		//cout << "#rays blocking my path: " << scan.indicesInFrontPos.size() << endl;
		bool hasObstacle = false;
		float nearestObsX = 0;
		float nearestObsY = 0;
		float distSquare = std::numeric_limits<float>::max();
		unsigned int winnerIndex = 0;
		for(unsigned int i=0; i<scan.indicesInFrontPos.size(); i++)
		{
			float currDistSquare = SQUARE(scan.positions.at(scan.indicesInFrontPos.at(i)).at(0)) + SQUARE(scan.positions.at(scan.indicesInFrontPos.at(i)).at(1));
			if(currDistSquare < distSquare)
			{
				nearestObsX = scan.positions.at(scan.indicesInFrontPos.at(i)).at(0);
				nearestObsY = scan.positions.at(scan.indicesInFrontPos.at(i)).at(1);
				distSquare = currDistSquare;
				winnerIndex = scan.indicesInFrontPos.at(i);
				hasObstacle = true;
			}
			//cout << "\t" << i << ": " << scan.positions.at(scan.indicesInFrontPos.at(i)).at(0) << ", " << scan.positions.at(scan.indicesInFrontPos.at(i)).at(1) << endl;
		}
		float distToNearestObs = sqrt(distSquare)*1000;

		// calculate the remaining distance to goal
		distToGoal = sqrt(SQUARE(currDestX-pose.x) + SQUARE(currDestY-pose.y));

		/*if(!(hasObstacle && allowRedefineTarget))
			cout << "this is why ... " << endl;
		cout << "distToNearestObs: " << distToNearestObs << ", distToGoal: " << distToGoal << endl;
		cout << "rayGlobPosX: " << scan.positionsGlob.at(winnerIndex).at(0) * 1000  << ", rayGlobPosY: " << scan.positionsGlob.at(winnerIndex).at(1) * 1000 << endl;*/

		// redefine goal: if obstacle in front of the original goal location, take a location in front of the obstacle as goal
		if(hasObstacle && allowRedefineTarget && distToNearestObs < distToGoal && fabs(distToNearestObs-distToOrigGoal) < MAX_DIST_TO_ORIG_GOAL)
		{
			float rayGlobPosX = scan.positionsGlob.at(winnerIndex).at(0) * 1000;
			float rayGlobPosY = scan.positionsGlob.at(winnerIndex).at(1) * 1000;
			currDestX = (pose.x > rayGlobPosX) ? (rayGlobPosX + 300) : (rayGlobPosX - 300);
			currDestY = (pose.y > rayGlobPosY) ? (rayGlobPosY + 300) : (rayGlobPosY - 300);
			distToGoal = sqrt(SQUARE(currDestX-pose.x) + SQUARE(currDestY-pose.y));
			cout << "update goal to " << currDestX << ", " << currDestY << endl;
		}

		// discard obstacles behind the goal
		if(hasObstacle && distToNearestObs > (distToGoal + 300))
			hasObstacle = false;

		// include frontal IR-sensors for obstacle detection
		if(readings[1] < 0.1)
		{
			nearestObsX = 0.2;
			nearestObsY = 0.2;
			hasObstacle = true;
		}
		if(readings[8] < 0.1)
		{
			nearestObsX = 0.2;
			nearestObsY = -0.2;
			hasObstacle = true;
		}



		// calculate the amount of sidewards velocity
		float propSidewards = 0.0f;
		float dirSidewards = 1.0f;
		if(hasObstacle)
		{
			if(nearestObsX < OBS_X_STILL_MAX)
				propSidewards = 1.0f;
			else
				propSidewards = (nearestObsX-OBS_X_END)/(OBS_X_STILL_MAX-OBS_X_END);

			if(nearestObsY > 0)
			{
				/*if(avoidToRight)
					lastDistToGoal = distToGoal;*/
				if(currDriveDir == DRIVE_DIR::BOTH)
					currDriveDir = DRIVE_DIR::ONLY_RIGHT;
				//avoidToRight = true;
			}
			else
			{
				/*if(avoidToLeft)
					lastDistToGoal = distToGoal;*/
				if(currDriveDir == DRIVE_DIR::BOTH)
					currDriveDir = DRIVE_DIR::ONLY_LEFT;
				//avoidToLeft = true;
			}

			/*if(avoidToRight && avoidToLeft && lastDistToGoal-distToGoal < 100)
			{
				if(currDriveDir == DRIVE_DIR::BOTH || currDriveDir == DRIVE_DIR::ONLY_RIGHT)
					currDriveDir = DRIVE_DIR::ONLY_LEFT;
				else if(currDriveDir == DRIVE_DIR::ONLY_LEFT)
					currDriveDir = DRIVE_DIR::ONLY_RIGHT;
			}*/

			if(currDriveDir == DRIVE_DIR::ONLY_RIGHT)
				dirSidewards = -1.0f;

			//cout << "nearest ray: " << nearestObsX << ", " << nearestObsY << " => propSidewards: " << propSidewards << ", dirSidewards: " << dirSidewards << endl;
		}
		else
		{
			//lastDistToGoal = distToGoal;
			//avoidToLeft = false;
			//avoidToRight = false;
			currDriveDir = DRIVE_DIR::BOTH;
			//cout << "no obstacle in front => propSidewards: " << propSidewards << ", dirSidewards: " << dirSidewards << endl;
		}


		// calculate the rotation part
		destPhiNow = getPositiveDegree(RADTODEG(atan2(currDestY - pose.y, currDestX - pose.x)));
		nowPhi = getPositiveDegree(pose.phi);
		distRotate = getDistDegree(destPhiNow, nowPhi);
		diffPhi = destPhiNow - nowPhi;

		if((diffPhi < -180) || (diffPhi > 0 && diffPhi < 180))
			dirRotate = 1;	// rotate anti-clockwise (= to the left)
		else
			dirRotate = -1;	// rotate clockwise (= to the right)

		// determine the final rotation speed -> only rotate if need to rotate and if *not* moving very sidewards
		speedRotate = 0;
		if(distRotate > MIN_DEGREE_DISTANCE && propSidewards < 0.95f)
		{
			speedRotate = abs(distRotate)*PHI_CONTROL_PROPORTIONAL_CONSTANT; // make rotation speed proportional to difference in between target and current phi
			if(speedRotate < MIN_ROT_SPEED)
				speedRotate = MIN_ROT_SPEED;
			if(speedRotate > MAX_ROT_SPEED)
				speedRotate = MAX_ROT_SPEED;

			speedRotate *= dirRotate; // give the speed the correct direction
		}
		else
			speedRotate = 0;

		// determine current base velocity
		currBaseVel = (1.0f-propSidewards)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
		if(distToGoal <= SLOW_DISTANCE_CF)
		{
			float baseVelForGoal = (MAX_SPEED-MIN_SPEED)/(SLOW_DISTANCE_CF-MIN_DISTANCE_CF) * (distToGoal-MIN_DISTANCE_CF) + MIN_SPEED;
			if(baseVelForGoal < currBaseVel)
				currBaseVel = baseVelForGoal;
		}


		//cout << "curr (" << pose.x << ", " << pose.y << ", " << pose.phi << ") to goal (" << currDestX << ", " << currDestY << ", " << destPhiNow << ")" << endl;


		if(this->sensorSrv->bumperHasContact())
		{
			cout << "bumper has contact. driving in opposed direction: " << -currVelX << ", " << -currVelY << endl;

			system("nohup play -v 0.05 /home/robotino/Yamaha-SY-35-Clarinet-C5.wav > /dev/null 2>&1 &");

			for(unsigned int i=0; i<600; i++)
			{
				setVelocity(0, 0, 0);
				boost::this_thread::sleep(boost::posix_time::milliseconds(5));
			}
			for(unsigned int i=0; i<100; i++)
			{
				setVelocity(-currVelX, -currVelY, 0);
				boost::this_thread::sleep(boost::posix_time::milliseconds(5));
			}
		}
		else if(hasObstacle && nearestObsX < OBS_X_SAFETY_BACK && nearestObsY > -0.25 && nearestObsY < 0.25 && !IRSensorIsBlocked(4) && !IRSensorIsBlocked(5))
		{
			cout << "drive backwards" << endl;
			currVelX = -MIN_SPEED;
			currVelY = 0;
			setVelocity(currVelX, currVelY, speedRotate);
		}
		else
		{
			bool sidewardsBlocked = false;
			if(dirSidewards > 0.0f)
			{
				if((propSidewards > 0.4f && IRSensorIsBlocked(1)) || (propSidewards > 0.6f && IRSensorIsBlocked(2)) || (propSidewards > 0.85f && IRSensorIsBlocked(3)))
					sidewardsBlocked = true;
			}
			else if(dirSidewards < 0.0f)
			{
				if((propSidewards > 0.4f && IRSensorIsBlocked(8)) || (propSidewards > 0.6f && IRSensorIsBlocked(7)) || (propSidewards > 0.85f && IRSensorIsBlocked(6)))
					sidewardsBlocked = true;
			}

			if(sidewardsBlocked)
			{
				cout << "sidewards way blocked. go the other way." << endl;
				setVelocity(0, 0, speedRotate);
				if(dirSidewards > 0.0f)
					currDriveDir = DRIVE_DIR::ONLY_RIGHT;
				else
					currDriveDir = DRIVE_DIR::ONLY_LEFT;
			}
			else
			{
				currVelX = (1-propSidewards)*currBaseVel;
				currVelY = dirSidewards*propSidewards*currBaseVel;
				//cout << "xVel: " << currVelX << ", yVel: " << currVelY << ", rot: " << speedRotate << endl;
				setVelocity(currVelX, currVelY, speedRotate);
			}
		}

		if(counter%50==0)
		{
			//FileLog::log_NOTICE("[MotorController] running: distToGoal ", FileLog::real(distToGoal), " distRotate: ", FileLog::real(distRotate), " speedRotate:", FileLog::real(speedRotate), " speedDrive:", FileLog::real(speedDrive));
		}
		counter++;
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));

	} while (distToGoal > MIN_DISTANCE_CF);

	slowStartTimer.reset();
	setVelocity(0, 0, 0);

	if(!wasTerminated)
	{
		sendReadyEvent();
	}
}

bool MotorController::IRSensorIsBlocked(unsigned int index)
{
	float posRobFrameX = cos(DEGTORAD(40)*index) * (0.23+IR_SENSOR_THRESHOLD);
	float posRobFrameY = sin(DEGTORAD(40)*index) * (0.23+IR_SENSOR_THRESHOLD);

	float posGlobX, posGlobY;
	this->sensorSrv->transformBase2World(posRobFrameX, posRobFrameY, posGlobX, posGlobY);

	//cout << "sensor " << index << ": " << posGlobX << ", " << posGlobY << endl;

	// IR sensor is blocked if it is physically blocked of if game field ends after a certain distance (= also IR_SENSOR_THRESHOLD)
	return (this->readings[index] < IR_SENSOR_THRESHOLD || posGlobX < 0 || posGlobX > this->msgEnvironment->x_max || posGlobY < 0 || posGlobY > this->msgEnvironment->y_max);
}

/* converts the API degree value (range: -180 - 180) into a positive degree value (range: 0 - 360) */
float MotorController::getPositiveDegree(float phi) const
{
	while(phi<0)
	{
		phi+=360.0f;
	}
	while(phi>=360.0f)
	{
		phi-=360.0f;
	}
	return phi;
}

/* returns the (smallest) distance between two degree values (range of the resulting distance: 0 - 180) */
float MotorController::getDistDegree(float phi1, float phi2) const
{
	float diff = getPositiveDegree(phi1) - getPositiveDegree(phi2);
	if(diff < 0)
		diff = -diff;

	if(diff < 180)
		return diff;
	else
		return 360-diff;
}

void MotorController::pause()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		//FileLog::log_NOTICE("MotorController is signaled PAUSED.");
		signal=MotorCtrlerSignal::PAUSE;
	}
}

void MotorController::run()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		//FileLog::log_NOTICE("MotorController is signaled RUN.");
		signal=MotorCtrlerSignal::RUN;
	}
	signal_cond.notify_all();
}

void MotorController::terminate()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		signal = MotorCtrlerSignal::TERMINATE;
	}
	signal_cond.notify_all();
	//FileLog::log_NOTICE("MotorController is signaled to TERMINATE.");
	if(execThread!=NULL)
	{
		execThread->join(); //Wait for execution thread to terminate
	}
}

/* returns true if signal is RUN
 * 		   false if signal is TERMINATE
 * 		   blocks when signal is PAUSE
 */
bool MotorController::checkSignalStatus()
{
	bool wasPaused=false;
	boost::unique_lock<boost::mutex> lock(signal_mutex);
	while(signal == MotorCtrlerSignal::PAUSE) //wait if signal is PAUSE
	{
		setVelocity(0,0,0);
		slowStartTimer.reset();
		wasPaused=true;
		//cout << "MotorController received signal PAUSED." << endl;
		FileLog::log(log_MotorController, "PAUSED");
		signal_cond.wait(lock); //waits for the notify, handles mutex locking/unlocking
	}
	//Check if signal is RUN or TERMINATE
	if(signal == MotorCtrlerSignal::RUN)
	{
		if(wasPaused)
		{
			slowStartTimer.start();
			//cout << "MotorController received signal RUN." << endl;
			FileLog::log(log_MotorController, "RUN");
		}
		return true;
	}
	else
	{
		//cout << "MotorController received signal TERMINATE." << endl;
		setVelocity(0,0,0);
		FileLog::log(log_MotorController, "TERMINATED");
		signal = MotorCtrlerSignal::PAUSE; //reset signal to PAUSE
		return false; //signal is TERMINATE
	}
}

void MotorController::setVelocity(float vx, float vy, float vphi)
{
	stateCtrl->getSensorControl()->setVelocity(vx,vy,vphi);
}

void MotorController::setStateBehaviorController(StateBehaviorController *stateCtrl_)
{
	this->stateCtrl = stateCtrl_;
}

void MotorController::sendReadyEvent()
{
	boost::shared_ptr<EvMotorCtrlReady> ev(new EvMotorCtrlReady());
	//FileLog::log_DEBUG("[MotorController, sendReadyEvent()] Current odometry: X: ", FileLog::real(sensorSrv->getX()), " Y: ", FileLog::real(sensorSrv->getY()), " Phi: ", FileLog::real(sensorSrv->getPhi()));
	FileLog::log_NOTICE("[MotorController] EvMotorCtrlReady ---->");
	stateCtrl->getAsyncStateMachine()->queueEvent(ev);
	return;
}

void MotorController::moveToAbsPosOnly(float destX, float destY, float myMaxSpeed)
{
	moveToAbsPos(destX, destY, sensorSrv->getPhi(), myMaxSpeed, 5);
}

void MotorController::moveToRelXPosAbsYPos(float destX, float destY, float destPhi, float myMaxSpeed)
{
	moveToAbsPos(sensorSrv->getX()+destX, destY, destPhi, myMaxSpeed);
}

void MotorController::moveToAbsXPosRelYPos(float destX, float destY, float destPhi, float myMaxSpeed)
{
	moveToAbsPos(destX, sensorSrv->getY()+destY, destPhi, myMaxSpeed);
}

void MotorController::rotateToAbsAngle(float destPhi, ForceRotationDirection::ForceRotationDirection forcedDir, float myMaxRotSpeed)
{
	vec2D pose;
	sensorSrv->getOdometry(pose);
	moveToAbsPos(pose.x, pose.y, destPhi, MAX_SPEED, myMaxRotSpeed, true, forcedDir);
}

void MotorController::rotateToRelAngle(float destPhi, ForceRotationDirection::ForceRotationDirection forcedDir, float myMaxRotSpeed)
{
	moveToRelPos(0, 0, destPhi, MAX_SPEED, myMaxRotSpeed, true, forcedDir);
}

void MotorController::moveToAbsPosOnlyCF(float destX, float destY, float myMaxSpeed, bool allowRedefineTarget)
{
	if(execThread != NULL)
	{
		//Terminate current thread
		terminate(); //Blocks until execThread is terminated
	}

	pause(); //prevent dangling TERMINATE signals by actively signaling PAUSE

	execThread = new boost::thread(&MotorController::moveToAbsPosCF_impl,this,destX,destY,myMaxSpeed,allowRedefineTarget);
	run(); //start the motor thread
}
