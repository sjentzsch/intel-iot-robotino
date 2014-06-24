/*
 * MotorController.cpp
 *
 *  Created on: 22.04.2011
 *      Author: root
 */

#include "MotorController.h"
#include "StateBehaviorController.h"
#include "StateMachineEvents.h"

const float MotorController::MIN_SPEED(40.0f); // original value 40.0f (03.06.11)
const float MotorController::MIN_ROT_SPEED(8.0f); // original value 5.0f (03.06.11), increased to 7.0f (29.0
const unsigned int MotorController::MIN_DISTANCE(15);
const unsigned int MotorController::MIN_DEGREE_DISTANCE(3);
const unsigned int MotorController::SLOW_DEGREE_DISTANCE(45);
const unsigned int MotorController::SLOW_START_DURATION(2000);
const unsigned int MotorController::SLOW_DISTANCE(250);
const float MotorController::PHI_CONTROL_PROPORTIONAL_CONSTANT(2.5);
const unsigned int MotorController::DEST_TO_TARGET_BEGIN_ROTATE = 1100; // original value 600
const float MotorController::MAX_ROT_SPEED(20.0f); // original value 30.0f (02.07.11)
const float MotorController::MAX_SPEED(300.0f);


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

		cout << "moveToAbsPos_impl: " << destX << " - " << destY << " - " << destPhi << " and mine: " << sensorSrv->getX() << " " << sensorSrv->getY() << " " << sensorSrv->getPhi() << endl;

		do
		{
			//cout << "[MotorController] Checkpoint 1" << endl;
			if(!checkSignalStatus()) //received TERMINATE signal
			{
				wasTerminated = true;
				break;
			}
			//cout << "[MotorController] Checkpoint 2" << endl;

			//GOT PAUSE SIGNAL?
			while(ModelProvider::getInstance()->gameStateIsPaused()){
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
