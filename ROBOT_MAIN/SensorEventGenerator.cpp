/*
 * SensorEventGenerator.cpp
 *
 *  Created on: 25.04.2011
 *      Author: root
 */

#include "config.h"
#include "SensorEventGenerator.h"
#include "SensorServer.h"
#include "StateBehaviorController.h"

const float SensorEventGenerator::FLOOR_SENSOR_DISTANCE(92.5);

SensorEventGenerator::SensorEventGenerator(SensorServer *sensorSrv_):sensorSrv(sensorSrv_)
{
	lastSentCameraPuckPos = vec3D();
	cameraPuckPosWasJustSent = false;
	lastPuckSendTimer.reset();

	lastSentCameraLampPos = vec3D();
	cameraLampPosWasJustSent = false;
	lastLampSendTimer.reset();

	cameraLightWasJustSent = false;
	lastLightSendTimer.reset();

	evObstacleIsCloseThrown = false;

	grid = new Grid();

	oldSensorState = NULL;
	getNewSensorValues();
	oldSensorState = newSensorState;

	lastLineCrossPhi = 0;
	lastLineCrossX = 0;
	lastLineCrossY = 0;
	lastLineCrossSensor = FloorSensor::INIT;

	pause();

	lastObstacleNodesInRange = NULL;
}

SensorEventGenerator::~SensorEventGenerator()
{
	if(oldSensorState != NULL)
		delete oldSensorState;

	if(newSensorState != NULL)
		delete newSensorState;
}

void SensorEventGenerator::pause()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] is signaled PAUSED");
		signal=SensorEventGenSignal::PAUSE;
	}
}

void SensorEventGenerator::run()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] is signaled RUN");
		signal=SensorEventGenSignal::RUN;
	}
	signal_cond.notify_all();
}

/* returns if signal is RUN
 * 		   blocks when signal is PAUSE
 */
void SensorEventGenerator::checkSignalStatus()
{
	boost::unique_lock<boost::mutex> lock(signal_mutex);
	bool wasRunning = true;
	while(signal == SensorEventGenSignal::PAUSE) //wait if signal is PAUSE
	{
		//cout << "SensorEventGenerator received signal PAUSED." << endl;
		FileLog::log(log_SensorEventGenerator, "PAUSED");
		wasRunning = false;
		signal_cond.wait(lock); //waits for the notify, handles mutex locking/unlocking
	}
	//got a run signal
	if(!wasRunning){
		FileLog::log(log_SensorEventGenerator, "RUN");
	}
}

void SensorEventGenerator::monitorSensors()
{
	bool firstRun = false;
	if(true){
		firstRun = true;
	}

	while(true)
	{
		//cout << "monitorSensors 1" << endl;

		if(ModelProvider::getInstance()->gameStateIsPaused())
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			//cout << "monitorSensors() paused" << endl;
			continue;
		}

		//cout << "monitorSensors 2" << endl;

		//Make FPS calculation
//		dtToLastImageInMs = (int)timer.msecsElapsed();
//		if(dtToLastImageInMs != 0) fps = 1000 / dtToLastImageInMs;
//		timer.reset();
//		timer.start();
//		if(coutCounter % 100 == 0)
//			FileLog::log_DEBUG("[SensorEventGenerator] Current FPS: ", FileLog::integer(fps));

		checkSignalStatus(); // blocks if thread is signaled to pause

		getNewSensorValues();

		// brightness sensor front left
		if(!oldSensorState->sensorFrontLeftObstacle && newSensorState->sensorFrontLeftObstacle)
		{
			boost::shared_ptr<EvSensorFrontLeftFoundObstacle> ev(new EvSensorFrontLeftFoundObstacle());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorFrontLeftFoundObstacle");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(oldSensorState->sensorFrontLeftObstacle && !newSensorState->sensorFrontLeftObstacle)
		{
			boost::shared_ptr<EvSensorFrontLeftIsFree> ev(new EvSensorFrontLeftIsFree());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorFrontLeftIsFree");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}


		// brightness sensor front right
		if(!oldSensorState->sensorFrontRightObstacle && newSensorState->sensorFrontRightObstacle)
		{
			boost::shared_ptr<EvSensorFrontRightFoundObstacle> ev(new EvSensorFrontRightFoundObstacle());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorFrontRightFoundObstacle");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(oldSensorState->sensorFrontRightObstacle && !newSensorState->sensorFrontRightObstacle)
		{
			boost::shared_ptr<EvSensorFrontRightIsFree> ev(new EvSensorFrontRightIsFree());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorFrontRightIsFree");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}

		// floor sensors
		//important: the floor sensors always find obstacles when the robot is driving
		//when the robot cross a line the sensor value returns false = 0
		if(!oldSensorState->sensorFloorLeftBlack && newSensorState->sensorFloorLeftBlack)
		{
			boost::shared_ptr<EvSensorFloorLeftIsBlack> ev(new EvSensorFloorLeftIsBlack());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorFloorLeftIsBlack");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);

			if(lastLineCrossSensor == FloorSensor::LEFT || lastLineCrossSensor == FloorSensor::INIT || lastLineCrossTimer.msecsElapsed() > 3000) // last line cross was triggered also by this sensor --> start new line cross
			{
				sensorSrv->getOdometry(lastLineCrossX, lastLineCrossY, lastLineCrossPhi);
				lastLineCrossTimer.reset();
				lastLineCrossTimer.start();
				lastLineCrossSensor = FloorSensor::LEFT;
			}
			else // last line cross was triggered by the other floor sensor --> calculate distance
			{
				handleLineCross();
			}
		}
		if(!oldSensorState->sensorFloorRightBlack && newSensorState->sensorFloorRightBlack)
		{
			boost::shared_ptr<EvSensorFloorRightIsBlack> ev(new EvSensorFloorRightIsBlack());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorFloorRightIsBlack");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);

			if(lastLineCrossSensor == FloorSensor::RIGHT || lastLineCrossSensor == FloorSensor::INIT || lastLineCrossTimer.msecsElapsed() > 3000) // last line cross was triggered also by this sensor --> start new line cross
			{
				sensorSrv->getOdometry(lastLineCrossX, lastLineCrossY, lastLineCrossPhi);
				lastLineCrossTimer.reset();
				lastLineCrossTimer.start();
				lastLineCrossSensor = FloorSensor::RIGHT;
			}
			else // last line cross was triggered by the other floor sensor --> calculate distance
			{
				handleLineCross();
			}
		}

		//check if robot has the puck by checking the puck light-sensor
		if(oldSensorState->sensorPuckBlack == true && newSensorState->sensorPuckBlack == false)
		{
			/*boost::shared_ptr<EvSensorLostPuck> ev(new EvSensorLostPuck());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorLostPuck");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);*/
		}
		else if(oldSensorState->sensorPuckBlack == false && newSensorState->sensorPuckBlack == true)
		{
			/*boost::shared_ptr<EvSensorHasPuck> ev(new EvSensorHasPuck());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorHasPuck");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);*/
		}


			// IR-Sensors
			if((firstRun || oldSensorState->sensorDistance[0] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[0] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance1Free> ev(new EvSensorDistance1Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance1Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[0] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[0] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance1Blocked> ev(new EvSensorDistance1Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance1Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[1] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[1] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance2Free> ev(new EvSensorDistance2Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance2Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[1] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[1] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance2Blocked> ev(new EvSensorDistance2Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance2Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[2] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[2] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance3Free> ev(new EvSensorDistance3Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance3Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[2] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[2] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance3Blocked> ev(new EvSensorDistance3Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance3Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[3] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[3] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance4Free> ev(new EvSensorDistance4Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance4Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[3] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[3] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance4Blocked> ev(new EvSensorDistance4Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance4Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[4] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[4] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance5Free> ev(new EvSensorDistance5Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance5Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[4] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[4] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance5Blocked> ev(new EvSensorDistance5Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance5Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[5] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[5] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance6Free> ev(new EvSensorDistance6Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance6Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[5] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[5] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance6Blocked> ev(new EvSensorDistance6Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance6Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[6] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[6] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance7Free> ev(new EvSensorDistance7Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance7Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[6] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[6] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance7Blocked> ev(new EvSensorDistance7Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance7Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[7] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[7] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance8Free> ev(new EvSensorDistance8Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance8Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[7] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[7] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance8Blocked> ev(new EvSensorDistance8Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance8Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			if((firstRun || oldSensorState->sensorDistance[8] >= SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[8] < SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance9Free> ev(new EvSensorDistance9Free());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance9Free");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}
			else if((firstRun || oldSensorState->sensorDistance[8] < SensorPuckBiasConstants::BLOCKED) && newSensorState->sensorDistance[8] >= SensorPuckBiasConstants::BLOCKED)
			{
				boost::shared_ptr<EvSensorDistance9Blocked> ev(new EvSensorDistance9Blocked());
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDistance9Blocked");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
			}


		// securing puck
		// TODO: apply: camera->getNearestPuck(&puckXPos, &puckYPos, &puckDist) && (puckXPos > camera->CATCH_BOTTOM_LEFT-20) && (puckXPos < camera->CATCH_BOTTOM_RIGHT+20) && (puckYPos > 7*camera->CAM_HEIGHT/8));
//		if(newSensorState->havingPuck && newSensorState->sensorPuckBias < SensorPuckBiasConstants::LOST && newSensorState->cameraPuckState == CameraPuckState::PUCK_IN_SIGHT && (!oldSensorState->havingPuck || oldSensorState->sensorPuckBias >= SensorPuckBiasConstants::LOST || oldSensorState->cameraPuckState != CameraPuckState::PUCK_IN_SIGHT))
//		{
//			boost::shared_ptr<EvRobotLostPuck> ev(new EvRobotLostPuck());
//			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvRobotLostPuck");
//			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
//		}

		// cout << CameraPuckState::cCameraPuckState[newSensorState->cameraPuckState] << endl;
		// camera puck detection

		if(newSensorState->cameraPuckState == CameraPuckState::OFF)
		{
			cameraPuckPosWasJustSent = false;
			lastPuckSendTimer.reset();
		}

		if(false)//newSensorState->cameraPuckState == CameraPuckState::PUCK_IN_SIGHT && newSensorState->gotCameraPuckPos && (oldSensorState->cameraPuckState != CameraPuckState::PUCK_IN_SIGHT || (cameraPuckPosWasJustSent && (BaseParameterProvider::getInstance()->getParams()->simulation_mode || abs(newSensorState->cameraPuckPos.phi - lastSentCameraPuckPos.phi) > 40) && lastPuckSendTimer.msecsElapsed() > 1000)))
		{
//			boost::shared_ptr<EvCameraPuckDetected> ev(new EvCameraPuckDetected(newSensorState->cameraPuckPos.x, newSensorState->cameraPuckPos.y));
//			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvCameraPuckDetected (", FileLog::real(newSensorState->cameraPuckPos.x), ", ", FileLog::real(newSensorState->cameraPuckPos.y), ")");
//			stateCtrl->getAsyncStateMachine()->queueEvent(ev);

			lastSentCameraPuckPos.x = newSensorState->cameraPuckPos.x;
			lastSentCameraPuckPos.y = newSensorState->cameraPuckPos.y;
			lastSentCameraPuckPos.phi = newSensorState->cameraPuckPos.phi;

			cameraPuckPosWasJustSent = true;
			lastPuckSendTimer.reset();
			lastPuckSendTimer.start();
		}



		// camera light detection

		if(newSensorState->cameraLightState == CameraLightState::OFF)
		{
			cameraLightWasJustSent = false;
			lastLightSendTimer.reset();
		}


		float myX, myY, myPhi;
		this->sensorSrv->getOdometry(myX, myY, myPhi);
		// Laserscanner Events: Only in non-simulation mode and if angle did not change too much between calculation and evaluation
		if(abs(newSensorState->obstacleBuffer.my_phi - myPhi) < 1)
		{
			// EvObstacleIsClose
			/*float minDist = std::numeric_limits<float>::max();
			unsigned int minClusterIndex = 0;
			for(unsigned int c=0; c<newSensorState->obstacleBuffer.obstacles.size(); c++)
			{
				float currDist = sqrt(SQUARE(newSensorState->obstacleBuffer.obstacles.at(c).at(0)-myX)+SQUARE(newSensorState->obstacleBuffer.obstacles.at(c).at(1)-myY));
				if(currDist < minDist)
				{
					minDist = currDist;
					minClusterIndex = c;
				}
			}
			if(minDist < 500 && !evObstacleIsCloseThrown)
			{
				boost::shared_ptr<EvObstacleIsClose> ev(new EvObstacleIsClose(newSensorState->obstacleBuffer.obstacles.at(minClusterIndex).at(0), newSensorState->obstacleBuffer.obstacles.at(minClusterIndex).at(1)));
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvObstacleIsClose");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);
				evObstacleIsCloseThrown = true;
			}
			else
				evObstacleIsCloseThrown = false;*/

			// EvObstacleMap
			const float OBSTACLE_RADIUS_GRID_SEARCH = 185.0f;
			const float obstacle_diag_radius_dist = sqrt(SQUARE(OBSTACLE_RADIUS_GRID_SEARCH)/2);



			vector<Node*>* currObstacleNodes = new vector<Node*>();
			for(unsigned int c=0; c<newSensorState->obstacleBuffer.obstacles.size(); c++)
			{
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0), newSensorState->obstacleBuffer.obstacles.at(c).at(1)));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0)+OBSTACLE_RADIUS_GRID_SEARCH, newSensorState->obstacleBuffer.obstacles.at(c).at(1)));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0)+obstacle_diag_radius_dist, newSensorState->obstacleBuffer.obstacles.at(c).at(1)+obstacle_diag_radius_dist));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0), newSensorState->obstacleBuffer.obstacles.at(c).at(1)+OBSTACLE_RADIUS_GRID_SEARCH));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0)-obstacle_diag_radius_dist, newSensorState->obstacleBuffer.obstacles.at(c).at(1)+obstacle_diag_radius_dist));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0)-OBSTACLE_RADIUS_GRID_SEARCH, newSensorState->obstacleBuffer.obstacles.at(c).at(1)));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0)-obstacle_diag_radius_dist, newSensorState->obstacleBuffer.obstacles.at(c).at(1)-obstacle_diag_radius_dist));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0), newSensorState->obstacleBuffer.obstacles.at(c).at(1)-OBSTACLE_RADIUS_GRID_SEARCH));
				this->addNodeIfNotFound(currObstacleNodes, grid->getNodeByCoord(newSensorState->obstacleBuffer.obstacles.at(c).at(0)+obstacle_diag_radius_dist, newSensorState->obstacleBuffer.obstacles.at(c).at(1)-obstacle_diag_radius_dist));
			}

			vector<Node*>* currObstacleNodesInRange = new vector<Node*>();
			const unsigned int radiusNodes = 2;
			Node* currPosNode = this->grid->getNodeByCoord(myX,myY);
			// get subset of obstacles around robot position


			float robotPhi = myPhi;
			if (robotPhi < 0) {
				robotPhi += 360;
			}

			float laserOpeningAngle = 180.0;

			for(unsigned int i = 0; i < currObstacleNodes->size(); i++){
				if (currPosNode->getX() == currObstacleNodes->at(i)->getX() && currPosNode->getY() == currObstacleNodes->at(i)->getY())
					continue;
				if(abs(currPosNode->getX() - currObstacleNodes->at(i)->getX()) <= radiusNodes &&
						abs(currPosNode->getY() - currObstacleNodes->at(i)->getY()) <= radiusNodes){

					// angle is within the range of the robotino. Now check if it is within the field of view of the laser scanner


					// get the signed angle to the x axis
					float angle = RADTODEG(atan2(currObstacleNodes->at(i)->getY()-currPosNode->getY(), currObstacleNodes->at(i)->getX()-currPosNode->getX()));
					if(angle < 0) {
						angle += 360;
					}
					if (abs(angle-robotPhi) <= laserOpeningAngle/2.0 || abs(angle-(robotPhi+360)) <= laserOpeningAngle/2.0) {
						currObstacleNodesInRange->push_back(currObstacleNodes->at(i));
					}

				}
			}

			delete currObstacleNodes;
			currObstacleNodes = NULL;

			for(unsigned int i=0; i<currObstacleNodesInRange->size(); i++) {
				if (lastObstacleNodesInRange == NULL || !(std::find(lastObstacleNodesInRange->begin(), lastObstacleNodesInRange->end(), currObstacleNodesInRange->at(i))!=lastObstacleNodesInRange->end())) {
					// current obstacle is a new one
				}
			}

			if (!isEqualNodeList(currObstacleNodesInRange, lastObstacleNodesInRange)) {
				boost::shared_ptr<EvObstacleMap> ev(new EvObstacleMap(*currObstacleNodesInRange));
				FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvObstacleMap");
				stateCtrl->getAsyncStateMachine()->queueEvent(ev);

				/*cout << "EvObstacleMap contains: " << endl;
				for(unsigned int i=0; i<currObstacleNodesInRange->size(); i++)
					cout << "- Node (x,y): " << currObstacleNodesInRange->at(i)->getX() << ", " << currObstacleNodesInRange->at(i)->getY() << endl;
				*/
			}

			delete lastObstacleNodesInRange;
			lastObstacleNodesInRange = currObstacleNodesInRange;
		}

		if(oldSensorState != NULL)
			delete oldSensorState;
		oldSensorState = newSensorState;

		if(true){
			firstRun = false;
		}

		//boost::this_thread::yield(); // give other threads a chance to run
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
}

void SensorEventGenerator::getNewSensorValues()
{
	newSensorState = new SensorEventGeneratorBuffer();
	sensorSrv->getNewSensorValues(newSensorState, oldSensorState);
}

void SensorEventGenerator::setStateBehaviorController(StateBehaviorController *stateCtrl_)
{
	stateCtrl = stateCtrl_;
}

void SensorEventGenerator::initiate()
{
	execThread = new boost::thread(&SensorEventGenerator::monitorSensors,this);
	run();
}

void SensorEventGenerator::handleLineCross()
{
	float x,y,dx,dy,phi,distance,alpha;
	long time;
	sensorSrv->getOdometry(x,y,phi);
	time = lastLineCrossTimer.msecsElapsed();

	dx = x - lastLineCrossX;
	dy = y - lastLineCrossY;
	distance = sqrt(dx*dx+dy*dy);
	// Refactor because of code duplication from below (if-case)
	if (distance == 0)
	{
		float beta = 0; // set beta to zero

		if(phi >= -45 && phi < 45) // crossed line normal in driving direction has angle 0°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(0,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(phi > 45 && phi < 135) // crossed line normal in driving direction has angle 90°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(90,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(phi >= 135 || phi <-135) // crossed line normal in driving direction has angle 180°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(180,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(phi >=-135 && phi < -45) // crossed line normal in driving direction has angle -90°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(-90,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
	}

	alpha = acos(dx/distance);
	if(dy < 0)
		alpha = -alpha; // alpha is the angle in which the robot was driving, range [-PI;+PI]
	alpha = RADTODEG(alpha);
	FileLog::log(log_SensorEventGenerator, "-------------> Driving angle by coordinates: ", FileLog::real(alpha));
	FileLog::log(log_SensorEventGenerator, "-------------> Driving angle+180 by coordinates: ", FileLog::real(alpha+180));
	FileLog::log(log_SensorEventGenerator, "-------------> Driving angle by odometry: ", FileLog::real(phi));

	if(getDistDegree(alpha, phi) < 5) // FORWARD
	{
		FileLog::log(log_SensorEventGenerator, "-------------> FULL LINE CROSS FORWARD detected: Distance: ", FileLog::real(distance), " Time: ", FileLog::integer(time));
		float ddrive, diag, dsens, beta;
		ddrive = distance/2;
		dsens = FLOOR_SENSOR_DISTANCE/2;
		diag = sqrt(ddrive*ddrive+dsens*dsens);
		beta = asin(dsens/diag);
		beta = RADTODEG(beta);
		beta = 90 - beta;

		if(lastLineCrossSensor == FloorSensor::LEFT)
			beta = -beta;

		FileLog::log(log_SensorEventGenerator, "-------------> calculated BETA: ", FileLog::real(beta));

		if(phi >= -45 && phi < 45) // crossed line normal in driving direction has angle 0°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(0,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(phi > 45 && phi < 135) // crossed line normal in driving direction has angle 90°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(90,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(phi >= 135 || phi <-135) // crossed line normal in driving direction has angle 180°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(180,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(phi >=-135 && phi < -45) // crossed line normal in driving direction has angle -90°
		{
			boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(-90,beta));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
	}
	else if(getDistDegree(alpha+180, phi) < 5)
		FileLog::log(log_SensorEventGenerator, "-------------> FULL LINE CROSS BACKWARD detected: Distance: ", FileLog::real(distance), " Time: ", FileLog::integer(time));
	else
	{
		FileLog::log(log_SensorEventGenerator, "-------------> Could not match direction to FORWARD or BACKWARD");
		boost::shared_ptr<EvAngleCalibration> ev(new EvAngleCalibration(0,0));
		FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvAngleCalibration");
		stateCtrl->getAsyncStateMachine()->queueEvent(ev);
	}
	lastLineCrossSensor = FloorSensor::INIT; // reset line cross detection
}

/* converts the API degree value (range: -180 - 180) into a positive degree value (range: 0 - 360) */
float SensorEventGenerator::getPositiveDegree(float phi) const
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
float SensorEventGenerator::getDistDegree(float phi1, float phi2) const
{
	float diff = getPositiveDegree(phi1) - getPositiveDegree(phi2);
	if(diff < 0)
		diff = -diff;

	if(diff < 180)
		return diff;
	else
		return 360-diff;
}

void SensorEventGenerator::addNodeIfNotFound(vector<Node*>* v, Node* n)
{
	if(n != NULL && std::find(v->begin(), v->end(), n) == v->end())
		v->push_back(n);
}

bool SensorEventGenerator::isEqualNodeList(vector<Node*>* v1, vector<Node*>* v2)
{
	if(v1 == NULL || v2 == NULL)
		return false;
	if(v1->size() != v2->size())
	   return false;
	std::sort(v1->begin(),v1->end());
	std::sort(v2->begin(),v2->end());
	return std::equal(v1->begin(), v1->end(), v2->begin());
}
