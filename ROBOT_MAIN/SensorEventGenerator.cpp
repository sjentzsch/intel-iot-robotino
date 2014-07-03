//
// SensorEventGenerator.cpp
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

#include "config.h"
#include "SensorEventGenerator.h"
#include "SensorServer.h"
#include "StateBehaviorController.h"

const float SensorEventGenerator::FLOOR_SENSOR_DISTANCE(92.5);

SensorEventGenerator::SensorEventGenerator(SensorServer *sensorSrv_):sensorSrv(sensorSrv_)
{
	evObstacleIsCloseThrown = false;

	oldSensorState = NULL;
	getNewSensorValues();
	oldSensorState = newSensorState;

	lastLineCrossPhi = 0;
	lastLineCrossX = 0;
	lastLineCrossY = 0;
	lastLineCrossSensor = FloorSensor::INIT;

	pause();
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
		// GOT PAUSE SIGNAL? Wait!
		while(!DataProvider::getInstance()->isRunning())
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			continue;
		}

		//Make FPS calculation
//		dtToLastImageInMs = (int)timer.msecsElapsed();
//		if(dtToLastImageInMs != 0) fps = 1000 / dtToLastImageInMs;
//		timer.reset();
//		timer.start();
//		if(coutCounter % 100 == 0)
//			FileLog::log_DEBUG("[SensorEventGenerator] Current FPS: ", FileLog::integer(fps));

		checkSignalStatus(); // blocks if thread is signaled to pause

		getNewSensorValues();

		/*cout << "**********************" << endl;
		cout << "Analog Value Drink 1: " << sensorSrv->valueDrink1() << endl;
		cout << "Analog Value Drink 2: " << sensorSrv->valueDrink2() << endl;
		cout << "Analog Value Drink 3: " << sensorSrv->valueDrink3() << endl;
		cout << "**********************" << endl;*/

		// force sensors (EvSensorDrinkTaken and EvSensorDrinkRefilled)
		// sensor for drink 1
		if(oldSensorState->sensorHasDrink1 && !newSensorState->sensorHasDrink1)
		{
			boost::shared_ptr<EvSensorDrinkTaken> ev(new EvSensorDrinkTaken(1));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDrinkTaken 1");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(!oldSensorState->sensorHasDrink1 && newSensorState->sensorHasDrink1)
		{
			boost::shared_ptr<EvSensorDrinkRefilled> ev(new EvSensorDrinkRefilled(1));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDrinkRefilled 1");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		// sensor for drink 2
		if(oldSensorState->sensorHasDrink2 && !newSensorState->sensorHasDrink2)
		{
			boost::shared_ptr<EvSensorDrinkTaken> ev(new EvSensorDrinkTaken(2));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDrinkTaken 2");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(!oldSensorState->sensorHasDrink2 && newSensorState->sensorHasDrink2)
		{
			boost::shared_ptr<EvSensorDrinkRefilled> ev(new EvSensorDrinkRefilled(2));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDrinkRefilled 2");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		// sensor for drink 3
		if(oldSensorState->sensorHasDrink3 && !newSensorState->sensorHasDrink3)
		{
			boost::shared_ptr<EvSensorDrinkTaken> ev(new EvSensorDrinkTaken(3));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDrinkTaken 3");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		else if(!oldSensorState->sensorHasDrink3 && newSensorState->sensorHasDrink3)
		{
			boost::shared_ptr<EvSensorDrinkRefilled> ev(new EvSensorDrinkRefilled(3));
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorDrinkRefilled 3");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}
		// combined force sensor events
		if((!oldSensorState->sensorHasDrink1 || !oldSensorState->sensorHasDrink2 || !oldSensorState->sensorHasDrink3) && (newSensorState->sensorHasDrink1 && newSensorState->sensorHasDrink2 && newSensorState->sensorHasDrink3))
		{
			boost::shared_ptr<EvSensorAllDrinksRefilled> ev(new EvSensorAllDrinksRefilled());
			FileLog::log(log_SensorEventGenerator, "[SensorEventGenerator] EvSensorAllDrinksRefilled");
			stateCtrl->getAsyncStateMachine()->queueEvent(ev);
		}

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
