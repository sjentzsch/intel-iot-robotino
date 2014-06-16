/*
 * TaskManager.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#include "TaskManager.h"

TaskManager::TaskManager(StateBehaviorController* stateBhvContrl,SensorServer *sensorSrv_):asyncStateMachine(stateBhvContrl->getAsyncStateMachine())
{
	nextTask_exec = NULL;
}

TaskManager::~TaskManager()
{
}

void TaskManager::nextTask()
{
	while(true)
	{
		try {
			if(nextTask_exec != NULL)
				nextTask_exec->join();

			nextTask_exec = new boost::thread(&TaskManager::nextTask_impl,this);

			break;
		} catch (std::exception& e) {
		   std::cerr << "[TaskManager] Caught exception: " << e.what() << std::endl;
		}

	}
}


void TaskManager::nextTask_impl()
{
	FileLog::log(log_Job,"[TaskManager] nextTask started");

	return;
}

void TaskManager::handleJob()
{

	// trigger blablablub
}

/*void TaskManager::triggerEventDeliverPuckToGate(Node* accessNode){
	FileLog::log(log_Job,"[TaskManager] EvDeliverPuckToGate");
	boost::shared_ptr<EvDeliverPuckToGate> ev(new EvDeliverPuckToGate(accessNode));
	asyncStateMachine->queueEvent(ev);
}*/
