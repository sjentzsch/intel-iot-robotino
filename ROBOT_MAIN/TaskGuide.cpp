/*
 * TaskGuide.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#include "TaskGuide.h"

TaskGuide::TaskGuide(StateBehaviorController* stateBhvContrl,SensorServer *sensorSrv_):asyncStateMachine(stateBhvContrl->getAsyncStateMachine())
{
	nextTask_exec = NULL;
}

TaskGuide::~TaskGuide()
{
}

void TaskGuide::nextTask()
{
	while(true){
		try{
			if(nextTask_exec !=NULL){
				nextTask_exec->join();
			}

			nextTask_exec = new boost::thread(&TaskGuide::nextTask_impl,this);

			break;
		}catch (std::exception& e) {
		   std::cerr << "[TaskGuide] Caught exception: " << e.what() << std::endl;
		}

	}
}


void TaskGuide::nextTask_impl()
{
	return;
}

void TaskGuide::handleJob()
{
	FileLog::log(log_Job,"[TaskGuide] handleJob Status STARTING");
	// trigger blablablub
}

/*void TaskGuide::triggerEventDeliverPuckToGate(Node* accessNode){
	FileLog::log(log_Job,"[TaskGuide] EvDeliverPuckToGate");
	boost::shared_ptr<EvDeliverPuckToGate> ev(new EvDeliverPuckToGate(accessNode));
	asyncStateMachine->queueEvent(ev);
}*/
