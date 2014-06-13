/*
 * AsyncStateMachine.cpp
 *
 *  Created on: 27.05.2011
 *      Author: root
 */

#include "AsyncStateMachine.h"
//#include "StateMachine_CameraTest.h"
#include "statemachine/StateMachine.h"
//#include "tests/StateMachine_POIMoveTest.h"
//#include "tests/StateMachine_OdometryTest.h"
//#include "tests/StateMachine_SensorTest.h"
//#include "tests/StateMachine_OdometryConstants.h"
//#include "tests/StateMachine_PathfinderTest.h"
//#include "tests/StateMachine_RegainPuck.h"
//#include "tests/StateMachine_DriveToPuck.h"

AsyncStateMachine::AsyncStateMachine(StateBehaviorController* stateBehavCtrl):sema_queue_stored(0){
	//sema_queue_stored = boost::interprocess::interprocess_semaphore(0);
	stateMachine = new StateMachine1(stateBehavCtrl);
}

AsyncStateMachine::~AsyncStateMachine() {
}

void AsyncStateMachine::initiate()
{
	stateMachine->initiate(); // start stateMachine into initial state
	queueProcessor = new boost::thread(&AsyncStateMachine::processEventQueue,this); // start processing the queue
}

void AsyncStateMachine::queueEvent(boost::shared_ptr<boost::statechart::event_base> event)
{
	queue_mutex.lock();
	eventQueue.push(event);
	queue_mutex.unlock();
	sema_queue_stored.post();
}

void AsyncStateMachine::processEventQueue()
{
	boost::shared_ptr<boost::statechart::event_base> current_event;
	while(true)
	{
		sema_queue_stored.wait(); // wait for an event in queue
		queue_mutex.lock(); // lock queue to pop event
		current_event = eventQueue.front();
		eventQueue.pop();
		queue_mutex.unlock();

		// process Event
		FileLog::log(log_SensorEventGenerator, "[AsyncStateMachine]: Started processing ", typeid(*current_event).name(), ", Queue size:", FileLog::integer(eventQueue.size()));
		stateMachine->process_event(*current_event);
		FileLog::log(log_SensorEventGenerator, "[AsyncStateMachine]: Finished processing ", typeid(*current_event).name());
	}
}
