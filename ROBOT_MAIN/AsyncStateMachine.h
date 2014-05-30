/*
 * AsyncStateMachine.h
 *
 *  Created on: 27.05.2011
 *      Author: root
 */

#ifndef ASYNCSTATEMACHINE_H_
#define ASYNCSTATEMACHINE_H_
#include <iostream>
#include <queue>
#include <typeinfo>
#include <boost/thread.hpp>
#include <boost/statechart/event.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include "utils/FileLogger.h"

using namespace std;

//Forward declarations
class StateMachine1;
class StateBehaviorController;


class AsyncStateMachine {
public:
	AsyncStateMachine(StateBehaviorController* stateBehavCtrl);
	virtual ~AsyncStateMachine();

	void queueEvent(boost::shared_ptr<boost::statechart::event_base> event);
	void initiate();
private:
	std::queue<boost::shared_ptr<boost::statechart::event_base> > eventQueue; //Event queue
	boost::mutex queue_mutex;
	boost::interprocess::interprocess_semaphore sema_queue_stored; // semaphore counting the available events in the queue

	StateMachine1* stateMachine;
	boost::thread* queueProcessor;
	void processEventQueue();
};

#endif /* ASYNCSTATEMACHINE_H_ */
