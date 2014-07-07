//
// AsyncStateMachine.cpp
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

#include "AsyncStateMachine.h"
//#include "statemachine/StateMachine.h"
#include "statemachine/StateMachine_Test.h"

AsyncStateMachine::AsyncStateMachine(StateBehaviorController* stateBehavCtrl):sema_queue_stored(0){
	//sema_queue_stored = boost::interprocess::interprocess_semaphore(0);
	stateMachine = new StateMachine1(stateBehavCtrl);
	queueProcessor = NULL;
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
