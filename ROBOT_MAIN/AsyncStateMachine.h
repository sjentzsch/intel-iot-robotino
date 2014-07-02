//
// AsyncStateMachine.h
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
