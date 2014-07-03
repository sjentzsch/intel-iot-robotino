//
// TaskManager.cpp
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

#include "TaskManager.h"
#include "communication/CloudComm.h"

TaskManager::TaskManager(StateBehaviorController* stateBhvContrl,SensorServer *sensorSrv_):asyncStateMachine(stateBhvContrl->getAsyncStateMachine()),sensorServer(sensorSrv_)
{
	FileLog::log_NOTICE("[TaskManager] Initial Drinks available: ", std::to_string(this->sensorServer->numDrinks()));
	currState = "NoStateYet";
	currCustomerOrder = NULL;
	nextTask_exec = NULL;
	sendBeacon_exec = NULL;
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

void TaskManager::startSendBeacon()
{
	sendBeacon_exec = new boost::thread(&TaskManager::sendBeacon_impl,this);
}

void TaskManager::nextTask_impl()
{
	FileLog::log_NOTICE("[TaskManager] nextTask started");

	if(this->sensorServer->numDrinks() < 1)
	{
		FileLog::log_NOTICE("[TaskManager] EvRefillDrinks");
		boost::shared_ptr<EvRefillDrinks> ev(new EvRefillDrinks());
		asyncStateMachine->queueEvent(ev);
	}
	else
	{
		MsgCustomerOrder* newCustomerOrders = NULL;

		vector< MsgCustomerOrder > vecMsgCustomerOrders = DataProvider::getInstance()->getMsgCustomerOrders();
		vector< MsgCustomerPos> vecMsgCustomerPoses = DataProvider::getInstance()->getMsgCustomerPoses();

		// Search for new (world) order ;)
		for(unsigned int i=0; i<vecMsgCustomerOrders.size(); i++)
		{
			// 1) Order should not be finished yet
			bool isAlreadyDone = false;
			for(unsigned int u=0; u<vecMsgRobotServed.size(); u++)
			{
				if(vecMsgCustomerOrders.at(i).order_id == vecMsgRobotServed.at(u).order_id)
				{
					isAlreadyDone = true;
					break;
				}
			}

			if(!isAlreadyDone)
			{
				// 2) Customer should be tracked, i.e. pose should exist
				for(unsigned int u=0; u<vecMsgCustomerPoses.size(); u++)
				{
					if(vecMsgCustomerOrders.at(i).customer_id == vecMsgCustomerPoses.at(u).customer_id)
					{
						// 3) Order is the newest
						if(newCustomerOrders == NULL)
							newCustomerOrders = new MsgCustomerOrder(vecMsgCustomerOrders.at(i));
						else if(vecMsgCustomerOrders.at(i).time < newCustomerOrders->time)
						{
							delete newCustomerOrders;
							newCustomerOrders = new MsgCustomerOrder(vecMsgCustomerOrders.at(i));
						}
						break;
					}
				}
			}
		}

		if(newCustomerOrders != NULL)
		{
			if(currCustomerOrder != NULL)
				delete currCustomerOrder;
			currCustomerOrder = newCustomerOrders;
			currCustomerOrder->print();

			FileLog::log_NOTICE("[TaskManager] EvServeCustomer");
			boost::shared_ptr<EvServeCustomer> ev(new EvServeCustomer());
			asyncStateMachine->queueEvent(ev);
		}
		else
		{
			FileLog::log_NOTICE("[TaskManager] EvRandomMovement");
			boost::shared_ptr<EvRandomMovement> ev(new EvRandomMovement());
			asyncStateMachine->queueEvent(ev);
		}
	}

	return;
}

void TaskManager::sendBeacon_impl()
{
	while(true)
	{
		float x, y, phi;
		this->sensorServer->getOdometry(x, y, phi);
		MsgRobotBeacon msgRobotBeacon((unsigned long)std::time(0), x, y, phi, this->currState, DataProvider::getInstance()->isRunning(), this->sensorServer->numDrinks());
		CloudComm::getInstance()->getCloudClient()->send(msgRobotBeacon.save());

		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}
}

void TaskManager::serveDrink()
{
	if(this->currCustomerOrder != NULL)
	{
		MsgRobotServed msgRobotServed((unsigned long)std::time(0), this->currCustomerOrder->order_id);
		CloudComm::getInstance()->getCloudClient()->send(msgRobotServed.save());
		this->vecMsgRobotServed.push_back(msgRobotServed);

		delete this->currCustomerOrder;
		this->currCustomerOrder = NULL;
	}
}

MsgCustomerOrder TaskManager::getCurrCustomerOrder()
{
	return *this->currCustomerOrder;
}

void TaskManager::setCurrState(std::string newState)
{
	this->currState = newState;
}
