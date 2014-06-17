/*
 * TaskManager.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: root
 */

#include "TaskManager.h"

TaskManager::TaskManager(StateBehaviorController* stateBhvContrl,SensorServer *sensorSrv_):asyncStateMachine(stateBhvContrl->getAsyncStateMachine())
{
	drinks_available = DRINKS_AVAILABLE_START;
	currCustomerOrder = NULL;
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
	FileLog::log_NOTICE("[TaskManager] nextTask started");

	if(this->drinks_available < 1)
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

void TaskManager::serveDrink()
{
	if(this->currCustomerOrder != NULL)
	{
		this->drinks_available--;
		this->vecMsgRobotServed.push_back(MsgRobotServed(42, this->currCustomerOrder->order_id));

		delete this->currCustomerOrder;
		this->currCustomerOrder = NULL;
	}
}


void TaskManager::refillDrinks()
{
	this->drinks_available = DRINKS_AVAILABLE_START;
}

MsgCustomerOrder TaskManager::getCurrCustomerOrder()
{
	return *this->currCustomerOrder;
}
