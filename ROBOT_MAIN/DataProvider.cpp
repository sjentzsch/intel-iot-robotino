//
// DataProvider.cpp
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

#include "DataProvider.h"

#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

DataProvider *DataProvider::instance = NULL;

DataProvider::DataProvider()
{
	this->msgEnvironment = NULL;
	this->msgRobotPos = NULL;
	this->dirBase = Direction::EAST;
}

DataProvider::~DataProvider()
{
	delete this->msgEnvironment;
	delete this->msgRobotPos;
	for(unsigned int i=0; i<this->vecMsgCustomerOrder.size(); i++)
		delete this->vecMsgCustomerOrder.at(i);
	this->vecMsgCustomerOrder.clear();
	for(unsigned int i=0; i<this->vecMsgCustomerPos.size(); i++)
		delete this->vecMsgCustomerPos.at(i);
	this->vecMsgCustomerPos.clear();
}

DataProvider* DataProvider::getInstance()
{
	if(instance==NULL)
		instance = new DataProvider();
	return instance;
}

void DataProvider::processMsg(::std::stringstream& msg)
{
	boost::property_tree::ptree pt;
	string message = "";

	try
	{
		boost::property_tree::json_parser::read_json(msg, pt);
		message = pt.get<string>("message");
	}
	catch(boost::property_tree::json_parser_error &exc)
	{
		std::cerr << "[DataProvider]: Could not parse the file and find the message type: " << exc.what() << std::endl;
	}

	::std::cout << "[DataProvider] Received message: " << message << ::std::endl;

	if(message == MsgEnvironment::Message())
	{
		boost::unique_lock< boost::shared_mutex > lock(mutexMsgEnvironment);
		if(this->msgEnvironment == NULL)
			this->msgEnvironment = new MsgEnvironment(pt);
		else
			this->msgEnvironment->load(pt);
		if(msgEnvironment->phi_base > -1 && msgEnvironment->phi_base < 1)
			dirBase = Direction::DIRECTION::SOUTH;
		else if(msgEnvironment->phi_base > 89 && msgEnvironment->phi_base < 91)
			dirBase = Direction::DIRECTION::EAST;
		else if(msgEnvironment->phi_base > 179 && msgEnvironment->phi_base < 181)
			dirBase = Direction::DIRECTION::NORTH;
		else if(msgEnvironment->phi_base > 269 && msgEnvironment->phi_base < 271)
			dirBase = Direction::DIRECTION::WEST;
		else
			::std::cout << "ERROR: environment phi_base could not be matched to direction! expected either 0, 90, 180, 270." << ::std::endl;

		::std::cout << "[DataProvider] Updated message: " << message << ::std::endl;
	}
	else if(message == MsgRobotPos::Message())
	{
		boost::unique_lock< boost::shared_mutex > lock(mutexMsgRobotPos);
		if(this->msgRobotPos == NULL)
			this->msgRobotPos = new MsgRobotPos(pt);
		else
			this->msgRobotPos->load(pt);
		::std::cout << "[DataProvider] Updated message: " << message << ::std::endl;
	}
	else if(message == MsgCustomerOrder::Message())
	{
		MsgCustomerOrder* newMsgCustomerOrder = new MsgCustomerOrder(pt);
		boost::unique_lock< boost::shared_mutex > lock(mutexVecMsgCustomerOrder);
		this->vecMsgCustomerOrder.push_back(newMsgCustomerOrder);
		::std::cout << "[DataProvider] Updated message: " << message << ::std::endl;
	}
	else if(message == MsgCustomerPos::Message())
	{
		MsgCustomerPos* newMsgCustomerPos = new MsgCustomerPos(pt);
		boost::unique_lock< boost::shared_mutex > lock(mutexVecMsgCustomerPos);
		bool alreadyContained = false;
		for(unsigned int i=0; i<vecMsgCustomerPos.size(); ++i)
		{
			if(vecMsgCustomerPos.at(i)->customer_id == newMsgCustomerPos->customer_id)
			{
				vecMsgCustomerPos.at(i)->load(pt);
				alreadyContained = true;
				break;
			}
		}
		if(!alreadyContained)
			this->vecMsgCustomerPos.push_back(newMsgCustomerPos);
	}
	else
	{
		::std::cout << "[DataProvider] Could not update message due to unknown type: " << message << ::std::endl;
	}
}

bool DataProvider::isValidMsgEnvironment()
{
	return this->msgEnvironment != NULL;
}

bool DataProvider::isValidMsgRobotPos()
{
	return this->msgRobotPos != NULL;
}

MsgEnvironment DataProvider::getLatestMsgEnvironment()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMsgEnvironment);
	return *this->msgEnvironment;
}

Direction::DIRECTION DataProvider::getLatestBaseDir()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMsgEnvironment);
	return this->dirBase;
}

MsgRobotPos DataProvider::getLatestMsgRobotPos()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMsgRobotPos);
	return *this->msgRobotPos;
}

vector< MsgCustomerOrder > DataProvider::getMsgCustomerOrders()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexVecMsgCustomerOrder);
	vector< MsgCustomerOrder > tempVecMsg;
	for(unsigned int i=0; i<vecMsgCustomerOrder.size(); ++i)
		tempVecMsg.push_back(*vecMsgCustomerOrder.at(i));
	return tempVecMsg;
}

vector< MsgCustomerPos > DataProvider::getMsgCustomerPoses()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexVecMsgCustomerPos);
	vector< MsgCustomerPos > tempVecMsg;
	for(unsigned int i=0; i<vecMsgCustomerPos.size(); ++i)
		tempVecMsg.push_back(*vecMsgCustomerPos.at(i));
	return tempVecMsg;
}
