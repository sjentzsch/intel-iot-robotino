/*
 * DataProvider.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "DataProvider.h"

#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

DataProvider *DataProvider::instance = NULL;

DataProvider::DataProvider()
{
	this->msgEnvironment = NULL;
	this->msgRobotPos = NULL;
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
