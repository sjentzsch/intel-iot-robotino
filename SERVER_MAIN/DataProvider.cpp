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
	this->msgRobotBeacon = NULL;
}

DataProvider::~DataProvider()
{
	delete this->msgRobotBeacon;
	for(unsigned int i=0; i<this->vecMsgRobotServed.size(); i++)
		delete this->vecMsgRobotServed.at(i);
	this->vecMsgRobotServed.clear();
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

	//::std::cout << "[DataProvider] Received message: " << message << ::std::endl;

	if(message == MsgRobotBeacon::Message())
	{
		boost::unique_lock< boost::shared_mutex > lock(mutexMsgRobotBeacon);
		if(this->msgRobotBeacon == NULL)
			this->msgRobotBeacon = new MsgRobotBeacon(pt);
		else
			this->msgRobotBeacon->load(pt);

		//::std::cout << "[DataProvider] Updated message: " << message << ::std::endl;
	}
	else if(message == MsgRobotServed::Message())
	{
		MsgRobotServed* newMsgRobotServed = new MsgRobotServed(pt);
		boost::unique_lock< boost::shared_mutex > lock(mutexVecMsgRobotServed);
		this->vecMsgRobotServed.push_back(newMsgRobotServed);
		//::std::cout << "[DataProvider] Updated message: " << message << ::std::endl;
	}
	else
	{
		::std::cout << "[DataProvider] Could not update message due to unknown type: " << message << ::std::endl;
	}
}

bool DataProvider::isValidMsgRobotBeacon()
{
	return this->msgRobotBeacon != NULL;
}

MsgRobotBeacon DataProvider::getLatestMsgRobotBeacon()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMsgRobotBeacon);
	return *this->msgRobotBeacon;
}

vector< MsgRobotServed > DataProvider::getMsgRobotServed()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexVecMsgRobotServed);
	vector< MsgRobotServed > tempVecMsg;
	for(unsigned int i=0; i<vecMsgRobotServed.size(); ++i)
		tempVecMsg.push_back(*vecMsgRobotServed.at(i));
	return tempVecMsg;
}
