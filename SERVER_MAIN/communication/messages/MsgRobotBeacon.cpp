/*
 * MsgRobotBeacon.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgRobotBeacon.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgRobotBeacon::MsgRobotBeacon(unsigned long time_, double x_, double y_, string state_, unsigned long drinks_available_)
{
	this->message = MsgRobotBeacon::Message();
	this->time = time_;
	this->x = x_;
	this->y = y_;
	this->state = state_;
	this->drinks_available = drinks_available_;
}

MsgRobotBeacon::MsgRobotBeacon(boost::property_tree::ptree& pt)
{
	this->load(pt);
}

MsgRobotBeacon::~MsgRobotBeacon()
{

}

void MsgRobotBeacon::load(boost::property_tree::ptree& pt)
{
	message = pt.get<string>("message");
	time = pt.get<unsigned long>("time");
	x = pt.get<double>("x");
	y = pt.get<double>("y");
	state = pt.get<string>("state");
	drinks_available = pt.get<unsigned long>("drinks_available");
}

::std::string MsgRobotBeacon::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time", time);
	pt.put("x", x);
	pt.put("y", y);
	pt.put("state", state);
	pt.put("drinks_available", drinks_available);
	boost::property_tree::json_parser::write_json(jsonMsgStream, pt, false);
	return jsonMsgStream.str();
}

void MsgRobotBeacon::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgRobotBeacon:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time);
	std::cout << PARAM_PRINT(x);
	std::cout << PARAM_PRINT(y);
	std::cout << PARAM_PRINT(state);
	std::cout << PARAM_PRINT(drinks_available);
}
