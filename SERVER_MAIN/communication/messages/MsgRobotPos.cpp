/*
 * MsgRobotPos.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgRobotPos.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgRobotPos::MsgRobotPos(unsigned long time_, double x_, double y_)
{
	this->message = MsgRobotPos::Message();
	this->time = time_;
	this->x = x_;
	this->y = y_;
}

MsgRobotPos::MsgRobotPos(boost::property_tree::ptree& pt)
{
	this->load(pt);
}

MsgRobotPos::~MsgRobotPos()
{

}

void MsgRobotPos::load(boost::property_tree::ptree& pt)
{
	message = pt.get<string>("message");
	time = pt.get<unsigned long>("time");
	x = pt.get<double>("x");
	y = pt.get<double>("y");
}

::std::string MsgRobotPos::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time", time);
	pt.put("x", x);
	pt.put("y", y);
	boost::property_tree::json_parser::write_json(jsonMsgStream, pt, false);
	return jsonMsgStream.str();
}

void MsgRobotPos::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgRobotPos:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time);
	std::cout << PARAM_PRINT(x);
	std::cout << PARAM_PRINT(y);
}
