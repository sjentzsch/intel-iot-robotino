/*
 * MsgRobotServed.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgRobotServed.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgRobotServed::MsgRobotServed(unsigned long time_, unsigned long order_id_)
{
	this->message = MsgRobotServed::Message();
	this->time = time_;
	this->order_id = order_id_;
}

MsgRobotServed::MsgRobotServed(boost::property_tree::ptree& pt)
{
	this->load(pt);
}

MsgRobotServed::~MsgRobotServed()
{

}

void MsgRobotServed::load(boost::property_tree::ptree& pt)
{
	message = pt.get<string>("message");
	time = pt.get<unsigned long>("time");
	order_id = pt.get<unsigned long>("order_id");
}

::std::string MsgRobotServed::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time", time);
	pt.put("order_id", order_id);
	boost::property_tree::json_parser::write_json(jsonMsgStream, pt, false);
	return jsonMsgStream.str();
}

void MsgRobotServed::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgRobotServed:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time);
	std::cout << PARAM_PRINT(order_id);
}
