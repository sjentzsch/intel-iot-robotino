/*
 * MsgCustomerPos.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgCustomerPos.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgCustomerPos::MsgCustomerPos(unsigned long time_, unsigned long customer_id_, string name_, double x_, double y_)
{
	this->message = MsgCustomerPos::Message();
	this->time = time_;
	this->customer_id = customer_id_;
	this->name = name_;
	this->x = x_;
	this->y = y_;
}

MsgCustomerPos::MsgCustomerPos(boost::property_tree::ptree& pt)
{
	this->load(pt);
}

MsgCustomerPos::~MsgCustomerPos()
{

}

void MsgCustomerPos::load(boost::property_tree::ptree& pt)
{
	message = pt.get<string>("message");
	time = pt.get<unsigned long>("time");
	customer_id = pt.get<unsigned long>("customer_id");
	name = pt.get<string>("name");
	x = pt.get<double>("x");
	y = pt.get<double>("y");
}

::std::string MsgCustomerPos::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time", time);
	pt.put("customer_id", customer_id);
	pt.put("name", name);
	pt.put("x", x);
	pt.put("y", y);
	boost::property_tree::json_parser::write_json(jsonMsgStream, pt, false);
	return jsonMsgStream.str();
}

void MsgCustomerPos::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgCustomerPos:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time);
	std::cout << PARAM_PRINT(customer_id);
	std::cout << PARAM_PRINT(name);
	std::cout << PARAM_PRINT(x);
	std::cout << PARAM_PRINT(y);
}
