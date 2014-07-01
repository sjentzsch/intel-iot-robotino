/*
 * MsgCustomerOrder.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgCustomerOrder.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgCustomerOrder::MsgCustomerOrder(unsigned long time_, unsigned long order_id_, unsigned long customer_id_)
{
	this->message = MsgCustomerOrder::Message();
	this->time = time_;
	this->order_id = order_id_;
	this->customer_id = customer_id_;
}

MsgCustomerOrder::MsgCustomerOrder(boost::property_tree::ptree& pt)
{
	this->load(pt);
}

MsgCustomerOrder::~MsgCustomerOrder()
{

}

void MsgCustomerOrder::load(boost::property_tree::ptree& pt)
{
	message = pt.get<string>("message");
	time = pt.get<unsigned long>("time");
	order_id = pt.get<unsigned long>("order_id");
	customer_id = pt.get<unsigned long>("customer_id");
}

::std::string MsgCustomerOrder::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time", time);
	pt.put("order_id", order_id);
	pt.put("customer_id", customer_id);
	boost::property_tree::json_parser::write_json(jsonMsgStream, pt, false);
	return jsonMsgStream.str();
}

void MsgCustomerOrder::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgCustomerOrder:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time);
	std::cout << PARAM_PRINT(order_id);
	std::cout << PARAM_PRINT(customer_id);
}
