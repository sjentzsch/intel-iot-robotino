//
// MsgCustomerPos.cpp
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
