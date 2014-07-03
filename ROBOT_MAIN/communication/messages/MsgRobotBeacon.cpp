//
// MsgRobotBeacon.cpp
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

#include "MsgRobotBeacon.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgRobotBeacon::MsgRobotBeacon(unsigned long time_, double x_, double y_, double phi_, string state_, bool running_, unsigned long drinks_available_)
{
	this->message = MsgRobotBeacon::Message();
	this->time = time_;
	this->x = x_;
	this->y = y_;
	this->phi = phi_;
	this->state = state_;
	this->running = running_;
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
	phi = pt.get<double>("phi");
	state = pt.get<string>("state");
	running = pt.get<bool>("running");
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
	pt.put("phi", phi);
	pt.put("state", state);
	pt.put("running", running);
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
	std::cout << PARAM_PRINT(phi);
	std::cout << PARAM_PRINT(state);
	std::cout << PARAM_PRINT(running);
	std::cout << PARAM_PRINT(drinks_available);
}
