//
// MsgEnvironment.cpp
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

#include "MsgEnvironment.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgEnvironment::MsgEnvironment(unsigned long time_, double x_max_, double y_max_, double x_base_robot_start_, double y_base_robot_start_, double x_base_left_corner_, double y_base_left_corner_, double phi_base_)
{
	this->message = MsgEnvironment::Message();
	this->time = time_;
	this->x_max = x_max_;
	this->y_max = y_max_;
	this->x_base_robot_start = x_base_robot_start_;
	this->y_base_robot_start = y_base_robot_start_;
	this->x_base_left_corner = x_base_left_corner_;
	this->y_base_left_corner = y_base_left_corner_;
	this->phi_base = phi_base_;
}

MsgEnvironment::MsgEnvironment(boost::property_tree::ptree& pt)
{
	this->load(pt);
}

MsgEnvironment::~MsgEnvironment()
{

}

void MsgEnvironment::load(boost::property_tree::ptree& pt)
{
	message = pt.get<string>("message");
	time = pt.get<unsigned long>("time");
	x_max = pt.get<double>("x_max");
	y_max = pt.get<double>("y_max");
	x_base_robot_start = pt.get<double>("x_base_robot_start");
	y_base_robot_start = pt.get<double>("y_base_robot_start");
	x_base_left_corner = pt.get<double>("x_base_left_corner");
	y_base_left_corner = pt.get<double>("y_base_left_corner");
	phi_base = pt.get<double>("phi_base");
}

::std::string MsgEnvironment::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time", time);
	pt.put("x_max", x_max);
	pt.put("y_max", y_max);
	pt.put("x_base_robot_start", x_base_robot_start);
	pt.put("y_base_robot_start", y_base_robot_start);
	pt.put("x_base_left_corner", x_base_left_corner);
	pt.put("y_base_left_corner", y_base_left_corner);
	pt.put("phi_base", phi_base);
	boost::property_tree::json_parser::write_json(jsonMsgStream, pt, false);
	return jsonMsgStream.str();
}

void MsgEnvironment::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgEnvironment:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time);
	std::cout << PARAM_PRINT(x_max);
	std::cout << PARAM_PRINT(y_max);
	std::cout << PARAM_PRINT(x_base_robot_start);
	std::cout << PARAM_PRINT(y_base_robot_start);
	std::cout << PARAM_PRINT(x_base_left_corner);
	std::cout << PARAM_PRINT(y_base_left_corner);
	std::cout << PARAM_PRINT(phi_base);
}
