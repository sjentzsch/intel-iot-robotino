/*
 * MsgEnvironment.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgEnvironment.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgEnvironment::MsgEnvironment(unsigned long time_start_, double x_max_, double y_max_, double x_robot_, double y_robot_, double phi_robot_, double x_base_start_, double y_base_start_, double x_base_end_, double y_base_end_, double phi_base_)
{
	this->message = MsgEnvironment::Message();
	this->time_start = time_start_;
	this->x_max = x_max_;
	this->y_max = y_max_;
	this->x_robot = x_robot_;
	this->y_robot = y_robot_;
	this->phi_robot = phi_robot_;
	this->x_base_start = x_base_start_;
	this->y_base_start = y_base_start_;
	this->x_base_end = x_base_end_;
	this->y_base_end = y_base_end_;
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
	time_start = pt.get<unsigned long>("time_start");
	x_max = pt.get<double>("x_max");
	y_max = pt.get<double>("y_max");
	x_robot = pt.get<double>("x_robot");
	y_robot = pt.get<double>("y_robot");
	phi_robot = pt.get<double>("phi_robot");
	x_base_start = pt.get<double>("x_base_start");
	y_base_start = pt.get<double>("y_base_start");
	x_base_end = pt.get<double>("x_base_end");
	y_base_end = pt.get<double>("y_base_end");
	phi_base = pt.get<double>("phi_base");
}

::std::string MsgEnvironment::save()
{
	::std::stringstream jsonMsgStream;
	boost::property_tree::ptree pt;
	pt.put("message", message);
	pt.put("time_start", time_start);
	pt.put("x_max", x_max);
	pt.put("y_max", y_max);
	pt.put("x_robot", x_robot);
	pt.put("y_robot", y_robot);
	pt.put("phi_robot", phi_robot);
	pt.put("x_base_start", x_base_start);
	pt.put("y_base_start", y_base_start);
	pt.put("x_base_end", x_base_end);
	pt.put("y_base_end", y_base_end);
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
	std::cout << PARAM_PRINT(time_start);
	std::cout << PARAM_PRINT(x_max);
	std::cout << PARAM_PRINT(y_max);
	std::cout << PARAM_PRINT(x_robot);
	std::cout << PARAM_PRINT(y_robot);
	std::cout << PARAM_PRINT(phi_robot);
	std::cout << PARAM_PRINT(x_base_start);
	std::cout << PARAM_PRINT(y_base_start);
	std::cout << PARAM_PRINT(x_base_end);
	std::cout << PARAM_PRINT(y_base_end);
	std::cout << PARAM_PRINT(phi_base);
}
