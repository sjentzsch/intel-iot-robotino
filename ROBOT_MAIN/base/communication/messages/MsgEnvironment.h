/*
 * MsgEnvironment.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef MSGENVIRONMENT_H_
#define MSGENVIRONMENT_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgEnvironment
{
public:
	MsgEnvironment(unsigned long time_start_, double x_max_, double y_max_, double x_robot_, double y_robot_, double phi_robot_, double x_base_, double y_base_, double phi_base_);
	MsgEnvironment(boost::property_tree::ptree& pt);
	virtual ~MsgEnvironment();

	static ::std::string Message() {return "msg_environment";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time_start;
	double x_max;
	double y_max;
	double x_robot;
	double y_robot;
	double phi_robot;
	double x_base;
	double y_base;
	double phi_base;
};

#endif /* MSGENVIRONMENT_H_ */
