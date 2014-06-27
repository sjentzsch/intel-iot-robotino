/*
 * MsgRobotBeacon.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef MSGROBOTBEACON_H_
#define MSGROBOTBEACON_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgRobotBeacon
{
public:
	MsgRobotBeacon(unsigned long time_, double x_, double y_, string state_, unsigned long drinks_available_);
	MsgRobotBeacon(boost::property_tree::ptree& pt);
	virtual ~MsgRobotBeacon();

	static ::std::string Message() {return "msg_robot_beacon";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time;
	double x;
	double y;
	string state;
	unsigned long drinks_available;
};

#endif /* MSGROBOTBEACON_H_ */
