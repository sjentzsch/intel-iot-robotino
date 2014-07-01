/*
 * MsgRobotServed.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef MSGROBOTSERVED_H_
#define MSGROBOTSERVED_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgRobotServed
{
public:
	MsgRobotServed(unsigned long time_, unsigned long order_id_);
	MsgRobotServed(boost::property_tree::ptree& pt);
	virtual ~MsgRobotServed();

	static ::std::string Message() {return "msg_robot_served";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time;
	unsigned long order_id;
};

#endif /* MSGROBOTSERVED_H_ */
