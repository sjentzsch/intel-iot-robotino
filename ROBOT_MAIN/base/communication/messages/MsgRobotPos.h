/*
 * MsgRobotPos.h
 *
 *  Created on: Jun 11, 2014
 *      Author: root
 */

#ifndef MSGROBOTPOS_H_
#define MSGROBOTPOS_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgRobotPos
{
public:
	MsgRobotPos(unsigned long time_, double x_, double y_);
	MsgRobotPos(boost::property_tree::ptree& pt);
	virtual ~MsgRobotPos();

	static ::std::string Message() {return "msg_robot_pos";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time;
	double x;
	double y;
};

#endif /* MSGROBOTPOS_H_ */
