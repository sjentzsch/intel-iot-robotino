/*
 * MsgCustomerPos.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef MSGCUSTOMERPOS_H_
#define MSGCUSTOMERPOS_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgCustomerPos
{
public:
	MsgCustomerPos(unsigned long time_, unsigned long customer_id_, string name_, double x_, double y_);
	MsgCustomerPos(boost::property_tree::ptree& pt);
	virtual ~MsgCustomerPos();

	static ::std::string Message() {return "msg_customer_pos";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time;
	unsigned long customer_id;
	string name;
	double x;
	double y;
};

#endif /* MSGCUSTOMERPOS_H_ */
