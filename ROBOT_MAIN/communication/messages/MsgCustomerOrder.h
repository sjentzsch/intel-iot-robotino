/*
 * MsgCustomerOrder.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef MSGCUSTOMERORDER_H_
#define MSGCUSTOMERORDER_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgCustomerOrder
{
public:
	MsgCustomerOrder(unsigned long time_, unsigned long order_id_, unsigned long customer_id_);
	MsgCustomerOrder(boost::property_tree::ptree& pt);
	virtual ~MsgCustomerOrder();

	static ::std::string Message() {return "msg_customer_order";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time;
	unsigned long order_id;
	unsigned long customer_id;
};

#endif /* MSGCUSTOMERORDER_H_ */
