/*
 * DataProvider.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef DATAPROVIDER_H_
#define DATAPROVIDER_H_

#include <boost/thread.hpp>

#include "communication/messages/MsgCustomerOrder.h"
#include "communication/messages/MsgCustomerPos.h"
#include "communication/messages/MsgEnvironment.h"
#include "communication/messages/MsgRobotBeacon.h"
#include "communication/messages/MsgRobotPos.h"
#include "communication/messages/MsgRobotServed.h"

class DataProvider
{
public:
	static DataProvider* getInstance();

	// used by CloudServer to process new messages
	void processMsg(::std::stringstream& msg);

	// used by readers/users
	MsgEnvironment getLatestMsgEnvironment();
	MsgRobotPos getLatestMsgRobotPos();
	vector< MsgCustomerOrder > getMsgCustomerOrders();
	vector< MsgCustomerPos > getMsgCustomerPoses();

private:
	static DataProvider *instance;

	MsgEnvironment* msgEnvironment;
	MsgRobotPos* msgRobotPos;
	vector< MsgCustomerOrder* > vecMsgCustomerOrder;
	vector< MsgCustomerPos* > vecMsgCustomerPos;
	boost::shared_mutex mutexMsgEnvironment;
	boost::shared_mutex mutexMsgRobotPos;
	boost::shared_mutex mutexVecMsgCustomerOrder;
	boost::shared_mutex mutexVecMsgCustomerPos;

protected:
	DataProvider();
	virtual ~DataProvider();
};


#endif /* DATAPROVIDER_H_ */
