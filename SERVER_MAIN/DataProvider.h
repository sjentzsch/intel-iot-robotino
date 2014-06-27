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
	bool isValidMsgRobotBeacon();
	MsgRobotBeacon getLatestMsgRobotBeacon();
	vector< MsgRobotServed > getMsgRobotServed();

private:
	static DataProvider *instance;

	MsgRobotBeacon* msgRobotBeacon;
	vector< MsgRobotServed* > vecMsgRobotServed;
	boost::shared_mutex mutexMsgRobotBeacon;
	boost::shared_mutex mutexVecMsgRobotServed;

protected:
	DataProvider();
	virtual ~DataProvider();
};


#endif /* DATAPROVIDER_H_ */
