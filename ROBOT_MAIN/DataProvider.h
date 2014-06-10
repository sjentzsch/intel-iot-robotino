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

	// following block should be used from the readers/users
	MsgEnvironment getLatestMsgEnvironment();

private:
	static DataProvider *instance;

	MsgEnvironment msgEnvironment;
	boost::shared_mutex mutexMsgEnvironment;

protected:
	DataProvider();
	virtual ~DataProvider();
};


#endif /* DATAPROVIDER_H_ */
