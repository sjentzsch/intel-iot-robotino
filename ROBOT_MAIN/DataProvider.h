//
// DataProvider.h
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Sören Jentzsch
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DATAPROVIDER_H_
#define DATAPROVIDER_H_

#include <boost/thread.hpp>

#include "communication/messages/MsgCustomerOrder.h"
#include "communication/messages/MsgCustomerPos.h"
#include "communication/messages/MsgEnvironment.h"
#include "communication/messages/MsgRobotBeacon.h"
#include "communication/messages/MsgRobotPos.h"
#include "communication/messages/MsgRobotServed.h"

namespace Direction
{
	enum DIRECTION { NORTH, EAST, SOUTH, WEST };
	static const int nDIRECTION = 4;
	static const char * cDIRECTION[nDIRECTION] = {"NORTH", "EAST", "SOUTH", "WEST"};
}

class DataProvider
{
public:
	static DataProvider* getInstance();

	// used by CloudServer to process new messages
	void processMsg(::std::stringstream& msg);

	// used by readers/users
	bool isValidMsgEnvironment();
	bool isValidMsgRobotPos();
	MsgEnvironment getLatestMsgEnvironment();
	Direction::DIRECTION getLatestBaseDir();
	MsgRobotPos getLatestMsgRobotPos();
	vector< MsgCustomerOrder > getMsgCustomerOrders();
	vector< MsgCustomerPos > getMsgCustomerPoses();

private:
	static DataProvider *instance;

	Direction::DIRECTION dirBase;
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
