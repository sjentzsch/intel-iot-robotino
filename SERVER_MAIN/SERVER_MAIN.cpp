//
// SERVER_MAIN.cpp
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

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <queue>
#include "config.h"
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include "utils/FileLoggerConfig.h"
#include "communication/CloudComm.h"

using namespace std;
using namespace boost;
using boost::lexical_cast;
using boost::bad_lexical_cast;

void initLog();
void pantheios_init();

// hit Enter key = send MsgRobotPause = toggle pause/unpause of the robot
void robotPauseThread_impl()
{
	string temp;
	while(true)
	{
		getline(cin, temp);
		MsgRobotPause msgRobotPause((unsigned long)std::time(0));
		CloudComm::getInstance()->getCloudClient()->send(msgRobotPause.save());
	}
}

int main(int argc, char* argv[])
{
	try
	{
		// initialize random seed
		srand (static_cast <unsigned> (time(0)));

		// initialize the logging class
		pantheios_init();

		// communication setup
		boost::asio::io_service io_service;
		boost::asio::io_service::work work(io_service);
		boost::thread* ioServiceThread = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

#if SIMULATION_MODE == 1
		CloudComm::getInstance()->init(new CloudServer(8197), new CloudClient(io_service, "localhost", 8198));
#else
		//CloudComm::getInstance()->init(new CloudServer(8190), new CloudClient(io_service, "172.26.1.1", 8191));
		CloudComm::getInstance()->init(new CloudServer(8190), new CloudClient(io_service, "192.168.1.244", 8191));
#endif

		CloudComm::getInstance()->getCloudServer()->handleConnections();

		//MsgEnvironment msgEnv((unsigned long)std::time(0), 2.8, 3.2, 1.55, 2.2, 1.05, 3.0, 90.0);
		MsgEnvironment msgEnv((unsigned long)std::time(0), 3.0, 5.0, 1.2, 3.9, 0.6, 4.8, 90.0);
		CloudComm::getInstance()->getCloudClient()->send(msgEnv.save());

		MsgCustomerOrder msgCustOrder1(0, 5, 22);
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder1.save());

		MsgCustomerOrder msgCustOrder2(2, 6, 23);
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder2.save());

		MsgCustomerOrder msgCustOrder3(1, 7, 22);
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder3.save());

		MsgCustomerOrder msgCustOrder4(2, 8, 24);
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder4.save());

		MsgCustomerPos msgCustPos1(0, 22, "John", 1.5, 1.5);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos1.save());

		MsgCustomerPos msgCustPos2(0, 22, "John", 1.0, 0.5);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos2.save());

		MsgCustomerPos msgCustPos4(0, 23, "Peter", 2.0, 2.5);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos4.save());

		MsgCustomerPos msgCustPos5(0, 24, "Klaus", 0.5, 1.5);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos5.save());

		FileLog::log_NOTICE("Waiting for receiving first MsgRobotBeacon ...");
		while(!DataProvider::getInstance()->isValidMsgRobotBeacon())
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));

		// Start async thread for enabling the user to pause/unpause the robot
		boost::thread* robotPauseThread = new boost::thread(robotPauseThread_impl);

		while(true)
		{
			MsgRobotBeacon msg = DataProvider::getInstance()->getLatestMsgRobotBeacon();
			cout << std::to_string(msg.time) << ": robot " << (msg.running ? "'RUN'" : "'PAUSE'") << " in state '" << msg.state << "' at " << msg.x << ", " << msg.y << ", " << msg.phi << " with " << msg.drinks_available << " drink(s) available" << endl;

			vector< MsgRobotServed > vecMsgRobotServed = DataProvider::getInstance()->getMsgRobotServed();
			cout << "robot served " << vecMsgRobotServed.size() << " drink(s) already: ";
			for(unsigned int i=0; i<vecMsgRobotServed.size(); i++)
				cout << std::to_string(vecMsgRobotServed.at(i).order_id) << " ";
			cout << endl;

			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		}

		// Debug stuff: print the messages ...
		/*boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
		DataProvider::getInstance()->getLatestMsgEnvironment().print();
		DataProvider::getInstance()->getLatestMsgRobotPos().print();
		vector< MsgCustomerOrder > vecMsgCustomerOrders = DataProvider::getInstance()->getMsgCustomerOrders();
		for(unsigned int i=0; i<vecMsgCustomerOrders.size(); i++)
			vecMsgCustomerOrders.at(i).print();
		vector< MsgCustomerPos > vecMsgCustomerPoses = DataProvider::getInstance()->getMsgCustomerPoses();
		for(unsigned int i=0; i<vecMsgCustomerPoses.size(); i++)
			vecMsgCustomerPoses.at(i).print();*/

	}
	catch( const std::exception& e )
	{
		FileLog::log_ERROR("[TBU_ACAPS] ", e.what());
	}

	return 0;
}

void initLog()
{
	// http://patorjk.com/software/taag/#p=display&f=Banner4&t=INTEL
	FileLog::log_NOTICE("#########################################");
	FileLog::log_NOTICE("                                         ");
	FileLog::log_NOTICE(".####.##....##.########.########.##......");
	FileLog::log_NOTICE("..##..###...##....##....##.......##......");
	FileLog::log_NOTICE("..##..####..##....##....##.......##......");
	FileLog::log_NOTICE("..##..##.##.##....##....######...##......");
	FileLog::log_NOTICE("..##..##..####....##....##.......##......");
	FileLog::log_NOTICE("..##..##...###....##....##.......##......");
	FileLog::log_NOTICE(".####.##....##....##....########.########");
	FileLog::log_NOTICE("_________________________________________");
	FileLog::log_NOTICE("#########################################");
	#ifdef DEBUG
	FileLog::log_NOTICE("##     DEBUG BUILD     ##################");
	FileLog::log_NOTICE("#########################################");
	#else
	FileLog::log_NOTICE("##     RELEASE BUILD   ##################");
	FileLog::log_NOTICE("#########################################");
	#endif
}


void pantheios_init(){

	int logCount = 0;

	filesystem::path logPath = filesystem::path("/home/robotino/log/");
	if(!filesystem::exists(logPath))
		filesystem::create_directory(logPath);

	while(filesystem::exists(logPath.string()+boost::lexical_cast<string>(logCount)+"/")){
		//cout << logPath.string() << boost::lexical_cast<string>(logCount) << "/" <<endl;
		logCount++;
	}
	filesystem::path nextLogFolder = filesystem::path(logPath.string()+boost::lexical_cast<string>(logCount)+"/");
	filesystem::create_directory(nextLogFolder);

	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_TBU_ACAPS")).c_str(),0);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Communication")).c_str(),4);

	initLog();
}
