//
// ROBOT_MAIN.cpp
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
#include "MotorController.h"
#include "SensorServer.h"
#include "SensorEventGenerator.h"
#include "StateBehaviorController.h"
#include "utils/FileLoggerConfig.h"
#include "communication/CloudComm.h"
#include "Simulation/SimApi2Com.h"
#include "Api2Com.h"

#include <rec/robotino/api2/all.h>

using namespace std;
using namespace boost;
using boost::lexical_cast;
using boost::bad_lexical_cast;

void initLog();
void pantheios_init();

void initApi2(rec::robotino::api2::Com *api2Com) {

	FileLog::log_NOTICE("[Api2Com] Initializing API2 Com");
	api2Com->setAddress( "127.0.0.1" );

	api2Com->connectToServer( true );

	if( !api2Com->isConnected() )
	{
		//TODO FileLogger
		FileLog::log_ERROR("[Api2Com] Could not connect to ",api2Com->address());
		exit( 1 );
	}
	else
	{
		FileLog::log_NOTICE("[Api2Com] connected to api server");
	}

	if (!api2Com->isLocalConnection()) {
		//TODO FileLoggerb
		FileLog::log_WARNING("[Api2Com] WARNING: Api2 connection isn't local (SharedMemory disabled). ",api2Com->address());
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
		CloudComm::getInstance()->init(new CloudServer(8198), new CloudClient(io_service, "localhost", 8197));
#else
		//CloudComm::getInstance()->init(new CloudServer(8191), new CloudClient(io_service, "172.26.1.114", 8190));
		//CloudComm::getInstance()->init(new CloudServer(8191), new CloudClient(io_service, "192.168.1.25", 8190));
		CloudComm::getInstance()->init(new CloudServer(8191), new CloudClient(io_service, "192.168.1.22", 8190));
#endif
		CloudComm::getInstance()->getCloudServer()->handleConnections();

#if SIMULATION_MODE == 1
		rec::robotino::api2::Com* api2Com = new SimApi2Com();
#else
		rec::robotino::api2::Com* api2Com = new Api2Com();
		initApi2(api2Com);
#endif

		// wait for MsgEnvironment to arrive ...
		FileLog::log_NOTICE("Waiting for receiving MsgEnvironment ...");
		while(!DataProvider::getInstance()->isValidMsgEnvironment())
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));

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

		SensorServer* sensorSrv = new SensorServer();
		FileLog::log_NOTICE("Instantiated SensorServer");
		MotorController* motorCtrl = new MotorController(sensorSrv);
		FileLog::log_NOTICE("Instantiated MotorController");
		SensorEventGenerator* sensorEvtGen = new SensorEventGenerator(sensorSrv);
		FileLog::log_NOTICE("Instantiated SensorEventGenerator");
		StateBehaviorController *stateCtrl = new StateBehaviorController(motorCtrl, sensorSrv, sensorEvtGen);
		FileLog::log_NOTICE("Instantiated StateBehaviorController, starting statemachine");
		stateCtrl->initiate();

		while(true)
		{
#if SIMULATION_MODE == 1
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
#else
			api2Com->processEvents();
			rec::robotino::api2::msleep(10);
#endif
		}
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
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_MotorController")).c_str(),1);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_SensorEventGenerator")).c_str(),2);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Communication")).c_str(),4);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Camera")).c_str(),5);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Job")).c_str(),6);

	initLog();
}
