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
#include "model/ModelProvider.h"
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
		//CloudComm::getInstance()->init(new CloudServer(8190), new CloudClient(io_service, "172.26.1.1", 8191));
		CloudComm::getInstance()->init(new CloudServer(8198), new CloudClient(io_service, "localhost", 8198));
		CloudComm::getInstance()->getCloudServer()->handleConnections();

		MsgEnvironment msgEnv(0, 2.0, 3.0, 0.23, 0.23, 0.0, 1.0, 2.0, 90.0);
		CloudComm::getInstance()->getCloudClient()->send(msgEnv.save());

		MsgRobotPos msgRobotPos1(0, 0.8, 0.9);
		CloudComm::getInstance()->getCloudClient()->send(msgRobotPos1.save());

		MsgCustomerOrder msgCustOrder1(0, 5, 22, "John");
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder1.save());

		MsgCustomerOrder msgCustOrder2(2, 6, 23, "Peter");
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder2.save());

		MsgCustomerOrder msgCustOrder3(1, 7, 22, "John");
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder3.save());

		MsgCustomerOrder msgCustOrder4(2, 8, 24, "Klaus");
		CloudComm::getInstance()->getCloudClient()->send(msgCustOrder4.save());

		MsgCustomerPos msgCustPos1(0, 22, "John", 1.0, 0.5);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos1.save());

		MsgCustomerPos msgCustPos2(0, 22, "John", 0.5, 1.5);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos2.save());

		MsgCustomerPos msgCustPos4(0, 23, "Peter", 1.5, 0.0);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos4.save());

		MsgCustomerPos msgCustPos5(0, 24, "Klaus", 1.0, 2.0);
		CloudComm::getInstance()->getCloudClient()->send(msgCustPos5.save());

		CloudComm::getInstance()->getCloudClient()->send("blalalalala!!");

		MsgRobotPos msgRobotPos2(0, 8.8, 9.9);
		CloudComm::getInstance()->getCloudClient()->send(msgRobotPos2.save());

		// wait for MsgEnvironment to arrive from the CloudServer ...
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

#if SIMULATION_MODE == 1
		rec::robotino::api2::Com* api2Com = new SimApi2Com();
#else
		rec::robotino::api2::Com* api2Com = new Api2Com();
		initApi2(api2Com);
#endif

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
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_AsyncWorldModelUpdater")).c_str(),3);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Communication")).c_str(),4);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Camera")).c_str(),5);
	pantheios_be_file_setFilePath((string(nextLogFolder.string()+"log_Job")).c_str(),6);

	initLog();
}
