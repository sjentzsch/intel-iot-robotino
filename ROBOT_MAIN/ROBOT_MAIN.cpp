#include <iostream>
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
#include "Api2Com.h"
#include "BaseParameterProvider.h"

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
	rec::robotino::api2::Com *api2Com;
	try
	{
		// initialize the logging class
		pantheios_init();

		// init base parameters
		std::string executable = argv[0];
		size_t idx = executable.find_last_of('/');
		std::string execdir = executable.substr(0,idx);
		BaseParameterProvider::setDirectory(execdir);
		BaseParameterProvider::getInstance()->getParams()->print();

		// communication setup
		boost::asio::io_service io_service;
		boost::asio::io_service::work work(io_service);
		boost::thread* ioServiceThread = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
		//CloudComm::getInstance()->init(new CloudServer(8190), new CloudClient(io_service, "172.26.1.1", 8191));
		CloudComm::getInstance()->init(new CloudServer(8190), new CloudClient(io_service, "localhost", 8190));
		CloudComm::getInstance()->getCloudServer()->handleConnections();

		//rec::robotino::api2::msleep(3000);

		CloudComm::getInstance()->getCloudClient()->sendTest();
		CloudComm::getInstance()->getCloudClient()->sendTest();

		while(true)
		{
			rec::robotino::api2::msleep(10);
		}

		api2Com = new Api2Com();
		initApi2(api2Com);

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
			api2Com->processEvents();
			rec::robotino::api2::msleep(10);
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
	FileLog::log_NOTICE("#################################################");
	FileLog::log_NOTICE("                                                 ");
	FileLog::log_NOTICE("....###.....######.....###....########...######..");
	FileLog::log_NOTICE("...##.##...##....##...##.##...##.....##.##....##.");
	FileLog::log_NOTICE("..##...##..##........##...##..##.....##.##.......");
	FileLog::log_NOTICE(".##.....##.##.......##.....##.########...######..");
	FileLog::log_NOTICE(".#########.##.......#########.##..............##.");
	FileLog::log_NOTICE(".##.....##.##....##.##.....##.##........##....##.");
	FileLog::log_NOTICE(".##.....##..######..##.....##.##.........######..");
	FileLog::log_NOTICE("_________________________________________________");
	FileLog::log_NOTICE("#################################################");
	#ifdef DEBUG
	FileLog::log_NOTICE("##     DEBUG BUILD     ##########################");
	FileLog::log_NOTICE("#################################################");
	#else
	FileLog::log_NOTICE("##     RELEASE BUILD   ##########################");
	FileLog::log_NOTICE("#################################################");
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
