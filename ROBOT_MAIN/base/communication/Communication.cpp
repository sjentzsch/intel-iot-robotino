//============================================================================
// Name        : Communication.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "Communication.h"
#include "ComDataObjectClient.h"
#include "WorldModelClientHandler.h"
#include "ImageServer.h"
#include "ImageClient.h"
#include "LaserScannerServer.h"
#include "LaserScannerClient.h"
#include "StateInformationClient.h"
#include "StateInformationServer.h"
#include "DynamicDataSender.h"
#include "../utils/FileLogger.h"


namespace communication{

// WARNING: BAD MEMORY LEAKS!!!
const char* getIpAdressAsString(IP_ADDRESS address){
	std::stringstream* str = new std::stringstream("");

	for(int i=0;i<3;i++){
		*str<<(int)address[i];
		*str<<".";
	}

	*str<<(int)address[3];
	return str->str().c_str();
}

void startWorldModelServer(){
	//thread_data *data;
	//data = (thread_data *) ptr;  /* type cast to a pointer to thdata */

	ID::ID id = ModelProvider::getInstance()->getID();
	int port = 0;
	if(id==ID::SERVER){
		port = ModelProvider::getInstance()->getComDataObject()->server_port;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports[id-1];
	}

	FileLog::log(log_Communication, "WorldModel-Server started on port ", FileLog::integer(port));
	FileLog::log_NOTICE("WorldModel-Server started on port ", FileLog::integer(port));


	WorldModelServer* server = new WorldModelServer(port);
	server->handleWorldModelClients();
}

void startStateInformationServer(){
	ID::ID id = ModelProvider::getInstance()->getID();
	int port = 0;
	if(id==ID::SERVER){
		port = ModelProvider::getInstance()->getComDataObject()->server_port+300;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports[id-1]+300;
	}

	for(int i=0;i<3;i++){
		FileLog::log(log_Communication, "StateInformation-Server started on port ", FileLog::integer(port+i));
		StateInformationServer* server = new StateInformationServer(port+i);
		server->handleConnections();
	}
}

void startComDataObjectServer(){
	//thread_data *data;
	//data = (thread_data *) ptr;  /* type cast to a pointer to thdata */

	ID::ID id = ModelProvider::getInstance()->getID();
	int port = 0;
	if(id==ID::SERVER){
		port = ModelProvider::getInstance()->getComDataObject()->server_port+100;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports[id-1]+100;
	}

	FileLog::log(log_Communication, "ComDataObject-Server started on port ", FileLog::integer(port));


	ComDataObjectServer* server = new ComDataObjectServer(port);
	server->handleConnections();
}

void startImageDataServer(){
	ID::ID id = ModelProvider::getInstance()->getID();
	int port = 0;
	if(id==ID::SERVER){
		port = ModelProvider::getInstance()->getComDataObject()->server_port+200;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports[id-1]+200;
	}

	for(int i=0;i<3;i++){
		FileLog::log(log_Communication, "ImageData-Server started on port ", FileLog::integer(port+i));
		ImageServer* server = new ImageServer(port+i);
		server->handleConnections();
	}
}

void startLaserScannerDataServer(){
	ID::ID id = ModelProvider::getInstance()->getID();
	int port = 0;
	if(id==ID::SERVER){
		port = ModelProvider::getInstance()->getComDataObject()->server_port+400;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports[id-1]+400;
	}

	for(int i=0;i<3;i++){
		FileLog::log(log_Communication, "LaserScannerData-Server started on port ", FileLog::integer(port+i));
		LaserScannerServer* server = new LaserScannerServer(port+i);
		server->handleConnections();
	}
}

//TODO: synchronize on comDataObject and model
void sendWorldModel(){
	FileLog::log(log_Communication, "Start sending World Model ...");
	WorldModelClientHandler::getInstance()->lock();
	WorldModelClientHandler::getInstance()->unlock();
	FileLog::log(log_Communication, "World Model sent!");
}

void sendComDataObject(){
	FileLog::log(log_Communication, "Sending ComDataObject!");
	for(int i=0;i<3;i++){
		if(ModelProvider::getInstance()->getComDataObject()->enabled[i]){
			boost::asio::io_service io_service;
			ComDataObjectClient client(io_service,i);
			client.sendComDataObject();
			io_service.run();
		}
	}
}

void sendImage(){
	ImageClient imageClient = ImageClient();
	imageClient.sendImageData();
}

void sendLaserScanner(){
	LaserScannerClient laserScannerClient = LaserScannerClient();
	laserScannerClient.sendLaserScannerData();
}

void sendStateInformation(){
	StateInformationClient stateInformationClient = StateInformationClient();
	stateInformationClient.sendStateInformation();
}

void startDynamicDataSender(){
	DynamicDataSender* sender = new DynamicDataSender();
	sender->start();
	FileLog::log(log_Communication, "DynamicDataSender started!");
}

}
