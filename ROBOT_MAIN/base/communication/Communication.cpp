//============================================================================
// Name        : Communication.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "Communication.h"
#include "ComDataObjectClient.h"
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

	int port = 0;
	if(true /*= SERVER*/){
		port = ModelProvider::getInstance()->getComDataObject()->server_port;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports;
	}

	FileLog::log(log_Communication, "WorldModel-Server started on port ", FileLog::integer(port));
	FileLog::log_NOTICE("WorldModel-Server started on port ", FileLog::integer(port));


	WorldModelServer* server = new WorldModelServer(port);
	server->handleWorldModelClients();
}

void startComDataObjectServer(){
	//thread_data *data;
	//data = (thread_data *) ptr;  /* type cast to a pointer to thdata */

	int port = 0;
	if(true /*= SERVER*/){
		port = ModelProvider::getInstance()->getComDataObject()->server_port+100;
	}else{
		port = ModelProvider::getInstance()->getComDataObject()->client_ports+100;
	}

	FileLog::log(log_Communication, "ComDataObject-Server started on port ", FileLog::integer(port));


	ComDataObjectServer* server = new ComDataObjectServer(port);
	server->handleConnections();
}

void sendComDataObject(){
	FileLog::log(log_Communication, "Sending ComDataObject!");
	boost::asio::io_service io_service;
	ComDataObjectClient client(io_service,0);
	client.sendComDataObject();
	io_service.run();
}

}
