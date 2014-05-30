/*
 * WorldModelClientHandler.cpp
 *
 *  Created on: Jun 8, 2011
 *      Author: root
 */

#include "WorldModelClientHandler.h"

WorldModelClientHandler* WorldModelClientHandler::handler = NULL;

WorldModelClientHandler::WorldModelClientHandler() {
}

WorldModelClientHandler::~WorldModelClientHandler() {
}

WorldModelClientHandler* WorldModelClientHandler::getInstance(){
	if(handler == NULL)
		handler = new WorldModelClientHandler();
	return handler;
}

void WorldModelClientHandler::lock()
{
	handlerMutex.lock();

	FileLog::log(log_Communication, "[WorldModelClientHandler] Lock all clients and the server!");

	for(unsigned int i=0; i<3; i++){

		if(BaseParameterProvider::getInstance()->getParams()->simulation_mode){
			clients[i] = new WorldModelClient((ID::ID)(i+1), 5000); // wait x seconds before kicking a robot
		}
		else{
			clients[i] = new WorldModelClient((ID::ID)(i+1), 20000); // wait x seconds before kicking a robot
		}
		FileLog::log(log_Communication, "[WorldModelClientHandler] Waiting for locking client ", FileLog::integer(i+1), " ...");
		clients[i]->lock();
		FileLog::log(log_Communication, "[WorldModelClientHandler] Finished locking client ", FileLog::integer(i+1));
	}
	server = new WorldModelClient(ID::SERVER, 250); // wait 250ms for a connection to the GUI
	FileLog::log(log_Communication, "[WorldModelClientHandler] Waiting for locking client 4 (server) ...");
	server->lock();
	FileLog::log(log_Communication, "[WorldModelClientHandler] Finished locking client 4 (server)");

	// now we have the lock for each agent and can manipulate the worldModel
	// first check if robots were kicked and change the world model accordingly
	for(unsigned int i=0; i<3; i++)
	{
		if(clients[i]->getRobotWasKicked())
			ModelProvider::getInstance()->clearWorldModelFromRobot((ID::ID)(i+1));
	}
}

void WorldModelClientHandler::unlock()
{
	{
		boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
		FileLog::log(log_Communication, "[WorldModelClientHandler] Incrementing WorldModel Version number (start), now it is ", FileLog::integer(ModelProvider::getInstance()->getWorldModel()->version));
		ModelProvider::getInstance()->getWorldModel()->version++;
		FileLog::log(log_Communication, "[WorldModelClientHandler] Incrementing WorldModel Version number (end), now it is ", FileLog::integer(ModelProvider::getInstance()->getWorldModel()->version));
	}
	FileLog::log(log_Communication, "[WorldModelClientHandler] Unlock all clients and the server!");
	for(unsigned int i=0; i<3; i++)
	{
		clients[i]->unlock();
		delete clients[i];
	}
	server->unlock();
	delete server;

	handlerMutex.unlock();
}
