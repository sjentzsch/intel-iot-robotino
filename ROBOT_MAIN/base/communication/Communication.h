/*
 * Communication.h
 *
 *  Created on: Apr 24, 2011
 *      Author: peter
 */


#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_



#include "WorldModelClient.h"
#include "WorldModelServer.h"
#include "ComDataObjectServer.h"
#include "../model/ModelProvider.h"

namespace communication{
	//Either use this or use an interface instead of a function pointer
	//synchronize on comDataObject and model

	const char* getIpAdressAsString(IP_ADDRESS address);

	void startWorldModelServer();			// GUI: main | Robot: main
	void sendWorldModel();					// GUI: startContinueCommand, sendWMCommand | Robot: If Paused Command is received

	void startComDataObjectServer();		// Robot: main
	void startDynamicDataSender();			// Robot: main
	void sendImage();						// Robot: DynamicDataSender
	void sendLaserScanner();				// Robot: DynamicDataSender
	void sendStateInformation();			// Robot: DynamicDataSender and StateMachine

	void startImageDataServer();			// GUI: main
	void startLaserScannerDataServer();		// GUI: main
	void startStateInformationServer();		// GUI: main
	void sendComDataObject();				// GUI: (several)
}

#endif /* COMMUNICATION_H_ */
