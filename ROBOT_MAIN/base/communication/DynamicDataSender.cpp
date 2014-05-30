/*
 * DynamicDataSender.cpp
 *
 *  Created on: Jun 24, 2011
 *      Author: root
 */

#include "DynamicDataSender.h"
#include "Communication.h"
#include <iostream>

DynamicDataSender::DynamicDataSender() {
	execThread = NULL;
}

void DynamicDataSender::runExec(){
	try
	{
	while(true){
		communication::sendImage();
		communication::sendLaserScanner();
		communication::sendStateInformation();
		boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	}
	}catch(std::exception& e)
	  {
		std::cerr << e.what() << "\n";
	  }
}

void DynamicDataSender::start(){
	execThread = new boost::thread(&DynamicDataSender::runExec,this);
}

DynamicDataSender::~DynamicDataSender() {
}
