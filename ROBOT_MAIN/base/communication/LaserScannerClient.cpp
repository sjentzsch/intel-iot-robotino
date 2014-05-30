/*
 * LaserScannerClient.cpp
 *
 *  Created on: Mar 31, 2014
 *      Author: root
 */

#include "LaserScannerClient.h"

LaserScannerClient::LaserScannerClient() {

}

LaserScannerClient::~LaserScannerClient() {

}

void LaserScannerClient::sendLaserScannerData(){

	if(ModelProvider::getInstance()->getComDataObject()->laserscanner_enabled[(int)ModelProvider::getInstance()->getID()-1]){
		try
		  {
			ComDataObject* comDataObject = ModelProvider::getInstance()->getComDataObject();

			boost::asio::io_service io_service;

			int offset = (int)ModelProvider::getInstance()->getID()-1;
			std::stringstream* str = new std::stringstream("");

			for(int i=0;i<3;i++){
				*str<<(int)comDataObject->server_address[i];
				*str<<".";
			}

			*str<<(int)comDataObject->server_address[3];
			std::string tmpAddress(str->str().c_str());

			boost::asio::ip::address tempAdd;
			tempAdd = boost::asio::ip::address::from_string(tmpAddress.c_str());

			udp::endpoint endpoint(tempAdd,comDataObject->server_port+400+offset);


			udp::socket socket(io_service);
			socket.open(udp::v4());


			char* byte_array = static_cast<char*>(static_cast<void*>(DynDataProvider::getInstance()->getLaserScannerData(ModelProvider::getInstance()->getID())));


			socket.send_to(boost::asio::buffer(byte_array,sizeof(LaserScannerData)), endpoint);
			delete str;

			//socket.close();	// TODO: check

			//	std::cout<<"sent LaserScannerData"<<std::endl;

			}
			catch (std::exception& e)
			{
			std::cerr << e.what() << std::endl;
			}
	}
}
