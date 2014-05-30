/*
 * StateInformationClient.cpp
 *
 *  Created on: Jun 9, 2011
 *      Author: root
 */

#include "StateInformationClient.h"


StateInformationClient::StateInformationClient() {

}

StateInformationClient::~StateInformationClient() {

}

void StateInformationClient::sendStateInformation(){
	ComDataObject* comDataObject = ModelProvider::getInstance()->getComDataObject();
	if(comDataObject->stateinfo_enabled[(int)ModelProvider::getInstance()->getID()-1]){
			try{
				boost::asio::io_service io_service;

				int offset = (int)ModelProvider::getInstance()->getID()-1;
				std::stringstream* str = new std::stringstream("");

				for(int i=0;i<3;i++){
					*str<<(int)comDataObject->server_address[i];
					*str<<".";
				}

				*str<<(int)comDataObject->server_address[3];
				std::string tmpAddress(str->str().c_str());
			//	std::cout << "addressString: " << tmpAddress << std::endl;
				boost::asio::ip::address tempAdd;
				tempAdd = boost::asio::ip::address::from_string(tmpAddress.c_str());

				udp::endpoint endpoint(tempAdd,comDataObject->server_port+300+offset);


				udp::socket socket(io_service);
				socket.open(udp::v4());


				char* byte_array = static_cast<char*>(static_cast<void*>(DynDataProvider::getInstance()->getStateInformation(ModelProvider::getInstance()->getID())));


				socket.send_to(boost::asio::buffer(byte_array,sizeof(StateInformation)), endpoint);
				delete str;
			}
			catch (std::exception& e)
			{
			std::cerr << e.what() << std::endl;
			}
	  }

}
