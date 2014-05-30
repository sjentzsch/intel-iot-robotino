/*
 * StateInformationServer.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: root
 */

#include "StateInformationServer.h"

#include "../model/StateInformation.h"
#include "../model/ModelProvider.h"

#include <boost/asio.hpp>
#include <iostream>
using boost::asio::ip::udp;
using boost::asio::ip::tcp;

StateInformationServer::StateInformationServer(int port) {
	this->port = port;
}

StateInformationServer::~StateInformationServer() {

}

void StateInformationServer::handleConnections(){
	exec_start = new boost::thread(&StateInformationServer::handleConnections_impl,this);
}

void StateInformationServer::handleConnections_impl(){

		try
		  {

			//tcp::endpoint endpoint(boost::asio::ip::address::from_string(),port);

		    boost::asio::io_service io_service;
		    udp::socket socket(io_service, udp::endpoint(udp::v4(), port));

		    StateInformation stateInformation = StateInformation();
		    for (;;)
		        {
		    		std::array<char, sizeof(StateInformation)> recv_buf;

		          //char* byte_array = static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getWorldModel()));


		          udp::endpoint remote_endpoint;
		          boost::system::error_code error;
		          socket.receive_from(boost::asio::buffer(recv_buf),
		              remote_endpoint, 0, error);



		          if (error && error != boost::asio::error::message_size)
		            throw boost::system::system_error(error);


		          memcpy(&stateInformation,recv_buf.data(),sizeof(StateInformation));
		          memcpy(DynDataProvider::getInstance()->getStateInformation((ID::ID)stateInformation.id),&stateInformation,sizeof(StateInformation));


				  if(DynDataProvider::getInstance()->getStateInformationListener((ID::ID)stateInformation.id)!=NULL){
					  DynDataProvider::getInstance()->getStateInformationListener((ID::ID)stateInformation.id)->notify();
					  //std::cout<<"received StateInformation from "<<ModelProvider::getInstance()->getStateInformation((ID::ID)stateInformation.id)->x<<std::endl;
				  }

				  if(DynDataProvider::getInstance()->getStateInformationListenerForSim((ID::ID)stateInformation.id)!=NULL){
					  DynDataProvider::getInstance()->getStateInformationListenerForSim((ID::ID)stateInformation.id)->notify((ID::ID)stateInformation.id);
					  //std::cout<<"received StateInformation from "<<ModelProvider::getInstance()->getStateInformation((ID::ID)stateInformation.id)->x<<std::endl;
				  }

				  if(DynDataProvider::getInstance()->getImageDataListener((ID::ID)stateInformation.id)!=NULL){
					  DynDataProvider::getInstance()->getImageDataListener((ID::ID)stateInformation.id)->notify();
					 // std::cout<<"received Image from "<<imageData->id<<std::endl;
				  }



			  }

		    	//socket.close(); // TODO: check
			}
		catch(std::exception& e)
		  {
			std::cerr << e.what() << "\n";
		  }
}
