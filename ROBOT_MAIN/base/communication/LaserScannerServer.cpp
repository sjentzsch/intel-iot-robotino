/*
 * LaserScannerServer.cpp
 *
 *  Created on: Mar 31, 2014
 *      Author: root
 */

#include "LaserScannerServer.h"

#include "../model/LaserScannerData.h"
#include "../model/DynDataProvider.h"

#include <boost/asio.hpp>
#include <iostream>
using boost::asio::ip::udp;
using boost::asio::ip::tcp;

LaserScannerServer::LaserScannerServer(int port) {
	this->port = port;
}

LaserScannerServer::~LaserScannerServer() {
}

void LaserScannerServer::handleConnections(){
	exec_start = new boost::thread(&LaserScannerServer::handleConnections_impl,this);
}

void LaserScannerServer::handleConnections_impl(){

		try
		  {

			//tcp::endpoint endpoint(boost::asio::ip::address::from_string(),port);

		    boost::asio::io_service io_service;
		    udp::socket socket(io_service, udp::endpoint(udp::v4(), port));

		    LaserScannerData laserScannerData = LaserScannerData();
		    for (;;)
		        {
		          std::array<char, sizeof(LaserScannerData)> recv_buf;

		          //char* byte_array = static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getWorldModel()));


		          udp::endpoint remote_endpoint;
		          boost::system::error_code error;
		          socket.receive_from(boost::asio::buffer(recv_buf),
		              remote_endpoint, 0, error);



		          if (error && error != boost::asio::error::message_size)
		            throw boost::system::system_error(error);


		          memcpy(&laserScannerData,recv_buf.data(),sizeof(LaserScannerData));
		          memcpy(DynDataProvider::getInstance()->getLaserScannerData((ID::ID)laserScannerData.id),&laserScannerData,sizeof(LaserScannerData));

		          /*std::cout << "Laser-Scanner-Data received: These are the final clusters: " << ::std::endl;
					for(unsigned int i=0; i<LASERSCANNER_MAX_DYN_OBSTACLES; i++)
					{
						std::cout << i << ": " << DynDataProvider::getInstance()->getLaserScannerData((ID::ID)laserScannerData.id)->dynObstacles[i][0] << "," << DynDataProvider::getInstance()->getLaserScannerData((ID::ID)laserScannerData.id)->dynObstacles[i][1] << ::std::endl;
					}*/

//				  if(DynDataProvider::getInstance()->getImageDataListener((ID::ID)imageData.id)!=NULL){
//					  DynDataProvider::getInstance()->getImageDataListener((ID::ID)imageData.id)->notify();
//					 // std::cout<<"received Image from "<<imageData->id<<std::endl;
//				  }
			  }

		    	//socket.close(); 	// TODO: check
			}
		catch(std::exception& e)
		  {
			std::cerr << e.what() << "\n";
		  }
}
