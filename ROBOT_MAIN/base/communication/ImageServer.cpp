/*
 * ImageServer.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: root
 */

#include "ImageServer.h"

#include "../model/ImageData.h"
#include "../model/DynDataProvider.h"

#include <boost/asio.hpp>
#include <iostream>
using boost::asio::ip::udp;
using boost::asio::ip::tcp;

ImageServer::ImageServer(int port) {
	this->port = port;
}

ImageServer::~ImageServer() {
	// TODO Auto-generated destructor stub
}

void ImageServer::handleConnections(){
	exec_start = new boost::thread(&ImageServer::handleConnections_impl,this);
}

void ImageServer::handleConnections_impl(){

		try
		  {

			//tcp::endpoint endpoint(boost::asio::ip::address::from_string(),port);

		    boost::asio::io_service io_service;
		    udp::socket socket(io_service, udp::endpoint(udp::v4(), port));

		    ImageData imageData = ImageData();
		    for (;;)
		        {
		          std::array<char, sizeof(ImageData)> recv_buf;

		          //char* byte_array = static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getWorldModel()));


		          udp::endpoint remote_endpoint;
		          boost::system::error_code error;
		          socket.receive_from(boost::asio::buffer(recv_buf),
		              remote_endpoint, 0, error);



		          if (error && error != boost::asio::error::message_size)
		            throw boost::system::system_error(error);


		          memcpy(&imageData,recv_buf.data(),sizeof(ImageData));
		          memcpy(DynDataProvider::getInstance()->getImageData((ID::ID)imageData.id),&imageData,sizeof(ImageData));

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
