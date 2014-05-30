/*
 * ImageClient.cpp
 *
 *  Created on: Jun 9, 2011
 *      Author: root
 */

#include "ImageClient.h"


ImageClient::ImageClient() {


}

ImageClient::~ImageClient() {

}

void ImageClient::sendImageData(){

	if(ModelProvider::getInstance()->getComDataObject()->image_enabled[(int)ModelProvider::getInstance()->getID()-1]){
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

			udp::endpoint endpoint(tempAdd,comDataObject->server_port+200+offset);


			udp::socket socket(io_service);
			socket.open(udp::v4());


			char* byte_array = static_cast<char*>(static_cast<void*>(DynDataProvider::getInstance()->getImageData(ModelProvider::getInstance()->getID())));


			socket.send_to(boost::asio::buffer(byte_array,sizeof(ImageData)), endpoint);
			delete str;

			//socket.close();	// TODO: check

			//	std::cout<<"sent image"<<std::endl;

			}
			catch (std::exception& e)
			{
			std::cerr << e.what() << std::endl;
			}
	}

}
