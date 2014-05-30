/*
 * ComDataObjectServer.cpp
 *
 *  Created on: May 14, 2011
 *      Author: root
 */

#include "ComDataObjectServer.h"

ComDataObjectServer::ComDataObjectServer(int port) {
	this->port = port;
}

ComDataObjectServer::~ComDataObjectServer() {

}

void ComDataObjectServer::handleConnections(){
	exec_start = new boost::thread(&ComDataObjectServer::handleConnections_impl,this);
}


void ComDataObjectServer::handleConnections_impl(){

	try
	  {
	    boost::asio::io_service io_service;
	    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));
	    acceptor.listen(3);
	while(true){

			tcp::socket socket(io_service);
		    acceptor.accept(socket);
		    std::array<char, sizeof(ComDataObject)> recv_buf;
			//printf("Connection to %s (ComDataObject)\n", inet_ntoa(cli.sin_addr));



			//printf("ComDataObject (Command:%d)\n",ModelProvider::getInstance()->getComDataObject()->command);
		    boost::system::error_code ignored_error;
		    boost::asio::read(socket,boost::asio::buffer(recv_buf));

		    memcpy(ModelProvider::getInstance()->getComDataObject(),recv_buf.data(),sizeof(ComDataObject));


			printf("received ComDataObject (Command:%d)\n",ModelProvider::getInstance()->getComDataObject()->command);

			if(ModelProvider::getInstance()->getComDataObject()->command == COMMAND::PAUSE)
			{
				communication::sendWorldModel();
				printf("Sent WorldModel because of pause\n");
			}

			if(ModelProvider::getInstance()->getCommandListener()!=NULL){
				ModelProvider::getInstance()->getCommandListener()->notify();
			}

			socket.close();
			printf("closed client\n");
	}
	}
	catch (std::exception& e)
	  {
		std::cerr << "[ComDataServer] "<<e.what() <<"\n";
	  }

}
