/*
 * CloudServer.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: root
 */

#include "CloudServer.h"

CloudServer::CloudServer(int port)
{
	this->exec_start = NULL;
	this->port = port;
}

CloudServer::~CloudServer()
{

}

void CloudServer::handleConnections()
{
	exec_start = new boost::thread(&CloudServer::handleConnections_impl,this);
}

void CloudServer::handleConnections_impl()
{
	try
	{
		boost::asio::io_service io_service;

		tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));

		// TODO: uncomment?
		// acceptor.listen(1);

		while(true)
		{
			tcp::socket socket(io_service);

			std::cout << "[CloudServer] Waiting for client ..." << std::endl;

			acceptor.accept(socket);

			std::cout << "[CloudServer] Client connected." << std::endl;

			::std::stringstream result;
			boost::asio::streambuf response;

		    boost::system::error_code error;
		    while(boost::asio::read(socket, response, boost::asio::transfer_at_least(1), error))
		    {
			    if(error && error != boost::asio::error::eof)
			      throw boost::system::system_error(error);
		    	result << &response;
		    }

		    // TODO: process result.str();

			// TODO: should the socket be really closed ?! or is it already closed by the client ?!
			socket.close();
			std::cout << "[CloudServer] Closed client." << std::endl;
		}
	}
	catch (std::exception& e)
	{
		std::cerr << "[CloudServer] "<< e.what() << std::endl;
	}
}
