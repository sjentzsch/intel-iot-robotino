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

		acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));

		while(true)
		{
			tcp::socket socket(io_service);

			FileLog::log(log_Communication, "[CloudServer] Waiting for client ...");

			acceptor.accept(socket);

			FileLog::log(log_Communication, "[CloudServer] Client connected.");

			::std::stringstream result;
			boost::asio::streambuf response;

		    boost::system::error_code error;
		    while(boost::asio::read(socket, response, boost::asio::transfer_at_least(1), error))
		    {
			    if(error && error != boost::asio::error::eof)
			      throw boost::system::system_error(error);
		    	result << &response;
		    }

		    FileLog::log(log_Communication, "[CloudServer] Received: ", result.str());

		    DataProvider::getInstance()->processMsg(result);

			//socket.close();
		    FileLog::log(log_Communication, "[CloudServer] Finished.");
		}
	}
	catch (std::exception& e)
	{
		FileLog::log(log_Communication, "[CloudServer] ", e.what());
	}
}
