//
// CloudServer.cpp
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Sören Jentzsch
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
