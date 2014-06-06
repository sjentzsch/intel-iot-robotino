/*
 * CloudClient.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: root
 */

#include "CloudClient.h"

#include <boost/lexical_cast.hpp>

CloudClient::CloudClient(boost::asio::io_service& io_service_, ::std::string host_, int port_)
:io_service(io_service_),socket(io_service_),timer(io_service_)
{
	this->host = host_;
	this->port = port_;
}

CloudClient::~CloudClient()
{

}

void CloudClient::handleConnect(const boost::system::error_code& ec)
{
	timer.cancel();

	if(!socket.is_open())
	{
		std::cout << "[CloudClient] Connect timed out" << std::endl;
	}
	else if(ec)
	{
		std::cout << "[CloudClient] Connect error: " << ec.message() << std::endl;
		socket.close();
	}
	else
	{
		std::cout << "[CloudClient] Connected" << std::endl;

		//boost::array<char, sizeof(ComDataObject)> buf;
		//buf.assign(static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getComDataObject())));

		std::string message = "blub !!!";

		boost::system::error_code ignored_error;
		boost::asio::write(socket, boost::asio::buffer(message), boost::asio::transfer_all(), ignored_error);

		socket.close();
	}
}

void CloudClient::checkDeadline()
{
	//if(closed)
	//	return;

	// Check whether the deadline has passed. We compare the deadline against
	// the current time since a new asynchronous operation may have moved the
	// deadline before this actor had a chance to run.
	if(timer.expires_at() <= deadline_timer::traits_type::now())
	{
		// The deadline has passed. The socket is closed so that any outstanding
		// asynchronous operations are cancelled.
		socket.close();

		// There is no longer an active deadline. The expiry is set to positive
		// infinity so that the actor takes no action until a new deadline is set.
		timer.expires_at(boost::posix_time::pos_infin);
	}

	// Put the actor back to sleep.
	timer.async_wait(boost::bind(&CloudClient::checkDeadline, this));
}

void CloudClient::sendTest()
{
	tcp::resolver resolver(io_service);
	tcp::resolver::query query(this->host, boost::lexical_cast< std::string >(this->port));
	auto endpoint_iterator = resolver.resolve(query);

	timer.expires_from_now(boost::posix_time::seconds(2));

	boost::asio::async_connect(socket, endpoint_iterator, boost::bind(&CloudClient::handleConnect,this,_1));

	// TODO: or:
	//socket.async_connect(*endpoint_iterator, boost::bind(&CloudClient::handleConnect,this,_1));

	timer.async_wait(boost::bind(&CloudClient::checkDeadline, this));

	std::cout << "[CloudClient] Trying to connect to " << endpoint_iterator->endpoint() << " ..." << std::endl;
}
