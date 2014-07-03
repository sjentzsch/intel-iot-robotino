//
// CloudClient.cpp
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

#include "CloudClient.h"

#include <boost/lexical_cast.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

CloudClient::CloudClient(boost::asio::io_service& io_service_, ::std::string host_, int port_)
:io_service(io_service_),socket(io_service_),timer(io_service_)
{
	this->host = host_;
	this->port = port_;

	tcp::resolver resolver(io_service);
	tcp::resolver::query query(this->host, boost::lexical_cast< std::string >(this->port));
	this->endpoint_iterator = resolver.resolve(query);
}

CloudClient::~CloudClient()
{

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

bool CloudClient::send(::std::string msg)
{
	boost::mutex::scoped_lock lock(send_mutex);

	//timer.expires_from_now(boost::posix_time::seconds(2));

	FileLog::log(log_Communication, "[CloudClient] Trying to connect to " , endpoint_iterator->endpoint().address().to_string(), ":", std::to_string(endpoint_iterator->endpoint().port()), " ...");

	//timer.async_wait(boost::bind(&CloudClient::checkDeadline, this));

	boost::system::error_code ec;
	socket.connect(this->endpoint_iterator->endpoint(), ec);

	//timer.cancel();

	if(!socket.is_open())
	{
		FileLog::log(log_Communication, "[CloudClient] Connect timed out");
		return false;
	}
	else if(ec)
	{
		FileLog::log(log_Communication, "[CloudClient] Connect error: ", ec.message());
		socket.close();
		return false;
	}
	else
	{
		FileLog::log(log_Communication, "[CloudClient] Connected. Start writing to server ...");
		boost::system::error_code ignored_error;
		boost::asio::write(socket, boost::asio::buffer(msg), boost::asio::transfer_all(), ignored_error);
		FileLog::log(log_Communication, "[CloudClient] Close socket ...");
		socket.close();
		FileLog::log(log_Communication, "[CloudClient] Socket closed.");
		return true;
	}
}
