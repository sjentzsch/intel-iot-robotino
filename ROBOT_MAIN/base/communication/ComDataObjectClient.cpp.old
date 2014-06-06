/*
 * ComDataObjectClient.cpp
 *
 *  Created on: May 30, 2011
 *      Author: root
 */

#include "ComDataObjectClient.h"

ComDataObjectClient::ComDataObjectClient(boost::asio::io_service& io_service, int i_):socket(io_service),timer(io_service),i(i_) {
	comDataObject = ModelProvider::getInstance()->getComDataObject();
	closed = false;
}

ComDataObjectClient::~ComDataObjectClient() {
	// TODO: check
	//if(socket.is_open())
	//	socket.close();
}

void ComDataObjectClient::handleConnect(const boost::system::error_code& ec){
	 	timer.cancel();
		if (!socket.is_open())
		{
		   std::cout << "[ComDataClient] Connect timed out\n";
		}
		else if (ec)
		{
		  std::cout << "[ComDataClient] Connect error: " << ec.message() << "\n";
		  socket.close();

		}
		else
		{
		  //std::cout << "[ComDataClient] Connected \n";


		  //boost::array<char, sizeof(ComDataObject)> buf;
		  //buf.assign(static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getComDataObject())));

		  char* byte_array = static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getComDataObject()));


		  boost::system::error_code ignored_error;
		  boost::asio::write(socket, boost::asio::buffer(byte_array,sizeof(ComDataObject)),
			   boost::asio::transfer_all(), ignored_error);

		  socket.close();
		}
		closed = true;
	}

void ComDataObjectClient::check_deadline()
{
  if (closed)
    return;

  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (timer.expires_at() <= deadline_timer::traits_type::now())
  {
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled.
	socket.close();

    // There is no longer an active deadline. The expiry is set to positive
    // infinity so that the actor takes no action until a new deadline is set.
	timer.expires_at(boost::posix_time::pos_infin);
  }

  // Put the actor back to sleep.
  timer.async_wait(boost::bind(&ComDataObjectClient::check_deadline, this));
}


void ComDataObjectClient::sendComDataObject(){
	std::stringstream* str = new std::stringstream("");

	for(int j=0;j<3;j++){
		*str<<(int)comDataObject->robot_address[j];
		*str<<".";
	}
	*str<<(int)comDataObject->robot_address[3];
	std::string tmpAddress(str->str().c_str());

	boost::asio::ip::address tempAdd;
	tempAdd = boost::asio::ip::address::from_string(tmpAddress.c_str()/*addressString.c_str()*/);
	tcp::endpoint endpoint(tempAdd,comDataObject->client_ports+100);
	delete str;
	//std::cout << "[ComDataClient] Trying to connect to " << endpoint << "...\n";

	timer.expires_from_now(boost::posix_time::seconds(2));
	socket.async_connect(endpoint, boost::bind(&ComDataObjectClient::handleConnect,this,_1));
	timer.async_wait(boost::bind(&ComDataObjectClient::check_deadline, this));
}
