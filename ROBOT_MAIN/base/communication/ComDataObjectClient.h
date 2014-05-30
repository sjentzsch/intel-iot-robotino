/*
 * ComDataObjectClient.h
 *
 *  Created on: May 30, 2011
 *      Author: root
 */

#ifndef COMDATAOBJECTCLIENT_H_
#define COMDATAOBJECTCLIENT_H_

#include <iostream>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "../model/ModelProvider.h"

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;


class ComDataObjectClient {
private:
	deadline_timer timer;
	tcp::socket socket;
	int i;
	ComDataObject* comDataObject;
	bool closed;
	void check_deadline();
	void handleConnect(const boost::system::error_code& ec);

public:
	ComDataObjectClient(boost::asio::io_service& io_service,int i_);
	virtual ~ComDataObjectClient();



	void sendComDataObject();
};

#endif /* COMDATAOBJECTCLIENT_H_ */
