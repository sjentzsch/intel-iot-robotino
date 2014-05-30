/*
 * StateInformationClient.h
 *
 *  Created on: Jun 9, 2011
 *      Author: root
 */

#ifndef STATEINFORMATIONCLIENT_H_
#define STATEINFORMATIONCLIENT_H_
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "Communication.h"
#include "../model/DynDataProvider.h"

using boost::asio::ip::udp;

class StateInformationClient {
public:
	StateInformationClient();
	virtual ~StateInformationClient();
	void sendStateInformation();
};

#endif /* STATEINFORMATIONCLIENT_H_ */
