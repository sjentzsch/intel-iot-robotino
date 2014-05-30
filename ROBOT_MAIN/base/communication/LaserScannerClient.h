/*
 * LaserScannerClient.h
 *
 *  Created on: Mar 31, 2014
 *      Author: root
 */

#ifndef LASERSCANNERCLIENT_H_
#define LASERSCANNERCLIENT_H_

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "Communication.h"
#include "../model/ModelProvider.h"
#include "../model/DynDataProvider.h"

using boost::asio::ip::udp;

class LaserScannerClient {
public:
	LaserScannerClient();
	virtual ~LaserScannerClient();
	void sendLaserScannerData();
};

#endif /* LASERSCANNERCLIENT_H_ */
