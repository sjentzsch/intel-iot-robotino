/*
 * ImageClient.h
 *
 *  Created on: Jun 9, 2011
 *      Author: root
 */

#ifndef IMAGECLIENT_H_
#define IMAGECLIENT_H_
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "Communication.h"
#include "../model/ModelProvider.h"
#include "../model/DynDataProvider.h"

using boost::asio::ip::udp;

class ImageClient {
public:
	ImageClient();
	virtual ~ImageClient();
	void sendImageData();
};

#endif /* IMAGECLIENT_H_ */
