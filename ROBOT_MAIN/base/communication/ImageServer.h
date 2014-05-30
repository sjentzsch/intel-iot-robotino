/*
 * ImageServer.h
 *
 *  Created on: Jun 7, 2011
 *      Author: root
 */

#ifndef IMAGESERVER_H_
#define IMAGESERVER_H_
#include "../model/ImageData.h"
#include "../model/DynDataProvider.h"
#include <boost/thread.hpp>

class ImageServer {
private:
	boost::thread *exec_start;
	void handleConnections_impl();
public:
	int port;
	ImageServer(int port);
	virtual ~ImageServer();

	void handleConnections();
};

#endif /* IMAGESERVER_H_ */
