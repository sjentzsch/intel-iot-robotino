/*
 * Communication.h
 *
 *  Created on: Apr 24, 2011
 *      Author: peter
 */

#ifndef CLOUDCOMM_H_
#define CLOUDCOMM_H_

#include "CloudClient.h"
#include "CloudServer.h"

class CloudComm
{
public:
	static CloudComm* getInstance();

	CloudServer* getCloudServer();
	CloudClient* getCloudClient();

protected:
	CloudComm();
	virtual ~CloudComm();

private:
	CloudServer* cloudServer;
	CloudClient* cloudClient;

	static CloudComm* cloudComm;
};

#endif /* CLOUDCOMM_H_ */
