/*
 * SimApi2Com.h
 *
 *  Created on: Apr 19, 2014
 *      Author: root
 */

#ifndef SIMAPI2COM_H_
#define SIMAPI2COM_H_

#include <iostream>
#include <rec/robotino/api2/Com.h>
#include <rec/robotino/api2/utils.h>

class SimApi2Com : public rec::robotino::api2::Com
{
public:
	virtual ~SimApi2Com();
	SimApi2Com();

	bool isConnected() const;
	void connectToServer( bool isBlocking = true );
	bool isLocalConnection() const;
	void processEvents();
};

#endif /* SIMAPI2COM_H_ */
