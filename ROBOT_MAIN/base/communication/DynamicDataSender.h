/*
 * DynamicDataSender.h
 *
 *  Created on: Jun 24, 2011
 *      Author: root
 */

#ifndef DYNAMICDATASENDER_H_
#define DYNAMICDATASENDER_H_
#include <boost/thread.hpp>

class DynamicDataSender {
private:
	boost::thread* execThread;
	void runExec();
public:
	DynamicDataSender();
	void start();
	virtual ~DynamicDataSender();
};

#endif /* DYNAMICDATASENDER_H_ */
