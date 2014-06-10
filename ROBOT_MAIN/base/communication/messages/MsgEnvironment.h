/*
 * MsgEnvironment.h
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#ifndef MSGENVIRONMENT_H_
#define MSGENVIRONMENT_H_

#include <string>

using namespace std;

class MsgEnvironment
{
public:
	MsgEnvironment();
	virtual ~MsgEnvironment();

	void load(::std::stringstream msg);
	void print();

	string message;
	unsigned long time_start;
	double x_max;
	double y_max;
	double x_robot;
	double y_robot;
	double phi_robot;
	double x_base;
	double y_base;
	double phi_base;
};


#endif /* MSGENVIRONMENT_H_ */
