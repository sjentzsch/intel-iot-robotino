/*
 * StateInformation.h
 *
 *  Created on: Jun 12, 2011
 *      Author: root
 */

#ifndef STATEINFORMATION_H_
#define STATEINFORMATION_H_

struct StateInformation {
	int id;
	float x,y,phi;
	char description[50];

	StateInformation(){};
	StateInformation(int id_):id(id_){x=0; y=0; phi=0;};
};

#endif /* STATEINFORMATION_H_ */
