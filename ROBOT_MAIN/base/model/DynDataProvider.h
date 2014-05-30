/*
 * DynDataProvider.h
 *
 *  Created on: Jun 16, 2011
 *      Author: root
 */

#ifndef DYNDATAPROVIDER_H_
#define DYNDATAPROVIDER_H_

#include <string>

#include "Observable.h"
#include "ImageData.h"
#include "LaserScannerData.h"
#include "StateInformation.h"
#include "ModelProvider.h"

class DynDataProvider {
private:
	ImageData imageData[3];
	LaserScannerData laserScannerData[3];
	StateInformation stateInformation[3];
	Observable* imageDataListener[3];
	Observable* stateInformationListener[3];
	Observable* stateInformationListenerForSim[3];

	static DynDataProvider* dynDataProvider;

public:
	static DynDataProvider* getInstance();

	void addImageDataListener(Observable* observable,ID::ID id);
	Observable* getImageDataListener(ID::ID id);

	void addStateInformationListener(Observable* observable,ID::ID id);
	Observable* getStateInformationListener(ID::ID id);

	void addStateInformationListenerForSim(Observable* observable,ID::ID id);
	Observable* getStateInformationListenerForSim(ID::ID id);

	ImageData* getImageData(ID::ID id);
	LaserScannerData* getLaserScannerData(ID::ID id);
	StateInformation* getStateInformation(ID::ID id);
protected:
	DynDataProvider();
	virtual ~DynDataProvider();
};

#endif /* DYNDATAPROVIDER_H_ */
