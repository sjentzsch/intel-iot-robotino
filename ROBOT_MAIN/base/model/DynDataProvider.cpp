/*
 * DynDataProvider.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: root
 */

#include "DynDataProvider.h"

DynDataProvider* DynDataProvider::dynDataProvider = NULL;

DynDataProvider::DynDataProvider() {
	imageData[0] = ImageData(1);
	imageData[1] = ImageData(2);
	imageData[2] = ImageData(3);

	laserScannerData[0] = LaserScannerData(1);
	laserScannerData[1] = LaserScannerData(2);
	laserScannerData[2] = LaserScannerData(3);

	stateInformation[0] = StateInformation(1);
	stateInformation[1] = StateInformation(2);
	stateInformation[2] = StateInformation(3);
}

DynDataProvider::~DynDataProvider() {

}

DynDataProvider* DynDataProvider::getInstance(){
	if(dynDataProvider==NULL){
		dynDataProvider= new DynDataProvider();
	}
	return dynDataProvider;
}

void DynDataProvider::addImageDataListener(Observable* observable,ID::ID id){
	imageDataListener[(int)id-1] = observable;
}

Observable* DynDataProvider::getImageDataListener(ID::ID id){
	return imageDataListener[(int)id-1];
}

void DynDataProvider::addStateInformationListener(Observable* observable,ID::ID id){
	stateInformationListener[(int)id-1] = observable;
}

Observable* DynDataProvider::getStateInformationListener(ID::ID id){
	return stateInformationListener[(int)id-1];
}

void DynDataProvider::addStateInformationListenerForSim(Observable* observable,ID::ID id){
	stateInformationListenerForSim[(int)id-1] = observable;
}

Observable* DynDataProvider::getStateInformationListenerForSim(ID::ID id){
	return stateInformationListenerForSim[(int)id-1];
}

ImageData* DynDataProvider::getImageData(ID::ID id){
	if(id!=ID::SERVER){
		return &imageData[(int)id-1];
	}

	return NULL;
}

LaserScannerData* DynDataProvider::getLaserScannerData(ID::ID id){
	if(id!=ID::SERVER){
		return &laserScannerData[(int)id-1];
	}

	return NULL;
}

StateInformation* DynDataProvider::getStateInformation(ID::ID id){
	if(id!=ID::SERVER){
		return &stateInformation[(int)id-1];
	}
	return NULL;
}
