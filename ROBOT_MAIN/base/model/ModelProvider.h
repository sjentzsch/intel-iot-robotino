/*
 * ModelProvider.h
 *
 *  Created on: May 27, 2011
 *      Author: root
 */

#ifndef MODELPROVIDER_H_
#define MODELPROVIDER_H_

#include <boost/thread.hpp>
#include <ostream>
#include <fstream>
#include "../utils/FileLogger.h"

#include "WorldModel.h"
#include "ComDataObject.h"
#include "GameData.h"
#include "Observable.h"
#include "RobotCameraEnums.h"

using namespace std;

class RefboxPeer;

class ModelProvider {
private:
	WorldModel worldModel;
	ComDataObject comDataObject;
	GameData gameData;

	Observable* worldModelListener;
	Observable* commandListener;

	RefboxPeer* refboxPeer;

	static ModelProvider* modelProvider;
	ID::ID ID;
	ID::ID HWID;

public:
	static ModelProvider* getInstance();

	WorldModel* getWorldModel();
	ComDataObject* getComDataObject();

	// following block should be used from the readers/users of GameData
	CommunicationInfo getLatestCommunicationInfo();
	ExplorationInfo getLatestExplorationInfo();
	GameState getLatestGameState();
	MachineInfo getLatestMachineInfo();
	MachineReportInfo getLatestMachineReportInfo();
	OrderInfo getLatestOrderInfo();

	bool isEqualLight(CameraLightState::CameraLightState tbuLight, vector<LightSpec> gdLight);

	// returns whether command is pause or refbox send paused state
	bool gameStateIsPaused();

	Time getGlobalGameTime();

	// following block should only be used from the writer class RefboxPeer
	GameData* getGameData();
	boost::shared_mutex mutexCommunicationInfo;
	boost::shared_mutex mutexExplorationInfo;
	boost::shared_mutex mutexGameState;
	boost::shared_mutex mutexMachineInfo;
	boost::shared_mutex mutexMachineReportInfo;
	boost::shared_mutex mutexOrderInfo;

	// following mutex should only be used from WorldModelClient and WorldModelServer
	boost::mutex wmMutex;

	void setRefboxPeer(RefboxPeer* refboxPeer_);
	RefboxPeer* getRefboxPeer();

	void saveWorldModel(const char* path);
	void loadWorldModel(const char* path);

	void saveComDataObject(const char* path);
	void loadComDataObject(const char* path);

	void addWorldModelChangeListener(Observable* observable);
	Observable* getWorldModelChangeListener();
	Observable* getCommandListener();

	void kickRobot(ID::ID id);
	void clearWorldModelFromRobot(ID::ID id);
	void enableRobot(ID::ID id);

	ID::ID getID();
	void setID(ID::ID id);
	ID::ID getHWID();
	void setHWID(ID::ID id);

protected:
	ModelProvider();
	virtual ~ModelProvider();
};

#endif /* MODELPROVIDER_H_ */
