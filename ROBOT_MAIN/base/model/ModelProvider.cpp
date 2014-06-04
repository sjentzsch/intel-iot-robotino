/*
 * ModelProvider.cpp
 *
 *  Created on: May 27, 2011
 *      Author: root
 */

#include "ModelProvider.h"

ModelProvider* ModelProvider::modelProvider = NULL;

ModelProvider::ModelProvider() {
	worldModel = WorldModel();
	comDataObject = ComDataObject();
	gameData = GameData();
}

ModelProvider::~ModelProvider() {

}

ModelProvider* ModelProvider::getInstance(){
	if(modelProvider==NULL){
		modelProvider= new ModelProvider();
	}
	return modelProvider;
}

WorldModel* ModelProvider::getWorldModel(){
	return &worldModel;
}

ComDataObject* ModelProvider::getComDataObject(){
	return &comDataObject;
}

GameData* ModelProvider::getGameData(){
	return &gameData;
}

CommunicationInfo ModelProvider::getLatestCommunicationInfo(){
	boost::shared_lock<boost::shared_mutex> r_lock(mutexCommunicationInfo);
	return gameData.communicationInfo;	// TODO: check if this is really a deep copy
}

ExplorationInfo ModelProvider::getLatestExplorationInfo(){
	boost::shared_lock<boost::shared_mutex> r_lock(mutexExplorationInfo);
	return gameData.explorationInfo;	// TODO: check if this is really a deep copy
}

GameState ModelProvider::getLatestGameState(){
	boost::shared_lock<boost::shared_mutex> r_lock(mutexGameState);
	return gameData.gameState;	// TODO: check if this is really a deep copy
}

MachineInfo ModelProvider::getLatestMachineInfo(){
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMachineInfo);
	return gameData.machineInfo;	// TODO: check if this is really a deep copy
}

MachineReportInfo ModelProvider::getLatestMachineReportInfo(){
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMachineReportInfo);
	return gameData.machineReportInfo;	// TODO: check if this is really a deep copy
}

OrderInfo ModelProvider::getLatestOrderInfo(){
	boost::shared_lock<boost::shared_mutex> r_lock(mutexOrderInfo);
	return gameData.orderInfo;	// TODO: check if this is really a deep copy
}

bool ModelProvider::gameStateIsPaused(){
	// no mutex here! yes, baby!
	return gameData.gameState.state == State::PAUSED || comDataObject.command == COMMAND::PAUSE;
}

Time ModelProvider::getGlobalGameTime()
{
	GameState currGameState = getLatestGameState();
	switch(currGameState.phase)
	{
	case Phase::PRE_GAME:
	case Phase::EXPLORATION:
		return currGameState.game_time;
	case Phase::PRODUCTION:
	case Phase::POST_GAME:
	default:
		currGameState.game_time.sec += 180;
		return currGameState.game_time;
	}
}

void ModelProvider::saveWorldModel(const char* path){
	ofstream ofs(path, ios::binary);
	ofs.write((char *)&worldModel, sizeof(WorldModel));
}

void ModelProvider::loadWorldModel(const char* path){
	ifstream ifs(path, ios::binary);
	ifs.read((char *)&worldModel, sizeof(WorldModel));
}

void ModelProvider::saveComDataObject(const char* path){
	ofstream ofs(path, ios::binary);
	ofs.write((char *)&comDataObject, sizeof(ComDataObject));
}

void ModelProvider::loadComDataObject(const char* path){
	ifstream ifs(path, ios::binary);
	ifs.read((char *)&comDataObject, sizeof(ComDataObject));
}

void ModelProvider::addWorldModelChangeListener(Observable* observable){
	worldModelListener = observable;
}

Observable* ModelProvider::getWorldModelChangeListener(){
	return worldModelListener;
}

Observable* ModelProvider::getCommandListener(){
	return commandListener;
}
