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

void ModelProvider::setRefboxPeer(RefboxPeer* refboxPeer_)
{
	this->refboxPeer = refboxPeer_;
}

RefboxPeer* ModelProvider::getRefboxPeer()
{
	return this->refboxPeer;
}

void ModelProvider::kickRobot(ID::ID id)
{
	if(getHWID() == id)
	{
		FileLog::log(log_Communication, "Asked to kick myself -> Not following this order");
		FileLog::log_NOTICE("Asked to kick myself -> Not following this order");
		return;
	}

	unsigned int index = (unsigned int)id-1;

	// TODO: check if this prevents reenabling robots by the RefboxPeer which were kicked by trying to connect to them
	// this clears the current/last BeaconStation-Information
	mutexCommunicationInfo.lock();
	getGameData()->communicationInfo.stations[index+1] = BeaconStation();
	mutexCommunicationInfo.unlock();

	comDataObject.enabled[index] = false;

	FileLog::log(log_Communication, "Kicked Robot ", FileLog::integer(index+1), " [new ComData-Enabled-Flags: ", FileLog::integer(comDataObject.enabled[0]), " ", FileLog::integer(comDataObject.enabled[1]), " ", FileLog::integer(comDataObject.enabled[2]), " ", FileLog::integer(comDataObject.server_enabled), "]");
	FileLog::log_NOTICE("Kicked Robot ", FileLog::integer(index+1), " [new ComData-Enabled-Flags: ", FileLog::integer(comDataObject.enabled[0]), " ", FileLog::integer(comDataObject.enabled[1]), " ", FileLog::integer(comDataObject.enabled[2]), " ", FileLog::integer(comDataObject.server_enabled), "]");
}

void ModelProvider::clearWorldModelFromRobot(ID::ID id)
{
	for(int x=0; x<X_GRID; x++){
		for(int y=0; y<Y_GRID; y++){
			if(worldModel.node[x][y].occupied == (int)id){
				worldModel.node[x][y].occupied = 0;
			}
		}
	}

	for(int u=0; u<NUMBER_OF_POIS; u++){
		if(worldModel.poi[u].occupied == (int)id){
			worldModel.poi[u].occupied = 0;
			/*if(worldModel.poi[i].type!=POIType::INPUT && worldModel.poi[i].type!=POIType::DELIVER && worldModel.poi[i].type!=POIType::LOAREA){
				worldModel.poi[i].status= POIStatus::OFFLINE;
			}*/
		}
	}

	FileLog::log(log_Communication, "Cleared WorldModel from Robot ", FileLog::integer((int)id));
	FileLog::log_NOTICE("Cleared WorldModel from Robot ", FileLog::integer((int)id));
}

void ModelProvider::enableRobot(ID::ID id)
{
	unsigned int index = (unsigned int)id-1;

	comDataObject.enabled[index] = true;

	FileLog::log(log_Communication, "Enabled Robot ", FileLog::integer(index+1), " [new ComData-Enabled-Flags: ", FileLog::integer(comDataObject.enabled[0]), " ", FileLog::integer(comDataObject.enabled[1]), " ", FileLog::integer(comDataObject.enabled[2]), " ", FileLog::integer(comDataObject.server_enabled), "]");
	FileLog::log_NOTICE("Enabled Robot ", FileLog::integer(index+1), " [new ComData-Enabled-Flags: ", FileLog::integer(comDataObject.enabled[0]), " ", FileLog::integer(comDataObject.enabled[1]), " ", FileLog::integer(comDataObject.enabled[2]), " ", FileLog::integer(comDataObject.server_enabled), "]");
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

void ModelProvider::setID(ID::ID id){
	ID = id;
}

ID::ID ModelProvider::getID(){
	return ID;
}

void ModelProvider::setHWID(ID::ID id){
	HWID = id;
}

ID::ID ModelProvider::getHWID(){
	return HWID;
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

bool ModelProvider::isEqualLight(CameraLightState::CameraLightState tbuLight, vector<LightSpec> gdLight)
{
	//	cout << "tbuLight: "<< tbuLight << endl;
	//	for(int i=0;i<3;i++){
	//		cout << "gdLight: color: "<< gdLight.at(i).color << " state: " << gdLight.at(i).state <<endl;
	//	}
	if(gdLight.at(0).state == LightState::ON)
	{
		if(gdLight.at(1).state == LightState::ON)
		{
			if(gdLight.at(2).state == LightState::ON)
				return tbuLight == CameraLightState::RED_YELLOW_GREEN;
			else
				return tbuLight == CameraLightState::RED_YELLOW;
		}
		else
		{
			if(gdLight.at(2).state == LightState::ON)
				return tbuLight == CameraLightState::RED_GREEN;
			else
				return tbuLight == CameraLightState::RED;
		}
	}
	else
	{
		if(gdLight.at(1).state == LightState::ON)
		{
			if(gdLight.at(2).state == LightState::ON)
				return tbuLight == CameraLightState::YELLOW_GREEN;
			else
				return tbuLight == CameraLightState::YELLOW;
		}
		else
		{
			if(gdLight.at(2).state == LightState::ON)
				return tbuLight == CameraLightState::GREEN;
			else
			{
				if(gdLight.at(0).state == LightState::BLINK)
					return tbuLight == CameraLightState::RED_FLASH;
				else if(gdLight.at(1).state == LightState::BLINK)
					return tbuLight == CameraLightState::YELLOW_FLASH;
				else
					return tbuLight == CameraLightState::OFFLINE;
			}
		}
	}
}
