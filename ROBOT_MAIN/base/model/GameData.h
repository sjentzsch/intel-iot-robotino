/*
 * GameData.h
 *
 *  Created on: Mar 27, 2013
 *      Author: root
 */

#ifndef GAMEDATA_H_
#define GAMEDATA_H_

#include <iostream>
#include <string>
#include <vector>

using namespace std;

struct Time
{
	unsigned long long sec;
	unsigned long long nsec;

	bool isNewerThan(Time otherTime)
	{
		if(this->sec > otherTime.sec || (this->sec == otherTime.sec && this->nsec > otherTime.nsec))
			return true;

		return false;
	}
};

struct Pose2D
{
	Time timestamp;
	float x;
	float y;
	float ori;	// in rad
};

struct BeaconSignal
{
	Time time;	// time in UTC
	unsigned long long seq;
	string team_name;
	string peer_name;
};

struct BeaconStation
{
	Time timeLastContact;	// local time in UTC
	unsigned long long seq;
	string team_name;
	string peer_name;
	Pose2D peer_odometry;
};

struct CommunicationInfo
{
	BeaconStation stations[4];
};

namespace TeamColor
{
	enum TeamColor {CYAN=0, MAGENTA};
	static const char * cTeamColor[2] = {"CYAN", "MAGENTA"};
}

struct ExplorationMachine
{
	string name;
	Pose2D pose;
	TeamColor::TeamColor color;
};


namespace LightColor
{
	enum LightColor {RED=0, YELLOW, GREEN};
	static const char * cLightColor[3] = {"RED", "YELLOW", "GREEN"};
}

namespace LightState
{
	enum LightState {OFF=0, ON, BLINK};
	static const char * cLightState[3] = {"OFF", "ON", "BLINK"};
}

struct LightSpec
{
	LightColor::LightColor color;
	LightState::LightState state;
};

struct ExplorationSignal
{
	string type;
	vector<LightSpec> lights;
};

struct ExplorationInfo
{
	vector<ExplorationSignal> explSignals;
	vector<ExplorationMachine> machines;
};

namespace State
{
	enum State {INIT=0, WAIT_START, RUNNING, PAUSED};
	static const char * cState[4] = {"INIT", "WAIT_START", "RUNNING", "PAUSED"};
}

namespace Phase
{
	enum Phase {PRE_GAME=0, SETUP=10, EXPLORATION=20, PRODUCTION=30, POST_GAME=40};
	static const char * cPhase[5] = {"PRE_GAME", "SETUP", "EXPLORATION", "PRODUCTION", "POST_GAME"};
}

struct GameState
{
	Time game_time;
	State::State state;
	Phase::Phase phase;
	unsigned int points_cyan;
	unsigned int points_magenta;
};

struct Machine
{
	string name;
	string type;
	TeamColor::TeamColor color;
};

struct MachineInfo
{
	vector<Machine> machines;
};

struct MachineReportEntry
{
	string name;
	string type;
};

struct MachineReport
{
	vector<MachineReportEntry> machines;
};

struct MachineReportInfo
{
	vector<string> reported_machines;
};

namespace ProductType
{
	enum ProductType {P1=1, P2, P3};
	static const char * cProductType[3] = {"P1", "P2", "P3"};
}

struct Order
{
	ProductType::ProductType product;
	unsigned int quantity_requested;
	unsigned int quantity_delivered;
	unsigned int delivery_period_begin;
	unsigned int delivery_period_end;
};

struct OrderInfo
{
	vector<Order> orders;
};

struct GameData
{
	CommunicationInfo communicationInfo;
	ExplorationInfo explorationInfo;
	GameState gameState;
	MachineInfo machineInfo;
	MachineReportInfo machineReportInfo;
	OrderInfo orderInfo;
};

#endif /* GAMEDATA_H_ */
