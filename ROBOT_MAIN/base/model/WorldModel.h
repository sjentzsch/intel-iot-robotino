/*
 * WorldModel.h
 *
 *  Created on: Apr 17, 2011
 *      Author: peter
 */

#ifndef WORLDMODEL_H_
#define WORLDMODEL_H_

#include "GameData.h"
#include "../config.h"

#define X_GRID 9 //Felder in X-Richtung
#define Y_GRID 19 //Felder in Y-Richtung
#define Y_GRID_WIDTH 560.0 //distance between two node center points in the grid in Y-direction
#define X_GRID_WIDTH 560.0 //distance between two node center points in the grid in X-direction
#define X_OFFSET 280.0 //OFFSET where 1 Grid point starts in X-direction
#define Y_OFFSET 280.0 //OFFSET where 1 Grid point starts in Y-direction

#define NUMBER_OF_POIS 32

namespace POIStatus
{
	enum POIStatus {UNKNOWN=0, READY, PROCESSING, OUT_OF_ORD_t, OUT_OF_ORDER_DURING_PROCESSING_t, OFFLINE};
	static const char * cPOIStatus[6] = {"UNKNOWN", "READY", "PROCESSING", "OUT_OF_ORD_t", "OUT_OF_ORDER_DURING_PROCESSING_t", "OFFLINE"};
}

namespace POIRequiredNext
{
	enum POIRequiredNext {S0=0, S1, S2, P1, P2, P3, RECYCLE, NON_EXISTING, CONSUMED};
	static const char * cPOIRequiredNext[9] = {"S0", "S1", "S2", "P1", "P2", "P3", "RECYCLE", "NON_EXISTING", "CONSUMED"};
}

namespace POIType
{
	enum POIType {UNKNOWN=0, T1, T2, T3, T4, T5, RECYCLE, DELIVER, INPUT};
	static const char * cPOIType[9] = {"UNKNO.", "T1", "T2", "T3", "T4", "T5", "RECYC.", "DELIVER", "INPUT"};
}

namespace POIAccessFrom
{
	enum POIAccessFrom {LEFT=0, FRONT, RIGHT, BACK};
	static const char * cPOIAccessFrom[4] = {"LEFT", "FRONT", "RIGHT", "BACK"};
}

namespace POIDirection
{
	enum POIDirection {NORTH=0, EAST, SOUTH, WEST};
	static const char *cPOIDirection[4] = {"NORTH", "EAST", "SOUTH", "WEST"};
}

namespace Puck
{
	enum Puck {UNKNOWN=0, NON_EXISTING, S0, S1, S2, P1, P2, P3, CONSUMED_IN_USE, CONSUMED_FREE_TO_RECYCLE_AT_T4, CONSUMED_FREE_TO_RECYCLE_AT_T3, CONSUMED_FREE_TO_RECYCLE_AT_T2};
	static const char * cPuck[12] = {"UNKNOWN", "NON_EXISTING", "S0", "S1", "S2", "P1", "P2", "P3", "CONSUMED_IN_USE", "CONSUMED_FREE_TO_RECYCLE_AT_T4", "CONSUMED_FREE_TO_RECYCLE_AT_T3", "CONSUMED_FREE_TO_RECYCLE_AT_T2"};
}

namespace LampStatus
{
	enum LampStatus {DUMMIE=0, UNKNOWN, OFFLINE, RED, YELLOW, YELLOW_FLASH, GREEN, RED_YELLOW, RED_GREEN, YELLOW_GREEN, RED_YELLOW_GREEN};
	static const int nLampStatus = 11;
	static const char * cLampStatus[nLampStatus] = {"DUMMIE", "UNKNOWN", "OFFLINE", "RED", "YELLOW", "YELLOW_FLASH", "GREEN", "RED_YELLOW", "RED_GREEN", "YELLOW_GREEN", "RED_YELLOW_GREEN"};
}

struct POI {
	int index;
	int x, y; // coordinates in grid, relevant for all POIs
	bool mine;	// true if belongs to my team
	POIDirection::POIDirection dir; // direction the machine is pointing to, hence opposite direction of the access direction
	POIType::POIType type;

	POIStatus::POIStatus status;
	Puck::Puck left, middle, right;
	int occupied;
	Time timestamp; // time when POI is available again
	bool isVisibleForLaser; //true if poi can be seen by laser scanner

	POI() {}

	void init(int index_, int x_, int y_, bool mine_, POIDirection::POIDirection dir_, POIType::POIType type_)
	{
		index = index_;
		x = x_;
		y = y_;
		mine = mine_;
		dir = dir_;
		type = type_;

		status = POIStatus::READY;
		left = Puck::NON_EXISTING;
		middle =  Puck::NON_EXISTING;
		right =  Puck::NON_EXISTING;
		occupied = 0;
		timestamp.sec = 0;	// TODO: check what goes wrong if this is not initialized with ULLONG_MAX
		timestamp.nsec = 0;
		isVisibleForLaser = true;

		if(type == POIType::INPUT)
		{
			middle = Puck::S0;
		}
	}
};

struct NodeRaw {
	int occupied;
	Time timestamp; // time when this node will most likely be available again

	NodeRaw()
	{
		occupied = 0;
		timestamp.sec = 0;
		timestamp.nsec = 0;
	}
};

struct WorldModel {
	int version;
	bool updatedTeamDistribution;
	float inputyLeft;
	float inputyRight;
	int pucksGrabbed;
	NodeRaw node[X_GRID][Y_GRID];
	POI poi[NUMBER_OF_POIS];
	bool updatedMachineTypes;
	int T5prodTime;

	WorldModel(){
		version = 0;
		updatedTeamDistribution = false;
		inputyLeft = 0;
		inputyRight = 0;
		pucksGrabbed = 0;
		updatedMachineTypes = false;
		T5prodTime = 0;

		// LEFT AREA

		// Production Machines
		poi[0].init(1,2,8,false,POIDirection::NORTH,POIType::UNKNOWN);
		poi[1].init(2,4,8,false,POIDirection::SOUTH,POIType::UNKNOWN);

		poi[2].init(3,2,6,false,POIDirection::WEST,POIType::UNKNOWN);
		poi[3].init(4,4,6,false,POIDirection::EAST,POIType::UNKNOWN);
		poi[4].init(5,6,6,false,POIDirection::EAST,POIType::UNKNOWN);

		poi[5].init(6,2,4,false,POIDirection::SOUTH,POIType::UNKNOWN);
		poi[6].init(7,6,4,false,POIDirection::NORTH,POIType::UNKNOWN);
		poi[7].init(8,8,4,false,POIDirection::EAST,POIType::UNKNOWN);

		poi[8].init(9,2,2,false,POIDirection::WEST,POIType::UNKNOWN);
		poi[9].init(10,4,2,false,POIDirection::SOUTH,POIType::UNKNOWN);
		poi[10].init(11,8,2,false,POIDirection::NORTH,POIType::UNKNOWN);

		poi[11].init(12,8,0,false,POIDirection::NORTH,POIType::UNKNOWN);

		// Recycling Machine
		poi[12].init(25,8,8,false,POIDirection::WEST,POIType::RECYCLE);

		// Puck Input Zone
		poi[13].init(27,0,5,false,POIDirection::SOUTH,POIType::INPUT);
		poi[13].isVisibleForLaser = false;
		poi[14].init(28,0,7,false,POIDirection::SOUTH,POIType::INPUT);
		poi[14].isVisibleForLaser = false;

		// Deliver Zone
		poi[15].init(31,4,0,false,POIDirection::EAST,POIType::DELIVER);
		poi[15].isVisibleForLaser = false;



		// RIGHT AREA

		// Production Machines
		poi[16].init(13,2,10,false,POIDirection::NORTH,POIType::UNKNOWN);
		poi[17].init(14,4,10,false,POIDirection::SOUTH,POIType::UNKNOWN);

		poi[18].init(15,2,12,false,POIDirection::EAST,POIType::UNKNOWN);
		poi[19].init(16,4,12,false,POIDirection::WEST,POIType::UNKNOWN);
		poi[20].init(17,6,12,false,POIDirection::WEST,POIType::UNKNOWN);

		poi[21].init(18,2,14,false,POIDirection::SOUTH,POIType::UNKNOWN);
		poi[22].init(19,6,14,false,POIDirection::NORTH,POIType::UNKNOWN);
		poi[23].init(20,8,14,false,POIDirection::WEST,POIType::UNKNOWN);

		poi[24].init(21,2,16,false,POIDirection::EAST,POIType::UNKNOWN);
		poi[25].init(22,4,16,false,POIDirection::SOUTH,POIType::UNKNOWN);
		poi[26].init(23,8,16,false,POIDirection::NORTH,POIType::UNKNOWN);

		poi[27].init(24,8,18,false,POIDirection::NORTH,POIType::UNKNOWN);

		// Recycling Machine
		poi[28].init(26,8,10,false,POIDirection::EAST,POIType::RECYCLE);

		// Puck Input Zone
		poi[29].init(29,0,11,false,POIDirection::SOUTH,POIType::INPUT);
		poi[29].isVisibleForLaser = false;
		poi[30].init(30,0,13,false,POIDirection::SOUTH,POIType::INPUT);
		poi[30].isVisibleForLaser = false;

		// Deliver Zone
		poi[31].init(32,4,18,false,POIDirection::WEST,POIType::DELIVER);
		poi[31].isVisibleForLaser = false;
	}
};

#endif /* WORLDMODEL_H_ */
