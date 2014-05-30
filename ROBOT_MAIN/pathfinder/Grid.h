#pragma once

#include "model/ModelProvider.h"
#include "model/WorldModel.h"
#include <list>
#include "../LSPBTrajectory/pose.h"
#include <set>
#include <string>
#include <sstream>

class Node;
class ExpandedNode;

using namespace std;

class Grid
{
private:
	Node* grid[X_GRID][Y_GRID];

public:
	Node* getAccessNode(POI* poi, POIAccessFrom::POIAccessFrom accDir=POIAccessFrom::FRONT);
	POIDirection::POIDirection getAccessDirection(POI* poi, POIAccessFrom::POIAccessFrom accDir);

	list<ExpandedNode>* normalizePath(list<ExpandedNode>* path);
	Grid();
	~Grid();
	Node* getNode(int x, int y);
	Node* getNodeByCoord(vec2D &pos);
	Node* getNodeByCoord(float x, float y);
	void clear(Node* currNode); //sets all occupied by robotino to zero except the one where he is standing now
	void clearAll();
	list<ExpandedNode>* getPath(Node* startNode,POIDirection::POIDirection startDirection, Node* goalNode, POIDirection::POIDirection goalDirection, const vector<Node*> blacklist = vector<Node*>());
	list<ExpandedNode>* getNormalizedPath(Node* startNode,POIDirection::POIDirection startDirection, Node* goalNode, POIDirection::POIDirection goalDirection);
	ExpandedNode getNextRelevantNode(list<ExpandedNode>* path);
	set<Node *> giveNodesInBetween(Node *start, Node *end);

};

class Node
{
private:
	float x_pos;
	float y_pos;
	int x, y;
	Grid* grid;
	bool blocked;
public:
	Node(float x_pos, float y_pos,int x, int y,Grid* grid,bool blocked);
	~Node();

	Node* getNeighbor(POIDirection::POIDirection dir);
	Node* getNeighbor(POIDirection::POIDirection dir, bool &isRegular);
	Node** getNeighbors();
	int getX();
	int getY();
	float getXPos();
	float getYPos();
	bool isBlocked(); //initialized value for nodes that are blocked by machines and input zone
	NodeRaw* getNodeRaw(); //returns the associated blocked value from the world model
	string toString();
};


class ExpandedNode {

private:
	Node* node;
	POIDirection::POIDirection dir;
	ExpandedNode* parent;

public:
	ExpandedNode(Node* node, POIDirection::POIDirection dir);
	ExpandedNode(Node* node, POIDirection::POIDirection dir,ExpandedNode* parent);
	~ExpandedNode();
	list<ExpandedNode*> expand();
	Node* getNode();
	POIDirection::POIDirection getDirection();
	float getPhi();
	ExpandedNode* getParent();
	string toString();
};

POIDirection::POIDirection getDirectionFromAngle(float phi);
