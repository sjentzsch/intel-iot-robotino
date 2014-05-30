//#pragma once
#include "Grid.h"
#include "model/ModelProvider.h"

Node::Node(float x_pos, float y_pos,int x, int y,Grid* grid,bool blocked)
	:x_pos(x_pos),y_pos(y_pos),x(x),y(y),grid(grid),blocked(blocked) {

}

Node::~Node(){

}

bool Node::isBlocked(){
	return blocked;
}

Node* Node::getNeighbor(POIDirection::POIDirection dir)
{
	bool isRegular = true;
	return getNeighbor(dir, isRegular);
}

Node* Node::getNeighbor(POIDirection::POIDirection dir, bool &isRegular){
	isRegular = true;
	switch(dir){
		case POIDirection::WEST:
			if(y>0){
				return this->grid->getNode(x,y-1);
			}else{
				isRegular = false;
				return this->grid->getNode(x,0);
			}
		case POIDirection::EAST:
			if(y<Y_GRID-1){
				return this->grid->getNode(x,y+1);
			}else{
				isRegular = false;
				return this->grid->getNode(x,Y_GRID-1);
			}
		case POIDirection::SOUTH:
			if(x<X_GRID-1){
				return this->grid->getNode(x+1,y);
			}else{
				isRegular = false;
				return this->grid->getNode(X_GRID-1,y);
			}
		case POIDirection::NORTH:
			if(x>0){
				return this->grid->getNode(x-1,y);
			}else{
				isRegular = false;
				return this->grid->getNode(0,y);
			}
		default:
			return 0;
	}
}

Node** Node::getNeighbors(){
	Node** neighbors = new Node *[4];
	for(int i=0;i<4;i++){
		neighbors[i] = this->getNeighbor((POIDirection::POIDirection)i);
	}

	return neighbors;
}

NodeRaw* Node::getNodeRaw(){
	return &(ModelProvider::getInstance()->getWorldModel()->node[x][y]);
}

/*void Node::setBlockedValue(int priority){
	 ModelProvider::getInstance()->getWorldModel()->node[x][y].occupied = priority;
}*/

int Node::getX(){
	return x;
}

int Node::getY(){
	return y;
}

float Node::getXPos(){
	return x_pos;
}

float Node::getYPos(){
	return y_pos;
}

string Node::toString(){
	stringstream buf;
	buf << "( " << x << ", " << y << " ) ";
	return buf.str();
}
