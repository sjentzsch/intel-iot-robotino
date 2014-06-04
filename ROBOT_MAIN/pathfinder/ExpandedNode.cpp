#include "Grid.h"

ExpandedNode::ExpandedNode(Node* node, POIDirection::POIDirection dir)
	:node(node), dir(dir),parent(0)
{

}

ExpandedNode::ExpandedNode(Node* node, POIDirection::POIDirection dir,ExpandedNode* parent)
	:node(node), dir(dir), parent(parent)
{

}

ExpandedNode::~ExpandedNode(){

}

list<ExpandedNode*> ExpandedNode::expand(){

	list<ExpandedNode*> nodeList = list<ExpandedNode*>();

	Node* neighbor = node->getNeighbor(dir);
	if(neighbor!=0 && !neighbor->isBlocked() &&
		(
		neighbor->getNodeRaw()->occupied == 0
		)){
		nodeList.push_back(new ExpandedNode(neighbor,dir,this));
		//std::cout<<this->node->getNeighbor(dir)->getX()<<"  "<<this->node->getNeighbor(dir)->getY()<<"\n";
	}

	//avoid turning robotino on spot
	//if(getParent()!=NULL){
		int exp_dir=0;
		for(int i=0;i<4;i++){
			if(exp_dir!=dir){
				//check for 180 degree rotation and avoid it
				//TODO: This does not seem to work yet
//				if(getParent()->getParent()!=NULL){
//					if((getParent()->getParent()->getDirection()+2)%4 != dir){
//						nodeList.push_back(new ExpandedNode(this->node,(POIDirection::POIDirection)exp_dir,this));
//					}
//
//				}else{
//					nodeList.push_back(new ExpandedNode(this->node,(POIDirection::POIDirection)exp_dir,this));
//				}

				nodeList.push_back(new ExpandedNode(this->node,(POIDirection::POIDirection)exp_dir,this));

				//std::cout<<this->node->getX()<<"  "<<this->node->getY()<<"\n";
			}
			exp_dir++;
		//}
	}

	return nodeList;
}

ExpandedNode* ExpandedNode::getParent(){
	return parent;
}

Node* ExpandedNode::getNode(){
	return node;
}

string ExpandedNode::toString(){
	stringstream buf;
	buf << "( " << getNode()->getX() << ", " << getNode()->getY() << ", " << getPhi() << " ) ";
	return buf.str();
}

float ExpandedNode::getPhi()
{
	switch(dir){
	case POIDirection::NORTH: return 180;
	case POIDirection::SOUTH: return 0;
	case POIDirection::EAST: return 90;
	case POIDirection::WEST: return -90;
	default: return 0;
	}
}

POIDirection::POIDirection ExpandedNode::getDirection(){
	return dir;
};
