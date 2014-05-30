#include "Grid.h"
#include "utils/FileLogger.h"

class Node;
class ExpandedNode;

Grid::Grid()
{
	float y_offset = 0.0;
	float x_offset = 0.0;

	for(int i=0; i<X_GRID; i++)
	{
		y_offset = Y_OFFSET;

		x_offset += X_GRID_WIDTH;
		
		//cout<<y_offset<<endl;
		for(int j=0; j<Y_GRID; j++)
		{
			if(j==0){
				y_offset += Y_OFFSET;
			}else{
				y_offset += Y_GRID_WIDTH;
			}

			//cout<<x_offset<<endl;
			bool blocked = false;

			// set the location of the production + recycling machines to blocked
			POI* poi;
			for(unsigned int p=0; p<26; p++)
			{
				if(p < 13)
					poi = &ModelProvider::getInstance()->getWorldModel()->poi[p];
				else
					poi = &ModelProvider::getInstance()->getWorldModel()->poi[p+3];
				if(poi->x == i && poi->y == j)
					blocked = true;
			}

			// block the first row, despite the 3 nodes in the middle
			if(i==0 && ((j>=0 && j<=7) || (j>=11 && j<=18)))
				blocked = true;

			// block the left delivery gate nodes (from team 1)
			if(j == 0 && (i == 3 || i == 4 || i == 5))
				blocked = true;

			// block the right delivery gate nodes (from team 2)
			if(j == 18 && (i == 3 || i == 4 || i == 5))
				blocked = true;
				
			grid[i][j] = new Node(x_offset,y_offset,i,j,this,blocked);
		}
	}
};

Grid::~Grid(){
	for(int i=0;i<X_GRID;i++)
		for(int j=0;j<Y_GRID;j++)
			delete grid[i][j];
};

POIDirection::POIDirection Grid::getAccessDirection(POI* poi, POIAccessFrom::POIAccessFrom accDir)
{

	//handle special case with recycle and scanner POIs (access node and node itself are the same);
	/*if(poi->type==POIType::RECYCLE || poi->type==POIType::SCANNER)
	{
		return (POIDirection::POIDirection)((poi->dir+2)%4);
	}
	else*/
	if(poi->type==POIType::INPUT || poi->type==POIType::DELIVER) // handle special case with input and deliver zone, basically ignore the AccessFrom-Direction
	{
		return (POIDirection::POIDirection)((poi->dir+2)%4);
	}
	else //ordinary production machine
	{
		switch(accDir)
		{
		case POIAccessFrom::FRONT:
			return (POIDirection::POIDirection)((poi->dir+2)%4);
		case POIAccessFrom::LEFT:
			return (POIDirection::POIDirection)((poi->dir+3)%4);
		case POIAccessFrom::RIGHT:
			return (POIDirection::POIDirection)((poi->dir+1)%4);
		default: //POIAccessFrom::BACK:
			return poi->dir;
		}
	}
}

Node* Grid::getAccessNode(POI* poi, POIAccessFrom::POIAccessFrom accDir)
{
	Node* node = getNode(poi->x, poi->y);
	//handle special case with recycle and scanner POIs (access node and node itself are the same);
	/*if(poi->type==POIType::RECYCLE || poi->type==POIType::SCANNER)
	{
		return node;
	}
	else*/
	if(poi->type==POIType::INPUT || poi->type==POIType::DELIVER) // handle special case with input and deliver zone, basically ignore the AccessFrom-Direction
	{
		return node->getNeighbor(poi->dir);
	}
	else //ordinary production machine
	{
		switch(accDir)
		{
		case POIAccessFrom::FRONT:
			return node->getNeighbor(poi->dir);
		case POIAccessFrom::LEFT:
			return node->getNeighbor((POIDirection::POIDirection)((poi->dir+1)%4));
		case POIAccessFrom::RIGHT:
			return node->getNeighbor((POIDirection::POIDirection)((poi->dir+3)%4));
		default: //POIAccessFrom::BACK:
			return node->getNeighbor((POIDirection::POIDirection)((poi->dir+2)%4));
		}
	}
}

Node* Grid::getNodeByCoord(vec2D &pos){
	return getNodeByCoord((float)(pos.x),(float)(pos.y));
}

Node* Grid::getNodeByCoord(float x, float y){
	int xGrid;
	int yGrid;

	yGrid = (int)((y-Y_OFFSET)/Y_GRID_WIDTH);
	xGrid = (int)((x-X_OFFSET)/X_GRID_WIDTH);
	if(yGrid<0){
		yGrid = 0;
	}
	if(xGrid<0){
		xGrid = 0;
	}
	if(xGrid>=X_GRID){
		xGrid = X_GRID-1;
	}
	if(yGrid>=Y_GRID){
		yGrid = Y_GRID-1;
	}

	if(	xGrid>=0 && xGrid<X_GRID &&
		yGrid>=0 && yGrid<Y_GRID){
		return getNode(xGrid,yGrid);
	}else{
		return NULL;
	}

}

ExpandedNode Grid::getNextRelevantNode(list<ExpandedNode>* path){
	list<ExpandedNode>::iterator it;
	POIDirection::POIDirection currDir;
	it = path->begin();
	currDir = it->getDirection();



	for ( it=path->begin() ; it != path->end(); it++ ){
		if(currDir!=it->getDirection()){
			return *it;
		}
	}

	return path->back();

}

list<ExpandedNode>* Grid::normalizePath(list<ExpandedNode>* path){ 

	if(path==NULL){
		return NULL;
	}

	list<ExpandedNode>::iterator it;
	list<ExpandedNode>* normalizedPath = new list<ExpandedNode>();
	POIDirection::POIDirection currDir;

	it = path->begin();
	normalizedPath->push_front(*it);
	currDir = it->getDirection();

	for ( it=path->begin() ; it != path->end(); it++ ){
		if(currDir!=it->getDirection()){
			normalizedPath->push_back(*it);
		}
		currDir = it->getDirection();
		
	}

	//refractor this if statement
	if(	normalizedPath->back().getDirection()!=path->back().getDirection()||
		normalizedPath->back().getNode()->getX()!=path->back().getNode()->getX()||
		normalizedPath->back().getNode()->getY()!=path->back().getNode()->getY()){
	normalizedPath->push_back(path->back()); 
	}
	
	return normalizedPath;
};

list<ExpandedNode>* Grid::getNormalizedPath(Node* startNode,POIDirection::POIDirection startDirection, Node* goalNode, POIDirection::POIDirection goalDirection){
	return normalizePath(getPath(startNode, startDirection, goalNode,goalDirection));
}

void Grid::clear(Node* currNode){
	for (int i = 0; i < X_GRID; ++i) {
		for (int j = 0; j < Y_GRID; ++j) {
			if(i!=currNode->getX()||j!=currNode->getY()){
				if(grid[i][j]->getNodeRaw()->occupied == ModelProvider::getInstance()->getID()){
					grid[i][j]->getNodeRaw()->occupied = 0;
				}
			}
		}
	}
}

void Grid::clearAll(){
	for (int i = 0; i < X_GRID; ++i) {
		for (int j = 0; j < Y_GRID; ++j) {
			if(grid[i][j]->getNodeRaw()->occupied == ModelProvider::getInstance()->getID()){
				grid[i][j]->getNodeRaw()->occupied = 0;
			}
		}
	}
}

//TODO: handle inaccessible nodes
list<ExpandedNode>* Grid::getPath(Node* startNode,POIDirection::POIDirection startDirection, Node* goalNode, POIDirection::POIDirection goalDirection, const vector<Node*> blacklist){

	bool expanded[X_GRID][Y_GRID][4];

	for (int row=0; row<X_GRID; row++) {
		for (int col=0; col<Y_GRID; col++) {
			for (int dir=0; dir<4; dir++) {
				expanded[row][col][dir] = false;
			}
		}
	}

	//block nodes that have an obstacle in them
	for (unsigned int i=0;i<blacklist.size();i++){
		expanded[blacklist[i]->getX()][blacklist[i]->getY()][0] = true;
		expanded[blacklist[i]->getX()][blacklist[i]->getY()][1] = true;
		expanded[blacklist[i]->getX()][blacklist[i]->getY()][2] = true;
		expanded[blacklist[i]->getX()][blacklist[i]->getY()][3] = true;
		}

	ExpandedNode* startExpNode = new ExpandedNode(startNode,startDirection);

	list<ExpandedNode*>* fringe = new list<ExpandedNode*>();
	fringe->push_front(startExpNode);
	
	while(!fringe->empty()){
		ExpandedNode* currElem = fringe->front();
		fringe->pop_front();
		Node* currNode = currElem->getNode();
		
		int x = currNode->getX();
		int y = currNode->getY();
		int dir = (int)currElem->getDirection();
		//cout<<"X:"<<x<<"  "<<"Y:"<<y<<"\n";
		if(expanded[x][y][dir]==false){
			
			list<ExpandedNode*> expandedList = currElem->expand();

			list<ExpandedNode*>::iterator it;

			for ( it=expandedList.begin() ; it != expandedList.end(); it++ ){
				fringe->push_back(*it);
			}


			expanded[x][y][dir]=true;
		}

		//GOALTEST
		if(	x==goalNode->getX()&&
			y==goalNode->getY()&&
			dir == goalDirection){

			ExpandedNode* parent = currElem;
			list<ExpandedNode>* path = new list<ExpandedNode>();

			while(parent!=0){
				parent->getNode()->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
				// TODO: set getNodeRaw()->timestamp ?!
				path->push_front(*parent);
				parent = parent->getParent();
			}
			//cout << "Path found." << endl;
			//cout << "From: " << startNode->getX() << ", " << startNode->getY() << "\t BlockedValue: " << startNode->getBlockedValue() <<endl;
			//cout << "To: " << goalNode->getX() << ", " << goalNode->getY() << "\t BlockedValue: " << goalNode->getBlockedValue() << endl;
			return path;
		}
	
	}
	//cout << "No path found." << endl;
	//cout << "From: " << startNode->getX() << ", " << startNode->getY() << "\t BlockedValue: " << startNode->getBlockedValue() <<endl;
	//cout << "To: " << goalNode->getX() << ", " << goalNode->getY() << "\t BlockedValue: " << goalNode->getBlockedValue() << endl;
	return NULL;

};

Node* Grid::getNode(int x, int y){
	if(x>=X_GRID || y >=Y_GRID || x<0  || y<0)
	{
		return NULL;	
	}
	else
	{
		return grid[x][y];
	}
}

/*
 * Returns the set of nodes between the line of start and end node, including start and end node
 */
set<Node *> Grid::giveNodesInBetween(Node *start, Node *end)
{
	set<Node *> result;

	if(start == NULL || end == NULL)
	{
		FileLog::log_NOTICE("[Grid] Warning: giveNodesInBetween was called with a null pointer.");
		return result;
	}

	if(start->getX() == end->getX()) // x is equal, iterate over all y
	{
		int x = start->getX();

		if(start->getY() > end->getY()) // change nodes so that start is the one with smaller Y coord
		{
			Node *temp = start;
			start = end;
			end = temp;
		}

		for(int i=start->getY(); i<=end->getY(); i++) // iterate over all y
		{
			result.insert(getNode(x,i));
		}
	}
	else if(start->getY() == end->getY()) // y is equal, iterate over all x
	{
		int y = start->getY();

		if(start->getX() > end->getX()) // change nodes so that start is the one with smaller X coord
		{
			Node *temp = start;
			start = end;
			end = temp;
		}

		for(int i=start->getX(); i<=end->getX(); i++) // iterate over all y
		{
			result.insert(getNode(i,y));
		}
	}
	else
	{
		// should not be the case
		FileLog::log_NOTICE("[Grid] Warning: giveNodesInBetween was called with two nodes not lying on a straight line.");
	}

	return result;
}

POIDirection::POIDirection getDirectionFromAngle(float phi)
{
	float phiStd = wrapDegToStdRange(phi);
	if(phiStd > -45.0 && phiStd <= 45.0)
		return POIDirection::SOUTH;
	else if(phiStd > 45.0 && phiStd <= 135.0)
		return POIDirection::EAST;
	else if(phiStd > 135.0 || phiStd <= -135.0)
		return POIDirection::NORTH;
	else
		return POIDirection::WEST;
}
