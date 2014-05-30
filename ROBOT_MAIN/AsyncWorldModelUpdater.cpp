/*
 * AsyncWorldModelUpdater.cpp
 *
 *  Created on: 05.06.2012
 *      Author: root
 */

#include "AsyncWorldModelUpdater.h"
#include "utils/FileLogger.h"
#include "pathfinder/Grid.h"

AsyncWorldModelUpdater::AsyncWorldModelUpdater() {
	active = false;
	execThread = NULL;
	grid = new Grid();
}

AsyncWorldModelUpdater::~AsyncWorldModelUpdater() {
	if(grid != NULL)
		delete grid;
}

bool AsyncWorldModelUpdater::releasePathExcept(vector<vec3D> upComingVias)
{
	FileLog::log(log_AsyncUpdate, "[AsyncWorldModelUpdater] requested async path freeing...");
	{
		boost::lock_guard<boost::mutex> lock(active_mutex);
		if(active == false)
		{
			active = true;
			FileLog::log(log_AsyncUpdate, "[AsyncWorldModelUpdater] started async path freeing...");
			execThread = new boost::thread(&AsyncWorldModelUpdater::releasePathExcept_impl,this,upComingVias);
			return true;
		}
		else
		{
			FileLog::log(log_AsyncUpdate, "[AsyncWorldModelUpdater] another path freeing is still running, returning...");
			return false;
		}
	}
}

void AsyncWorldModelUpdater::releasePathExcept_impl(vector<vec3D> upComingVias)
{
	// convert to grid nodes
	vector<Node *> upComingNodes;
	for(unsigned int i=0; i<upComingVias.size(); i++)
	{
		upComingNodes.push_back(grid->getNodeByCoord(upComingVias[i].x, upComingVias[i].y));
	}

	set<Node *> sum;
	// get all nodes in between
	for(unsigned int i=0; i<upComingNodes.size()-1; i++)
	{
		set<Node *> tmp = grid->giveNodesInBetween(upComingNodes[i], upComingNodes[i+1]);
		sum.insert(tmp.begin(), tmp.end());
	}

	WorldModelClientHandler::getInstance()->lock();

	FileLog::log(log_AsyncUpdate, "Remaining nodes:");
	grid->clearAll(); // clear all nodes
	set<Node *>::iterator it;
	for(it=sum.begin(); it!=sum.end(); it++) // block all upcoming nodes
	{
		(*it)->getNodeRaw()->occupied = ModelProvider::getInstance()->getID();
		// TODO: optional: set new getNodeRaw()->timestamp values for a better estimation
		FileLog::log(log_AsyncUpdate, "( ", FileLog::integer((*it)->getX()), ", ", FileLog::integer((*it)->getY()), ")");
	}

	WorldModelClientHandler::getInstance()->unlock();

	{
		boost::lock_guard<boost::mutex> lock(active_mutex);
		active = false;
	}

}

void AsyncWorldModelUpdater::join()
{
	// TODO: is this safe?
	if(execThread!=NULL)
	{
		execThread->join(); //Wait for execution thread to terminate
	}
}
