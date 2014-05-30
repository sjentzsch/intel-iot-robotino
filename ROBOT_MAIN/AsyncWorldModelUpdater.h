/*
 * AsyncWorldModelUpdater.h
 *
 *  Created on: 05.06.2012
 *      Author: root
 */

#ifndef ASYNCWORLDMODELUPDATER_H_
#define ASYNCWORLDMODELUPDATER_H_

#include <boost/thread.hpp>
#include "model/WorldModel.h"
#include <vector>
#include "model/ModelProvider.h"
#include "communication/Communication.h"
#include "communication/WorldModelClientHandler.h"
#include "LSPBTrajectory/pose.h"

using namespace std;

class Grid;

class AsyncWorldModelUpdater {
private:
	boost::thread *execThread; //thread of execution
	boost::mutex active_mutex; //mutex for the active flag
	bool active;

	Grid *grid;

	void releasePathExcept_impl(vector<vec3D> upComingVias);
public:
	AsyncWorldModelUpdater();
	void join(); // wait for the current path freeing to be finished
	bool releasePathExcept(vector<vec3D> upComingVias);

	virtual ~AsyncWorldModelUpdater();
};

#endif /* ASYNCWORLDMODELUPDATER_H_ */
