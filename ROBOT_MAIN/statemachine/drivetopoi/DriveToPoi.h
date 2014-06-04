/*
 * StateMachine.h
 *
 *  Created on: 24.04.2011
 *      Author: root
 */

#ifndef DRIVETOPOI_H_
#define DRIVETOPOI_H_
#include "../StateMachine.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

//States
struct dtpInit;
struct dtpWaiting;
struct dtpDriving;
struct dtpObstacleHandling;
struct dtpPathDriven;


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
//    DriveToPoi
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

struct DriveToPoi : sc::state<DriveToPoi,StateMachine1, dtpInit>
{
	DriveToPoi(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("DriveToPoi");
		stateBehavCtrl = context<StateMachine1>().getStateBehavController();
		poi = context<StateMachine1>().poiTo;
		accessDirection = context<StateMachine1>().accessDirectionTo;
	} // entry

	~DriveToPoi() {
		context<StateMachine1>().havePathToPOI = false;
	} // exit

	StateBehaviorController* getStateBehavController(){ return stateBehavCtrl;}

	void findPath(){
		stateBehavCtrl->getMotorCtrl()->terminate();
		//stateBehavCtrl->getPathFinder()->findPathTo(poi,accessDirection);
	}

	void findPathMotor(const EvMotorCtrlReady&){
		//context<StateMachine1>().getStateBehavController()->getAsyncWorldModelUpdater()->join();
		findPath();
	}

	void findPathInit(){
		findPath();
	}

	void findPathWaiting(const EvNoPathFound&){
		findPath();
	}

	void pathFound(float x_,float y_,float rotation_, vector<struct vec3D> vias_,float maxSpeed){
		EvPathFound event(x_, y_, rotation_, vias_);

		FileLog::log_NOTICE("[DriveToPoi] ",FileLog::real(event.x), ",", FileLog::real(event.y), ",", FileLog::real(event.rotation));

		/*if(stateBehavCtrl->getPathFinder()->isNextRelevantInSameNodeMM(event.x,event.y) && context<StateMachine1>().driveWithPuck){
			FileLog::log_NOTICE("[DriveToPoi] Rotating to drive direction:", FileLog::real(event.rotation));
			stateBehavCtrl->getMotorCtrl()->rotateToAbsAngle(event.rotation);
		}else{
			FileLog::log_NOTICE("[DriveToPoi] Driving trajectory.");
			vias = event.vias;
			if(stateBehavCtrl->getPathFinder()->isNextRelevantInSameNodeMM(event.vias[0].x,event.vias[0].y,event.vias[0].phi))
			{
				FileLog::log_NOTICE("[DriveToPoi] Leaving out first via, cause it's so close.");
				vias.erase(vias.begin());
			}
			nodes = context<StateMachine1>().getStateBehavController()->getPathFinder()->convertViaPointsToNodes(vias);
			stateBehavCtrl->getMotorCtrl()->driveLSPBTrajectory(context<DriveToPoi>().vias, maxSpeed,600,180,180);
		}*/
	}

	vector<vec3D> vias;
	vector<Node*> nodes;

	//Reactions
	typedef mpl::list<
		sc::transition<EvSuccess, Dispatcher, StateMachine1, &StateMachine1::taskSuccess>,
		sc::deferral<EvAngleCalibration>
	> reactions;

private:
	StateBehaviorController *stateBehavCtrl;
	POI* poi;
	POIAccessFrom::POIAccessFrom accessDirection;
};

struct dtpInit : sc::state< dtpInit, DriveToPoi>
{
	dtpInit(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dtpInit");
		post_event(EvInitPathFinder());
	} // entry

	~dtpInit() {
	} // exit

	sc::result react(const EvInitPathFinder&)
	{
		if(context<StateMachine1>().havePathToPOI)
		{
			FileLog::log_NOTICE("[DriveToPoi] Having Path already.");
			context<DriveToPoi>().vias = context<StateMachine1>().pathToPOI->vias;
			//context<DriveToPoi>().nodes = context<StateMachine1>().getStateBehavController()->getPathFinder()->convertViaPointsToNodes(context<DriveToPoi>().vias);
			context<DriveToPoi>().pathFound(context<StateMachine1>().pathToPOI->x, context<StateMachine1>().pathToPOI->y, context<StateMachine1>().pathToPOI->rotation, context<StateMachine1>().pathToPOI->vias,800);
			return transit<dtpDriving>();
		}
		else
		{
			FileLog::log_NOTICE("[DriveToPoi] Searching Path.");
			context<DriveToPoi>().findPathInit();
			return transit<dtpWaiting>();
		}
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvInitPathFinder>
	> reactions;
};

struct dtpWaiting : sc::state< dtpWaiting, DriveToPoi>
{
	dtpWaiting(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dtpWaiting");
	} // entry

	~dtpWaiting() {
	} // exit

	sc::result react(const EvPathFound& ev)
	{
		/*if(context<StateMachine1>().poiFrom != NULL && context<StateMachine1>().poiFrom->type == POIType::INPUT && context<StateMachine1>().poiFrom->occupied == ModelProvider::getInstance()->getID()){
			WorldModelClientHandler::getInstance()->lock();
			cout << "Releasing input zone." << endl;
			context<StateMachine1>().poiFrom->occupied = 0;
			WorldModelClientHandler::getInstance()->unlock();
		}*/
		context<DriveToPoi>().vias = ev.vias;
		context<DriveToPoi>().pathFound(ev.x, ev.y, ev.rotation, ev.vias, 800);
		return transit<dtpDriving>();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvPathFound>,
		sc::transition<EvNoPathFound,dtpWaiting,DriveToPoi, &DriveToPoi::findPathWaiting>,
		sc::transition<EvPathDriven, dtpPathDriven>
	> reactions;
};

struct dtpDriving : sc::state< dtpDriving, DriveToPoi>
{
	dtpDriving(my_context ctx) : my_base(ctx) {
		if(context<StateMachine1>().driveWithPuck)
		{
			context<StateMachine1>().logAndDisplayStateName("dtpDriving (with puck)");
			context<StateMachine1>().getStateBehavController()->getSensorControl()->setHavingPuck(true);
		}
		else
		{
			context<StateMachine1>().logAndDisplayStateName("dtpDriving");
		}
	} // entry

	~dtpDriving() {
		if(context<StateMachine1>().driveWithPuck)
			context<StateMachine1>().getStateBehavController()->getSensorControl()->setHavingPuck(false);
	} // exit

	sc::result react(const EvSensorLostPuck&)
	{
		context<StateMachine1>().regainPuckReturn = RegainPuckReturn::dtpInit;
		return transit<RegainPuck>();
	}

	sc::result react(const EvFinishedTrajSegment& ev)
	{
		/*if(context<StateMachine1>().getStateBehavController()->getAsyncWorldModelUpdater()->releasePathExcept(ev.upComingVias))
			FileLog::log_NOTICE("[EvFinishedTrajSegment] starting async path freeing...");
		else
			FileLog::log_NOTICE("[EvFinishedTrajSegment] AsyncWorldModelUpdate busy, discarding event...");*/
		return discard_event();
		//return discard_event();
	}

	sc::result react(const EvObstacleMap& ev){

		cout << "[DriveToPOI] EvObstacleMap start ..." << endl;

		float myX = context<StateMachine1>().getStateBehavController()->getSensorControl()->getRobotX();
		float myY = context<StateMachine1>().getStateBehavController()->getSensorControl()->getRobotY();
		Node* myNode = context<StateMachine1>().getGrid()->getNodeByCoord(myX, myY);

		vector<Node*> nodesToDrive;
		bool reachedMyNode = false;
		for(unsigned int i=0; i<context<DriveToPoi>().nodes.size(); i++)
		{
			if(context<DriveToPoi>().nodes.at(i)->getX() == myNode->getX() && context<DriveToPoi>().nodes.at(i)->getY() == myNode->getY())
				reachedMyNode = true;

			if(reachedMyNode)
				nodesToDrive.push_back(context<DriveToPoi>().nodes.at(i));
		}

		for(unsigned int i=0; i<nodesToDrive.size(); i++)
		{
			cout << "nodesToDrive " << i << " : " << nodesToDrive.at(i)->getX() << ", " << nodesToDrive.at(i)->getY() << endl;
		}

		for(unsigned int i=0; i<ev.obstacleNodes.size(); i++)
		{
			cout << "ev.obstacleNodes " << i << " : " << ev.obstacleNodes.at(i)->getX() << ", " << ev.obstacleNodes.at(i)->getY() << endl;
		}

		bool someNodeOnPathIsBlocked = false;
		for(unsigned int i=0; i<nodesToDrive.size(); i++)
		{
			for(unsigned int u=0; u<ev.obstacleNodes.size(); u++)
			{
				if(nodesToDrive.at(i)->getX() == ev.obstacleNodes.at(u)->getX() && nodesToDrive.at(i)->getY() == ev.obstacleNodes.at(u)->getY())
				{
					someNodeOnPathIsBlocked = true;
					break;
				}
			}
			if(someNodeOnPathIsBlocked)
				break;
		}

		// DEBUG beide Node-Listen hier !!!


		//ev.obstacles => NodeList
		//vector<Node*> blacklist = ev.obstacleNodes;
		if(someNodeOnPathIsBlocked)
		{
			cout << "[DriveToPOI] Replan ..." << endl;
			FileLog::log_NOTICE("[DriveToPOI] Replan ...");
			context<StateMachine1>().getStateBehavController()->getMotorCtrl()->terminate();
			//context<StateMachine1>().getStateBehavController()->getPathFinder()->findPathTo(context<StateMachine1>().poiTo, context<StateMachine1>().accessDirectionTo, NULL, POIAccessFrom::FRONT, ev.obstacleNodes);
			return transit<dtpWaiting>();
		}
		else
		{
			cout << "[DriveToPOI] Do not replan ..." << endl;
			FileLog::log_NOTICE("[DriveToPOI] Do not replan ...");
			return discard_event();
		}
	}

	sc::result react(const EvObstacleIsClose& ev){
		context<StateMachine1>().getStateBehavController()->getMotorCtrl()->terminate();
		return discard_event();
	}

	//Reactions
	typedef mpl::list<
		sc::custom_reaction<EvSensorLostPuck>,
		/*sc::custom_reaction<EvObstacleIsClose>,*/
		sc::custom_reaction<EvObstacleMap>,
		sc::transition<EvMotorCtrlReady, dtpWaiting, DriveToPoi, &DriveToPoi::findPathMotor>,
		sc::custom_reaction<EvFinishedTrajSegment>
	> reactions;
};

struct dtpPathDriven : sc::state< dtpPathDriven, DriveToPoi>
{
	dtpPathDriven(my_context ctx) : my_base(ctx) {
		context<StateMachine1>().logAndDisplayStateName("dtpPathDriven");
		post_event(EvSuccess());
	} // entry

	~dtpPathDriven() {
	} // exit

};

#endif /* DRIVETOPOI_H */
