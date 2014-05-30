/*
 * LSPBTrajectory.h
 *
 *  Created on: 17.05.2012
 *      Author: root
 */

#ifndef LSPBTRAJECTORY_H_
#define LSPBTRAJECTORY_H_

#include "timespec_utils.h"
#include "pose.h"
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/unordered_map.hpp> // hash map
#include "utils/FileLogger.h"

using namespace std;

class LSPBTrajectory2 {
private:
	int nSegments;
	vector<struct vec3D> viaPointsWithInit;
	vector<double> t_total_segment; // minimal total time for traveling from via point i to via point i+1; i = 0 ... n-1 in microseconds, first segment is the path from initial position to first via point
	vector<vec3D> linVelMax_segment; // signed maximum linear velocity for each segment and each dof in mm/s
	vector<vec3D> linAccMax_viaPoint; // signed maximun acceleration for each via point and each dof in mm/s^2
	vector<vec3D> t_blend_viaPoint; // blend time for each via point and each dof in microseconds
	vector<vec3D> linVel_segment; // actual linear velocity for each segment and dof in mm/s
	vector<vec3D> t_linear_segment; // linear time for each segmment and each dof in microseconds
	vector<double> t_maximum_allowed_segment_time; // when the determined time of a segment exceeds the maximum segmemt time, the trial is aborted and restarted with smaller velocities

	double sampleTime;
	vector<vec3D> traj_pos;
	vector<vec3D> traj_vel;

	// current differential constraints
	double curMaxLinVel;
	double curMaxLinAcc;
	double curMaxAngVel;
	double curMaxAngAcc;



	int signumWZero(double x);
	int signumWOZero(double x);
	void logVectorVec2DToFile(vector<struct vec3D> vec, const char * filename);
	bool tryDefineTraj(vector<struct vec3D> viaPoints, struct vec3D poseStart, double maxLinVel=300 , double maxAcc=300, double maxAngVel=70, double maxAngAcc=45);
public:
	LSPBTrajectory2();
	virtual ~LSPBTrajectory2();

	void defineTraj(vector<struct vec3D> viaPoints, struct vec3D poseStart, double maxLinVel=300 , double maxAcc=300, double maxAngVel=70, double maxAngAcc=45);
	void generateTraj(double sampleTime_);
	void getTrajPose(double curTime, vec3D& pos, vec3D& vel);
	int getCurSegment(double curTime);
	int getCurSegment(double curTime, vec2D position);
	void logTrajToFile(const char *dataSetName);
    double getCurMaxAngAcc() const;
    double getCurMaxAngVel() const;
    double getCurMaxLinAcc() const;
    double getCurMaxLinVel() const;
    void setCurMaxAngAcc(double curMaxAngAcc);
	
	template <typename T>
	string printVector(vector<T> vec);
};

#endif /* LSPBTRAJECTORY_H_ */
