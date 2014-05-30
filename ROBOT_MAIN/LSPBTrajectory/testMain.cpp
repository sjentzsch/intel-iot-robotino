///*
// * testMain.cpp
// *
// *  Created on: 17.05.2012
// *      Author: root
// */
//#include <iostream>
//#include "../FileLoggerConfig.h"
//#include "../FileLogger.h"
//
////std::ostream& operator <<(std::ostream &o, const struct vec2D& p)
////{
////    return o << "(" << p.x << ", " << p.y << ")" << std::endl;
////}
//
////#include "LSPBTrajectory.h"
//#include "LSPBTrajectory2.h"
//
//using namespace std;
//
//int main(int argc, char* argv[])
//{
//	vector<struct vec3D> vias;
//	vias.push_back(vec3D(560,0,0));
//	vias.push_back(vec3D(560,0,0));
//	vias.push_back(vec3D(1120,0,0));
//
//	//ViaPointsWithInit:  (210, 1635, 0) (560, 1680, 0) (1120, 1680, 90) (1120, 2240, 0) (1680, 2240, 0) (2240, 2240, 0) (2800, 2240, 0) (3360, 2240, 90) (3360, 2800, 180)
//
////ViaPointsWithInit:  (2801.87, 2229.02, -178.266) (1120, 2240, 180)
//
//	LSPBTrajectory2 traj;
//	traj.defineTraj(vias, vec3D(0, 0, 0),600,600,90,90);
//	traj.generateTraj(5000.0);
//	traj.logTrajToFile("test");
//
//	return 0;
//}
