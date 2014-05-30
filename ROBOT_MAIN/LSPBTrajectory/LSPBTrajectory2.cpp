/*
 * LSPBTrajectory.cpp
 *
 *  Created on: 17.05.2012
 *      Author: root
 */

#include "LSPBTrajectory2.h"
#include <math.h>
#include <cmath>

LSPBTrajectory2::LSPBTrajectory2() {
	// TODO Auto-generated constructor stub

}

LSPBTrajectory2::~LSPBTrajectory2() {
	// TODO Auto-generated destructor stub
}

// params: maxLinVel in mm/s, maxAcc in mm/s^2, all poses in mm
bool LSPBTrajectory2::tryDefineTraj(vector<struct vec3D> viaPoints, struct vec3D poseStart, double maxLinVel, double maxLinAcc, double maxAngVel, double maxAngAcc)
{
	// reset variables
	t_total_segment.clear();
	linVelMax_segment.clear();
	linAccMax_viaPoint.clear();
	t_blend_viaPoint.clear();
	linVel_segment.clear();
	t_linear_segment.clear();
	t_maximum_allowed_segment_time.clear();
	traj_pos.clear();
	traj_vel.clear();

	nSegments = viaPoints.size();
	viaPointsWithInit = viaPoints;
	viaPointsWithInit.insert(viaPointsWithInit.begin(), poseStart);
	vector<vec3D> linAccMax_viaPoint_before;

	FileLog::log(log_MotorController, "ViaPointsWithInit: ", printVec3D(viaPointsWithInit));

	// eliminate the angle wrap around to create a continous space in which interpolation/blending works
	for(unsigned int i=1; i<viaPointsWithInit.size(); i++)
	{
		viaPointsWithInit[i].phi = eliminateAngleWrapAround(viaPointsWithInit[i-1].phi,viaPointsWithInit[i].phi);
	}

	// ###################################################
	// calculate signed max velocity for each segment
	// cout << "calculate signed max velocity for each segment..." << endl;
	for(int i=0; i < nSegments; i++)
	{
		double vel_x = maxLinVel * signumWZero(viaPointsWithInit[i+1].x-viaPointsWithInit[i].x);
		double vel_y = maxLinVel * signumWZero(viaPointsWithInit[i+1].y-viaPointsWithInit[i].y);
		double vel_phi = maxAngVel * signumWZero(viaPointsWithInit[i+1].phi-viaPointsWithInit[i].phi);
		linVelMax_segment.push_back(vec3D(vel_x, vel_y, vel_phi));
	}

	// ##################################################
	// calculate signed max acceleration for each via point
	// cout << "calculate signed max acceleration for each via point" << endl;

	// first segment special case
	double acc_x = maxLinAcc * signumWZero(viaPointsWithInit[1].x - viaPointsWithInit[0].x);
	double acc_y = maxLinAcc * signumWZero(viaPointsWithInit[1].y - viaPointsWithInit[0].y);
	double acc_phi = maxAngAcc * signumWZero(viaPointsWithInit[1].phi - viaPointsWithInit[0].phi);
	linAccMax_viaPoint.push_back(vec3D(acc_x, acc_y, acc_phi));

	// via segments
	for(int i=0; i < nSegments-1; i++)
	{
		double acc_x = maxLinAcc * signumWZero(linVelMax_segment[i+1].x-linVelMax_segment[i].x);
		double acc_y = maxLinAcc * signumWZero(linVelMax_segment[i+1].y-linVelMax_segment[i].y);
		double acc_phi = maxAngAcc * signumWZero(linVelMax_segment[i+1].phi-linVelMax_segment[i].phi);

		// if velocities are equal let him still accelerate in the velocities direction
		if(linVelMax_segment[i+1].x == linVelMax_segment[i].x){
			acc_x = maxLinAcc * signumWZero(linVelMax_segment[i+1].x);
		}
		if(linVelMax_segment[i+1].y == linVelMax_segment[i].y){
			acc_y = maxLinAcc * signumWZero(linVelMax_segment[i+1].y);
		}
		if(linVelMax_segment[i+1].phi == linVelMax_segment[i].phi){
			acc_phi = maxAngAcc * signumWZero(linVelMax_segment[i+1].phi);
		}

		linAccMax_viaPoint.push_back(vec3D(acc_x, acc_y, acc_phi));
	}

	// last segment special case
	acc_x = maxLinAcc * signumWZero(viaPointsWithInit[nSegments-1].x - viaPointsWithInit[nSegments].x); // here [n-1] - [n] because we want to brake
	acc_y = maxLinAcc * signumWZero(viaPointsWithInit[nSegments-1].y - viaPointsWithInit[nSegments].y); // here [n-1] - [n] because we want to brake
	acc_phi = maxAngAcc * signumWZero(viaPointsWithInit[nSegments-1].phi - viaPointsWithInit[nSegments].phi); // here [n-1] - [n] because we want to brake
	linAccMax_viaPoint.push_back(vec3D(acc_x, acc_y, acc_phi));

	// save linAcc values, in special cases acceleration needs to be reverted
	for(unsigned int i=0; i<linAccMax_viaPoint.size();i++)
	{
		linAccMax_viaPoint_before.push_back(linAccMax_viaPoint[i]);
	}

	// calc maximum allowed segment time
	for(int i=0; i<nSegments; i++)
	{
		int factor = 4;
		list<double> temp;
		float diffx = abs(viaPointsWithInit[i].x - viaPointsWithInit[i+1].x);
		float diffy = abs(viaPointsWithInit[i].y - viaPointsWithInit[i+1].y);
		float diffphi = abs(viaPointsWithInit[i].phi - viaPointsWithInit[i+1].phi);
		if(diffx == 0 && diffy == 0 && diffphi == 0) // this means i+1 is the same viapoint as i
		{
			if(i == 0)
			{
				temp.push_back(5000000); // default value of 5 milliseconds
			}
			else
			{
				temp.push_back(t_maximum_allowed_segment_time.back()); // push the time from the lase segment
			}
		}
		else
		{
			temp.push_back(factor * 1000000.0 * diffx / maxLinVel);
			temp.push_back(factor * 1000000.0 * diffy / maxLinVel);
			temp.push_back(factor * 1000000.0 * diffphi / maxAngVel);
		}
		temp.sort();
		t_maximum_allowed_segment_time.push_back(temp.back()); // save maximum time
	}

	FileLog::log(log_MotorController, "ViaPointsWithInit (after wrapping): ", printVec3D(viaPointsWithInit));

	if(nSegments > 1) // we need a special case for trajectories with only one segment
	{
		/*
		 * Determine first segment total time by increasing it until certain velocity and blend time constraints are met
		 */
		{
			double t_segment = 100000; // init with 100ms
			double increment = 100000;

			// result variables
			vec3D t_blend;
			vec3D linVel;
			vec3D t_linear;

			// helper variables
			vec3D linAcc = linAccMax_viaPoint[0];
			vec3D x1 = viaPointsWithInit[1];
			vec3D x0 = viaPointsWithInit[0];

			do
			{

				// calc resulting blend times
				if(x1.x > x0.x){
					t_blend.x = 1000000.0*(linAcc.x * t_segment/1000000.0 - sqrt((pow(linAcc.x,2)*pow(t_segment/1000000.0,2))-(2*linAcc.x*x1.x)+(2*linAcc.x*x0.x)))/linAcc.x;
				}
				else{
					t_blend.x = 1000000.0*(linAcc.x * t_segment/1000000.0 + sqrt((pow(linAcc.x,2)*pow(t_segment/1000000.0,2))-(2*linAcc.x*x1.x)+(2*linAcc.x*x0.x)))/linAcc.x;
				}
				if(x1.y > x0.y){
					t_blend.y = 1000000.0*(linAcc.y * t_segment/1000000.0 - sqrt((pow(linAcc.y,2)*pow(t_segment/1000000.0,2))-(2*linAcc.y*x1.y)+(2*linAcc.y*x0.y)))/linAcc.y;
				}
				else{
					t_blend.y = 1000000.0*(linAcc.y * t_segment/1000000.0 + sqrt((pow(linAcc.y,2)*pow(t_segment/1000000.0,2))-(2*linAcc.y*x1.y)+(2*linAcc.y*x0.y)))/linAcc.y;
				}
				if(x1.phi > x0.phi){
					t_blend.phi = 1000000.0*(linAcc.phi * t_segment/1000000.0 - sqrt((pow(linAcc.phi,2)*pow(t_segment/1000000.0,2))-(2*linAcc.phi*x1.phi)+(2*linAcc.phi*x0.phi)))/linAcc.phi;
				}
				else{
					t_blend.phi = 1000000.0*(linAcc.phi * t_segment/1000000.0 + sqrt((pow(linAcc.phi,2)*pow(t_segment/1000000.0,2))-(2*linAcc.phi*x1.phi)+(2*linAcc.phi*x0.phi)))/linAcc.phi;
				}

				// check for division by zero
				t_blend.x = linAcc.x == 0.0 ? 0 : t_blend.x;
				t_blend.y = linAcc.y == 0.0 ? 0 : t_blend.y;
				t_blend.phi = linAcc.phi == 0.0 ? 0 : t_blend.phi;

				t_blend.x = abs(t_blend.x);
				t_blend.y = abs(t_blend.y);
				t_blend.phi = abs(t_blend.phi);

				linVel.x = linAcc.x * t_blend.x/1000000.0;
				linVel.y = linAcc.y * t_blend.y/1000000.0;
				linVel.phi = linAcc.phi * t_blend.phi/1000000.0;

				t_linear.x = t_segment - t_blend.x;
				t_linear.y = t_segment - t_blend.y;
				t_linear.phi = t_segment - t_blend.phi;

//				cout << "------------------" << endl;
//				cout << "First segment: " << "t_segment: " << t_segment << endl;
//				cout << "linAcc: " << linAcc.x << ", " << linAcc.y << ", " << linAcc.phi << endl;
//				cout << "x1: " << x1.x << ", " << x1.y << ", " << x1.phi << endl;
//				cout << "x0: " << x0.x << ", " << x0.y << ", " << x0.phi << endl;
//				cout << "Calculated Values ---|" << endl;
//				cout << "t_blend: " << t_blend.x << ", " << t_blend.y << "," << t_blend.phi << endl;
//				cout << "linVel: " << linVel.x << ", " << linVel.y << "," << linVel.phi << endl;
//				cout << "t_maximum_allowed_time: " << t_maximum_allowed_segment_time[0] << endl;
//				cout << "------------------" << endl;


				// check if constraints are fulfilled
				if(abs(linVel.x) <= maxLinVel &&
				   abs(linVel.y) <= maxLinVel &&
				   abs(linVel.phi) <= maxAngVel &&
				   t_blend.x <= 0.5*t_segment &&
				   t_blend.y <= 0.5*t_segment &&
				   t_blend.phi <= 0.5*t_segment)
				{
					break; // out, t_segment big enough
				}
				else
				{
					t_segment += increment;
					 //wait for input
//					char lol;
//					cin >> lol;
//					lol = lol +1;

					if(t_segment > t_maximum_allowed_segment_time[0])
					{
						return false; // break this trial, because iteration won't probably converge
					}
				}
			}
			while(true);

			// set determined values
			t_blend_viaPoint.push_back(t_blend);
			t_linear_segment.push_back(t_linear);
			linVel_segment.push_back(linVel);
			t_total_segment.push_back(t_segment);


		}


		/*
		 * Determine middle segment total time by increasing it until certain velocity and blend time constraints are met
		 */
		for(int n=1; n<nSegments-1; n++)
		{
			double t_segment = 100000; // init with 100ms
			double increment = 100000;

			// result variables
			vec3D t_blend_n;
			vec3D linVel_n;
			vec3D t_linear_n;

			// helper variables
			vec3D linAcc = linAccMax_viaPoint[n];
			vec3D xnp1 = viaPointsWithInit[n+1];
			vec3D xn = viaPointsWithInit[n];
			vec3D linVel_nm1 = linVel_segment[n-1];
			double t_segment_nm1 = t_total_segment[n-1];

			do
			{

				linVel_n.x = (xnp1.x - xn.x) / (t_segment/1000000.0);
				linVel_n.y = (xnp1.y - xn.y) / (t_segment/1000000.0);
				linVel_n.phi = (xnp1.phi - xn.phi) / (t_segment/1000000.0);

				t_blend_n.x = 1000000.0 * (linVel_n.x - linVel_nm1.x)/linAcc.x;
				t_blend_n.y = 1000000.0 * (linVel_n.y - linVel_nm1.y)/linAcc.y;
				t_blend_n.phi = 1000000.0 * (linVel_n.phi - linVel_nm1.phi)/linAcc.phi;

				// check for division by zero
				t_blend_n.x = linAcc.x == 0.0 ? 0 : t_blend_n.x;
				t_blend_n.y = linAcc.y == 0.0 ? 0 : t_blend_n.y;
				t_blend_n.phi = linAcc.phi == 0.0 ? 0 : t_blend_n.phi;

				// test if blend times are negative (this means that acceleration has to flip sign)
				if(t_blend_n.x < 0.0){
					t_blend_n.x = -t_blend_n.x;
					linAccMax_viaPoint[n].x = -linAccMax_viaPoint_before[n].x;
				}
				if(t_blend_n.y < 0.0){
					t_blend_n.y = -t_blend_n.y;
					linAccMax_viaPoint[n].y = -linAccMax_viaPoint_before[n].y;
				}
				if(t_blend_n.phi < 0.0){
					t_blend_n.phi = -t_blend_n.phi;
					linAccMax_viaPoint[n].phi = -linAccMax_viaPoint_before[n].phi;
				}

				t_linear_n.x = t_segment - 0.5 * t_blend_n.x;
				t_linear_n.y = t_segment - 0.5 * t_blend_n.y;
				t_linear_n.phi = t_segment - 0.5 * t_blend_n.phi;


//				cout << "------------------" << endl;
//				cout << "Middle segment: " << n << "; t_segment =" << t_segment << endl;
//				cout << "linAcc: " << linAcc.x << ", " << linAcc.y << ", " << linAcc.phi << endl;
//				cout << "xnp1: " << xnp1.x << ", " << xnp1.y << ", " << xnp1.phi << endl;
//				cout << "xn: " << xn.x << ", " << xn.y << ", " << xn.phi << endl;
//				cout << "linVel_nm1: " << linVel_nm1.x << ", " << linVel_nm1.y << ", " << linVel_nm1.phi << endl;
//				cout << "t_segment_nm1: " << t_segment_nm1 << endl;
//				cout << "Calculated Values ---|" << endl;
//				cout << "t_blend_n: " << t_blend_n.x << ", " << t_blend_n.y << "," << t_blend_n.phi << endl;
//				cout << "linVel_n: " << linVel_n.x << ", " << linVel_n.y << "," << linVel_n.phi << endl;
//				cout << "t_maximum_allowed_time: " << t_maximum_allowed_segment_time[n] << endl;
//				cout << "------------------" << endl;

				// check if constraints are met
				if(abs(linVel_n.x) <= maxLinVel &&
				   abs(linVel_n.y) <= maxLinVel &&
				   abs(linVel_n.phi) <= maxAngVel &&
				   0.5*t_blend_n.x <= 0.5*t_segment_nm1 &&
				   0.5*t_blend_n.y <= 0.5*t_segment_nm1 &&
				   0.5*t_blend_n.phi <= 0.5*t_segment_nm1 &&
				   0.5*t_blend_n.x <= 0.5*t_segment &&
				   0.5*t_blend_n.y <= 0.5*t_segment &&
				   0.5*t_blend_n.phi <= 0.5*t_segment)
				{
					break; // out
				}
				else
				{
					t_segment += increment;
					// wait for input
//					char lol;
//					cin >> lol;
//					lol = lol +1;

					if(t_segment > t_maximum_allowed_segment_time[n])
					{
						return false; // break this trial, because iteration won't probably converge
					}
				}
			}
			while(true);

			// set determined values
			t_blend_viaPoint.push_back(t_blend_n);
			t_linear_segment.push_back(t_linear_n);
			linVel_segment.push_back(linVel_n);
			t_total_segment.push_back(t_segment);
		}

		/*
		 * Determine last segment total time by increasing it until certain velocity and blend time constraints are met
		 */
		{
			double t_segment = 100000; // init with 100ms
			double increment = 100000;

			// result variables
			vec3D t_blend_n;
			vec3D t_blend_nm1;
			vec3D linVel_n;
			vec3D t_linear_n;

			// helper variables
			vec3D linAcc_n = linAccMax_viaPoint[nSegments];
			vec3D linAcc_nm1 = linAccMax_viaPoint[nSegments-1];
			vec3D xn = viaPointsWithInit[nSegments];
			vec3D xnm1 = viaPointsWithInit[nSegments-1];
			vec3D linVel_nm1 = linVel_segment[nSegments-2];
			double t_segment_nm1 = t_total_segment[nSegments-2];

			do
			{
				if(xn.y < xnm1.y){
					t_blend_n.y = 1000000.0 * ((t_segment/1000000.0)*linAcc_n.y - sqrt((pow(t_segment/1000000.0,2)*pow(linAcc_n.y,2))+(2*linAcc_n.y*xn.y)-(2*linAcc_n.y*xnm1.y)))/(linAcc_n.y);
				}
				else{ // difference is the plus/minus in front of sqrt
					t_blend_n.y = 1000000.0 * ((t_segment/1000000.0)*linAcc_n.y + sqrt((pow(t_segment/1000000.0,2)*pow(linAcc_n.y,2))+(2*linAcc_n.y*xn.y)-(2*linAcc_n.y*xnm1.y)))/(linAcc_n.y);
				}

				if(xn.x < xnm1.x){
					t_blend_n.x = 1000000.0 * ((t_segment/1000000.0)*linAcc_n.x - sqrt((pow(t_segment/1000000.0,2)*pow(linAcc_n.x,2))+(2*linAcc_n.x*xn.x)-(2*linAcc_n.x*xnm1.x)))/(linAcc_n.x);
				}
				else{
					t_blend_n.x = 1000000.0 * ((t_segment/1000000.0)*linAcc_n.x + sqrt((pow(t_segment/1000000.0,2)*pow(linAcc_n.x,2))+(2*linAcc_n.x*xn.x)-(2*linAcc_n.x*xnm1.x)))/(linAcc_n.x);
				}
				if(xn.phi < xnm1.phi){
					t_blend_n.phi = 1000000.0 * ((t_segment/1000000.0)*linAcc_n.phi - sqrt((pow(t_segment/1000000.0,2)*pow(linAcc_n.phi,2))+(2*linAcc_n.phi*xn.phi)-(2*linAcc_n.phi*xnm1.phi)))/(linAcc_n.phi);
				}
				else{ // difference is the plus/minus in front of sqrt
					t_blend_n.phi = 1000000.0 * ((t_segment/1000000.0)*linAcc_n.phi + sqrt((pow(t_segment/1000000.0,2)*pow(linAcc_n.phi,2))+(2*linAcc_n.phi*xn.phi)-(2*linAcc_n.phi*xnm1.phi)))/(linAcc_n.phi);
				}

				// check for division by zero
				t_blend_n.x = linAcc_n.x == 0.0 ? 0 : t_blend_n.x;
				t_blend_n.y = linAcc_n.y == 0.0 ? 0 : t_blend_n.y;
				t_blend_n.phi = linAcc_n.phi == 0.0 ? 0 : t_blend_n.phi;

				// test if blend times are negative (this means that acceleration has to flip sign)
				if(t_blend_n.x < 0.0){
					t_blend_n.x = -t_blend_n.x;
					linAccMax_viaPoint[nSegments].x = -linAccMax_viaPoint_before[nSegments].x;
				}
				if(t_blend_n.y < 0.0){
					t_blend_n.y = -t_blend_n.y;
					linAccMax_viaPoint[nSegments].y = -linAccMax_viaPoint_before[nSegments].y;
				}
				if(t_blend_n.phi < 0.0){
					t_blend_n.phi = -t_blend_n.phi;
					linAccMax_viaPoint[nSegments].phi = -linAccMax_viaPoint_before[nSegments].phi;
				}

				linVel_n.x = - linAcc_n.x * (t_blend_n.x / 1000000.0);
				linVel_n.y = - linAcc_n.y * (t_blend_n.y / 1000000.0);
				linVel_n.phi = - linAcc_n.phi * (t_blend_n.phi / 1000000.0);

				t_blend_nm1.x = 1000000.0 * (linVel_n.x - linVel_nm1.x)/linAcc_nm1.x;
				t_blend_nm1.y = 1000000.0 * (linVel_n.y - linVel_nm1.y)/linAcc_nm1.y;
				t_blend_nm1.phi = 1000000.0 * (linVel_n.phi - linVel_nm1.phi)/linAcc_nm1.phi;

				// check for division by zero
				t_blend_nm1.x = linAcc_nm1.x == 0.0 ? 0 : t_blend_nm1.x;
				t_blend_nm1.y = linAcc_nm1.y == 0.0 ? 0 : t_blend_nm1.y;
				t_blend_nm1.phi = linAcc_nm1.phi == 0.0 ? 0 : t_blend_nm1.phi;

				// test if blend times are negative (this means that acceleration has to flip sign)
				if(t_blend_nm1.x < 0.0){
					t_blend_nm1.x = -t_blend_nm1.x;
					linAccMax_viaPoint[nSegments-1].x = -linAccMax_viaPoint_before[nSegments-1].x;
				}
				if(t_blend_nm1.y < 0.0){
					t_blend_nm1.y = -t_blend_nm1.y;
					linAccMax_viaPoint[nSegments-1].y = -linAccMax_viaPoint_before[nSegments-1].y;
				}
				if(t_blend_nm1.phi < 0.0){
					t_blend_nm1.phi = -t_blend_nm1.phi;
					linAccMax_viaPoint[nSegments-1].phi = -linAccMax_viaPoint_before[nSegments-1].phi;
				}

				t_linear_n.x = t_segment - t_blend_n.x;
				t_linear_n.y = t_segment - t_blend_n.y;
				t_linear_n.phi = t_segment - t_blend_n.phi;

//				cout << "------------------" << endl;
//				cout << "linAcc_n: " << linAcc_n.x << ", " << linAcc_n.y << ", " << linAcc_n.phi << endl;
//				cout << "linAcc_nm1: " << linAcc_nm1.x << ", " << linAcc_nm1.y << ", " << linAcc_nm1.phi  << endl;
//				cout << "xn: " << xn.x << ", " << xn.y << ", " << xn.phi  << endl;
//				cout << "xnm1: " << xnm1.x << ", " << xnm1.y << ", " << xnm1.phi  << endl;
//				cout << "linVel_nm1: " << linVel_nm1.x << ", " << linVel_nm1.y << ", " << linVel_nm1.phi  << endl;
//				cout << "t_segment_nm1: " << t_segment_nm1 << endl;
//				cout << "Calculated Values ---|" << endl;
//				cout << "Last segment: t_segment =" << t_segment << endl;
//				cout << "t_blend_n: " << t_blend_n.x << ", " << t_blend_n.y << ", " << t_blend_n.phi  << endl;
//				cout << "linVel_n: " << linVel_n.x << ", " << linVel_n.y << ", " << linVel_n.phi  << endl;
//				cout << "t_blend_nm1: " << t_blend_nm1.x << ", " << t_blend_nm1.y << ", " << t_blend_nm1.phi  << endl;
//				cout << "t_maximum_allowed_time: " << t_maximum_allowed_segment_time[nSegments-1] << endl;
//				cout << "------------------" << endl;

				// check constraints
				if(abs(linVel_n.x) <= maxLinVel &&
				   abs(linVel_n.y) <= maxLinVel &&
				   abs(linVel_n.phi) <= maxAngVel &&
				   0.5*t_blend_nm1.x <= 0.5*t_segment_nm1 &&
				   0.5*t_blend_nm1.y <= 0.5*t_segment_nm1 &&
				   0.5*t_blend_nm1.phi <= 0.5*t_segment_nm1 &&
				   0.5*t_blend_nm1.x <= 0.5*t_segment &&
				   0.5*t_blend_nm1.y <= 0.5*t_segment &&
				   0.5*t_blend_nm1.phi <= 0.5*t_segment &&
				   t_blend_n.x <= 0.5*t_segment &&
				   t_blend_n.y <= 0.5*t_segment &&
				   t_blend_n.phi <= 0.5*t_segment)
				{
					break; // out of while
				}
				else
				{
					t_segment += increment;
					// wait for input
//					char lol;
//					cin >> lol;
//					lol = lol +1;

					if(t_segment > t_maximum_allowed_segment_time[nSegments-1])
					{
						return false; // break this trial, because iteration won't probably converge
					}
				}
			}
			while(true);

			// set determined values
			t_blend_viaPoint.push_back(t_blend_nm1);
			t_blend_viaPoint.push_back(t_blend_n);
			linVel_segment.push_back(linVel_n);
			t_linear_segment.push_back(t_linear_n);
			t_total_segment.push_back(t_segment);
		}
	}
	else // traj has only one segment
	{
		double t_segment = 100000; // init with 100ms
		double increment = 100000;

		// values to be calculated
		vec3D t_blend;
		vec3D linVel;
		vec3D t_linear;

		// helper variables
		vec3D x1 = viaPointsWithInit[1];
		vec3D x0 = viaPointsWithInit[0];
		vec3D linAcc = linAccMax_viaPoint_before[0];

		do
		{
			if(x1.x > x0.x){
				t_blend.x = 1000000.0 * 0.5 * ((linAcc.x*(t_segment/1000000.0) - sqrt(pow(linAcc.x,2)*pow(t_segment/1000000.0,2) - 4*linAcc.x*x1.x + 4*linAcc.x*x0.x))/linAcc.x);
			}
			else{
				t_blend.x = 1000000.0 * 0.5 * ((linAcc.x*(t_segment/1000000.0) + sqrt(pow(linAcc.x,2)*pow(t_segment/1000000.0,2) - 4*linAcc.x*x1.x + 4*linAcc.x*x0.x))/linAcc.x);
			}

			if(x1.y > x0.y){
				t_blend.y = 1000000.0 * 0.5 * ((linAcc.y*(t_segment/1000000.0) - sqrt(pow(linAcc.y,2)*pow(t_segment/1000000.0,2) - 4*linAcc.y*x1.y + 4*linAcc.y*x0.y))/linAcc.y);
			}
			else{
				t_blend.y = 1000000.0 * 0.5 * ((linAcc.y*(t_segment/1000000.0) + sqrt(pow(linAcc.y,2)*pow(t_segment/1000000.0,2) - 4*linAcc.y*x1.y + 4*linAcc.y*x0.y))/linAcc.y);
			}

			if(x1.phi > x0.phi){
				t_blend.phi = 1000000.0 * 0.5 * ((linAcc.phi*(t_segment/1000000.0) - sqrt(pow(linAcc.phi,2)*pow(t_segment/1000000.0,2) - 4*linAcc.phi*x1.phi + 4*linAcc.phi*x0.phi))/linAcc.phi);
			}
			else{
				t_blend.phi = 1000000.0 * 0.5 * ((linAcc.phi*(t_segment/1000000.0) + sqrt(pow(linAcc.phi,2)*pow(t_segment/1000000.0,2) - 4*linAcc.phi*x1.phi + 4*linAcc.phi*x0.phi))/linAcc.phi);
			}

//			t_blend.x = 1000000.0 * ((t_segment/1000000.0)/2.0) * (sqrt((pow(linAcc.x,2)*pow(t_segment/1000000.0,2))-(4*linAcc.x*(x1.x-x0.x)))/(2*linAcc.x));
//			t_blend.y = 1000000.0 * ((t_segment/1000000.0)/2.0) * (sqrt((pow(linAcc.y,2)*pow(t_segment/1000000.0,2))-(4*linAcc.y*(x1.y-x0.y)))/(2*linAcc.y));
//			t_blend.phi = 1000000.0 * ((t_segment/1000000.0)/2.0) * (sqrt((pow(linAcc.phi,2)*pow(t_segment/1000000.0,2))-(4*linAcc.phi*(x1.phi-x0.phi)))/(2*linAcc.phi));

			// check for division by zero
			t_blend.x = linAcc.x == 0.0 ? 0 : t_blend.x;
			t_blend.y = linAcc.y == 0.0 ? 0 : t_blend.y;
			t_blend.phi = linAcc.phi == 0.0 ? 0 : t_blend.phi;

			linVel.x = (t_blend.x / 1000000.0) * linAcc.x;
			linVel.y = (t_blend.y / 1000000.0) * linAcc.y;
			linVel.phi = (t_blend.phi / 1000000.0) * linAcc.phi;

			t_linear.x = t_segment - 2*t_blend.x;
			t_linear.y = t_segment - 2*t_blend.y;
			t_linear.phi = t_segment - 2*t_blend.phi;

//			cout << "------------------" << endl;
//			cout << "First segment: " << "t_segment: " << t_segment << endl;
//			cout << "linAcc: " << linAcc.x << ", " << linAcc.y << ", " << linAcc.phi << endl;
//			cout << "x1: " << x1.x << ", " << x1.y << ", " << x1.phi << endl;
//			cout << "x0: " << x0.x << ", " << x0.y << ", " << x0.phi << endl;
//			cout << "Calculated Values ---|" << endl;
//			cout << "t_blend: " << t_blend.x << ", " << t_blend.y << "," << t_blend.phi << endl;
//			cout << "linVel: " << linVel.x << ", " << linVel.y << "," << linVel.phi << endl;
//			cout << "------------------" << endl;

			// check constraints
			if(abs(linVel.x) <= maxLinVel &&
			   abs(linVel.y) <= maxLinVel &&
			   abs(linVel.phi) <= maxAngVel &&
			   t_blend.x <= 0.5*t_segment &&
			   t_blend.y <= 0.5*t_segment &&
			   t_blend.phi <= 0.5*t_segment
			   )
			{
				break; // out of while
			}
			else
			{
				t_segment += increment;
				// wait for input
//				char lol;
//				cin >> lol;
//				lol = lol +1;

				if(t_segment > t_maximum_allowed_segment_time[0])
				{
					return false; // break this trial, because iteration won't probably converge
				}
			}
		}
		while(true);

		t_blend_viaPoint.push_back(t_blend); // acc
		t_blend_viaPoint.push_back(t_blend); // deacc
		t_linear_segment.push_back(t_linear);
		linVel_segment.push_back(linVel);
		t_total_segment.push_back(t_segment);
	}

	// Yeah we found a traj, print some details about it:
	FileLog::log(log_MotorController, "Signed Linear Max Velocity in mm/s: ", printVec3D(linVelMax_segment));
	FileLog::log(log_MotorController, "Signed Linear Max Acceleration in mm/s^2 (before): ", printVec3D(linAccMax_viaPoint_before));
	FileLog::log(log_MotorController, "Signed Linear Max Acceleration in mm/s^2 (after): ", printVec3D(linAccMax_viaPoint));
	FileLog::log(log_MotorController, "t_segment in microseconds: ", printVector(t_total_segment));
	FileLog::log(log_MotorController, "t_blend_viaPoint in microseconds:", printVec3D(t_blend_viaPoint));
	FileLog::log(log_MotorController, "t_linear_segment in microseconds:", printVec3D(t_linear_segment));
	FileLog::log(log_MotorController, "linVel_segment in mm/s:", printVec3D(linVel_segment));

	return true;
}

void LSPBTrajectory2::defineTraj(vector<struct vec3D> viaPoints, struct vec3D poseStart, double maxLinVel , double maxLinAcc, double maxAngVel, double maxAngAcc)
{
	curMaxAngAcc = maxAngAcc;
	curMaxLinAcc = maxLinAcc;
	curMaxLinVel = maxLinVel;
	curMaxAngVel = maxAngVel;

	while(!tryDefineTraj(viaPoints, poseStart, curMaxLinVel, curMaxLinAcc, curMaxAngVel, curMaxAngAcc))
	{
		//FileLog::log_NOTICE();
		FileLog::log_NOTICE("[LSPBTrajectory] No traj found with current vel limits, I'm lowering the limits and trying again");
		FileLog::log_NOTICE("[LSPBTrajectory] Old limits, LinVel: ", FileLog::real(curMaxLinVel), ", AngVel: ", FileLog::real(curMaxAngVel));
		curMaxLinVel = 0.833 * curMaxLinVel; // reduce to 5/6 of old velocity
		curMaxAngVel = 0.833 * curMaxAngVel;
		FileLog::log_NOTICE("[LSPBTrajectory] New limits, LinVel: ", FileLog::real(curMaxLinVel), ", AngVel: ", FileLog::real(curMaxAngVel));
	}
}

void LSPBTrajectory2::generateTraj(double sampleTime_)
{
	sampleTime = sampleTime_; // sampling time in microseconds
	int curSegment = 0; // current segment (index)
	double totalTime = 0; // total trajectory time in microseconds

	vector<double> traj_pos_x;
	vector<double> traj_vel_x;

	vector<double> traj_pos_y;
	vector<double> traj_vel_y;

	vector<double> traj_pos_phi;
	vector<double> traj_vel_phi;


	double curTotalTime = 0; // current accumulated time in microseconds
	double curAccSegTime = 0; // accumulated time of all already passed segments

	//calc total time in micro seconds
	for(int i=0; i < (int)t_total_segment.size(); i++)
	{
		totalTime += t_total_segment[i];
	}

	if(nSegments > 1) // we need a special case for trajs with only one segment
	{
		// #######################################
		// generate trajectory for x
		for(curTotalTime = 0; curTotalTime < totalTime; curTotalTime += sampleTime)
		{
			// check if we just stepped into a new segment
			if(curTotalTime > curAccSegTime + t_total_segment[curSegment] + 0.5 * t_blend_viaPoint[curSegment+1].x && !(curSegment >= nSegments-1)) // each segment contains its linear part and the following acceleration part t_blend_viaPoint[curSegment+1]
			{
				curAccSegTime += t_total_segment[curSegment];
				curSegment++;
			}

			// get the curSegTime
			double curSegTime = curTotalTime - curAccSegTime; // current time since start of the current segment in microseconds

			if(curSegment == 0) // first segment special case
			{
				if(curSegTime < t_blend_viaPoint[0].x) // initial  acceleration, linear and second acceleration
				{
					traj_pos_x.push_back(viaPointsWithInit[0].x + 0.5 * linAccMax_viaPoint[0].x * pow(curSegTime/1000000.0f,2));
					traj_vel_x.push_back(linAccMax_viaPoint[0].x * (curSegTime/1000000.0f));
				}
				else if(curSegTime < t_blend_viaPoint[0].x + t_linear_segment[0].x - 0.5 * t_blend_viaPoint[1].x) // first segment linear phase
				{
					double linTime = curSegTime - t_blend_viaPoint[0].x; // time since beginning of linear phase
					traj_pos_x.push_back(viaPointsWithInit[0].x + linTime / 1000000.0 * linVel_segment[0].x + 0.5 * linAccMax_viaPoint[0].x * pow(t_blend_viaPoint[0].x/1000000.0f,2));
					traj_vel_x.push_back(linVel_segment[0].x);
				}
				else // acceleration phase of first via point
				{
					double accTime = curSegTime - t_blend_viaPoint[0].x - (t_linear_segment[0].x - 0.5 * t_blend_viaPoint[1].x); // time since beginning of acceleration phase
					traj_pos_x.push_back(viaPointsWithInit[0].x + (t_linear_segment[0].x - 0.5 * t_blend_viaPoint[1].x) / 1000000.0 * linVel_segment[0].x + 0.5 * linAccMax_viaPoint[0].x * pow(t_blend_viaPoint[0].x/1000000.0f,2) + accTime/1000000.0f * linVel_segment[0].x + 0.5 * linAccMax_viaPoint[1].x * pow(accTime/1000000.0f,2));
					traj_vel_x.push_back(linVel_segment[0].x + linAccMax_viaPoint[1].x * accTime/1000000.0);
				}
			}
			else if(curSegment == nSegments-1) // end segment
			{
				if(curSegTime < t_linear_segment[curSegment].x) // linear phase
				{
					traj_pos_x.push_back(viaPointsWithInit[curSegment].x + curSegTime/1000000.0 * linVel_segment[curSegment].x);
					traj_vel_x.push_back(linVel_segment[curSegment].x);
				}
				else // last deacceleration phase
				{
					double accTime = curSegTime - t_linear_segment[curSegment].x;
					traj_pos_x.push_back(viaPointsWithInit[curSegment].x + curSegTime/1000000.0 * linVel_segment[curSegment].x + 0.5 * linAccMax_viaPoint[curSegment+1].x * pow(accTime/1000000.0f,2));
					traj_vel_x.push_back(linVel_segment[curSegment].x + accTime/1000000.0 * linAccMax_viaPoint[curSegment+1].x);
				}
			}
			else // middle segments
			{
				if(curSegTime < 0.5 * t_blend_viaPoint[curSegment].x + t_linear_segment[curSegment].x - 0.5 * t_blend_viaPoint[curSegment+1].x) // linear phase
				{
					traj_pos_x.push_back(viaPointsWithInit[curSegment].x + curSegTime/1000000.0 * linVel_segment[curSegment].x);
					traj_vel_x.push_back(linVel_segment[curSegment].x);
				}
				else // acceleration phase of the following via point
				{
					double accTime = curSegTime - 0.5 * t_blend_viaPoint[curSegment].x - (t_linear_segment[curSegment].x - 0.5 * t_blend_viaPoint[curSegment+1].x);
					traj_pos_x.push_back(viaPointsWithInit[curSegment].x + curSegTime/1000000.0 * linVel_segment[curSegment].x + 0.5 * linAccMax_viaPoint[curSegment+1].x * pow(accTime/1000000.0f,2));
					traj_vel_x.push_back(linVel_segment[curSegment].x + accTime/1000000.0 * linAccMax_viaPoint[curSegment+1].x);
				}
			}
		}

		curSegment = 0; // reset everything
		curTotalTime = 0;
		curAccSegTime = 0;
		// #######################################
		// generate trajectory for y
		for(curTotalTime = 0; curTotalTime < totalTime; curTotalTime += sampleTime)
		{
			// check if we just stepped into a new segment
			if(curTotalTime > curAccSegTime + t_total_segment[curSegment] + 0.5 * t_blend_viaPoint[curSegment+1].y && !(curSegment >= nSegments-1)) //  !curSegment >= nSegments-1 prevents the statement to switch beyond the maximum segment number
			{
				curAccSegTime += t_total_segment[curSegment];
				curSegment++;
			}

			// get the curSegTime
			double curSegTime = curTotalTime - curAccSegTime;

			if(curSegment == 0) // first segment special case
			{
				if(curSegTime < t_blend_viaPoint[0].y) // initial acceleration, linear and second acceleration phase
				{
					traj_pos_y.push_back(viaPointsWithInit[0].y + 0.5 * linAccMax_viaPoint[0].y * pow(curSegTime/1000000.0f,2));
					traj_vel_y.push_back(linAccMax_viaPoint[0].y * (curSegTime/1000000.0f));
				}
				else if(curSegTime < t_blend_viaPoint[0].y + t_linear_segment[0].y - 0.5 * t_blend_viaPoint[1].y) // first segment linear phase
				{
					double linTime = curSegTime - t_blend_viaPoint[0].y; // time since beginning of linear phase
					traj_pos_y.push_back(viaPointsWithInit[0].y + linTime/1000000.0 * linVel_segment[0].y + 0.5 * linAccMax_viaPoint[0].y * pow(t_blend_viaPoint[0].y/1000000.0f,2));
					traj_vel_y.push_back(linVel_segment[0].y);
				}
				else // acceleration phase of first via point
				{
					double accTime = curSegTime - t_blend_viaPoint[0].y - (t_linear_segment[0].y - 0.5 * t_blend_viaPoint[1].y); // time since beginning of acceleration phase
					traj_pos_y.push_back(viaPointsWithInit[0].y + (t_linear_segment[0].y - 0.5 * t_blend_viaPoint[1].y)/1000000.0 * linVel_segment[0].y + 0.5 * linAccMax_viaPoint[0].y * pow(t_blend_viaPoint[0].y/1000000.0f,2) + accTime/1000000.0f * linVel_segment[0].y + 0.5 * linAccMax_viaPoint[1].y * pow(accTime/1000000.0f,2));
					traj_vel_y.push_back(linVel_segment[0].y + linAccMax_viaPoint[1].y * accTime/1000000.0);
				}
			}
			else if(curSegment == nSegments-1) // end segment
			{
				if(curSegTime < t_linear_segment[curSegment].y) // linear phase
				{
					traj_pos_y.push_back(viaPointsWithInit[curSegment].y + curSegTime/1000000.0 * linVel_segment[curSegment].y);
					traj_vel_y.push_back(linVel_segment[curSegment].y);
				}
				else // last deacceleration phase
				{
					double accTime = curSegTime - t_linear_segment[curSegment].y;
					traj_pos_y.push_back(viaPointsWithInit[curSegment].y + curSegTime/1000000.0 * linVel_segment[curSegment].y + 0.5 * linAccMax_viaPoint[curSegment+1].y * pow(accTime/1000000.0f,2));
					traj_vel_y.push_back(linVel_segment[curSegment].y + accTime/1000000.0 * linAccMax_viaPoint[curSegment+1].y);
				}
			}
			else // middle segments
			{
				if(curSegTime < 0.5 * t_blend_viaPoint[curSegment].y + t_linear_segment[curSegment].y - 0.5 * t_blend_viaPoint[curSegment+1].y) // linear phase
				{
					traj_pos_y.push_back(viaPointsWithInit[curSegment].y + curSegTime/1000000.0 * linVel_segment[curSegment].y);
					traj_vel_y.push_back(linVel_segment[curSegment].y);
				}
				else // acceleration phase of the following via point
				{
					double accTime = curSegTime - 0.5 * t_blend_viaPoint[curSegment].y - (t_linear_segment[curSegment].y - 0.5 * t_blend_viaPoint[curSegment+1].y);
					traj_pos_y.push_back(viaPointsWithInit[curSegment].y + curSegTime/1000000.0 * linVel_segment[curSegment].y + 0.5 * linAccMax_viaPoint[curSegment+1].y * pow(accTime/1000000.0f,2));
					traj_vel_y.push_back(linVel_segment[curSegment].y + accTime/1000000.0 * linAccMax_viaPoint[curSegment+1].y);
				}
			}
		}

		curSegment = 0; // reset everything
		curTotalTime = 0;
		curAccSegTime = 0;
		// #######################################
		// generate trajectory for phi
		for(curTotalTime = 0; curTotalTime < totalTime; curTotalTime += sampleTime)
		{
			// check if we just stepped into a new segment
			if(curTotalTime > curAccSegTime + t_total_segment[curSegment] + 0.5 * t_blend_viaPoint[curSegment+1].phi && !(curSegment >= nSegments-1)) //  !curSegment >= nSegments-1 prevents the statement to switch beyond the maximum segment number
			{
				curAccSegTime += t_total_segment[curSegment];
				curSegment++;
			}

			// get the curSegTime
			double curSegTime = curTotalTime - curAccSegTime;

			if(curSegment == 0) // first segment special case
			{
				if(curSegTime < t_blend_viaPoint[0].phi) // initial acceleration, linear and second acceleration phase
				{
					traj_pos_phi.push_back(viaPointsWithInit[0].phi + 0.5 * linAccMax_viaPoint[0].phi * pow(curSegTime/1000000.0f,2));
					traj_vel_phi.push_back(linAccMax_viaPoint[0].phi * (curSegTime/1000000.0f));
				}
				else if(curSegTime < t_blend_viaPoint[0].phi + t_linear_segment[0].phi - 0.5 * t_blend_viaPoint[1].phi) // first segment linear phase
				{
					double linTime = curSegTime - t_blend_viaPoint[0].phi; // time since beginning of linear phase
					traj_pos_phi.push_back(viaPointsWithInit[0].phi + linTime/1000000.0 * linVel_segment[0].phi + 0.5 * linAccMax_viaPoint[0].phi * pow(t_blend_viaPoint[0].phi/1000000.0f,2));
					traj_vel_phi.push_back(linVel_segment[0].phi);
				}
				else // acceleration phase of first via point
				{
					double accTime = curSegTime - t_blend_viaPoint[0].phi - (t_linear_segment[0].phi - 0.5 * t_blend_viaPoint[1].phi); // time since beginning of acceleration phase
					traj_pos_phi.push_back(viaPointsWithInit[0].phi + (t_linear_segment[0].phi - 0.5 * t_blend_viaPoint[1].phi)/1000000.0 * linVel_segment[0].phi + 0.5 * linAccMax_viaPoint[0].phi * pow(t_blend_viaPoint[0].phi/1000000.0f,2) + accTime/1000000.0f * linVel_segment[0].phi + 0.5 * linAccMax_viaPoint[1].phi * pow(accTime/1000000.0f,2));
					traj_vel_phi.push_back(linVel_segment[0].phi + linAccMax_viaPoint[1].phi * accTime/1000000.0);
				}
			}
			else if(curSegment == nSegments-1) // end segment
			{
				if(curSegTime < t_linear_segment[curSegment].phi) // linear phase
				{
					traj_pos_phi.push_back(viaPointsWithInit[curSegment].phi + curSegTime/1000000.0 * linVel_segment[curSegment].phi);
					traj_vel_phi.push_back(linVel_segment[curSegment].phi);
				}
				else // last deacceleration phase
				{
					double accTime = curSegTime - t_linear_segment[curSegment].phi;
					traj_pos_phi.push_back(viaPointsWithInit[curSegment].phi + curSegTime/1000000.0 * linVel_segment[curSegment].phi + 0.5 * linAccMax_viaPoint[curSegment+1].phi * pow(accTime/1000000.0f,2));
					traj_vel_phi.push_back(linVel_segment[curSegment].phi + accTime/1000000.0 * linAccMax_viaPoint[curSegment+1].phi);
				}
			}
			else // middle segments
			{
				if(curSegTime < 0.5 * t_blend_viaPoint[curSegment].phi + t_linear_segment[curSegment].phi - 0.5 * t_blend_viaPoint[curSegment+1].phi) // linear phase
				{
					traj_pos_phi.push_back(viaPointsWithInit[curSegment].phi + curSegTime/1000000.0 * linVel_segment[curSegment].phi);
					traj_vel_phi.push_back(linVel_segment[curSegment].phi);
				}
				else // acceleration phase of the following via point
				{
					double accTime = curSegTime - 0.5 * t_blend_viaPoint[curSegment].phi - (t_linear_segment[curSegment].phi - 0.5 * t_blend_viaPoint[curSegment+1].phi);
					traj_pos_phi.push_back(viaPointsWithInit[curSegment].phi + curSegTime/1000000.0 * linVel_segment[curSegment].phi + 0.5 * linAccMax_viaPoint[curSegment+1].phi * pow(accTime/1000000.0f,2));
					traj_vel_phi.push_back(linVel_segment[curSegment].phi + accTime/1000000.0 * linAccMax_viaPoint[curSegment+1].phi);
				}
			}
		}
	}
	else
	{
		for(curTotalTime = 0; curTotalTime < totalTime; curTotalTime += sampleTime)
		{
			double curSegTime = curTotalTime;

			// x
			if(curSegTime < t_blend_viaPoint[0].x) // initial acceleration, linear and second acceleration phase
			{
				traj_pos_x.push_back(viaPointsWithInit[0].x + 0.5 * linAccMax_viaPoint[0].x * pow(curSegTime/1000000.0f,2));
				traj_vel_x.push_back(linAccMax_viaPoint[0].x * (curSegTime/1000000.0f));
			}
			else if(curSegTime < t_blend_viaPoint[0].x + t_linear_segment[0].x) // first segment linear phase
			{
				double linTime = curSegTime - t_blend_viaPoint[0].x; // time since beginning of linear phase
				traj_pos_x.push_back(viaPointsWithInit[0].x + linTime/1000000.0 * linVel_segment[0].x + 0.5 * linAccMax_viaPoint[0].x * pow(t_blend_viaPoint[0].x/1000000.0f,2));
				traj_vel_x.push_back(linVel_segment[0].x);
			}
			else // deacceleration phase
			{
				double accTime = curSegTime - t_blend_viaPoint[0].x - t_linear_segment[0].x; // time since beginning of acceleration phase
				traj_pos_x.push_back(viaPointsWithInit[0].x + (curSegTime - t_blend_viaPoint[0].x)/1000000.0 * linVel_segment[0].x + 0.5 * linAccMax_viaPoint[0].x * pow(t_blend_viaPoint[0].x/1000000.0f,2) + 0.5 * linAccMax_viaPoint[1].x * pow(accTime/1000000.0f,2));
				traj_vel_x.push_back(linVel_segment[0].x + linAccMax_viaPoint[1].x * accTime/1000000.0);
			}

			// y
			if(curSegTime < t_blend_viaPoint[0].y) // initial acceleration, linear and second acceleration phase
			{
				traj_pos_y.push_back(viaPointsWithInit[0].y + 0.5 * linAccMax_viaPoint[0].y * pow(curSegTime/1000000.0f,2));
				traj_vel_y.push_back(linAccMax_viaPoint[0].y * (curSegTime/1000000.0f));
			}
			else if(curSegTime < t_blend_viaPoint[0].y + t_linear_segment[0].y) // first segment linear phase
			{
				double linTime = curSegTime - t_blend_viaPoint[0].y; // time since beginning of linear phase
				traj_pos_y.push_back(viaPointsWithInit[0].y + linTime/1000000.0 * linVel_segment[0].y + 0.5 * linAccMax_viaPoint[0].y * pow(t_blend_viaPoint[0].y/1000000.0f,2));
				traj_vel_y.push_back(linVel_segment[0].y);
			}
			else // deacceleration phase
			{
				double accTime = curSegTime - t_blend_viaPoint[0].y - t_linear_segment[0].y; // time since beginning of acceleration phase
				traj_pos_y.push_back(viaPointsWithInit[0].y + (curSegTime - t_blend_viaPoint[0].y)/1000000.0 * linVel_segment[0].y + 0.5 * linAccMax_viaPoint[0].y * pow(t_blend_viaPoint[0].y/1000000.0f,2) + 0.5 * linAccMax_viaPoint[1].y * pow(accTime/1000000.0f,2));
				traj_vel_y.push_back(linVel_segment[0].y + linAccMax_viaPoint[1].y * accTime/1000000.0);
			}

			// phi
			if(curSegTime < t_blend_viaPoint[0].phi) // initial acceleration, linear and second acceleration phase
			{
				traj_pos_phi.push_back(viaPointsWithInit[0].phi + 0.5 * linAccMax_viaPoint[0].phi * pow(curSegTime/1000000.0f,2));
				traj_vel_phi.push_back(linAccMax_viaPoint[0].phi * (curSegTime/1000000.0f));
			}
			else if(curSegTime < t_blend_viaPoint[0].phi + t_linear_segment[0].phi) // first segment linear phase
			{
				double linTime = curSegTime - t_blend_viaPoint[0].phi; // time since beginning of linear phase
				traj_pos_phi.push_back(viaPointsWithInit[0].phi + linTime/1000000.0 * linVel_segment[0].phi + 0.5 * linAccMax_viaPoint[0].phi * pow(t_blend_viaPoint[0].phi/1000000.0f,2));
				traj_vel_phi.push_back(linVel_segment[0].phi);
			}
			else // deacceleration phase
			{
				double accTime = curSegTime - t_blend_viaPoint[0].phi - t_linear_segment[0].phi; // time since beginning of acceleration phase
				traj_pos_phi.push_back(viaPointsWithInit[0].phi + (curSegTime - t_blend_viaPoint[0].phi)/1000000.0 * linVel_segment[0].phi + 0.5 * linAccMax_viaPoint[0].phi * pow(t_blend_viaPoint[0].phi/1000000.0f,2) + 0.5 * linAccMax_viaPoint[1].phi * pow(accTime/1000000.0f,2));
				traj_vel_phi.push_back(linVel_segment[0].phi + linAccMax_viaPoint[1].phi * accTime/1000000.0);
			}
		}
	}

	// put vectors together
	for(int i=0; i < (int)traj_pos_x.size(); i++)
	{
		traj_pos.push_back(vec3D(traj_pos_x[i],traj_pos_y[i], traj_pos_phi[i]));
		traj_vel.push_back(vec3D(traj_vel_x[i],traj_vel_y[i], traj_vel_phi[i]));
	}
}

// interpolates the traj pose
// curTime in microseconds
// output into pos and vel
void LSPBTrajectory2::getTrajPose(double curTime, vec3D& pos, vec3D& vel)
{
	// calculate vector index
	unsigned int index_low = floor(curTime / sampleTime);
	unsigned int index_high = index_low+1;
	index_low = index_low > traj_pos.size()-1 ? traj_pos.size()-1 : index_low; // check out of bounds
	index_high = index_high > traj_pos.size()-1 ? traj_pos.size()-1 : index_high; // check out of bounds

	double ratio = (curTime - index_low * sampleTime) / sampleTime; // ratio for interpolation, value between 0...1

	// interpolate
	vec2D linPos = linearInterpolateVec2D(vec2D(traj_pos[index_low].x,traj_pos[index_low].y), vec2D(traj_pos[index_high].x,traj_pos[index_high].y), ratio);
	vec2D linVel = linearInterpolateVec2D(vec2D(traj_vel[index_low].x,traj_vel[index_low].y), vec2D(traj_vel[index_high].x,traj_vel[index_high].y), ratio);
	double angPos = linearInterpolateAngle(traj_pos[index_low].phi, traj_pos[index_high].phi, ratio);
	double angVel = linearInterpolateScalar(traj_vel[index_low].phi, traj_vel[index_high].phi, ratio);

	// set values;
	pos.x = linPos.x;
	pos.y = linPos.y;
	pos.phi = angPos;

	vel.x = linVel.x;
	vel.y = linVel.y;
	vel.phi = angVel;
}

/*
 * returns the current segment, starting with 0 to nSegments-1
 */
int LSPBTrajectory2::getCurSegment(double curTime)
{
	int curSegment = 0;
	double accSegTime = 0;
	for(unsigned int i=0; i<t_total_segment.size(); i++)
	{
		accSegTime += t_total_segment[i];
		if(curTime < accSegTime) // first time current smaller than accumulated time we are in the right segment
		{
			return curSegment;
		}
		curSegment++;
	}
	curSegment--;
	return curSegment; // if we are here than the curTime was actually larger than the total trajectory time --> return the last segment
}

int LSPBTrajectory2::getCurSegment(double curTime, vec2D position)
{
	int index = floor(curTime / sampleTime);
	int curLeastDistanceIndex = 0;
	double curLeastDistance = 1000000000000.0; // in mm

	// start at time curTime and look for the sampled pose which is closest to the given pose
	while(distanceVec2D(position, vec2D(traj_pos[index].x,traj_pos[index].y)) < curLeastDistance)
	{

		curLeastDistance = distanceVec2D(position, vec2D(traj_pos[index].x,traj_pos[index].y));
		curLeastDistanceIndex = index;
		index++;
	}
	return getCurSegment(sampleTime * curLeastDistanceIndex);
}

int LSPBTrajectory2::signumWZero(double x)
{
	if(x < 0.0)
		return -1;
	else if(x == 0)
		return 0;
	else
		return 1;
}

int LSPBTrajectory2::signumWOZero(double x)
{
	if(x < 0.0)
		return -1;
	else
		return 1;
}



//struct vec2D LSPBTrajectory::generateCurrentPose()
//{
//	return NULL;
//}



double LSPBTrajectory2::getCurMaxAngAcc() const
{
    return curMaxAngAcc;
}

double LSPBTrajectory2::getCurMaxAngVel() const
{
    return curMaxAngVel;
}

double LSPBTrajectory2::getCurMaxLinAcc() const
{
    return curMaxLinAcc;
}

double LSPBTrajectory2::getCurMaxLinVel() const
{
    return curMaxLinVel;
}

void LSPBTrajectory2::logTrajToFile(const char *dataSetName)
{
	string namePos;
	string nameVel;
	namePos.append(dataSetName);
	nameVel.append(dataSetName);
	namePos.append("Pos.csv");
	nameVel.append("Vel.csv");

	logVectorVec2DToFile(traj_pos, namePos.c_str());
	logVectorVec2DToFile(traj_vel, nameVel.c_str());
}

void LSPBTrajectory2::logVectorVec2DToFile(vector<struct vec3D> vec, const char * filename)
{
	ofstream myfile;
	myfile.open(filename);

	for(int i=0; i < (int)vec.size(); i++)
	{
		myfile << vec[i].x << ", " << vec[i].y << ", " << vec[i].phi << ", ";
	}
	myfile.close();
}

template <typename T>
string LSPBTrajectory2::printVector(vector<T> vec)
{
	stringstream buf;
	for (int i=0; i<(int)vec.size(); i++)
	{
		buf << vec[i] << ", ";
	}
	buf << endl;
	return buf.str();
}

template string LSPBTrajectory2::printVector<double>(vector<double> vec);
template string LSPBTrajectory2::printVector<long>(vector<long> vec);



