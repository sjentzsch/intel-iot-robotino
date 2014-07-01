#ifndef POSE_H_
#define POSE_H_

#include <iostream>
//#include <math>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <sstream>
#include "config.h"

using namespace std;

// struct for via-points for trajectory generation
struct vec2D
{
	double x;
	double y;
	vec2D(){}
	vec2D(double x_, double y_) : x(x_), y(y_){}

	string toString(){
		stringstream buf;
		buf << "( " << x << ", " << y << " ) ";
		return buf.str();
	};
};

// struct for MotorController
struct vec3D
{
	float x;
	float y;
	float phi;
	vec3D(){}
	vec3D(float x_, float y_, float phi_) : x(x_), y(y_), phi(phi_){}
	void set(float x_, float y_, float phi_)
	{
		this->x = x_;
		this->y = y_;
		this->phi = phi_;
	}

	string toString(){
		stringstream buf;
		buf << "( " << x << ", " << y << ", "<< phi << ") ";
		return buf.str();
	};
};

/*
 * wraps every angle in degtree to the standard range -180°..+180°
 */
double wrapDegToStdRange(double phi);

/*
 * return the signed smallest difference between to angles
 * angles given in degree, return value in degree (-180 .. +180)
 * e.g.: goal = 10°, phi = 5°---> return 5°
 * e.g.  goal = 10°, phi = 15° --> return -5°
 * e.g.  goal = 170°, phi = -170° --> return -20°
 *
 */
double diffAngle(double goal, double phi);

/*
 * converts the angular direction of 'next' into a degree representation which is closest to start
 * e.g.: start = 170°, next = -170° ---> return 190°
 *
 * return value is NOT in -180..+180
 */
double eliminateAngleWrapAround(double start, double next);

/*
 * // ration: value between 0...1 determining how close we are to in_high (0 --> returns in_low, 1 --> returns in_high)
 * interpolate linearly a vec2D
 */
vec2D linearInterpolateVec2D(struct vec2D in_low, struct vec2D in_high, double ratio);

/*
 * returns the euclidean distance between to positions
 */
double distanceVec2D(vec2D a, vec2D b);

/*
 * // ration: value between 0...1 determining how close we are to in_high (0 --> returns in_low, 1 --> returns in_high)
 * interpolate linear between angles
 */
double linearInterpolateAngle(double phi_low, double phi_high, double ratio);

/*
 * // ration: value between 0...1 determining how close we are to in_high (0 --> returns in_low, 1 --> returns in_high)
 * interpolate linear between a scalar
 */
double linearInterpolateScalar(double a_low, double a_high, double ratio);


string printVec2D(vector<struct vec2D> vec);

string printVec3D(vector<struct vec3D> vec);

string printVec3D(list<struct vec3D> vec);



#endif /* POSE_H_ */

