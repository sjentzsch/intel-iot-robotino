#include "pose.h"

/*
 * wraps every angle in degtree to the standard range -180°..+180°
 */
double wrapDegToStdRange(double phi)
{
	double rad = DEGTORAD(phi);
	return RADTODEG(atan2(sin(rad), cos(rad)));
}

/*
 * return the signed smallest difference between to angles
 * angles given in degree, return value in degree (-180 .. +180)
 * e.g.: goal = 10°, phi = 5°---> return 5°
 * e.g.  goal = 10°, phi = 15° --> return -5°
 * e.g.  goal = 170°, phi = -170° --> return -20°
 *
 */
double diffAngle(double goal, double phi)
{
	goal = wrapDegToStdRange(goal);
	phi = wrapDegToStdRange(phi);
	double diffPhi = goal - phi;
	if(diffPhi < -180.0)
	{
		diffPhi += 360;
	}
	else if(diffPhi > 180.0)
	{
		diffPhi -= 360;
	}
	return diffPhi;
}

/*
 * converts the angular direction of 'next' into a degree representation which is closest to start
 * e.g.: start = 170°, next = -170° ---> return 190°
 *
 * return value is NOT in -180..+180
 */
double eliminateAngleWrapAround(double start, double next)
{
	double startInRange = wrapDegToStdRange(start);
	double nextInRange = wrapDegToStdRange(next);
	double diff = diffAngle(nextInRange, startInRange);
	return start + diff;
}

/*
 * // ration: value between 0...1 determining how close we are to in_high (0 --> returns in_low, 1 --> returns in_high)
 * interpolate linearly a vec2D
 */
vec2D linearInterpolateVec2D(struct vec2D in_low, struct vec2D in_high, double ratio)
{
	struct vec2D result;
	result.x = in_low.x + (in_high.x - in_low.x)*ratio;
	result.y = in_low.y + (in_high.y - in_low.y)*ratio;
	return result;
}

/*
 * returns the euclidean distance between to positions
 */
double distanceVec2D(vec2D a, vec2D b)
{
	return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

/*
 * // ration: value between 0...1 determining how close we are to in_high (0 --> returns in_low, 1 --> returns in_high)
 * interpolate linear between angles
 */
double linearInterpolateAngle(double phi_low, double phi_high, double ratio)
{
	double phi_low_InRange = wrapDegToStdRange(phi_low);
	double phi_high_InRange = wrapDegToStdRange(phi_high);
	double diff = diffAngle(phi_high_InRange, phi_low_InRange);
	return wrapDegToStdRange(phi_low + ratio*diff);
}

/*
 * // ration: value between 0...1 determining how close we are to in_high (0 --> returns in_low, 1 --> returns in_high)
 * interpolate linear between a scalar
 */
double linearInterpolateScalar(double a_low, double a_high, double ratio)
{
	return a_low + (a_high - a_low)*ratio;
}


string printVec2D(vector<struct vec2D> vec)
{
	stringstream buf;
	for (int i=0; i<(int)vec.size(); i++)
	{
		buf << vec[i].toString();
	}
	buf << endl;
	return buf.str();
}

string printVec3D(vector<struct vec3D> vec)
{
	stringstream buf;
	for (int i=0; i<(int)vec.size(); i++)
	{
		buf << vec[i].toString();
	}
	buf << endl;
	return buf.str();
}

string printVec3D(list<struct vec3D> vec)
{
	stringstream buf;
	list<vec3D>::iterator it;
	for (it=vec.begin(); it!=vec.end(); it++)
	{
		buf << it->toString();
	}
	buf << endl;
	return buf.str();
}

