/*
 * Geometry.cpp
 *
 *  Created on: May 3, 2014
 *      Author: root
 */

#include <cstdlib>
#include <cmath>
#include "Geometry.h"
using namespace std;

void Point2D::rotateByAngle(float theta) {
	float cs = cos(theta);
	float sn = sin(theta);

	float tmpX = x * cs - y * sn;
	float tmpY = x * sn + y * cs;
	this->x = tmpX;
	this->y = tmpY;
}

ostream& operator<<(ostream& os, const Point2D& p)
{
    os << p.x << "|" << p.y;
    return os;
}


std::vector<Point2D>* Line2D::getIntersection(const Line2D *line) const {
	//http://flassari.is/2008/11/line-line-intersection-in-cplusplus/

	// Store the values for fast access and easy
	// equations-to-code conversion
	const float x1 = line->p1.x, x2 = line->p2.x, x3 = this->p1.x, x4 = this->p2.x;
	const float y1 = line->p1.y, y2 = line->p2.y, y3 = this->p1.y, y4 = this->p2.y;

	float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (fabs(d) <= 1e-6) return NULL;

	// Get the x and y
	float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
	float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
	float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

	// Check if the x and y coordinates are within both lines
	if ( x < min(x1, x2) || x > max(x1, x2) ||
	x < min(x3, x4) || x > max(x3, x4) ) return NULL;
	if ( y < min(y1, y2) || y > max(y1, y2) ||
	y < min(y3, y4) || y > max(y3, y4) ) return NULL;

	// Return the point of intersection
	std::vector<Point2D> *ret = new std::vector<Point2D>();
	ret->push_back(Point2D(x,y));
	return ret;
}


std::vector<Point2D>* Line2D::getIntersection(const Circle2D *circle) const {
	const Point2D *C = &circle->center;
	const Point2D *A = &this->p1;
	const Point2D *B = &this->p2;
	const float r = circle->radius;

    float dx = B->x - A->x;
    float dy = B->y - A->y;

	float a = dx * dx + dy * dy;
	float b = 2 * (dx * (A->x - C->x) +dy * (A->y - C->y));
	float cc = C->x * C->x + C->y * C->y + A->x * A->x + A->y * A->y - 2 * (C->x * A->x + C->y * A->y) - r * r;
	float deter = b * b - 4 * a * cc;
	if (deter <= 0 ) {
        // No real solutions.
        return NULL;
	} else {
		float e = sqrt(deter);
		float u1 = ( - b + e ) / (2 * a );
		float u2 = ( - b - e ) / (2 * a );
		if ((u1 < 0 || u1 > 1) && (u2 < 0 || u2 > 1)) {
			// either within circle or not intersecting
			return NULL;
		} else {

	    	std::vector<Point2D> *ret = new std::vector<Point2D>();
			if (0 <= u2 && u2 <= 1) {
		        ret->push_back(Point2D(A->x + u1 * dx, A->y + u1 * dy));
			}
			if (0 <= u1 && u1 <= 1) {
		        ret->push_back(Point2D(A->x + u2 * dx, A->y + u2 * dy));
			}
			return ret;
		}
	}
}

/*
std::vector<Point2D>* Line2D::getIntersection(const Circle2D *circle) const {
	//http://blog.csharphelper.com/2010/03/28/determine-where-a-line-intersects-a-circle-in-c.aspx

	const float cx = circle->center.x;
	const float cy = circle->center.y;
	const float radius = circle->radius;
	const Point2D *point1 = &this->p1;
	const Point2D *point2 = &this->p2;

    float dx, dy, A, B, C, det, t;

    dx = point2->x - point1->x;
    dy = point2->y - point1->y;

    A = dx * dx + dy * dy;
    B = 2 * (dx * (point1->x - cx) + dy * (point1->y - cy));
    C = (point1->x - cx) * (point1->x - cx) + (point1->y - cy) * (point1->y - cy) - radius * radius;

    det = B * B - 4 * A * C;
    if ((A <= 0.0000001) || (det < 0))
    {
        // No real solutions.
        return NULL;
    }
    else if (det == 0)
    {
        // One solution.
        t = -B / (2 * A);
    	std::vector<Point2D> *ret = new std::vector<Point2D>();
    	ret->push_back(Point2D(point1->x + t * dx, point1->y + t * dy));
    	return ret;
    }
    else
    {
    	cout << "\t2 Points: " << circle->center << " r= " << radius <<  << endl;
        // Two solutions.
    	std::vector<Point2D> *ret = new std::vector<Point2D>();
    	float detSqrt = sqrt(det);
        t = (float)((-B + detSqrt) / (2 * A));
        ret->push_back(Point2D(point1->x + t * dx, point1->y + t * dy));
        t = (float)((-B - detSqrt) / (2 * A));
        ret->push_back(Point2D(point1->x + t * dx, point1->y + t * dy));
        return ret;
    }
}
*/
std::vector<Point2D>* Circle2D::getIntersection(const Line2D *line) const {
	return line->getIntersection(this);
}

std::vector<Point2D>* Circle2D::getIntersection(const Circle2D *circle) const {
	// http://paulbourke.net/geometry/circlesphere/

	const float x0 = this->center.x, y0 = this->center.y, r0 = this->radius;
	const float x1 = circle->center.x,y1 = circle->center.y,r1 = circle->radius;

	double a, dx, dy, d, h, rx, ry;
	double x2, y2;

	/* dx and dy are the vertical and horizontal distances between
	* the circle centers.
	*/
	dx = x1 - x0;
	dy = y1 - y0;

	/* Determine the straight-line distance between the centers. */
	//d = sqrt((dy*dy) + (dx*dx));
	d = hypot(dx,dy); // Suggested by Keith Briggs

	/* Check for solvability. */
	if (d > (r0 + r1))
	{
		/* no solution. circles do not intersect. */
		return NULL;
	}
	if (d < fabs(r0 - r1))
	{
		/* no solution. one circle is contained in the other */
		return NULL;
	}

	/* 'point 2' is the point where the line through the circle
	* intersection points crosses the line between the circle
	* centers.
	*/

	/* Determine the distance from point 0 to point 2. */
	a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

	/* Determine the coordinates of point 2. */
	x2 = x0 + (dx * a/d);
	y2 = y0 + (dy * a/d);

	/* Determine the distance from point 2 to either of the
	* intersection points.
	*/
	h = sqrt((r0*r0) - (a*a));

	/* Now determine the offsets of the intersection points from
	* point 2.
	*/
	rx = -dy * (h/d);
	ry = dx * (h/d);

	/* Determine the absolute intersection points. */
	std::vector<Point2D> *ret = new std::vector<Point2D>();
    ret->push_back(Point2D(x2 + rx, y2 + ry));
    ret->push_back(Point2D(x2 - rx, y2 - ry));
	return ret;
}
