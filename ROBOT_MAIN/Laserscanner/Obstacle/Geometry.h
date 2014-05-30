/*
 * Geometry.h
 *
 *  Created on: May 3, 2014
 *      Author: root
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include "IObstacle.h"
#include <iostream>
using namespace std;

class Point2D {
public:
	float x;
	float y;
	Point2D():x(0),y(0){};
	Point2D(float _x, float _y) : x(_x), y(_y){};
	void rotateByAngle(float theta);
	inline float distToSqrt(Point2D &p){
		return pow(this->x - p.x,2)+pow(this->y - p.y,2);
	};

    friend ostream& operator<<(ostream& os, const Point2D& dt);
};

class Line2D : public IObstacle {
public:
	Point2D p1;
	Point2D p2;
	Line2D():p1(), p2(){};
	Line2D(Point2D _p1, Point2D _p2) : p1(_p1), p2(_p2){};
    virtual std::vector<Point2D>* getIntersection(const Line2D* line) const;
    virtual std::vector<Point2D>* getIntersection(const Circle2D* circle) const;
};

class Circle2D : public IObstacle {
public:
	Point2D center;
	float radius;
	Circle2D():center(), radius(0){};
	Circle2D(Point2D _center, float _radius) : center(_center), radius(_radius){};
    virtual std::vector<Point2D>* getIntersection(const Line2D* line) const;
    virtual std::vector<Point2D>* getIntersection(const Circle2D* circle) const;
};

#endif /* GEOMETRY_H_ */
