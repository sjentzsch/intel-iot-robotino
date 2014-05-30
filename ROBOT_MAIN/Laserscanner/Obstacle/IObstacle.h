/*
 * IObstacle.h
 *
 *  Created on: May 3, 2014
 *      Author: sprofanter
 */

#ifndef IOBSTACLE_H_
#define IOBSTACLE_H_

class Point2D;
class Line2D;
class Circle2D;
#include <vector>

class IObstacle
{
    public:
        virtual ~IObstacle() {}
        /**
         * Get all intersection points of this obstacle and a line in 2D.
         * You need to delete the returned point array after usage.
         * @param line the line to check intersection with
         * @return NULL if no intersection or array containing the points.
         */
        virtual std::vector<Point2D>* getIntersection(const Line2D* line) const = 0;

        /**
         * Get all intersection points of this obstacle and a circle in 2D.
         * You need to delete the returned point array after usage.
         * @param line the line to check intersection with
         * @return NULL if no intersection or array containing the points.
         */
        virtual std::vector<Point2D>* getIntersection(const Circle2D* circle) const = 0;
};



#endif /* IOBSTACLE_H_ */
