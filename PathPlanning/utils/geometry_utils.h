//
// Created by chh3213 on 23-11-5.
//

#ifndef CHHROBOTICS_CPP_GEOMETRY_UTILS_H
#define CHHROBOTICS_CPP_GEOMETRY_UTILS_H

#include <vector>
#include <cmath>

using namespace std;

struct Point {
    Point(double x_in, double y_in) : x(x_in), y(y_in) {};

    double x, y;
};

double distanceBetweenPoints(const Point &p1, const Point &p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

/**
 * compute segment p1p2's closed point to point p.
 * @param p1
 * @param p2
 * @param p
 * @return
 */
Point closestPointOnSegment(const Point &p1, const Point &p2, const Point &p) {
    double l2 = pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2);
    double t = ((p.x - p1.x) * (p2.x - p1.x) + (p.y - p1.y) * (p2.y - p1.y)) / l2;
    t = max(0.0, min(1.0, t));

    double closestX = p1.x + t * (p2.x - p1.x);
    double closestY = p1.y + t * (p2.y - p1.y);
    return {closestX, closestY};
}


#endif //CHHROBOTICS_CPP_GEOMETRY_UTILS_H
