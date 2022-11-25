//
// Created by chh3213 on 2022/11/25.
//

#ifndef CHHROBOTICS_CPP_BEZIERCURVE_H
#define CHHROBOTICS_CPP_BEZIERCURVE_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

double factorial(int n);

Vector2d bezierCommon(vector<Vector2d>Ps,  double t);

Vector2d bezierRecursion(vector<Vector2d>Ps, double t);

#endif //CHHROBOTICS_CPP_BEZIERCURVE_H
