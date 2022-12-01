//
// Created by chh3213 on 2022/11/30.
//

#ifndef CHHROBOTICS_CPP_REEDSSHEPP_H
#define CHHROBOTICS_CPP_REEDSSHEPP_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define EPS  1e-4
#define PI 3.14159265354

struct Path{
    vector<double> lengths;
    string modes;
    double L;
    vector<double>x,y,yaw,directions;
    //Path(const vector<double> &lengths, const string &modes, double l, const vector<double> &x,
    //     const vector<double> &y, const vector<double> &yaw, vector<double> directions);

};
/**
 * 根据[pythonRobotics代码](https://atsushisakai.github.io/PythonRobotics/modules/path_planning/reeds_shepp_path/reeds_shepp_path.html)写成c++代码
 */
class ReedsShepp {
private:

    double mod2Pi(double theta);

    vector<double> polar(double  x,double y);

    double PI2PI(double angle);

public:

    pair<bool,vector<double>>straightLeftStraight(double x, double y,double phi);

    vector<Path>setPath(vector<Path>paths,vector<double> lengths,string modes, double step_size);

    vector<Path>straightCurveStraight(double x, double y,double phi,vector<Path>paths,double step_size);

    pair<bool,vector<double>>leftStraightLeft(double x, double y,double phi);


    pair<bool,vector<double>>leftRightLeft(double x, double y,double phi);

    vector<Path>curveCurveCurve(double x, double y,double phi,vector<Path>paths,double step_size);

    vector<Path>curveStraightCurve(double x, double y,double phi,vector<Path>paths,double step_size);


    pair<bool,vector<double>>leftStraightRight(double x, double y,double phi);

    vector<Path>generatePath(vector<double>q0,vector<double>q1,double max_curvature, double step_size);

    vector<vector<double>>calInterpolateDistsList(vector<double> lengths,double step_size);

    vector<vector<double>>generateLocalCourse(vector<double> lengths,string modes,double max_curvature,double step_size);

    vector<double>interpolate(double dist,double length,char mode,double max_curvature,double origin_x, double origin_y, double origin_yaw);

    vector<Path>calPath(vector<double>start,vector<double>goal,double maxc,double step_size);

    Path reedsSheppPathPlanning(vector<double>start,vector<double>goal,double maxc,double step_size);

    void plotArrow(vector<double>  x,vector<double>  y,vector<double>  yaw, double length=1.0, double width=0.5, string fc="r", string ec="k");
};


#endif //CHHROBOTICS_CPP_REEDSSHEPP_H
