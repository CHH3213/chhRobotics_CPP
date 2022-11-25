//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_MYREFERENCEPATH_H
#define CHHROBOTICS_CPP_MYREFERENCEPATH_H
#include <iostream>
#include <vector>
#include<cmath>
#include <algorithm>
using namespace std;
#define PI 3.1415926

class MyReferencePath {
public:
    //refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
    vector<vector<double>>refer_path;
    vector<double>refer_x,refer_y;
public:
    MyReferencePath();
    vector<double> calcTrackError(vector<double>robot_state);
    double normalizeAngle(double angle);
};


#endif //CHHROBOTICS_CPP_MYREFERENCEPATH_H
