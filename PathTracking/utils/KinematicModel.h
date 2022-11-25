//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_KINEMATICMODEL_H
#define CHHROBOTICS_CPP_KINEMATICMODEL_H
#include <iostream>
#include <vector>
#include <cmath>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel {
public:
    double x,y,psi,v,L,dt;
public:
    KinematicModel();

    KinematicModel(double x, double y, double psi, double v, double l, double dt);

   vector<double>getState();

    void updateState(double accel, double delta_f);

    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);

};


#endif //CHHROBOTICS_CPP_KINEMATICMODEL_H
