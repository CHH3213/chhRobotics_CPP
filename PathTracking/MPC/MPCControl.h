//
// Created by chh3213 on 2022/11/25.
//

#ifndef CHHROBOTICS_CPP_MPCCONTROL_H
#define CHHROBOTICS_CPP_MPCCONTROL_H

#include <iostream>
#include<vector>
#include<Eigen/Dense>
#include "../utils/KinematicModel.h"
#include "OsqpEigen/OsqpEigen.h"
using namespace std;
using namespace Eigen;
using namespace OsqpEigen;

class MPCControl {
private:
    int NX,NU,T;
public:
    MPCControl(int nx, int nu, int t);

    double linearMPCControl(vector<double>xref,double x0, vector<double> ref_delta, KinematicModel ugv);
};


#endif //CHHROBOTICS_CPP_MPCCONTROL_H
