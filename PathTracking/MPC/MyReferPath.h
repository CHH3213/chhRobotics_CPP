//
// Created by chh3213 on 2022/12/23.
//

#ifndef CHHROBOTICS_CPP_MYREFERPATH_H
#define CHHROBOTICS_CPP_MYREFERPATH_H
#include "../utils/MyReferencePath.h"
#include<Eigen/Dense>

using namespace Eigen;
struct refTraj{
    MatrixXd xref,dref;
    int ind;
};

struct parameters{
    int L;
    int NX,NU,T;
    double dt;
};

class MyReferPath: public MyReferencePath{

public:
    refTraj calc_ref_trajectory(vector<double> robot_state,parameters param, double dl = 1.0);


};


#endif //CHHROBOTICS_CPP_MYREFERPATH_H
