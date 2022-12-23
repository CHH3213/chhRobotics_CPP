//
// Created by chh3213 on 2022/11/25.
//

#include "MPCControl.h"

MPCControl::MPCControl(int nx, int nu, int t) : NX(nx), NU(nu), T(t) {}

vector<double> MPCControl::linearMPCControl(MatrixXd xref,double x0, MatrixXd ref_delta, KinematicModel ugv) {

    Solver solver;
    solver.data()->setNumberOfVariables(NX);


}
