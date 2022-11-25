//
// Created by chh3213 on 2022/11/25.
//

#include "MPCControl.h"

MPCControl::MPCControl(int nx, int nu, int t) : NX(nx), NU(nu), T(t) {}

double MPCControl::linearMPCControl(vector<double> xref, double x0, vector<double> ref_delta, KinematicModel ugv) {



}
