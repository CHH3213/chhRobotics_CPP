//
// Created by chh3213 on 2022/11/25.
//

#ifndef CHHROBOTICS_CPP_LQRCONTROL_H
#define CHHROBOTICS_CPP_LQRCONTROL_H

#define EPS 1.0e-4
#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;


class LQRControl {
private:
    int N;

public:
    LQRControl(int n);

    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    double lqrControl(vector<double>robot_state,vector<vector<double>>refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
};


#endif //CHHROBOTICS_CPP_LQRCONTROL_H
