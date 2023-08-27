//
// Created by chh3213 on 23-8-19.
//

#ifndef CHHROBOTICS_CPP_LATERALERRORMODEL_H
#define CHHROBOTICS_CPP_LATERALERRORMODEL_H
#include <iostream>
#include <vector>
#include <cmath>
#include<Eigen/Dense>

using namespace std;

class LateralErrorModel {
    public:
        LateralErrorModel(double m, double Vx, double C_alpha_f, double C_alpha_r, double l_f, double l_r, double I_z, double g);

        std::vector<Eigen::MatrixXd> GenerateStateSpace();
        Eigen::VectorXd compute_state_derivative(const Eigen::VectorXd& state, double delta, double psi_des, double phi);
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DiscreteStateSpace(double dt);
    private:
        double m, Vx, C_alpha_f, C_alpha_r, l_f, l_r, I_z, g;
};


#endif //CHHROBOTICS_CPP_LATERALERRORMODEL_H
