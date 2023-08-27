//
// Created by chh3213 on 23-8-19.
//
#include "LateralErrorModel.h"

LateralErrorModel::LateralErrorModel(double m, double Vx, double C_alpha_f, double C_alpha_r, double l_f, double l_r, double I_z, double g)
        : m(m), Vx(Vx), C_alpha_f(C_alpha_f), C_alpha_r(C_alpha_r), l_f(l_f), l_r(l_r), I_z(I_z), g(g) {}

std::vector<Eigen::MatrixXd> LateralErrorModel::GenerateStateSpace() {
    Eigen::MatrixXd A(4, 4);
    A << 0, 1, 0, 0,
            0, -(2 * C_alpha_f + 2 * C_alpha_r) / (m * Vx), (2 * C_alpha_f + 2 * C_alpha_r) / m, (-2 * C_alpha_f * l_f + 2 * C_alpha_r * l_r) / (m * Vx),
            0, 0, 0, 1,
            0, -(2 * l_f * C_alpha_f - 2 * l_r * C_alpha_r) / (I_z * Vx), (2 * l_f * C_alpha_f - 2 * l_r * C_alpha_r) / I_z, (-2 * l_f * l_f * C_alpha_f + 2 * l_r * l_r * C_alpha_r) / (I_z * Vx);

    Eigen::MatrixXd B(4, 1);
    B << 0, 2 * C_alpha_f / m, 0, 2 * l_f * C_alpha_f / I_z;

    Eigen::MatrixXd C(4, 1);
    C << 0, (-2 * C_alpha_f * l_f + 2 * C_alpha_r * l_r) / (m * Vx) - Vx, 0, (-2 * l_f * l_f * C_alpha_f + 2 * l_r * l_r * C_alpha_r) / (I_z * Vx);

    Eigen::MatrixXd D(4, 1);
    D << 0, g, 0, 0;

    return {A, B, C, D};
}

Eigen::VectorXd LateralErrorModel::compute_state_derivative(const Eigen::VectorXd& state, double delta, double psi_des, double phi) {
    auto ABCD = GenerateStateSpace();
    const Eigen::MatrixXd& A = ABCD[0];
    const Eigen::MatrixXd& B = ABCD[1];
    const Eigen::MatrixXd& C = ABCD[2];
    const Eigen::MatrixXd& D = ABCD[3];

    Eigen::VectorXd state_dot = A * state + B * delta + C * psi_des + D * std::sin(phi);
    return state_dot;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> LateralErrorModel::DiscreteStateSpace(double dt) {
    auto ABCD = GenerateStateSpace();
    const Eigen::MatrixXd& A = ABCD[0];
    const Eigen::MatrixXd& B = ABCD[1];

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd A_bar = I + A * dt;
    Eigen::MatrixXd B_bar = B * dt;

    return std::make_pair(A_bar, B_bar);
}