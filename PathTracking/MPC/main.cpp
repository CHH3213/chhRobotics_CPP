//
// Created by chh3213 on 2022/11/25.
//
#include "MPCControl.h"
#include "../utils/MyReferencePath.h"
#include "../utils/KinematicModel.h"
#include "../utils/NormalizeAngle.hpp"
#include "../../matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
    parameters param;
    //Number of states.
    param.NX = 3;
    // Number of control inputs.
    param.NU = 3;
    // Horizon length.
    param.T = 3;
    param.L = 2.0;
    param.dt = 0.1;


    // Initialize MPC controller.
    MPCControl mpc(param.NX, param.NU, param.T);

    // Define initial state and reference trajectory.
    Eigen::VectorXd x0(param.NX);
    x0 << 0.0, -3.0, 0.0;  // Initial state [x, y, yaw]
    vector<double> robot_state = {0.0, -3.0, 0.0, 0.0};
    // Create an instance of your KinematicModel class
    double dt = 0.1;  // Time interval
    double L = 2.0;   // Vehicle wheelbase
    // Define reference trajectory using your MyReferPath class
    MyReferencePath reference_path;
    auto reference_trajectory = reference_path.calc_ref_trajectory(robot_state, param, 1.0);
    KinematicModel ugv(x0(0), x0(1), x0(2), 2.0, L, dt);


    // Main loop
    std::vector<double> x_history;
    std::vector<double> y_history;

    for (int i = 0; i < param.T; ++i) {
        Eigen::MatrixXd xref = reference_trajectory.xref;
        Eigen::VectorXd xref_i = xref.col(i);
        Eigen::VectorXd ref_delta = reference_trajectory.dref.col(i);

        // Call the MPC controller to compute control inputs
        std::vector<double> control_inputs = mpc.linearMPCControl(xref_i, x0, ref_delta, ugv);

        // Update the state using your KinematicModel class
        ugv.updateState(control_inputs[0], control_inputs[1]);

        // Store state for plotting
        x_history.push_back(ugv.getState()[0]);
        y_history.push_back(ugv.getState()[1]);

        // Update the initial state for the next iteration
        const auto state = ugv.getState();
        x0 << state[0], state[1], state[2];  // Initial state [x, y, yaw]

    }

    // Plot the results using matplotlibcpp or any other plotting library of your choice
    //plt::plot(reference_trajectory.xref.row(0), reference_trajectory.xref.row(1), "-.b");
    plt::plot(x_history, y_history, "-r");
    //plt::plot(reference_trajectory.xref(0, param.T), reference_trajectory.xref(1, param.T), "go", "target");
    plt::grid(true);
    plt::show();
//
    return 0;
}

