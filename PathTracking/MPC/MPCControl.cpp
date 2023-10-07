#include "MPCControl.h"

MPCControl::MPCControl(int nx, int nu, int t) : NX(nx), NU(nu), T(t) {}

vector<double> MPCControl::linearMPCControl(MatrixXd xref, Vector3d x0, MatrixXd ref_delta, KinematicModel ugv) {
    int NX = xref.rows();
    int NU = ref_delta.rows();
    int T = xref.cols() - 1;  // Horizon length.

    // Define optimization variables.
    MatrixXd x(NX, T + 1);
    MatrixXd u(NU, T);
    // Store A matrices.
    vector<MatrixXd> A_vec;
    // Store B matrices.
    vector<MatrixXd> B_vec;

    // Initialize A and B matrices.
    for (int t = 0; t < T; ++t) {
        auto state_space = ugv.stateSpace(ref_delta(1, t), xref(2, t));
        A_vec.push_back(state_space[0]);
        B_vec.push_back(state_space[1]);
    }

    // Define the optimization problem.
    VectorXd cost(T + 1);
    // List of constraint indices.
    vector<vector<int>> constraints;

    for (int t = 0; t < T; ++t) {
        cost(t) = (u.col(t) - ref_delta.col(t)).transpose() * R * (u.col(t) - ref_delta.col(t));

        if (t != 0) {
            cost(t) += (x.col(t) - xref.col(t)).transpose() * Q * (x.col(t) - xref.col(t));
        }

        MatrixXd A = A_vec[t];
        MatrixXd B = B_vec[t];

        constraints.push_back({(t + 1) * NX, (t + 1) * NX + NX});  // State constraints.
        constraints.push_back({t * NU, t * NU + NU});  // Input constraints.

        x.col(t + 1) = A * x.col(t) + B * (u.col(t) - ref_delta.col(t));
    }

    // Final state cost.
    cost(T) = (x.col(T) - xref.col(T)).transpose() * Qf * (x.col(T) - xref.col(T));

    // Set initial state.
    x.col(0) = x0;

    // Set up bounds.
    VectorXd lower_bound(T * NU);
    VectorXd upper_bound(T * NU);

    for (int t = 0; t < T; ++t) {
        lower_bound.segment(t * NU, NU) << -MAX_VEL, -MAX_STEER;
        upper_bound.segment(t * NU, NU) << MAX_VEL, MAX_STEER;
    }

    // Solve the optimization problem.
    OsqpEigen::Solver solver;

    solver.data()->setNumberOfVariables(NX * (T + 1));
    solver.data()->setNumberOfConstraints(T * (NU + NX));

    // Define the Hessian matrix dimension.
    int N = NX * (T + 1);

    // Define and set the Hessian matrix.
    Eigen::SparseMatrix<double> P(N, N);

    for (int t = 0; t < T; ++t) {
        for (int i = 0; i < NU; ++i) {
            P.coeffRef(t * NU + i, t * NU + i) = R(i, i);
        }
    }

    if (!solver.data()->setHessianMatrix(P)) {
        cerr << "Error setting Hessian matrix." << endl;
        return {};
    }

    // Define the gradient vector (cost vector).
    VectorXd q(N);

    for (int t = 0; t < T; ++t) {
        q.segment(t * NU, NU) = cost(t) * VectorXd::Ones(NU);
    }

    if (!solver.data()->setGradient(q)) {
        cerr << "Error setting gradient vector." << endl;
        return {};
    }

    // Define the linear equality constraint matrix Aeq.
    int M = T * NX;  // Number of equality constraints.
    Eigen::SparseMatrix<double> Aeq(M, N);

    // Define the equality constraint vector beq.
    VectorXd beq(M);

    // You should populate Aeq and beq based on your state dynamics.

    // Set lower and upper bounds for variables and constraints.
    if (!solver.data()->setLowerBound(lower_bound)) {
        cerr << "Error setting lower bound." << endl;
        return {};
    }

    if (!solver.data()->setUpperBound(upper_bound)) {
        cerr << "Error setting upper bound." << endl;
        return {};
    }

    // Initialize the solver.
    if (!solver.initSolver()) {
        cerr << "Error initializing solver." << endl;
        return {};
    }

    // Solve the problem.
    if (solver.solve() != 0) {
        cerr << "Error solving the optimization problem." << endl;
        return {};
    }
    VectorXd optimal_solution = solver.getSolution();

    // Extract optimal control inputs.
    vector<double> optimal_input;

    for (int t = 0; t < T; ++t) {
        VectorXd u_t = optimal_solution.segment(t * NU, NU);
        optimal_input.push_back(u_t(0));  // Extract the velocity input.
    }

    return optimal_input;
}

