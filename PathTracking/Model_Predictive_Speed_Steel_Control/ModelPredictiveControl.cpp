//
// Created by chh3213 on 2022/12/23.
//

#include "ModelPredictiveControl.h"

/**
 * 车辆状态更新
 * @param state
 * @param a 加速度
 * @param delta 转角控制量
 */
void update_state(State &state, double a, double delta) {
    if (delta >= MAX_STEER) delta = MAX_STEER;
    if (delta <= - MAX_STEER) delta = - MAX_STEER;

    state.x = state.x + state.v * cos(state.yaw) * DT;
    state.y = state.y + state.v * sin(state.yaw) * DT;
    state.yaw = state.yaw + state.v / WB * CppAD::tan(delta) * DT;
    state.v = state.v + a * DT;

    if (state.v > MAX_SPEED) state.v = MAX_SPEED;
    if (state.v < MIN_SPEED) state.v = MIN_SPEED;
}

/**
 * 计算参考轨迹上离当前状态最近的路点
 * @param state
 * @param cx
 * @param cy
 * @param cyaw
 * @param pind
 * @return
 */
int calc_nearest_index(State state, vector<double> cx, vector<double> cy, vector<double> cyaw, int pind) {
    double mind = numeric_limits<double>::max();
    double ind = 0;
    for(int i=pind; i<pind+N_IND_SEARCH; i++){
        double idx = cx[i] - state.x;
        double idy = cy[i] - state.y;
        double d_e = idx*idx + idy*idy;

        if (d_e<mind){
            mind = d_e;
            ind = i;
        }
    }

    // double dxl = cx[ind] - state.x;
    // double dyl = cy[ind] - state.y;
    // double angle = YAW_P2P(cyaw[ind] - atan2(dyl, dxl));
    // if (angle < 0) mind = mind * -1;

    return ind;
}


vector<double> calc_speed_profile(vector<double> rx, vector<double> ry, vector<double> ryaw, double target_speed) {
    vector<double> speed_profile(rx.size(), target_speed);

    double direction = 1.0; //forward
    //Set stop point
    for( int i=0; i < rx.size()-1; i++){
        double dx = rx[i+1] - rx[i];
        double dy = ry[i+1] - ry[i];
        double move_direction = atan2(dy, dx);

        if (dx != 0.0 && dy != 0.0){
            double dangle = abs(YAW_P2P(move_direction - ryaw[i]));
            if (dangle >= M_PI/4.0) direction = -1.0;
            else direction = 1.0;
        }

        if (direction != 1.0) speed_profile[i] = -1 * target_speed;
        else speed_profile[i] = target_speed;

    }
    speed_profile[rx.size()-1] = 0.0;

    return speed_profile;
}

/**
 * 参考轨迹
 * @param state
 * @param cx
 * @param cy
 * @param cyaw
 * @param ck
 * @param sp
 * @param dl
 * @param target_ind
 * @param xref
 */
void calc_ref_trajectory(State state, vector<double> cx, vector<double> cy, vector<double> cyaw, vector<double> ck,
                         vector<double> sp, double dl, int &target_ind, M_XREF &xref) {
    xref = M_XREF::Zero();
    // dref = Matrix<double, 1, T>::Zero();

    int ncourse = cx.size();

    int ind = calc_nearest_index(state, cx, cy, cyaw, target_ind);
    if (target_ind >= ind) ind = target_ind;

    xref(0, 0) = cx[ind];
    xref(1, 0) = cy[ind];
    xref(2, 0) = cyaw[ind];
    xref(3, 0) = sp[ind];

    double travel = 0.0;

    for(int i=0; i<T; i++){
        travel += abs(state.v) * DT;
        int dind = (int)round(travel/dl);
        // int dind = i;


        if ((ind+dind)<ncourse){
            xref(0, i) = cx[ind + dind];
            xref(1, i) = cy[ind + dind];
            xref(2, i) = cyaw[ind + dind];
            xref(3, i) = sp[ind + dind];
            // dref(0, i) = 0.0;
        }else{
            xref(0, i) = cx[ncourse - 1];
            xref(1, i) = cy[ncourse - 1];
            xref(2, i) = cyaw[ncourse - 1];
            xref(3, i) = sp[ncourse - 1];
            // dref(0, i) = 0.0;
        }
    }

    target_ind = ind;

}

void smooth_yaw(vector<double> &cyaw) {
    for(int i=0; i<cyaw.size()-1; i++){
        double dyaw = cyaw[i+1] - cyaw[i];

        while (dyaw > M_PI/2.0){
            cyaw[i+1] -= M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
        while (dyaw < -M_PI/2.0){
            cyaw[i+1] += M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
    }
}

vector<double> mpc_solve(State x0, M_XREF traj_ref) {
    typedef CPPAD_TESTVECTOR(double) Dvector;
    double x = x0.x;
    double y = x0.y;
    double yaw = x0.yaw;
    double v = x0.v;

    size_t n_vars = T * 4 + (T - 1) * 2;
    size_t n_constraints = T * 4;

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++){
        vars[i] = 0.0;
    }

    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = v;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    // NOTE there mush be both lower and upper bounds for all vars!!!!!
    for (int i = 0; i < n_vars; i++) {
        vars_lowerbound[i] = -10000000.0;
        vars_upperbound[i] = 10000000.0;
    }

    for (int i = delta_start; i < delta_start+T-1; i++) {
        vars_lowerbound[i] = -MAX_STEER;
        vars_upperbound[i] = MAX_STEER;
    }

    for (int i = a_start; i < a_start+T-1; i++) {
        vars_lowerbound[i] = -MAX_ACCEL;
        vars_upperbound[i] = MAX_ACCEL;
    }

    for (int i = v_start; i < v_start+T; i++) {
        vars_lowerbound[i] = MIN_SPEED;
        vars_upperbound[i] = MAX_SPEED;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[yaw_start] = yaw;
    constraints_lowerbound[v_start] = v;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[yaw_start] = yaw;
    constraints_upperbound[v_start] = v;

    FG_EVAL fg_eval(traj_ref);

    // options
    string options;
    options += "Integer print_level  0\n";
    // options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Integer max_iter      50\n";
    // options += "Numeric tol          1e-6\n";
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_EVAL>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    vector<double> result;
    for (int i =0 ; i < n_vars; i++) {
        result.push_back((double)solution.x[i]);
    }
    return result;
}

void mpc_simulation(vector<double> cx, vector<double> cy, vector<double> cyaw, vector<double> ck,
                    vector<double> speed_profile, vector<double> goal) {

    State state(cx[0], cy[0], cyaw[0], speed_profile[0]);

    if ((state.yaw - cyaw[0]) >= M_PI) state.yaw -= M_PI * 2.0;
    else if ((state.yaw - cyaw[0]) <= -1.0*M_PI) state.yaw += M_PI * 2.0;

    double goal_dis = 0.5;
    int iter_count = 0;

    int target_ind = 0;
    calc_nearest_index(state, cx, cy, cyaw, target_ind);

    smooth_yaw(cyaw);

    // visualization

    vector<double> x_h,y_h,time_h,vel_h;//画图用

    M_XREF xref;

    while (MAX_TIME >= iter_count){
        calc_ref_trajectory(state, cx, cy, cyaw, ck, speed_profile, 1.0, target_ind, xref);

        vector<double> output = mpc_solve(state, xref);


        //cout<<output.size()<<endl;
        update_state(state, output[a_start], output[delta_start]);

        double steer = output[delta_start];

        double dx = state.x - goal[0];
        double dy = state.y - goal[1];
        if (sqrt(dx*dx + dy*dy) <= goal_dis) {
            cout<<("Goal")<<endl;
            break;
        }

        x_h.push_back(state.x);
        y_h.push_back(state.y);
        time_h.push_back(iter_count);
        vel_h.push_back(state.v);

        // visualization
        plt::cla();
        plt::figure(1);
        plt::plot(x_h,y_h,"ob");//实际轨迹
        plt::plot(cx,cy,"-r");//参考轨迹
        plt::plot({cx[target_ind]}, {cy[target_ind]}, "xg");//当前跟踪目标点
        plt::plot({goal[0]}, {goal[1]}, "og");//终点

        plt::xlabel("x[m]");
        plt::ylabel("y[m]");

        plt::axis("equal");
        plt::grid(true);

        plt::figure(2);
        plt::plot(time_h,vel_h,"-g");
        plt::xlabel("Time [s]");
        plt::ylabel("Speed [kmh]");
        plt::grid(true);

        plt::pause(0.0001);
        iter_count++;
    }
    // save figure
    const char* filename = "./mpc_speed_steel_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
}

FG_EVAL::FG_EVAL(const M_XREF &trajRef) : traj_ref(trajRef) {}

void FG_EVAL::operator()(FG_EVAL::ADvector &fg, const FG_EVAL::ADvector &vars) {
    fg[0] = 0;

    for(int i=0; i<T-1; i++){
        fg[0] +=  0.01 * CppAD::pow(vars[a_start+i], 2);
        fg[0] += 0.01 * CppAD::pow(vars[delta_start+i], 2);
    }

    for(int i=0; i<T-2; i++){
        fg[0] += 0.01 * CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
        fg[0] += 1 * CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
    }

    // fix the initial state as a constraint
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];

    // fg[0] += CppAD::pow(traj_ref(0, 0) - vars[x_start], 2);
    // fg[0] += CppAD::pow(traj_ref(1, 0) - vars[y_start], 2);
    // fg[0] += 0.5 * CppAD::pow(traj_ref(2, 0) - vars[yaw_start], 2);
    // fg[0] += 0.5 * CppAD::pow(traj_ref(3, 0) - vars[v_start], 2);

    // The rest of the constraints
    for (int i = 0; i < T - 1; i++) {
        // The state at time t+1 .
        AD<double> x1 = vars[x_start + i + 1];
        AD<double> y1 = vars[y_start + i + 1];
        AD<double> yaw1 = vars[yaw_start + i + 1];
        AD<double> v1 = vars[v_start + i + 1];

        // The state at time t.
        AD<double> x0 = vars[x_start + i];
        AD<double> y0 = vars[y_start + i];
        AD<double> yaw0 = vars[yaw_start + i];
        AD<double> v0 = vars[v_start + i];

        // Only consider the actuation at time t.
        AD<double> delta0 = vars[delta_start + i];
        AD<double> a0 = vars[a_start + i];

        // constraint with the dynamic model
        fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * DT);
        fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * DT);
        fg[2 + yaw_start + i] = yaw1 - (yaw0 + v0 * CppAD::tan(delta0) / WB * DT);
        fg[2 + v_start + i] = v1 - (v0 + a0 * DT);
        // cost with the ref traj
        fg[0] += CppAD::pow(traj_ref(0, i+1) - (x0 + v0 * CppAD::cos(yaw0) * DT), 2);
        fg[0] += CppAD::pow(traj_ref(1, i+1) - (y0 + v0 * CppAD::sin(yaw0) * DT), 2);
        fg[0] += 0.5 * CppAD::pow(traj_ref(2, i+1) - (yaw0 + v0 * CppAD::tan(delta0) / WB * DT), 2);
        fg[0] += 0.5 * CppAD::pow(traj_ref(3, i+1) - (v0 + a0 * DT), 2);
    }
}




