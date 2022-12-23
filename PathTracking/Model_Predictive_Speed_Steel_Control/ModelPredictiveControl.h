//
// Created by chh3213 on 2022/12/23.
//

#ifndef CHHROBOTICS_CPP_MODELPREDICTIVECONTROL_H
#define CHHROBOTICS_CPP_MODELPREDICTIVECONTROL_H

#include <iostream>
#include<vector>
#include<Eigen/Dense>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>
#include "cubic_spline.hpp"
#include<cmath>

using namespace std;
using namespace Eigen;

using CppAD::AD;

#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

#define NX 4
#define T 6

#define DT 0.2
#define MAX_STEER 45.0/180*M_PI
#define MAX_DSTEER  30.0/180*M_PI

#define MAX_ITER 3
#define DU_TH 0.1

#define N_IND_SEARCH 10
#define MAX_TIME 5000

#define WB 2.5
#define MAX_SPEED   55.0/3.6
#define MIN_SPEED  -20.0/3.6
#define MAX_ACCEL 1.0


#define LENGTH  4.5
#define WIDTH 2.0
#define BACKTOWHEEL 1.0
#define WHEEL_LEN 0.3
#define WHEEL_WIDTH 0.2
#define TREAD 0.7
#define WB 2.5
#define YAW_P2P(angle) fmod(fmod((angle)+M_PI, 2*M_PI)-2*M_PI, 2*M_PI)+M_PI


using M_XREF=Matrix<double, NX, T>;
static int x_start = 0;
static int y_start = x_start + T;
static int yaw_start = y_start + T;
static int v_start = yaw_start + T;

static int delta_start = v_start + T;
static int a_start = delta_start + T-1;

struct State{
    double x;
    double y;
    double yaw;
    double v;
    State(double x_, double y_, double yaw_, double v_){
        x = x_;
        y = y_;
        yaw = yaw_;
        v = v_;
    };
};

void update_state(State& state, double a, double delta);

int calc_nearest_index(State state, vector<double> cx, vector<double> cy, vector<double> cyaw, int pind);

vector<double> calc_speed_profile(vector<double> rx, vector<double> ry, vector<double> ryaw, double target_speed);


void calc_ref_trajectory(State state, vector<double> cx, vector<double> cy, vector<double> cyaw, vector<double> ck, vector<double> sp, double dl, int& target_ind, M_XREF& xref);

void smooth_yaw(vector<double>& cyaw);



vector<double> mpc_solve(State x0, M_XREF traj_ref);

void mpc_simulation(vector<double> cx, vector<double> cy, vector<double> cyaw, vector<double> ck, vector<double> speed_profile, vector<double> goal);



class FG_EVAL{
public:
    // Eigen::VectorXd coeeffs;
    M_XREF traj_ref;

    FG_EVAL(const M_XREF &trajRef);//constructor

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars);
    
};

#endif //CHHROBOTICS_CPP_MODELPREDICTIVECONTROL_H
