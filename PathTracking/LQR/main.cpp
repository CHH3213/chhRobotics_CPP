//
// Created by chh3213 on 2022/11/25.
//

#include "LQRControl.h"
#include "../utils/MyReferencePath.h"
#include "../utils/KinematicModel.h"
#include "../utils/NormalizeAngle.hpp"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
    double dt=0.1; // 时间间隔，单位：s
    double L=2; // 车辆轴距，单位：m
    double v = 2; // 初始速度
    double x_0=0; // 初始x
    double y_0=-3; //初始y
    double psi_0=0; // 初始航向角
    int N = 100;//迭代范围

    MatrixXd Q(3,3);
    Q<<3,0,0,
            0,3,0,
            0,0,3;
    MatrixXd R(2,2);
    R<<2.0,0.0,
            0.0,2;

    //保存机器人（小车）运动过程中的轨迹
    vector<double>x_,y_;
    MyReferencePath referencePath;
    KinematicModel ugv(x_0,y_0,psi_0,v,L,dt);
    LQRControl lqr(N);
    vector<double>robot_state;



    for(int i=0;i<500;i++){
        plt::clf();
        robot_state = ugv.getState();
        vector<double>one_trial = referencePath.calcTrackError(robot_state);
        double k = one_trial[1],ref_yaw = one_trial[2], s0=one_trial[3];

        double ref_delta = atan2(L*k,1);
        vector<MatrixXd>state_space = ugv.stateSpace(ref_delta,ref_yaw);

        double delta = lqr.lqrControl(robot_state, referencePath.refer_path, s0, state_space[0], state_space[1], Q, R);
        delta = delta+ref_delta;

        ugv.updateState(0,delta);//加速度设为0，恒速

        x_.push_back(ugv.x);
        y_.push_back(ugv.y);
        //画参考轨迹
        plt::plot(referencePath.refer_x,referencePath.refer_y,"b--");
        plt::grid(true);
        plt::ylim(-5,5);
        //画图
        plt::plot(x_, y_,"r");
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./lqr_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}