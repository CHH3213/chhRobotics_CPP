//
// Created by chh3213 on 2022/11/24.
//
#include "RearWheelFeedback.h"
#include "../utils/MyReferencePath.h"
#include "../../matplotlibcpp.h"
#include "../utils/KinematicModel.h"
namespace plt = matplotlibcpp;

#define PI 3.1415926

int main(){

    double Kpsi=3, K2=1.5;//利亚普诺夫系数

    double dt=0.1; // 时间间隔，单位：s
    double L=2; // 车辆轴距，单位：m
    double v = 2; // 初始速度
    double x_0=0; // 初始x
    double y_0=-3; //初始y
    double psi_0=0; // 初始航向角
    //保存机器人（小车）运动过程中的轨迹
    vector<double>x_,y_;
    MyReferencePath referencePath;
    KinematicModel ugv(x_0,y_0,psi_0,v,L,dt);
    RearWheelFeedback rwf(Kpsi,K2,L);
    vector<double>robot_state;



    for(int i=0;i<500;i++){
        plt::clf();
        robot_state = ugv.getState();
        vector<double>one_trial = referencePath.calcTrackError(robot_state);
        double e = one_trial[0],k = one_trial[1],ref_psi = one_trial[2];
        double delta = rwf.rearWheelFeedbackControl(robot_state,e,k,ref_psi);

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
    const char* filename = "./rear_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}
