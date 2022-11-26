//
// Created by chh3213 on 2022/11/26.
//

#ifndef CHHROBOTICS_CPP_DWA_H
#define CHHROBOTICS_CPP_DWA_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

#define PI 3.1415926

class DWA {
private:
    double dt; //采样时间
    double v_min,v_max,w_min,w_max; //线速度角速度边界
    double predict_time;//轨迹推算时间长度
    double a_vmax,a_wmax; //线加速度和角加速度最大值
    double v_sample,w_sample; //采样分辨率
    double alpha,beta,gamma; //轨迹评价函数系数
    double radius; // 用于判断是否到达目标点
    double judge_distance; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
private:
    vector<double> calVelLimit();
    vector<double> calAccelLimit(double v, double w);
    vector<double> calObstacleLimit(VectorXd state, vector<Vector2d> obstacle);
    vector<double> calDynamicWindowVel(double v, double w,VectorXd state, vector<Vector2d> obstacle);
    double _dist(VectorXd state, vector<Vector2d>obstacle);
    vector<VectorXd> trajectoryPredict(VectorXd state, double v, double w);
    pair<vector<double>,vector<VectorXd>> trajectoryEvaluation(VectorXd state, Vector2d goal, vector<Vector2d>obstacle);

    double _heading(vector<VectorXd> trajectory,Vector2d goal);
    double _velocity(vector<VectorXd> trajectory);
    double _distance(vector<VectorXd> trajectory,vector<Vector2d> obstacle);


public:
    DWA(double dt, double vMin, double vMax, double wMin, double wMax, double predictTime, double aVmax, double aWmax,
        double vSample, double wSample, double alpha, double beta, double gamma, double radius, double judgeDistance);

    VectorXd kinematicModel(VectorXd state, vector<double>control,double dt);

    pair<vector<double>,vector<VectorXd>> dwaControl(VectorXd state, Vector2d goal, vector<Vector2d>obstacle);
};


#endif //CHHROBOTICS_CPP_DWA_H
