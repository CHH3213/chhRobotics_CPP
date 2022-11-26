//
// Created by chh3213 on 2022/11/26.
//

#ifndef CHHROBOTICS_CPP_APF_H
#define CHHROBOTICS_CPP_APF_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

class APF {
private:
    double Eta_att,Eta_rep_ob,Eta_rep_edge,d_max,n;//引力的增益系数,斥力的增益系数,道路边界斥力的增益系数,障碍影响的最大距离, n系数
    Vector2d target_pos;
    vector<Vector2d>obstacle_pos;
    double d,w; //道路标准宽度，汽车宽度
    double len_step; //步长
public:
    APF(double etaAtt, double etaRepOb, double etaRepEdge, double dmax, double n);

    void setTargetPos(const Vector2d &targetPos);

    void setObstaclePos(const vector<Vector2d> &obstaclePos);

    void setEtaAtt(double etaAtt);

    void setEtaRepOb(double etaRepOb);

    void setEtaRepEdge(double etaRepEdge);

    void setDMax(double dMax);

    void setN(double n);

    void setD(double d);

    void setW(double w);

    void setLenStep(double lenStep);

    Vector2d computeForce(VectorXd robot_state);

    VectorXd runAPF(VectorXd robot_state);

};


#endif //CHHROBOTICS_CPP_APF_H
