//
// Created by chh3213 on 2022/11/26.
//

#include "APF.h"

APF::APF(double etaAtt, double etaRepOb, double etaRepEdge, double dmax, double n) : Eta_att(etaAtt),
                                                                                   Eta_rep_ob(etaRepOb),
                                                                                   Eta_rep_edge(etaRepEdge), d_max(dmax),
                                                                                   n(n) {}

void APF::setTargetPos(const Vector2d &targetPos) {
    target_pos = targetPos;
}

void APF::setObstaclePos(const vector<Vector2d> &obstaclePos) {
    obstacle_pos = obstaclePos;
}

void APF::setEtaAtt(double etaAtt) {
    Eta_att = etaAtt;
}

void APF::setEtaRepOb(double etaRepOb) {
    Eta_rep_ob = etaRepOb;
}

void APF::setEtaRepEdge(double etaRepEdge) {
    Eta_rep_edge = etaRepEdge;
}



void APF::setN(double n) {
    APF::n = n;
}

void APF::setD(double d) {
    APF::d = d;
}

void APF::setW(double w) {
    APF::w = w;
}

void APF::setDMax(double dMax) {
    d_max = dMax;
}

void APF::setLenStep(double lenStep) {
    len_step = lenStep;
}

/**
 * 计算引力斥力
 * @param robot_state 车辆状态：x,y,v
 * @return 单位合力方向
 */
Vector2d APF::computeForce(VectorXd robot_state) {
    // 引力势场计算
    Vector2d delta_att = target_pos-robot_state.head(2);
    double dist_att = delta_att.norm();
    Vector2d unite_att_vec = delta_att/dist_att;
    Vector2d F_att = Eta_att*delta_att;

    //合力
    Vector2d F = F_att;

    //障碍物斥力势场
    //在斥力势场函数增加目标调节因子（即车辆至目标距离），以使车辆到达目标点后斥力也为0
    for(Vector2d obs:obstacle_pos){
        Vector2d delta = robot_state.head(2)-obs;
        double dist = delta.norm();
        Vector2d  unite_rep_vec = delta/dist;
        Vector2d F_rep_ob;
        if(dist>=d_max){
            F_rep_ob = Vector2d (0,0);
        }else{
            //障碍物的斥力1，方向由障碍物指向车辆
            //斥力1
            double  F_rep1_norm = Eta_rep_ob*(1/dist-1/d_max)*pow(dist_att,n)/pow(dist,2);
            Vector2d F_rep_ob1 = F_rep1_norm*unite_rep_vec;
            //斥力2
            double F_rep2_norm = n/2*Eta_rep_ob*pow(1/dist-1/d_max,2)*pow(dist_att,n-1);
            Vector2d F_rep_ob2 = F_rep2_norm*unite_att_vec;
            F_rep_ob = F_rep_ob1+F_rep_ob2;
        }
        F += F_rep_ob;

    }

    //道路边界斥力势场
    Vector2d F_rep_edge;
    double v = robot_state(2);//车辆速度
    if(robot_state(1)>-d+w/2&&robot_state(1)<=-d/2){
        F_rep_edge = Vector2d (0,Eta_rep_edge * v * exp(-d / 2 - robot_state(1)));
    }else if(robot_state(1)>-d/2&&robot_state(1)<=-w/2){
        F_rep_edge = Vector2d (0,1/3*Eta_rep_edge*pow(robot_state(1),2));
    }else if(robot_state(1)>w/2&&robot_state(1)<=d/2){
        F_rep_edge = Vector2d (0,-1/3*Eta_rep_edge*pow(robot_state(1),2));
    }else if(robot_state(1)>d/2&&robot_state(1)<=d-w/2){
        F_rep_edge = Vector2d (0,Eta_rep_edge * v * exp( robot_state(1)-d / 2 ));
    }
    F+=F_rep_edge;

    Vector2d unit_F = F/F.norm();
    return unit_F;
}

/**
 * 人工势场法控制器
 * @param robot_state
 * @return 车辆下一步位置
 */
VectorXd APF::runAPF(VectorXd robot_state) {
    Vector2d unit_F = computeForce(robot_state);
    Vector2d next_pos = robot_state.head(2)+len_step * unit_F;

    robot_state <<next_pos(0),next_pos(1),robot_state(2);
    return robot_state;
}



