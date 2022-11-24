//
// Created by chh3213 on 2022/11/24.
//

#include "RearWheelFeedback.h"

/**
 * 角度归一化到【-PI,PI】
 * @param angle
 * @return
 */
double RearWheelFeedback::normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}

/**
 * 后轮位置反馈控制
 * @param robot_state 机器人位姿，包括x,y,yaw,v
 * @param e 误差
 * @param k 曲率
 * @param ref_psi 参考轨迹上点的切线方向的角度
 * @return
 */
double RearWheelFeedback::rearWheelFeedbackControl(vector<double> robot_state, double e, double k, double ref_psi) {
    double psi = robot_state[2],v = robot_state[3];
    double psi_e = normalizeAngle(psi-ref_psi);//psi_e=yaw-ref_yaw
    // 公式17
    double psi_dot = v * k * cos(psi_e) / (1.0 - k * e)  - K2 * v * sin(psi_e) * e / psi_e- Kpsi * abs(v) * psi_e;

    if(psi_e == 0.0 || psi_dot == 0.0)return 0.0;

    // 公式21
    double delta = atan2(L * psi_dot, v);

    return delta;

}

/**
 * 构造函数
 * @param Kpsi
 * @param K2
 * @param L
 */
RearWheelFeedback::RearWheelFeedback(double Kpsi, double K2, double L) {
    this->Kpsi=Kpsi;
    this->K2=K2;
    this->L=L;
}


