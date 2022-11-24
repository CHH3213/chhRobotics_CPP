//
// Created by chh3213 on 2022/11/24.
//

#include "KinematicModel.h"
/**
 * 机器人运动学模型构造
 * @param x 位置x
 * @param y 位置y
 * @param psi 偏航角
 * @param v 速度
 * @param l 轴距
 * @param dt 采样时间
 */
KinematicModel::KinematicModel(double x, double y, double psi, double v, double l, double dt) : x(x), y(y), psi(psi),
                                                                                                v(v), L(l), dt(dt) {}
/**
 * 控制量为转向角delta_f和加速度a
 * @param accel 加速度
 * @param delta_f 转向角控制量
 */
void KinematicModel::updateState(double accel, double delta_f) {
    x = x + v* cos(psi)*dt;
    y = y + v*sin(psi)*dt;
    psi = psi + v / L * tan(delta_f)*dt;
    v = v + accel*dt;
}

/**
 * 状态获取
 * @return
 */
vector<double> KinematicModel::getState() {
    return {x,y,psi,v};
}


