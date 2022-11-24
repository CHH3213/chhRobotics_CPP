//
// Created by chh3213 on 2022/11/24.
//
#include "Stanley.h"

Stanley::Stanley(double k) {
    this->k = k;
}

/**
 * 搜索目标邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @return
 */
double Stanley::calTargetIndex(vector<double> robot_state, vector<vector<double>> refer_path) {
    vector<double>dists;
    for (vector<double>xy:refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
}

/**
 * 角度归一化到【-PI,PI】
 * @param angle
 * @return
 */
double Stanley::normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}


/**
 * stanley控制
 * @param robot_state 机器人位姿，包括x,y,yaw,v
 * @param refer_path 参考轨迹的位置和参考轨迹上点的切线方向的角度 x,y,theta
 * @return 控制量+目标点索引
 */
vector<double> Stanley::stanleyControl(vector<double> robot_state, vector<vector<double>> refer_path) {
    double current_target_index = calTargetIndex(robot_state,refer_path);
    //cout<<current_target_index<<endl;
    vector<double>  current_ref_point;

    // 当计算出来的目标临近点索引大于等于参考轨迹上的最后一个点索引时
    if(current_target_index>=refer_path.size()){
        current_target_index = refer_path.size()-1;
        current_ref_point = refer_path[refer_path.size()-1];
    }else{
        current_ref_point = refer_path[current_target_index];
    }
    double e_y;
    double psi_t = current_ref_point[2];

    // 计算横向误差e_y
    // 参考自https://blog.csdn.net/renyushuai900/article/details/98460758
    if((robot_state[0]-current_ref_point[0])*psi_t-(robot_state[1]-current_ref_point[1])>0){
    //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
        e_y = sqrt(pow(current_ref_point[0]-robot_state[0],2)+pow(current_ref_point[1]-robot_state[1],2));
    }else{
        e_y = -sqrt(pow(current_ref_point[0]-robot_state[0],2)+pow(current_ref_point[1]-robot_state[1],2));
    }
    //# 通过公式(5)计算转角,符号保持一致
    double psi = robot_state[2];
    double v = robot_state[3];
    double theta_e = psi_t - psi;
    double delta_e = atan2(k*e_y, v);
    double delta = normalizeAngle(delta_e+theta_e);

    return {delta,current_target_index};
}
