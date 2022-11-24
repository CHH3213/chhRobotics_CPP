//
// Created by chh3213 on 2022/11/24.
//

#include "PurePursuit.h"

/**
 * 计算邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @param l_d 前向距离
 * @return
 */
double PurePursuit::calTargetIndex(vector<double> robot_state, vector<vector<double>> refer_path, double l_d) {
    vector<double>dists;
    for (vector<double>xy:refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    double min_ind = min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标

    double delta_l = sqrt(pow(refer_path[min_ind][0]-robot_state[0],2)+pow(refer_path[min_ind][1]-robot_state[1],2));

    while (l_d>delta_l && min_ind<refer_path.size()-1){
        delta_l = sqrt(pow(refer_path[min_ind+1][0]-robot_state[0],2)+pow(refer_path[min_ind+1][1]-robot_state[1],2));
        min_ind+=1;
    }
    return min_ind;
}

/**
 * purePursuitControl
 * @param robot_state 当前机器人位置
 * @param current_ref_point 参考轨迹点
 * @param l_d 前向距离
 * @param psi 机器人航向角
 * @param L 轴距
 * @return 转角控制量
 */
double PurePursuit::purePursuitControl(vector<double> robot_state, vector<double> current_ref_point, double l_d, double psi, double L) {
    double alpha = atan2(current_ref_point[1]-robot_state[1],current_ref_point[0]-robot_state[0])-psi;

    double delta = atan2(2*L*sin(alpha),l_d);

    return delta;

}


