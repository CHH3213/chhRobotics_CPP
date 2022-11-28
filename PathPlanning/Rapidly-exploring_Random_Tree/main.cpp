//
// Created by chh3213 on 2022/11/27.
//




#include "RRT.h"

int main(){
    vector<vector<double>>obstacle_list{
            {5, 5, 1},
            {3, 6, 2},
            {3, 8, 2},
            {3, 10, 2},
            {7, 5, 2},
            {9, 5, 2},
            {8,10,1},
            {6,12,1}
    };
    RRT::Node*begin = new RRT::Node(0.0,0.0);
    RRT::Node*end = new RRT::Node(6.0,10.0);
    vector<double>rnd_area{-2,15};
    vector<double>play_area{-2,12,0,14};
    double radius = 0.8;
    double expand_dis=3;//扩展的步长
    double goal_sample_rate=5;//采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点
    int max_iter=500;
    RRT rrt(obstacle_list,rnd_area,play_area,radius,expand_dis,goal_sample_rate, max_iter);
    rrt.setBegin(begin);
    rrt.setAnEnd(end);

    pair<vector<double>, vector<double>>traj = rrt.planning();


    plt::plot(traj.first,traj.second,"r");
    const char* filename = "./rrt_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}