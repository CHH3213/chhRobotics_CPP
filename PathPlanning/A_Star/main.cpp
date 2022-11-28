//
// Created by chh3213 on 2022/11/28.
//

#include "Astar.h"


int main(){
    vector<double>start{-5,-5},goal{50,50};
    double grid_size = 2.0;
    double robot_radius = 1.0;

    vector<double> ox;
    vector<double> oy;

    // add 障碍物边缘
    for(double i=-10; i<60; i++){
        ox.push_back(i);
        oy.push_back(-10.0);
    }
    for(double i=-10; i<60; i++){
        ox.push_back(60.0);
        oy.push_back(i);
    }
    for(double i=-10; i<61; i++){
        ox.push_back(i);
        oy.push_back(60.0);
    }
    for(double i=-10; i<61; i++){
        ox.push_back(-10.0);
        oy.push_back(i);
    }
    for(double i=-10; i<40; i++){
        ox.push_back(20.0);
        oy.push_back(i);
    }
    for(double i=0; i<40; i++){
        ox.push_back(40.0);
        oy.push_back(60.0 - i);
    }
    Astar astar(grid_size, robot_radius);
    astar.setGo(goal);//设置目标点
    astar.setSt(start);//设置起点
    astar.setOx(ox);//设置障碍物
    astar.setOy(oy);

    astar.calObstacleMap(ox, oy);
    astar.getMotionModel();


    pair<vector<double>, vector<double>> xy = astar.planning(start, goal);
    plt::plot(xy.first,xy.second,"-r");

    const char* filename = "./astar_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}
