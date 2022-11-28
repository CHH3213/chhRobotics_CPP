//
// Created by chh3213 on 2022/11/28.
//

#include "Dijkstra.h"


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
    Dijkstra dijkstra(grid_size, robot_radius);
    dijkstra.setGo(goal);//设置目标点
    dijkstra.setSt(start);//设置起点
    dijkstra.setOx(ox);//设置障碍物
    dijkstra.setOy(oy);

    dijkstra.calObstacleMap(ox,oy);
    dijkstra.getMotionModel();


    pair<vector<double>, vector<double>> xy = dijkstra.planning(start, goal);
    plt::plot(xy.first,xy.second,"-r");

    const char* filename = "./dijkstra_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}
