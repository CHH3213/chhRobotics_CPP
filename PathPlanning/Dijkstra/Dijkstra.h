//
// Created by chh3213 on 2022/11/28.
//

#ifndef CHHROBOTICS_CPP_DIJKSTRA_H
#define CHHROBOTICS_CPP_DIJKSTRA_H


#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define EPS  1e-4
#define PI 3.14159265354

class Dijkstra {
public:
    struct Node{
            double x;
            double y;
            float cost;
            //Node* p_node;
            double parent_index;

        Node(double x, double y, float cost, double parentIndex);
    };


private:
    double resolution;//栅格大小
    double robot_radius;
    double min_x,min_y,max_x,max_y;//地图范围
    double x_width,y_width;//长宽
    vector<vector<bool>>obstacle_map;//障碍物地图
    vector<vector<double>>motion;//障碍物地图
    vector<double>st,go;
    vector<double> ox,oy;


public:

    Dijkstra( double resolution, double robotRadius);

    void calObstacleMap(const vector<double> &ox, const vector<double> &oy);

    double calPosition(double index,double minp);

    vector<vector<double>>getMotionModel();

    double calXyIndex(double position, double minp);

    double calIndex(Node*node);

    bool verifyNode(Node*node);

    pair<vector<double>,vector<double>>calFinalPath(Node*goal_node, map<double,Node*>closed_set);

    pair<vector<double>,vector<double>>planning(vector<double>start,vector<double>goal);

    void plotGraph(Node*current);

    void setSt(const vector<double> &st);

    void setGo(const vector<double> &go);

    void setOx(const vector<double> &ox);

    void setOy(const vector<double> &oy);
};


#endif //CHHROBOTICS_CPP_DIJKSTRA_H
