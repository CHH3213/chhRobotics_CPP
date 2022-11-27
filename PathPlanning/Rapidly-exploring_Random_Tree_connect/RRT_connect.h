//
// Created by chh3213 on 2022/11/27.
//

#ifndef CHHROBOTICS_CPP_RRT_H
#define CHHROBOTICS_CPP_RRT_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>

#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define PI 3.14159265354

class RRTConnect {
public:
    struct Node{
        double x,y;//节点坐标
        vector<double>path_x={},path_y={};//# 路径，作为画图的数据
        Node(double x, double y);
        Node*parent;

        bool operator==(const Node &rhs) const;

        bool operator!=(const Node &rhs) const;
    };
private:


    vector<vector<double>>obstacle_list;//障碍物位置列表 [[x,y,size],...]
    vector<double>rand_area,play_area;//采样区域 x,y ∈ [min,max];约束随机树的范围 [xmin,xmax,ymin,ymax]

    double robot_radius;//机器人半径
    double expand_dis;//扩展的步长
    double goal_sample_rate;//采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点
    vector<Node*>node_list_1,node_list_2;//与RRT不同的地方
    Node* begin;//根节点
    Node* end;//终节点

    int max_iter;



public:

    RRTConnect(const vector<vector<double>> &obstacleList,
               const vector<double> &randArea, const vector<double> &playArea, double robotRadius, double expandDis,
               double goalSampleRate, int maxIter);

    vector<double>calDistanceAngle(Node* from_node, Node*to_node); // 计算两个节点间的距离和方位角

    bool obstacleFree(Node*node);//判断是否有障碍物

    bool isInsidePlayArea(Node*node);//判断是否在可行区域里面

    int getNearestNodeIndex(vector<Node*>node_list,Node*rnd_node);//计算最近的节点

    Node* sampleFree();//采样生成节点

    double calDistToGoal(double x,double y); //计算(x,y)离目标点的距离

    pair<vector<double>,vector<double>>generateFinalCourse();//生成路径，与RRT不同的地方

    Node* steer(Node* from_node, Node* to_node, double extend_length=numeric_limits<double>::max());//连线方向扩展固定步长查找x_new

    pair<vector<double>,vector<double>>planning();//与RRT不同的地方

    void setBegin(Node *begin);

    void setAnEnd(Node *anEnd);

    void plotCircle(double x, double y, double size, string color = "b");//画圆
    void draw(Node*node1= nullptr, Node*node2= nullptr);

};


#endif //CHHROBOTICS_CPP_RRT_H
