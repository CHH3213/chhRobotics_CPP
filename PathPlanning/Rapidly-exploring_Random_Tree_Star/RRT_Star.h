//
// Created by chh3213 on 2022/11/27.
//

#ifndef CHHROBOTICS_CPP_RRT_STAR_H
#define CHHROBOTICS_CPP_RRT_STAR_H

#include "../Rapidly-exploring_Random_Tree/RRT.h"
class RRT_Star: public RRT{
public:

    double connect_circle_dist;
    bool search_until_max_iter;


    RRT_Star(const vector<vector<double>> &obstacleList, const vector<double> &randArea, const vector<double> &playArea,
             double robotRadius, double expandDis, double goalSampleRate, int maxIter, double connectCircleDist,
             bool searchUntilMaxIter);

    pair<vector<double>, vector<double>> planning();

    vector<int>findNearInds(Node* new_node);//找出邻近节点集合

    void propagateCostToLeaves(Node* parent_node );

    double calcNewCost(Node*from_node,Node*to_node);

    void rewire(Node* new_node, vector<int>near_inds);

    int findBestGoalInd();

    Node* chooseParent(Node* new_node, vector<int>near_inds);
};


#endif //CHHROBOTICS_CPP_RRT_STAR_H
