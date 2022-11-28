//
// Created by chh3213 on 2022/11/27.
//

#include "RRT_Star.h"

RRT_Star::RRT_Star(const vector<vector<double>> &obstacleList, const vector<double> &randArea,
                   const vector<double> &playArea, double robotRadius, double expandDis, double goalSampleRate,
                   int maxIter, double connectCircleDist, bool searchUntilMaxIter) : RRT(obstacleList, randArea,
                                                                                         playArea, robotRadius,
                                                                                         expandDis, goalSampleRate,
                                                                                         maxIter), connect_circle_dist(
        connectCircleDist), search_until_max_iter(searchUntilMaxIter) {}

pair<vector<double>, vector<double>> RRT_Star::planning() {
    node_list.push_back(begin);//将起点作为根节点x_{init}，加入到随机树的节点集合中。
    for(int i=0;i<max_iter;i++){
        //从可行区域内随机选取一个节点x_{rand}
        Node*rnd_node = sampleFree();
        cout<<"随机树节点个数："<<node_list.size()<<endl;
        //已生成的树中利用欧氏距离判断距离x_{rand}最近的点x_{near}。
        int nearest_ind = getNearestNodeIndex(node_list,rnd_node);
        Node* nearest_node = node_list[nearest_ind];
        //从x_{near}与x_{rand}的连线方向上扩展固定步长u，得到新节点 x_{new}
        Node*new_node = steer(nearest_node,rnd_node,expand_dis);
        //计算代价,欧氏距离
        new_node->cost = nearest_node->cost+sqrt(pow(new_node->x-nearest_node->x,2)+pow(new_node->y-nearest_node->y,2));

        //如果在可行区域内，且x_{near}与x_{new}之间无障碍物
        if(isInsidePlayArea(new_node) && obstacleFree(new_node)) {
            vector<int> near_ind = findNearInds(new_node);//找到x_new的邻近节点
            Node *node_with_updated_parent = chooseParent(new_node, near_ind);//重新选择父节点
            //如果父节点更新了(非空）
            if (node_with_updated_parent) {
                //重布线
                rewire(node_with_updated_parent, near_ind);
                node_list.push_back(node_with_updated_parent);
            } else {
                node_list.push_back(new_node);
            }
        }

        draw(rnd_node);

        if((!search_until_max_iter)&&new_node){// reaches goal
            int last_index = findBestGoalInd();
            if(last_index!=-1){
                cout<<"reaches the goal!"<<endl;
                return generateFinalCourse(last_index);
            }
        }
    }
    cout<<"达到最大回合数"<<endl;
    int last_index = findBestGoalInd();
    if(last_index!=-1)return generateFinalCourse(last_index);
    return {};
}

/**
 * 计算周围一定半径内的所有邻近节点
 * @param new_node
 * @return 所有邻近节点索引
 */
vector<int> RRT_Star::findNearInds(RRT::Node *new_node) {
    int nnode = node_list.size()+1;
    vector<int>inds;
    double r = connect_circle_dist*sqrt(log(nnode)/nnode);
    for(int i=0;i<node_list.size();i++){
        Node* n_ = node_list[i];
        if (pow(n_->x-new_node->x,2)+ pow(n_->y-new_node->y,2)<r*r){
            inds.push_back(i);
        }
    }
    return inds;
}



void RRT_Star::propagateCostToLeaves(RRT::Node *parent_node) {
    for(Node*node:node_list){
        if(node->parent==parent_node){
            node->cost = calcNewCost(parent_node,node);
            propagateCostToLeaves(node);
        }
    }
}

/**
 * 计算代价
 * @param from_node
 * @param to_node
 * @return
 */
double RRT_Star::calcNewCost(RRT::Node *from_node, RRT::Node *to_node) {
    vector<double>da = calDistanceAngle(from_node,to_node);
    return from_node->cost+da[0];
}

/**
 * 布线
 * @param new_node
 * @param near_inds
 */
void RRT_Star::rewire(RRT::Node *new_node, vector<int> near_inds) {
    for(int i:near_inds){
        Node* near_node = node_list[i];
        Node* edge_node = steer(new_node,near_node);
        if(!edge_node)continue;
        edge_node->cost = calcNewCost(new_node,near_node);

        if(obstacleFree(edge_node)&&near_node->cost>edge_node->cost){
            near_node->x = edge_node->x;
            near_node->y = edge_node->y;
            near_node->cost = edge_node->cost;
            near_node->path_x = edge_node->path_x;
            near_node->path_y = edge_node->path_y;
            near_node->parent = edge_node->parent;
            propagateCostToLeaves(new_node);
        }

    }

}

/**
 * 计算离目标点的最佳索引
 * @return
 */
int RRT_Star::findBestGoalInd() {
    vector<int>goal_inds,safe_goal_inds;
    for(int i=0;i<node_list.size();i++){
        Node* node = node_list[i];
        double dist = calDistToGoal(node->x,node->y);
        if(dist<=expand_dis){
            goal_inds.push_back(i);
        }
    }

    for(int goal_ind:goal_inds){
        Node*  t_node = steer(node_list[goal_ind],end);
        if(obstacleFree(t_node)){
            safe_goal_inds.push_back(goal_ind);
        }
    }
    if(safe_goal_inds.empty())return -1;
    double min_cost = numeric_limits<double>::max();
    int safe_ind = -1;
    for(int ind:safe_goal_inds){
        if(min_cost>node_list[ind]->cost){
            min_cost = node_list[ind]->cost;
            safe_ind = ind;
        }
    }
    return safe_ind;

}

/**
 * 在新产生的节点 $x_{new}$ 附近以定义的半径范围$r$内寻找所有的近邻节点 $X_{near}$，
        作为替换 $x_{new}$ 原始父节点 $x_{near}$ 的备选
	    我们需要依次计算起点到每个近邻节点 $X_{near}$ 的路径代价 加上近邻节点 $X_{near}$ 到 $x_{new}$ 的路径代价，
        取路径代价最小的近邻节点$x_{min}$作为 $x_{new}$ 新的父节点
 * @param new_node
 * @param near_inds
 * @return
 */
RRT::Node *RRT_Star::chooseParent(RRT::Node *new_node, vector<int> near_inds) {
    if(near_inds.empty())return nullptr;

    //double min_cost = numeric_limits<double>::max();
    //Node* determine_node;
    //for(int i:near_inds){
    //    Node* near_node = node_list[i];
    //    Node* t_node = steer(near_node,new_node);
    //    if(t_node&& obstacleFree(t_node)){
    //        double cost = calcNewCost(near_node,new_node);
    //        if(cost<min_cost){
    //            min_cost = cost;
    //            determine_node = t_node;
    //        }
    //    }else{
    //        min_cost =  numeric_limits<double>::max();
    //    }
    //
    //}
    //if(min_cost==numeric_limits<double>::max())return nullptr;
    //determine_node->cost = min_cost;
    //return determine_node;

    vector<double>costs;
    for(int i:near_inds){
        Node* near_node = node_list[i];
        Node* t_node = steer(near_node,new_node);
        if(t_node&& obstacleFree(t_node)){
            costs.push_back(calcNewCost(near_node,new_node));
        }else{
            costs.push_back(numeric_limits<double>::max());//the cost of collision node
        }
    }
    double min_cost = *min_element(costs.begin(),costs.end());

    if(min_cost==numeric_limits<double>::max()){
        cout<<"There is no good path.(min_cost is inf)"<<endl;
        return nullptr;
    }
    int min_ind = near_inds[min_element(costs.begin(),costs.end())-costs.begin()];

    Node* determine_node = steer(node_list[min_ind],new_node);
    determine_node->cost = min_cost;
    return determine_node;



}
