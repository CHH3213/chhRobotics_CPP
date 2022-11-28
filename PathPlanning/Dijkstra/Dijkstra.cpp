//
// Created by chh3213 on 2022/11/28.
//

#include "Dijkstra.h"

Dijkstra::Node::Node(double x, double y, float cost, double parentIndex) : x(x), y(y), cost(cost), parent_index(parentIndex) {}

Dijkstra::Dijkstra( double resolution, double robotRadius) :  resolution(resolution), robot_radius(robotRadius) {}

/**
 * 得到障碍物信息图，有障碍物的地方标记为true，没有标记为false
 * @param ox 障碍物x坐标集合
 * @param oy 障碍物y坐标集合
 */
void Dijkstra::calObstacleMap(const vector<double> &ox, const vector<double> &oy) {
    min_x = round(*min_element(ox.begin(),ox.end()));
    min_y = round(*min_element(oy.begin(),oy.end()));
    max_x = round(*max_element(ox.begin(),ox.end()));
    max_y = round(*max_element(oy.begin(),oy.end()));

    cout<<"min_x:"<<min_x<<"   min_y:"<<min_y<<"  max_x:"<<max_x<<"  max_y:"<<max_y<<endl;

    x_width = round((max_x-min_x)/resolution);
    y_width = round((max_y-min_y)/resolution);
    cout<<"x_width:"<<x_width<<"  y_width:"<<y_width<<endl;

    obstacle_map=vector<vector<bool>>(x_width,vector<bool>(y_width,false));

    for(double i=0;i<x_width;i++){
        double x = calPosition(i,min_x);
        for(double j = 0;j<y_width;j++){
            double y = calPosition(j,min_y);
            for(double k=0;k<ox.size();k++){
                double d = sqrt(pow(ox[k]-x,2)+pow(oy[k]-y,2));
                if(d<=robot_radius){
                    obstacle_map[i][j]=true;
                    break;
                }
            }
        }
    }
}

/**
 * 计算栅格在地图中的位置
 * @param index
 * @param minp
 * @return
 */
double Dijkstra::calPosition(double index, double minp) {
    double pos = index*resolution+minp;
    return pos;
}

/**
 * 标记移动代价
 * @return
 */
vector<vector<double>> Dijkstra::getMotionModel() {
    //x,y,cost
    motion = {{1, 0, 1},
              {0, 1, 1},
              {-1, 0, 1},
            {0, -1, 1},
              {-1, -1, sqrt(2)},
              {-1, 1, sqrt(2)},
              {1, -1, sqrt(2)},
              {1, 1, sqrt(2)}};
    return motion;
}

/**
 * 计算起点终点的栅格索引
 * @param position
 * @param minp
 * @return
 */
double Dijkstra::calXyIndex(double position, double minp) {
    return round((position-minp)/resolution);
}

/**
 * 计算栅格索引
 * @param node
 * @return
 */
double Dijkstra::calIndex(Dijkstra::Node *node) {
    //cout<<node->x<<","<<node->y<<endl;
    return (node->y-min_y)*x_width+(node->x-min_x);
}

/**
 * 判断节点是否有效，即是否超出边界和碰到障碍物
 * @param node
 * @return
 */
bool Dijkstra::verifyNode(Dijkstra::Node *node) {
    double px = calPosition(node->x,min_x);
    double py = calPosition(node->y,min_y);
    if(px<min_x)return false;
    if(py<min_y)return false;
    if(px>=max_x)return false;
    if(py>=max_y)return false;
    if(obstacle_map[node->x][node->y])return false;
    return true;
}

/**
 * 计算路径，便于画图
 * @param goal_node
 * @param closed_set
 * @return
 */
pair<vector<double>, vector<double>> Dijkstra::calFinalPath(Dijkstra::Node *goal_node, map<double, Node *> closed_set) {
    vector<double>rx,ry;
    rx.push_back(calPosition(goal_node->x,min_x));
    ry.push_back(calPosition(goal_node->y,min_y));

    double parent_index = goal_node->parent_index;

    while(parent_index!=-1){
        Node* node = closed_set[parent_index];
        rx.push_back(calPosition(node->x,min_x));
        ry.push_back(calPosition(node->y,min_y));

        parent_index = node->parent_index;
    }
    return {rx,ry};

}

/**
 * 规划
 * @param start 起点
 * @param goal 终点
 * @return 规划后的路径
 */
pair<vector<double>, vector<double>> Dijkstra::planning(vector<double> start, vector<double> goal) {
    double sx = start[0],sy=start[1];
    double gx = goal[0],gy=goal[1];
    Node* start_node = new Node(calXyIndex(sx,min_x), calXyIndex(sy,min_y),0.0,-1);
    Node* goal_node = new Node(calXyIndex(gx,min_x), calXyIndex(gy,min_y),0.0,-1);

    map<double,Node*>open_set, closed_set;
    //将起点加入到open set
    open_set[calIndex(start_node)]=start_node;
    //cout<<calIndex(start_node)<<endl;
    Node*current;
    while(true){
        double c_id = numeric_limits<double>::max();
        double cost = numeric_limits<double>::max();
        //计算代价最小的节点
        for(auto it=open_set.begin();it!=open_set.end();it++){
            if(it->second->cost<cost){
                cost = it->second->cost;
                c_id = it->first;
            }
            //cout<<it->first<<","<<it->second->cost<<endl;
        }
        current = open_set[c_id];

        plotGraph(current);//画图



        if(abs(current->x - goal_node->x)<EPS && abs(current->y - goal_node->y)<EPS) {
            cout << "Find goal" << endl;
            goal_node->parent_index = current->parent_index;
            goal_node->cost = current->cost;
            break;
        }

        //从open set中去除
        auto iter = open_set.find(c_id);
        open_set.erase(iter);
        //将其加入到closed set
        closed_set[c_id] = current;

        // expand search grid based on motion model
        for(vector<double>move:motion){
            //cout<<move[0]<<move[1]<<move[2]<<endl;
            Node*node = new Node(current->x+move[0],current->y+move[1],current->cost+move[2],c_id);
            double n_id = calIndex(node);

            if(closed_set.find(n_id)!=closed_set.end())continue;//如果已经在closed_set中了

            if(!verifyNode(node))continue;//如果超出边界或者碰到障碍物了

            if(open_set.find(n_id)==open_set.end()){//如果open set中没有这个节点
                open_set[n_id]=node;
            }else{//如果open set中已经存在这个节点
                if(open_set[n_id]->cost>=node->cost){
                    open_set[n_id]=node;
                }
            }
        }
    }
    return calFinalPath(goal_node,closed_set);
}

/**
 * 画图
 * @param current
 */
void Dijkstra::plotGraph(Dijkstra::Node *current) {
    //plt::clf();
    plt::plot(ox,oy,".k");
    plt::plot(vector<double>{st[0]}, vector<double>{st[1]}, "og");
    plt::plot(vector<double>{go[0]}, vector<double>{go[1]}, "xb");
    plt::grid(true);
    plt::plot(vector<double>{calPosition(current->x,min_x)},vector<double>{calPosition(current->y,min_y)},"xc");
    plt::pause(0.001);

}

void Dijkstra::setSt(const vector<double> &st) {
    Dijkstra::st = st;
}

void Dijkstra::setGo(const vector<double> &go) {
    Dijkstra::go = go;
}

void Dijkstra::setOx(const vector<double> &ox) {
    Dijkstra::ox = ox;
}

void Dijkstra::setOy(const vector<double> &oy) {
    Dijkstra::oy = oy;
}

