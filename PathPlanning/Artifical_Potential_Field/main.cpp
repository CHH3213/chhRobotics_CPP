//
// Created by chh3213 on 2022/11/26.
//

#include "APF.h"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
   //初始化车的参数
    double d = 3.5;  // 道路标准宽度

    double w = 1.8;  //  汽车宽度

    double L = 4.7;  // 车长

    double Eta_att = 2;  // 引力的增益系数

    double Eta_rep_ob = 1;  // 斥力的增益系数

    double Eta_rep_edge = 10;   // 道路边界斥力的增益系数

    double d_max = 5;  // 障碍影响的最大距离


    double len_step = 0.5; // 步长

    double  n=1;

    int Num_iter = 300;  // 最大循环迭代次数

    Vector2d target(99,0);
    vector<Vector2d>obstacle_pos={Vector2d (15, 7 / 4),Vector2d (30, - 3 / 2),Vector2d (45, 3 / 2),Vector2d (60, - 3 / 4),Vector2d (80, 3/2)};

    APF apf(Eta_att,Eta_rep_ob,Eta_rep_edge,d_max,n);
    apf.setD(d);
    apf.setW(w);
    apf.setLenStep(len_step);
    apf.setTargetPos(target);
    apf.setObstaclePos(obstacle_pos);
    //robot_state: 为x,y,v
    VectorXd robot_state(3),init_pos(2);
    robot_state<<0,0,2;
    init_pos<<0,0;
    vector<double>x_,y_;

    //画图
    double len_line = 100;
    vector<double>greyZone_x{-5,-5,len_line,len_line};
    vector<double>greyZone_y{- d - 0.5,d + 0.5,d + 0.5,- d - 0.5};

    for(int i=0;i<Num_iter;i++){
        plt::clf();
        robot_state = apf.runAPF(robot_state);
        x_.push_back(robot_state(0));
        y_.push_back(robot_state(1));

        //画分界线
        map<string, string> keywords;
        keywords["color"] = "grey";
        plt::fill(greyZone_x,greyZone_y,keywords);
        plt::plot({-5,len_line},{0,0},"w--");
        plt::plot({-5,len_line},{d,d},"w--");
        plt::plot({-5,len_line},{-d,-d},"w--");
        for(Vector2d obs:obstacle_pos){
            plt::plot(vector<double>{obs(0)},vector<double>{obs(1)},"ro");//障碍物位置
        }
        plt::plot(vector<double>{target(0)},vector<double>{target(1)}, "gv");//目标位置
        plt::plot(vector<double>{init_pos(0)},vector<double>{init_pos(1)}, "bs");//起点位置

        //画轨迹
        plt::plot(x_, y_,"r");

        plt::grid(true);
        plt::ylim(-10,10);
        plt::pause(0.01);
        if ((robot_state.head(2) - target).norm() < 1) { break; }

    }
    //// save figure
    const char* filename = "./apf_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}