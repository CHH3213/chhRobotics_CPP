//
// Created by chh3213 on 2022/11/26.
//

#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include "DWA.h"

/**
 * 画箭头
 * @param x
 * @param y
 * @param yaw
 * @param len
 * @param width
 */
void plotArrow(double x,double y,double yaw,double len = 0.5, double width=0.1){
    plt::arrow(x,y,len*cos(yaw),len*sin(yaw));
    plt::plot(vector<double>{x},vector<double>{y});
}


void plotRobot(double x, double y, double yaw, double radius=1.0){
    plt::plot(vector<double>{x},vector<double>{y},"bs");
}

int main(){
    VectorXd state(5);
    state<<0.0, 0.0, PI / 8.0, 0.0, 0.0;//[x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    Vector2d goal(10,10); //目标点
    double dt=0.1; //采样时间
    double v_min=-0.5,v_max=1.0,w_min=-40*PI/180,w_max=40*PI/180; //线速度角速度边界
    double predict_time=3.0;//轨迹推算时间长度
    double a_vmax=0.2,a_wmax=40*PI/180; //线加速度和角加速度最大值
    double v_sample=0.01,w_sample=0.1*PI/180; //采样分辨率
    double alpha=0.15,beta=1.0,gamma=1.0; //轨迹评价函数系数
    double radius=1.0; // 用于判断是否到达目标点
    double judge_distance=10; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    vector<Vector2d>obstacle;//障碍物位置
    obstacle.push_back(Vector2d(-1, -1));
    obstacle.push_back(Vector2d(0, 2));
    obstacle.push_back(Vector2d(4, 2));
    obstacle.push_back(Vector2d(5, 4));
    obstacle.push_back(Vector2d(5,5));
    obstacle.push_back(Vector2d(5,6));
    obstacle.push_back(Vector2d(5,9));
    obstacle.push_back(Vector2d(8,9));
    obstacle.push_back(Vector2d(7,9));
    obstacle.push_back(Vector2d(8,10));
    obstacle.push_back(Vector2d(9,11));
    obstacle.push_back(Vector2d(12,13));
    obstacle.push_back(Vector2d(12,12));
    obstacle.push_back(Vector2d(15,15));
    obstacle.push_back(Vector2d(13,13));


    vector<VectorXd >trajectory;
    trajectory.push_back(state);
    //初始化
    DWA dwa(dt,v_min,v_max,w_min,w_max,predict_time,a_vmax,a_wmax,v_sample,w_sample,alpha,beta,gamma,radius,judge_distance);

    //保存画图数据
    vector<double>x_,y_,predict_x,predict_y;
    while(true){
        pair<vector<double>, vector<VectorXd>> res = dwa.dwaControl(state,goal,obstacle);
        state = dwa.kinematicModel(state,res.first,dt);
        trajectory.push_back(state);

        x_.push_back(state(0));
        y_.push_back(state(1));

        //画图
        plt::clf();
        plt::plot(vector<double>{state(0)},vector<double>{state(1)},"xr");
        plt::plot(vector<double>{goal(0)},vector<double>{goal(1)},"xb");//目标
        for(Vector2d obs:obstacle){//障碍物
            plt::plot(vector<double>{obs(0)},vector<double>{obs(1)},"ok");
        }
        plotArrow(state(0),state(1),state(2));
        plotRobot(state(0), state(1), state(2), radius);

        for(VectorXd s:res.second){//画出推算的轨迹
            predict_x.push_back(s(0));
            predict_y.push_back(s(1));
        }
        plt::plot(predict_x,predict_y,"-g");
        predict_x = {};
        predict_y = {};

        //画机器人轨迹
        plt::plot(x_,y_,"r");
        plt::grid(true);
        plt::pause(0.01);

        double dist_to_goal = (state.head(2)-goal).norm();
        if(dist_to_goal<=radius)break;
    }

    //// save figure
    const char* filename = "./dwa_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;

}