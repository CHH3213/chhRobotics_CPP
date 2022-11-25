//
// Created by chh3213 on 2022/11/24.
//

#include "Stanley.h"
#include "../../matplotlibcpp.h"
#include "../utils/KinematicModel.h"
namespace plt = matplotlibcpp;

#define PI 3.1415926





int main(){

    double x0 = 0.0, y0=-1.0,psi = 0.5,v=2,L=2,dt=0.1; //初始参数
    double k = 3.; //增益系数
    vector<vector<double>>refer_path(1000,vector<double>(3));
    vector<double>refer_x,refer_y; //保存参考数据用于画图
    // 生成参考轨迹
    for(int i=0;i<1000;i++){
        refer_path[i][0]=0.1*i;
        refer_path[i][1]=2*sin(refer_path[i][0]/3.0)+2.5*cos(refer_path[i][0]/2.0);
        for(int i=0;i<999;i++){
            refer_path[i][2]= atan2((refer_path[i+1][1]-refer_path[i][1]),(refer_path[i+1][0]-refer_path[i][0]));
        }
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
//        cout<<refer_path[i][0]<<" ,"<<refer_path[i][1]<<endl;
    }

    // 运动学模型
    KinematicModel ugv(x0,y0,psi,v,L,dt);

    //保存机器人（小车）运动过程中的轨迹
    vector<double>x_,y_;
    //机器人状态
    vector<double>robot_state(4);
    Stanley stanley(k);

    for(int i=0;i<600;i++){
        plt::clf();
        robot_state = ugv.getState();
        //参考博客中的公式
        vector<double>delta_index = stanley.stanleyControl(robot_state,refer_path);

        ugv.updateState(0,delta_index[0]); //加速度设为0，恒速

        x_.push_back(ugv.x);
        y_.push_back(ugv.y);
        //画图
        plt::plot(refer_x,refer_y,"b--");
        plt::plot(x_, y_,"r");
        plt::grid(true);
        plt::ylim(-5,5);
        plt::pause(0.01);
        if(delta_index[1]>=refer_path.size()-1)break;
    }
    // save figure
    const char* filename = "./stanley_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}