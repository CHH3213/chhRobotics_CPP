//
// Created by chh3213 on 2022/11/24.
//

#include "MyReferencePath.h"

/**
 * 构造函数，求解出参考轨迹点上的曲率等信息
 */
MyReferencePath::MyReferencePath() {
    refer_path = vector<vector<double>>(1000, vector<double>(4));
    // 生成参考轨迹
    for (int i = 0; i < 1000; i++) {
        refer_path[i][0] = 0.1 * i;
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0) + 2.5 * cos(refer_path[i][0] / 2.0);

        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
    }
    double dx,dy,ddx,ddy;
    for(int i=0;i<refer_path.size();i++){
        if (i==0){
             dx = refer_path[i+1][0] - refer_path[i][0];
             dy = refer_path[i+1][1] - refer_path[i][1];
             ddx = refer_path[2][0] + refer_path[0][0] - 2*refer_path[1][0];
             ddy = refer_path[2][1] + refer_path[0][1] - 2*refer_path[1][1];
        }else if(i==refer_path.size()-1){
             dx = refer_path[i][0] - refer_path[i-1][0];
             dy = refer_path[i][1] - refer_path[i-1][1];
             ddx = refer_path[i][0] + refer_path[i-2][0] - 2*refer_path[i-1][0];
             ddy = refer_path[i][1] + refer_path[i-2][1] - 2*refer_path[i-1][1];
        }else{
             dx = refer_path[i+1][0] - refer_path[i][0];
             dy = refer_path[i+1][1] - refer_path[i][1];
             ddx = refer_path[i+1][0] + refer_path[i-1][0] - 2*refer_path[i][0];
             ddy = refer_path[i+1][1] + refer_path[i-1][1] - 2*refer_path[i][1];
        }
        refer_path[i][2] = atan2(dy,dx);//yaw
        //计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        //参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
        refer_path[i][3]= (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3 / 2) ;// 曲率k计算
    }
}

/**
 * 计算跟踪误差
 * @param robot_state  机器人状态
 * @return
 */
vector<double> MyReferencePath::calcTrackError(vector<double> robot_state) {
    double x = robot_state[0],y=robot_state[1];
    vector<double>d_x(refer_path.size()),d_y(refer_path.size()),d(refer_path.size());
    for(int i=0;i<refer_path.size();i++) {
        d_x[i]=refer_path[i][0]-x;
        d_y[i]=refer_path[i][1]-y;
        d[i] = sqrt(d_x[i]*d_x[i]+d_y[i]*d_y[i]);

    }
    double min_index = min_element(d.begin(),d.end())-d.begin();
    //cout<<min_index<<endl;
    double yaw = refer_path[min_index][2];
    double k = refer_path[min_index][3];
    double angle = normalizeAngle(yaw- atan2(d_y[min_index],d_x[min_index]));
    double error = d[min_index];//误差
    if(angle<0)error*=-1;
    return {error,k,yaw,min_index};
}

/**
 * 角度归一化
 * @param angle
 * @return
 */
double MyReferencePath::normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}


