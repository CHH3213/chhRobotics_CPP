//
// Created by chh3213 on 2022/12/23.
//

#include "ModelPredictiveControl.h"



int main(){
    //生成参考路线
    vector<double> wx({0.0, 60.0, 125.0,  50.0,   75.0,  35.0,  -10.0});
     //vector<double> wy({0.0,  4.0,  -4.0,  4.0,  -4.0,   4.0,  0.0});
    vector<double> wy({0.0,  0.0,  50.0,  65.0,   30.0,  50.0,  -20.0});

    Spline2D csp_obj(wx, wy);
    vector<double> r_x;
    vector<double> r_y;
    vector<double> ryaw;
    vector<double> rcurvature;
    vector<double> rs;
    for(double i=0; i<csp_obj.s.back(); i+=1.0){
        vector<double> point_= csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }

    double target_speed = 10.0 / 3.6;
    vector<double> speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);

    mpc_simulation(r_x, r_y, ryaw, rcurvature, speed_profile, {{wx.back(), wy.back()}});


    return 0;
}