//
// Created by chh3213 on 2022/12/1.
//

#include "Dubins.h"

/**
 * 角度对2pi取模，double类型
 * @param angle
 * @return
 */
double Dubins::PI2PI(double angle) {
    return fmod(angle+PI,2*PI)-PI;
}

vector<double> Dubins::polar(double x, double y) {
    double r = sqrt(x*x+y*y);
    double theta = atan2(y,x);
    return {r,theta};
}

/**
 * theta对2pi取模运算
 * @param theta
 * @return
 */
double Dubins::mod2Pi(double theta) {

    return theta - 2.0*PI* floor(theta/2.0/PI);

}

Dubins::Path Dubins::left_straight_left(double alpha, double beta, double d) {
    Path path;

    double sa = sin(alpha);
    double sb = sin(beta);
    double ca = cos(alpha);
    double cb = cos(beta);
    double c_ab = cos(alpha-beta);

    double tmp0 = d+sa-sb;

    string mode = "LSL";
    path.mode = mode;
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));

    if(p_squared<0.){
        return path;
    }

    double tmp1 = atan2(cb-ca,tmp0);

    path.t = mod2Pi(-alpha + tmp1);
    path.p = sqrt(p_squared);
    path.q = mod2Pi(beta - tmp1);

    return path;
}

Dubins::Path Dubins::right_straight_right(double alpha, double beta, double d) {
    Path path;

    double sa = sin(alpha);
    double sb = sin(beta);
    double ca = cos(alpha);
    double cb = cos(beta);
    double c_ab = cos(alpha-beta);

    double tmp0 = d-sa+sb;

    string mode = "RSR";
    path.mode = mode;
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));

    if(p_squared<0.){
        return path;
    }

    double tmp1 = atan2(ca-cb,tmp0);

    path.t = mod2Pi(alpha - tmp1);
    path.p = sqrt(p_squared);
    path.q = mod2Pi(-beta + tmp1);

    return path;
}

Dubins::Path Dubins::left_straight_right(double alpha, double beta, double d) {
    Path path;

    double sa = sin(alpha);
    double sb = sin(beta);
    double ca = cos(alpha);
    double cb = cos(beta);
    double c_ab = cos(alpha-beta);


    string mode = "LSR";
    path.mode = mode;
    double p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sb + sa));

    if(p_squared<0.){
        return path;
    }
    double p = sqrt(p_squared);
    double tmp1 = atan2(-ca-cb,d+sa+sb)- atan2(-2.0,p);

    path.t = mod2Pi(-alpha + tmp1);
    path.p = p;
    path.q = mod2Pi(-mod2Pi(beta) + tmp1);
    return path;
}

Dubins::Path Dubins::right_straight_left(double alpha, double beta, double d) {
    Path path;

    double sa = sin(alpha);
    double sb = sin(beta);
    double ca = cos(alpha);
    double cb = cos(beta);
    double c_ab = cos(alpha-beta);


    string mode = "RSL";
    path.mode = mode;
    double p_squared = -2 + (d * d) + (2 * c_ab) - (2 * d * (sb + sa));

    if(p_squared<0.){
        return path;
    }
    double p = sqrt(p_squared);
    double tmp1 = atan2(ca + cb, d - sa - sb) - atan2(2.0, p);

    path.t = mod2Pi(alpha - tmp1);
    path.p = p;
    path.q = mod2Pi(beta - tmp1);

    return path;
}

Dubins::Path Dubins::right_left_right(double alpha, double beta, double d) {
    Path path;

    double sa = sin(alpha);
    double sb = sin(beta);
    double ca = cos(alpha);
    double cb = cos(beta);
    double c_ab = cos(alpha-beta);


    string mode = "RLR";
    path.mode = mode;
    double tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;

    if(abs(tmp_rlr) > 1.0){
        return path;
    }


    path.p = mod2Pi(2 * PI - acos(tmp_rlr));
    path.t = mod2Pi(alpha - atan2(ca - cb, d - sa + sb)+ mod2Pi(path.p/2.0));
    path.q = mod2Pi(alpha - beta - path.t + mod2Pi(path.p));

    return path;
}

Dubins::Path Dubins::left_right_left(double alpha, double beta, double d) {
    Path path;

    double sa = sin(alpha);
    double sb = sin(beta);
    double ca = cos(alpha);
    double cb = cos(beta);
    double c_ab = cos(alpha-beta);


    string mode = "LRL";
    path.mode = mode;
    double tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0;

    if(abs(tmp_rlr) > 1.0){
        return path;
    }


    path.p = mod2Pi(2 * PI - acos(tmp_rlr));
    path.t = mod2Pi(-alpha - atan2(ca - cb, d + sa - sb)+ path.p/2.0);
    path.q = mod2Pi(mod2Pi(beta) - alpha - path.t + mod2Pi(path.p));

    return path;
}

vector<vector<double>>
Dubins::interpolate(double ind, double length, char m, double max_curvature, double origin_x, double origin_y,
                    double origin_yaw, vector<double> path_x, vector<double> path_y, vector<double> path_yaw,
                    vector<double> directions) {
    //for(int i=0;i<path_x.size();i++){
    //    cout<<path_x[i]<<",";
    //}
    if(m == 'S'){
        path_x[ind] = origin_x + length / max_curvature * cos(origin_yaw);
        path_y[ind] = origin_y + length / max_curvature * sin(origin_yaw);
        path_yaw[ind] = origin_yaw;
    }
    else {  // curve
        double ldx = sin(length) / max_curvature;
        double ldy = 0.0;
        if(m == 'L') {// left turn
            ldy = (1.0 - cos(length)) / max_curvature;
        }
        else if( m == 'R') {// right turn
            ldy = (1.0 - cos(length)) / -max_curvature;
        }
        double gdx = cos(-origin_yaw) * ldx + sin(-origin_yaw) * ldy;
        double gdy = -sin(-origin_yaw) * ldx + cos(-origin_yaw) * ldy;
        path_x[ind] = origin_x + gdx;
        path_y[ind] = origin_y + gdy;
    }



    if(m == 'L') {// left turn
        path_yaw[ind] = origin_yaw + length;
    }
    else if( m == 'R') {// right turn
        path_yaw[ind] = origin_yaw - length;
    }

    if(length > 0.0){
        directions[ind] = 1;
    }else {
        directions[ind] = -1;
    }
    //cout<<endl;
    //for(int i=0;i<path_x.size();i++){
    //    cout<<path_x[i]<<",";
    //}
    return {path_x, path_y, path_yaw, directions};
}

vector<vector<double>>
Dubins::generate_local_course(double total_length, vector<double> lengths, string modes, double max_curvature,
                              double step_size) {
    double n_point = trunc(total_length / step_size) + lengths.size() + 4;//trunc返回整数部分，忽略小数部分。
    vector<double>p_x(n_point,0.0),p_y(n_point,0.0),p_yaw(n_point,0.0),directions(n_point,0.0);
    double ind = 1;
    if(lengths[0]>0.0){
        directions[0]=1;
    }else{
        directions[0]=-1;
    }
    double ll=0.;
    double dist;
    double pd;
    for(int i=0;i<modes.size();i++){
        if(lengths[i]==0.)continue;
        else if(lengths[i]>0.0) dist = step_size;
        else dist = -step_size;

        // set origin state
        double origin_x=p_x[ind], origin_y=p_y[ind], origin_yaw = p_yaw[ind];

        ind -= 1;
        if(i >= 1 && (lengths[i - 1] * lengths[i]) > 0) {
            pd = -dist - ll;
        }else {
            pd = dist - ll;
        }
        while(abs(pd) <= abs(lengths[i])){
            ind += 1;
            vector<vector<double>>inter = interpolate(ind, pd, modes[i],
                                                      max_curvature,
                                                      origin_x,
                                                      origin_y,
                                                      origin_yaw,
                                                      p_x, p_y,
                                                      p_yaw,
                                                      directions);
            p_x = inter[0], p_y = inter[1], p_yaw = inter[2], directions = inter[3];
            pd += dist;
            //cout<<ind<<","<<pd<<","<< modes[i]<<","<<max_curvature<<","<<origin_x<<","<<origin_y<<","<<origin_yaw<<endl;
        }
        ll = lengths[i] - pd - dist;//calc remain length
        ind += 1;
        vector<vector<double>>inter = interpolate(ind, lengths[i], modes[i],
                                                  max_curvature,
                                                  origin_x, origin_y,
                                                  origin_yaw,
                                                  p_x, p_y, p_yaw,
                                                  directions);
        p_x = inter[0], p_y = inter[1], p_yaw = inter[2], directions = inter[3];

    }
    if (p_x.size()<=1){
        cout<<"no path"<<endl;
        return {};
    }
    while(p_x.size() >= 1 && p_x[p_x.size()-1] == 0){
        p_x.pop_back();
        p_y.pop_back();
        p_yaw.pop_back();
        directions.pop_back();
    }
    return {p_x, p_y, p_yaw, directions};
}

Dubins::ResultDubins Dubins::dubins_path_planning_from_origin(Vector3d goal, double curvature, double step_size) {
    double dx = goal[0];
    double dy = goal[1];
    double D = hypot(dx, dy);//Return `sqrt(X*X + Y*Y)'
    double d = D * curvature;
    double theta = mod2Pi(atan2(dy, dx));
    double alpha = mod2Pi(- theta);
    double beta = mod2Pi(goal[2] - theta);

    Path best_path;
    Path path1 = left_straight_left(alpha,beta,d);
    Path path2 = right_straight_right(alpha,beta,d);
    Path path3 = left_straight_right(alpha,beta,d);
    Path path4 = right_straight_left(alpha,beta,d);
    Path path5 = right_left_right(alpha,beta,d);
    Path path6 = left_right_left(alpha,beta,d);
    double cost1 = abs(path1.t)+abs(path1.p)+abs(path1.q);
    double cost2 = abs(path2.t)+abs(path2.p)+abs(path2.q);
    double cost3 = abs(path3.t)+abs(path3.p)+abs(path3.q);
    double cost4 = abs(path4.t)+abs(path4.p)+abs(path4.q);
    double cost5 = abs(path5.t)+abs(path5.p)+abs(path5.q);
    double cost6 = abs(path6.t)+abs(path6.p)+abs(path6.q);
    vector<double>costs{cost1,cost2,cost3,cost4,cost5,cost6};
    double best_cost=numeric_limits<double>::max();
    for(int i=0;i<costs.size();i++){
        if(costs[i]<best_cost&&costs[i]>0){//除去〇值后取最小
            best_cost=costs[i];
        }
        //cout<<costs[i]<<endl;
    }
    vector<double>lengths;
    string best_mode;
    if(best_cost==cost1){
        lengths={path1.t,path1.p,path1.q};
        best_mode=path1.mode;
    }else if(best_cost==cost2){
        lengths={path2.t,path2.p,path2.q};
        best_mode=path2.mode;
    }else if(best_cost==cost3){
        lengths={path3.t,path3.p,path3.q};
        best_mode=path3.mode;
    }else if(best_cost==cost4){
        lengths={path4.t,path4.p,path4.q};
        best_mode=path4.mode;
    }else if(best_cost==cost5){
        lengths={path5.t,path5.p,path5.q};
        best_mode=path5.mode;
    }else if(best_cost==cost6){
        lengths={path6.t,path6.p,path6.q};
        best_mode=path6.mode;
    }

    vector<vector<double>>glc = generate_local_course(lengths[0]+lengths[1]+lengths[2],
                                                      lengths,
                                                      best_mode,
                                                      curvature,
                                                      step_size);

    ResultDubins rd;
    rd.p_x=glc[0],rd.p_y=glc[1],rd.p_yaw=glc[2],rd.directions=glc[3];rd.mode=best_mode;
    //for(int i=0;i<rd.p_x.size();i++){
    //    cout<<rd.p_x[i]<<endl;
    //}
    vector<double>correct_len;
    for(double l:lengths){
        correct_len.push_back(l/curvature);
    }
    rd.lengths=correct_len;
    return rd;
}

Dubins::ResultDubins
Dubins::dubins_path_planning(Vector3d start, Vector3d goal, double curvature, double step_size) {
    ResultDubins res;
    goal[0]-=start[0];
    goal[1]-=start[1];
    Matrix3d l_rot;
    l_rot = AngleAxisd(start[2], Eigen::Vector3d::UnitZ());//从欧拉角中获取旋转矩阵
    MatrixXd block_rot = l_rot.block(0,0,2,2);//截取旋转矩阵前面两行两列
    MatrixXd goal_xy(1,2);
    goal_xy<<goal[0],goal[1];
    MatrixXd le_xy = goal_xy*block_rot;
    double le_yaw = goal[2]-start[2];

    ResultDubins rd = dubins_path_planning_from_origin(Vector3d(le_xy(0,0),le_xy(0,1),le_yaw),curvature,step_size);
    Matrix3d rot;
    rot = AngleAxisd(-start[2], Eigen::Vector3d::UnitZ());//从欧拉角中获取旋转矩阵
    assert(rd.p_x.size()==rd.p_y.size()&&rd.p_y.size()==rd.p_yaw.size());//确保维度一致
    int sz =rd.p_x.size();
    MatrixXd pxy(sz,2);

    for(int i=0;i<sz;i++){
        //cout<<rd.p_x[i]<<endl;
        pxy(i,0)=rd.p_x[i];
        pxy(i,1)=rd.p_y[i];
    }


    MatrixXd block_rot2 = rot.block(0,0,2,2);//截取旋转矩阵前面两行两列
    MatrixXd converted_xy = pxy*block_rot2;

    vector<double>x_list(sz),y_list(sz),yaw_list(sz);
    for(int i=0;i<sz;i++){
        x_list[i]=converted_xy(i,0)+start[0];
        y_list[i]=converted_xy(i,1)+start[1];
        yaw_list[i] = PI2PI(rd.p_yaw[i]+start[2]);
    }

    res.p_x=x_list;
    res.p_y=y_list;
    res.p_yaw=yaw_list;
    res.lengths=rd.lengths;
    res.mode = rd.mode;
    res.directions=rd.directions;
    return res;
}
