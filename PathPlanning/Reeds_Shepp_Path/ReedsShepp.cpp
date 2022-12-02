//
// Created by chh3213 on 2022/11/30.
//

#include "ReedsShepp.h"

//Path::Path(const vector<double> &lengths, const string &modes, double l, const vector<double> &x,
//           const vector<double> &y, const vector<double> &yaw, vector<double> directions) : lengths(lengths), modes(modes), L(l),
//                                                                                 x(x), y(y), yaw(yaw),
//                                                                                 directions(directions) {}

/**
 * theta对2pi取模运算
 * @param theta
 * @return
 */
double ReedsShepp::mod2Pi(double theta) {

    return theta - 2.0*PI* floor(theta/2.0/PI);

}

/**
 * SLS
 * @param x
 * @param y
 * @param phi
 * @return
 */
pair<bool, vector<double>> ReedsShepp::straightLeftStraight(double x, double y, double phi) {
    phi = mod2Pi(phi);
    if(phi>0.0&&phi<PI*0.99){
        double xd = - y / tan(phi) + x;
        double t = xd - tan(phi / 2.0);
        double u = phi;
        double v;
        if(y>0.0){
            v = sqrt(pow(x - xd, 2) + pow(y, 2)) - tan(phi / 2.0);
        }else if(y<0.0){
            v = -sqrt(pow(x - xd, 2) + pow(y, 2)) - tan(phi / 2.0);
        }else {
            return {false, {0.0, 0.0, 0.0}};
        }
        return {true,{t,u,v}};
    }
    return {false, {0.0, 0.0, 0.0}};

}

vector<Path> ReedsShepp::setPath(vector<Path>paths, vector<double> lengths, string modes, double step_size) {
    Path path;
    path.modes=modes;
    path.lengths=lengths;
    double s = 0;
    for(int  i=0;i<lengths.size();i++){
        s+=abs(lengths[i]);
    }
    path.L=s;

    for(Path i_path:paths){
        bool type_is_same = i_path.modes==path.modes;

        double tmp_sum = 0.0;
        for(int i=0;i<i_path.lengths.size();i++){
            tmp_sum+=abs(i_path.lengths[i]);
        }
        tmp_sum = tmp_sum-path.L;
        bool length_is_close = tmp_sum<=step_size;
        if(type_is_same && length_is_close)return paths;
    }
    //如果路径太短，则直接返回原路径
    if(path.L<=step_size)return paths;

    paths.push_back(path);
    return paths;
}

/**
 * SCS
 * @param x
 * @param y
 * @param phi
 * @param paths
 * @param step_size
 * @return
 */
vector<Path> ReedsShepp::straightCurveStraight(double x, double y, double phi, vector<Path> paths, double step_size) {
    pair<bool, vector<double>>res = straightLeftStraight(x,y,phi);
    bool flag = res.first;
    if(flag){
        paths = setPath(paths,res.second,"SLS",step_size);
    }
    res = straightLeftStraight(x,-y,-phi);
    flag = res.first;
    if(flag){
        paths = setPath(paths,res.second,"SRS",step_size);
    }
    return paths;

}

vector<double> ReedsShepp::polar(double x, double y) {
    double r = sqrt(x*x+y*y);
    double theta = atan2(y,x);
    return {r,theta};
}

/**
 * LSL
 * @param x
 * @param y
 * @param phi
 * @return
 */
pair<bool, vector<double>> ReedsShepp::leftStraightLeft(double x, double y, double phi) {
    vector<double>res = polar(x-sin(phi),y-1.0+cos(phi));
    double t = res[1];
    double u = res[0];
    if(t>=0.0){
        double v = mod2Pi(phi-t);
        if(v>=0.0){
            return {true,{t,u,v}};
        }
    }
    return {false,{0.0,0.0,0.0}};
}

/**
 * LRL
 * @param x
 * @param y
 * @param phi
 * @return
 */
pair<bool, vector<double>> ReedsShepp::leftRightLeft(double x, double y, double phi) {
    vector<double>res = polar(x-sin(phi),y-1.0+cos(phi));
    double t1 = res[1];
    double u1 = res[0];
    if(u1<=4.0){
        double u = -2.0* asin(0.25*u1);
        double t = mod2Pi(t1+0.5*u+PI);
        double v= mod2Pi(phi-t+u);
        if(t>=0.0&&u<=0.0){
            return {true,{t,u,v}};
        }
    }
    return {false,{0.0,0.0,0.0}};
}

/**
 * CCC
 * @param x
 * @param y
 * @param phi
 * @param paths
 * @param step_size
 * @return
 */
vector<Path> ReedsShepp::curveCurveCurve(double x, double y, double phi, vector<Path> paths, double step_size) {
    pair<bool, vector<double>>lrl = leftRightLeft(x,y,phi);
    bool flag = lrl.first;
    if(flag){
        paths = setPath(paths,lrl.second,"LRL",step_size);
    }

    lrl = leftRightLeft(-x,y,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[0],-lrl.second[1],-lrl.second[2]},"LRL",step_size);
    }

    lrl = leftRightLeft(x,-y,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{lrl.second[0],lrl.second[1],lrl.second[2]},"RLR",step_size);
    }

    lrl = leftRightLeft(-x,-y,phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[0],-lrl.second[1],-lrl.second[2]},"RLR",step_size);
    }

    // backwards
    double xb = x * cos(phi) + y * sin(phi);
    double yb = x * sin(phi) - y * cos(phi);

    lrl = leftRightLeft(xb,yb,phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{lrl.second[2],lrl.second[1],lrl.second[0]},"LRL",step_size);
    }

    lrl = leftRightLeft(-xb,yb,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[2],-lrl.second[1],-lrl.second[0]},"LRL",step_size);
    }

    lrl = leftRightLeft(xb,-yb,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{lrl.second[2],lrl.second[1],lrl.second[0]},"RLR",step_size);
    }

    lrl = leftRightLeft(-xb,-yb,phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[2],-lrl.second[1],-lrl.second[0]},"RLR",step_size);
    }

    return paths;

}

/**
 * CSC
 * @param x
 * @param y
 * @param phi
 * @param paths
 * @param step_size
 * @return
 */
vector<Path> ReedsShepp::curveStraightCurve(double x, double y, double phi, vector<Path> paths, double step_size) {
    pair<bool, vector<double>>lrl = leftStraightLeft(x,y,phi);
    bool flag = lrl.first;
    if(flag){
        paths = setPath(paths,lrl.second,"LSL",step_size);
    }

    lrl = leftStraightLeft(-x,y,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[0],-lrl.second[1],-lrl.second[2]},"LSL",step_size);
    }

    lrl = leftStraightLeft(x,-y,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{lrl.second[0],lrl.second[1],lrl.second[2]},"RSR",step_size);
    }

    lrl = leftStraightLeft(-x,-y,phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[0],-lrl.second[1],-lrl.second[2]},"RSR",step_size);
    }

    lrl = leftStraightRight(x,y,phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{lrl.second[0],lrl.second[1],lrl.second[2]},"LSR",step_size);
    }

    lrl = leftStraightRight(-x,y,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[0],-lrl.second[1],-lrl.second[2]},"LSR",step_size);
    }

    lrl = leftStraightRight(x,-y,-phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{lrl.second[0],lrl.second[1],lrl.second[2]},"RSL",step_size);
    }

    lrl = leftStraightRight(-x,-y,phi);
    flag = lrl.first;
    if(flag){
        paths = setPath(paths,{-lrl.second[0],-lrl.second[1],-lrl.second[2]},"RSL",step_size);
    }

    return paths;
}

/**
 * LSR
 * @param x
 * @param y
 * @param phi
 * @return
 */
pair<bool, vector<double>> ReedsShepp::leftStraightRight(double x, double y, double phi) {
    vector<double>res = polar(x+sin(phi),y-1.0-cos(phi));
    double t1 = res[1];
    double u1 = res[0];
    u1 *=u1;
    if(u1>=4.0){
        double u = sqrt(u1-4.0);
        double theta = atan2(2.0,u);
        double t = mod2Pi(t1+theta);
        double v= mod2Pi(t-phi);
        if(t>=0.0&&v>=0.0){
            return {true,{t,u,v}};
        }
    }
    return {false,{0.0,0.0,0.0}};
}

vector<Path> ReedsShepp::generatePath(vector<double> q0, vector<double> q1, double max_curvature, double step_size) {
    double dx = q1[0]-q0[0];
    double dy = q1[1]-q0[1];
    double dth = q1[2]-q0[2];
    double c = cos(q0[2]);
    double s = sin(q0[2]);
    double x = (c*dx+s*dy)*max_curvature;
    double y = (-s*dx+c*dy)*max_curvature;

    vector<Path>paths;
    paths= straightCurveStraight(x,y,dth,paths,step_size);
    paths= curveStraightCurve(x,y,dth,paths,step_size);
    paths= curveCurveCurve(x,y,dth,paths,step_size);
    return paths;
}

vector<vector<double>> ReedsShepp::calInterpolateDistsList(vector<double> lengths, double step_size) {
    vector<vector<double>>interpolate_dists_list;
    for(double length:lengths){
        double d_dist;
        if(length>=0){
             d_dist = step_size;
        }else{
            d_dist = -step_size;
        }
        vector<double>interp_dists;
        for(double i=0;i<length;i+=d_dist){
            interp_dists.push_back(i);
        }
        interp_dists.push_back(length);
        interpolate_dists_list.push_back(interp_dists);
    }
    return interpolate_dists_list;

}

vector<vector<double>>
ReedsShepp::generateLocalCourse(vector<double> lengths, string modes, double max_curvature, double step_size) {
    vector<vector<double>>interpolate_dists_list= calInterpolateDistsList(lengths,step_size);
    double origin_x=0,origin_y=0,origin_yaw=0;

    vector<double>xs,ys,yaws,directions;

    for(int i=0;i<interpolate_dists_list.size();i++){
        vector<double>interp_dists=interpolate_dists_list[i];
        char mode = modes[i];
        double length = lengths[i];

        for(double dist:interp_dists){
            vector<double>inter  = interpolate(dist, length, mode,max_curvature, origin_x, origin_y, origin_yaw);
            xs.push_back(inter[0]);
            ys.push_back(inter[1]);
            yaws.push_back(inter[2]);
            directions.push_back(inter[3]);
        }
        origin_x = xs[xs.size()-1];
        origin_y = ys[ys.size()-1];
        origin_yaw = yaws[yaws.size()-1];
    }
    return {xs, ys, yaws, directions};
}

vector<double>
ReedsShepp::interpolate(double dist, double length, char mode, double max_curvature, double origin_x, double origin_y,
                        double origin_yaw) {
    double x,y,yaw;
    if(mode=='S'){
        x = origin_x + dist / max_curvature * cos(origin_yaw);
        y = origin_y + dist / max_curvature * sin(origin_yaw);
        yaw = origin_yaw;
    }else{
        double ldx = sin(dist)/max_curvature;
        double ldy = 0.0;
        yaw = 0.0;
        if(mode=='L'){
            ldy = (1.0-cos(dist))/max_curvature;
            yaw = origin_yaw + dist;
        }else if(mode=='R'){
            ldy = (1.0 - cos(dist)) / -max_curvature;
            yaw = origin_yaw - dist;
        }
        double gdx = cos(-origin_yaw)*ldx+sin(-origin_yaw)*ldy;
        double gdy = -sin(-origin_yaw)*ldx+cos(-origin_yaw)*ldy;
        x = origin_x + gdx;
        y = origin_y + gdy;
    }
    double k;
    if(length>0.0)k = 1;
    else k=-1;
    return {x,y,yaw,k};
}

/**
 * 角度对2pi取模，double类型
 * @param angle
 * @return
 */
double ReedsShepp::PI2PI(double angle) {
    return fmod(angle+PI,2*PI)-PI;
}

vector<Path> ReedsShepp::calPath(vector<double> start, vector<double> goal, double maxc, double step_size) {
    vector<Path>paths = generatePath(start,goal,maxc,step_size);
    vector<Path>ps ;
    for(Path path:paths){
        vector<vector<double>>gLC= generateLocalCourse(path.lengths,path.modes,maxc,step_size*maxc);
        vector<double>xs, ys, yaws, dirs;
        xs=gLC[0],ys=gLC[1],yaws=gLC[2],dirs=gLC[3];
        for(int i=0;i<xs.size();i++){
            double ix=xs[i],iy=ys[i],yaw=yaws[i];
            path.x.push_back(cos(-start[2])*ix+sin(-start[2])*iy+start[0]);
            path.y.push_back(-sin(-start[2])*ix+cos(-start[2])*iy+start[1]);
            path.yaw.push_back(PI2PI(yaw+start[2]));
        }
        path.directions = dirs;
        for(int i=0;i<path.lengths.size();i++){
            double len = path.lengths[i]/maxc;
            path.lengths[i] = len;
        }
        path.L = path.L/maxc;
        ps.push_back(path);
    }
    return ps;
}

Path ReedsShepp::reedsSheppPathPlanning(vector<double> start, vector<double> goal, double maxc, double step_size=0.2) {
    vector<Path>paths = calPath(start,goal,maxc,step_size);
    if(paths.empty()){
        cout<<"could not generate any path"<<endl;
    }
    double best_path_index=-1;
    double min_L=numeric_limits<double>::max();
    for(int i=0;i<paths.size();i++){
        Path p = paths[i];
        if(min_L>abs(p.L)){
            min_L=abs(p.L);
            best_path_index = i;
        }
    }
    Path b_path = paths[best_path_index];
    return b_path;
}

//void ReedsShepp::plotArrow(vector<double> x, vector<double>  y, vector<double>  yaw, double length, double width, string fc, string ec) {
//    for(int i=0;i<x.size();i++){
//        //plt::arrow(vector<double>{x[i]},vector<double>{y[i]},vector<double>{length*cos(yaw[i])},vector<double>{length*sin(yaw[i])},fc,ec,vector<double>{width},vector<double>{width});
//        plt::arrow(vector<double>{x[i]},vector<double>{y[i]},vector<double>{length*cos(yaw[i])},vector<double>{length*sin(yaw[i])},fc,ec,vector<double>{width},vector<double>{width});
//        //plt::plot(vector<double>{x[i]},vector<double>{y[i]});
//    }
//}









