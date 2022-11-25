//
// Created by chh3213 on 2022/11/25.
//

#include "BezierCurve.h"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
    vector<Vector2d>Ps{Vector2d (0,0),Vector2d(1,1),Vector2d(2,1),Vector2d(3,0),Vector2d(4,2)};
    //vector<Vector2d>Ps{Vector2d (0,0),Vector2d(1,1)};

    vector<double>x_ref,y_ref;
    for(int i=0;i<Ps.size();i++){
        x_ref.push_back(Ps[i][0]);
        y_ref.push_back(Ps[i][1]);
    }
    vector<double>x_,y_;
    for(int t=0;t<100;t++){
        plt::clf();
        Vector2d pos = bezierCommon(Ps,(double)t/100);
        //cout<<pos[0]<<","<<pos[1]<<endl;
        x_.push_back(pos[0]);
        y_.push_back(pos[1]);

        //画图
        //plt::xlim(0,1);
        plt::plot(x_, y_,"r");
        plt::plot(x_ref,y_ref);
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./bezier_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}