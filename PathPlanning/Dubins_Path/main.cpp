//
// Created by chh3213 on 2022/12/1.
//

#include "Dubins.h"

int main(){
    Vector3d start(1.0,1.0,(double)45/180*PI);
    Vector3d goal(-3.0,-3.0,(double)-45/180*PI);
    double curvature = 1;
    double step_size = 0.1;
    Dubins dubins;
    Dubins::ResultDubins rd = dubins.dubins_path_planning(start,goal,curvature,step_size);
    plt::plot(rd.p_x,rd.p_y);
    plt::plot(vector<double>{start[0]},vector<double>{start[1]},"og");
    plt::plot(vector<double>{goal[0]},vector<double>{goal[1]},"xb");
    plt::title("mode: "+rd.mode);

    const char* filename = "./dubins_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    cout<<"mode: "<<rd.mode<<endl;
    return 0;
}