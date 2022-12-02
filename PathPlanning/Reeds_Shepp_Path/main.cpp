#include "ReedsShepp.h"


int main(){
    vector<double>start{1.0,-4.0,(double)20/180*PI};
    vector<double>goal{5.0,5.0,(double)100/180*PI};
    double curvature = 0.1;
    double step_size = 0.05;
    ReedsShepp reedsShepp;

    Path path = reedsShepp.reedsSheppPathPlanning(start,goal,curvature,step_size);
    plt::plot(path.x,path.y,"r");
    plt::plot(vector<double>{start[0]},vector<double>{start[1]},"og");
    plt::plot(vector<double>{goal[0]},vector<double>{goal[1]},"xb");
    plt::title("mode: "+path.modes);
    //for(int i=0;i<path.x.size();i++){
    //    plt::clf();
    //    cout<<path.x[i]<<","<<path.y[i]<<endl;
    //    cout<<path.lengths[i]<<endl;
    //    plt::plot(vector<double>{path.x[i]},vector<double>{path.x[i]},"-r");
    //    //reedsShepp.plotArrow(vector<double>{start[0]},vector<double>{start[1]},vector<double>{start[2]});
    //    plt::pause(0.001);
    //}
    const char* filename = "./reedsShepp_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    cout<<"mode: "<<path.modes<<endl;
    return 0;
}