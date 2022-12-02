#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类
#include<vector>
#include<cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
#define PI 3.14159265354
#include "PathPlanning/Rapidly-exploring_Random_Tree/RRT.h"
int main()
{
    //Vector3d a(1,2,3);
    //Vector3d b(1,2,3);
    //a[0]-=b[0];
    //cout<<a<<endl;
    vector<double>x{1,2,3};
    cout<<x[x.size()-1]<<endl;
    cout<<x[-1]<<endl;
    return 0;

}
