#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类
#include<vector>
#include<cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
#define PI 3.14159265354

int main()
{
    //Eigen::MatrixXd m(2,2);   // MatrixXd 表示的是动态数组，初始化的时候指定数组的行数和列数
    //m(0,0) = 3;               //m(i,j) 表示第i行第j列的值，这里对数组进行初始化
    //m(1,0) = 2.5;
    //m(0,1) = -1;
    //m(1,1) = m(1,0) + m(0,1);
    //Eigen::MatrixXd b(2,2);
    //b<<1,1,
    //1,1;
    //Eigen::MatrixXd a(1,2);
    vector<double>x_t,y_t;
    double size = 0.5;
    for(int i=0;i<=360;i+=5){
        x_t.push_back(0+size*cos((double)i/180*PI));
        y_t.push_back(0+size*sin((double)i/180*PI));

    }
    plt::plot(x_t,y_t,"r");
    plt::show();

}
