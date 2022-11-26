#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类
#include<vector>
#include<cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
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
    VectorXd v(3);
    v<<1,2,3;
    double y = 1;
    double x = v(1)-y;
    cout<<x<<endl;
    plt::plot(v(0),v(1),"blue");
    plt::show();
    //cout<<v<<endl;
}
