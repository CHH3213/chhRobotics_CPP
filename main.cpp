#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类
#include<vector>
#include<cmath>
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
    VectorXd v(6);
    v<<1,2,3,4,5,6;
    MatrixXd m(2,6);
    m<<1,2,3,4,5,6,7,8,9,10,11,12;
    cout<<m(1)<<endl;
    MatrixXd xc(6,1);
    xc<<1,2,3,4,5,6;
    xc<<1,5,4653,4,5,6;
    MatrixXd res = m*xc;
    cout<<res(0,0)<<endl;

    //cout<<v<<endl;
}
