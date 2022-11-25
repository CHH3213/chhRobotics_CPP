#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类
#include<vector>
using namespace std;
int main()
{
    Eigen::MatrixXd m(2,2);   // MatrixXd 表示的是动态数组，初始化的时候指定数组的行数和列数
    m(0,0) = 3;               //m(i,j) 表示第i行第j列的值，这里对数组进行初始化
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    Eigen::MatrixXd b(2,2);
    b<<1,1,
    1,1;
    Eigen::MatrixXd a(1,2);
    a<<1,1;
    vector<Eigen::MatrixXd>vec;
    vec.push_back(a);
    vec.push_back(b);
    std::cout << vec[1] << std::endl;     // eigen重载了<<运算符，可以直接输出eigen矩阵的值
}
