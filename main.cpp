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
    vector<Vector2d>obstacle;//障碍物位置
    obstacle.push_back(Vector2d(-1, -1));
    obstacle.push_back(Vector2d(0, 2));
    obstacle.push_back(Vector2d(4, 2));
    obstacle.push_back(Vector2d(5, 4));
    obstacle.push_back(Vector2d(5,5));
    obstacle.push_back(Vector2d(5,6));
    obstacle.push_back(Vector2d(5,9));
    obstacle.push_back(Vector2d(8,9));
    obstacle.push_back(Vector2d(7,9));
    obstacle.push_back(Vector2d(8,10));
    obstacle.push_back(Vector2d(9,11));
    obstacle.push_back(Vector2d(12,13));
    obstacle.push_back(Vector2d(12,12));
    obstacle.push_back(Vector2d(15,15));
    obstacle.push_back(Vector2d(13,13));
    for(Vector2d obs:obstacle){//障碍物
        //cout<<vector<double>(obs(0))<<endl;
        plt::plot(vector<double>{obs(0)},vector<double>{obs(1)},"ok");
    }
    plt::show();

}
