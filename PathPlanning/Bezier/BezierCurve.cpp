//
// Created by chh3213 on 2022/11/25.
//

#include "BezierCurve.h"


/**
 * 阶乘实现
 * @param n
 * @return
 */
double factorial(int n) {
    if(n<=1)return 1;
    return factorial(n-1)*n;
}

/**
 * 贝塞尔公式
 * @param Ps
 * @param t
 * @return
 */
Vector2d bezierCommon(vector<Vector2d> Ps, double t) {

    if(Ps.size()==1)return Ps[0];

    Vector2d p_t(0.,0.);
    int n = Ps.size()-1;
    for(int i=0;i<Ps.size();i++){
        double C_n_i = factorial(n)/ (factorial(i)* factorial(n-i));
        p_t +=  C_n_i*pow((1-t),(n-i))*pow(t,i)*Ps[i];
        //cout<<t<<","<<1-t<<","<<n-i<<","<<pow((1-t),(n-i))<<endl;
    }
    return p_t;
}



