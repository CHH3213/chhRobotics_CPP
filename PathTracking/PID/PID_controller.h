//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_PID_CONTROLLER_H
#define CHHROBOTICS_CPP_PID_CONTROLLER_H
#include <iostream>

using namespace std;

/**
 * 位置式PID实现
 */
class PID_controller {
private:
    double kp,  ki, kd, target, upper,  lower;
    double error=0.0,pre_error=0.0,sum_error=0.0;
public:
    PID_controller(double kp, double ki, double kd, double target, double upper, double lower);

    void setTarget(double target);

    void setK(double kp,double ki,double kd );

    void setBound(double upper,double lower);

    double calOutput(double state);

    void reset();

    void setSumError(double sum_error);
};


#endif //CHHROBOTICS_CPP_PID_CONTROLLER_H
