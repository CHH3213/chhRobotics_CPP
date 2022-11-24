//
// Created by chh3213 on 2022/11/24.
//

#include "PID_controller.h"




PID_controller::PID_controller(double kp, double ki, double kd, double target, double upper, double lower) : kp(kp),
                                                                                                             ki(ki),
                                                                                                             kd(kd),
                                                                                                             target(target),
                                                                                                             upper(upper),
                                                                                                             lower(lower) {}

void PID_controller::setTarget(double target) {
    PID_controller::target = target;
}

void PID_controller::setK(double kp, double ki, double kd) {
    this->kp=kp;
    this->ki=ki;
    this->kd=kd;
}

void PID_controller::setBound(double upper, double lower) {
    this->upper=upper;
    this->lower=lower;
}

/**
 * 计算控制输出
 * @param state 当前状态量
 * @return
 */
double PID_controller::calOutput(double state) {
    this->error = this->target-state;
    double u = this->error*this->kp+this->sum_error*this->ki+(this->error-this->pre_error)*this->kd;
    if(u<this->lower)u=this->lower;
    else if(u>this->upper)u=this->upper;
    this->pre_error=this->error;
    this->sum_error =this->sum_error+this->error;
    return u;
}

void PID_controller::reset() {
    error=0.0,pre_error=0.0,sum_error=0.0;
}

void PID_controller::setSumError(double sum_error) {
    this->sum_error = sum_error;
}
