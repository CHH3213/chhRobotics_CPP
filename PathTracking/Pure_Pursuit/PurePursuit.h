//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_PUREPURSUIT_H
#define CHHROBOTICS_CPP_PUREPURSUIT_H
#include <iostream>
#include <vector>
#include<cmath>
#include <algorithm>

using namespace std;
class PurePursuit {
public:
    double calTargetIndex(vector<double>robot_state, vector<vector<double>>refer_path, double l_d);

    double purePursuitControl(vector<double>robot_state, vector<double>current_ref_point,double l_d, double psi, double L);
};


#endif //CHHROBOTICS_CPP_PUREPURSUIT_H
