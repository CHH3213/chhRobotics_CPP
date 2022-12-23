//
// Created by chh3213 on 2022/12/23.
//

#include "MyReferPath.h"

/**
 * 计算参考轨迹点，统一化变量数组，便于后面MPC优化使用
 * @param robot_state 车辆的状态(x,y,yaw,v)
 * @param param 超參數
 * @param dl Defaults to 1.0.
 * @return {xref, ind, dref}結構體
 */
refTraj MyReferPath::calc_ref_trajectory(vector<double> robot_state,parameters param, double dl) {
    vector<double>track_error = calcTrackError(robot_state);//e,k,ref_yaw,ind
    double e = track_error[0],k=track_error[1],ref_yaw=track_error[2],ind = track_error[3];
    refTraj ref_traj;
    ref_traj.xref=MatrixXd (param.NX,param.T+1);
    ref_traj.dref = MatrixXd (param.NU,param.T);
    int ncourse = refer_path.size();
    ref_traj.xref(0,0)=refer_path[ind][0];
    ref_traj.xref(1,0)=refer_path[ind][1];
    ref_traj.xref(2,0)=refer_path[ind][2];
    //参考控制量[v,delta]
    double ref_delta = atan2(param.L*k,1);
    for(int i=0;i<param.T;i++){
        ref_traj.dref(0,i)=robot_state[3];
        ref_traj.dref(1,i)=ref_delta;
    }

    double travel = 0.0;

    for(int i=0;i<param.T+1;i++){
        travel+=abs(robot_state[3])*param.dt;

        double dind = (int)round(travel/dl);

        if(ind+dind<ncourse){
            ref_traj.xref(0,i)=refer_path[ind+dind][0];
            ref_traj.xref(1,i)=refer_path[ind+dind][1];
            ref_traj.xref(2,i)=refer_path[ind+dind][2];
        }else{
            ref_traj.xref(0,i)=refer_path[ncourse-1][0];
            ref_traj.xref(1,i)=refer_path[ncourse-1][1];
            ref_traj.xref(2,i)=refer_path[ncourse-1][2];
        }
    }
    return ref_traj;






}
