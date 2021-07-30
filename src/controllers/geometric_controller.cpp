#include <controllers/geometric_controller.h>

namespace online_planner{

void GeometricController::setState(nav_msgs::Odometry odom){
    curr_odom = navMsgToOdomState(odom, is_body_twist);
}

RpyThCmd GeometricController::computeControl(ControlState target_state){
    RpyThCmd cmd;

    Eigen::Vector3d p_d_ = target_state.pos;
    Eigen::Vector3d v_d_ = target_state.vel;
    Eigen::Vector3d a_d_ = target_state.acc;
    Eigen::Matrix3d R_d_ = target_state.R;

    Eigen::Vector3d p_ = curr_odom.pos;
    Eigen::Vector3d v_ = curr_odom.vel;
    Eigen::Matrix3d R_ = curr_odom.R;
    
    Eigen::Vector3d e_p_ = p_ - p_d_;
    Eigen::Vector3d e_v_ = v_ - v_d_;
    // compute thrust, assuming upward z axis(NWU frame)
    double thrust = (-k_p * e_p_ - k_v*e_v_ + m*g*Eigen::Vector3d(0, 0, 1) + m*a_d_).dot(R_.col(2));
    Eigen::Vector3d euler = transform_utils::utils::R2rpy(R_);

    Eigen::Matrix3d A = R_.transpose()*R_d_;
    Eigen::Vector3d e_R = transform_utils::from_skewsym_to_vec3(0.5*(A.transpose() - A));
    Eigen::Matrix3d D = A.trace() * Eigen::Matrix3d::Identity() - A;
    Eigen::Vector3d w_d_ = (-1.0)*D.inverse()*(k_R.asDiagonal()*e_R); //desired angular rate
    
    double phi = euler(0); double theta = euler(1); double psi = euler(2);
    Eigen::Matrix<double,3,3> T;
	T(0,0) = 1.0; T(0,1) = sin(phi)*tan(theta); T(0,2) = cos(phi)*tan(theta);
	T(1,0) = 0.0; T(1,1) = cos(phi); 			T(1,2) = -sin(phi);
	T(2,0) = 0.0; T(2,1) = sin(phi)/cos(theta); T(2,2) = cos(phi)/cos(theta);   
    Eigen::Vector3d euler_rate_d_ = T*w_d_;

    cmd.is_rp_rate = true;
    cmd.is_yaw_rate = true;
    cmd.rpy = euler_rate_d_;
    cmd.thrust = thrust;
    return cmd;
}

};