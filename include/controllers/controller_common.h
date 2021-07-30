#ifndef CONTROLLER_COMMON_H_
#define CONTROLLER_COMMON_H_

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <transform_utils/transform_utils.h>

namespace online_planner{
//setpoint & odom state
struct OdomState{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Matrix3d R;
};

struct ControlState{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Matrix3d R;
};

struct RpyThCmd{
    bool is_rp_rate; 
    bool is_yaw_rate;
    Eigen::Vector3d rpy;
    double thrust;
};

struct VelCmd{
    bool is_yaw_rate;
    Eigen::Vector3d vel;
    double yaw_val; 
};

struct PhysicsParams{
    double m_;
    double g_;
};

inline OdomState navMsgToOdomState(nav_msgs::Odometry nav, bool is_body_twist = true){
    OdomState odom;
    auto pos = nav.pose.pose.position;
    auto vel = nav.twist.twist.linear;
    auto rot = nav.pose.pose.orientation;
    odom.pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
    odom.R = transform_utils::utils::gquat2rot(rot);
    if(is_body_twist){ //nav.twist.twist.linear = body frame twist
        odom.vel = odom.R * Eigen::Vector3d(vel.x, vel.y, vel.z);
    }
    else{ // nav.twist.twist.linear = world frame velocity
        odom.vel = Eigen::Vector3d(vel.x, vel.y, vel.z);
    }
    return odom;
}

}

#endif