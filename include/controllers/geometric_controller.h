#pragma once


#include <algorithm>
#include <geometry_msgs/TwistStamped.h>

#include <controllers/controller_common.h>

namespace online_planner{
class GeometricController{
public:
struct Params{
    double k_p;
    double k_v;
    Eigen::Vector3d k_R;
    double control_frequency;
    double m, g; //gravity and mass
    bool is_body_twist;
    bool is_ned;
};
    GeometricController(Params p):
    k_p(p.k_p), k_v(p.k_v), k_R(p.k_R), 
    control_frequency(p.control_frequency), is_body_twist(p.is_body_twist){}
    ~GeometricController(){}
    void setState(nav_msgs::Odometry odom);
    RpyThCmd computeControl(ControlState target_state);
private:
    OdomState curr_odom; //current state
    
    //loaded from params
    double k_p;
    double k_v;
    Eigen::Vector3d k_R;
    double control_frequency;
    double m;
    double g;
    bool is_body_twist;
};
}