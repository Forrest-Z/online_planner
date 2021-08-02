#ifndef ONLINE_ROS_UTILS_H_
#define ONLINE_ROS_UTILS_H_

#include <ros/ros.h>
#include <traj_lib/MavState.h>
#include <mapping/octomap_handler.h>
#include <planner/local_planner/motion_primitives_planner.h>

namespace online_planner{
inline OctomapHandler::Param loadOctomapHandlerParam(ros::NodeHandle nh_private){
    OctomapHandler::Param param;
    nh_private.param("oct_res", param.oct_res, 0.1);
    nh_private.param("max_range", param.max_range, 7.0);
    nh_private.param("undersample_rate", param.undersample_rate, 4);
    nh_private.param("print_map_update", param.verbose, false);
    double x_min, y_min, z_min, x_max, y_max, z_max;
    nh_private.param("x_min", x_min, -1.0);
    nh_private.param("y_min", y_min, -5.0);
    nh_private.param("z_min", z_min, -0.2);
    nh_private.param("x_max", x_max,  20.0);
    nh_private.param("y_max", y_max,  10.0);
    nh_private.param("z_max", z_max,  5.0);
    nh_private.param("is_perspective", param.is_perspective, false);
    param.bbxMin.x() = x_min;
    param.bbxMin.y() = y_min;
    param.bbxMin.z() = z_min;
    param.bbxMax.x() = x_max;
    param.bbxMax.y() = y_max;
    param.bbxMax.z() = z_max;
    return param;
}

inline MpEvaluator::Param loadMpEvalParam(ros::NodeHandle nh_private){
    MpEvaluator::Param param;
    nh_private.param("k_col", param.k_col, 15.0);
    nh_private.param("k_per", param.k_per, 5.0);
    nh_private.param("k_prox", param.k_prox, 2.0);
    nh_private.param("k_end", param.k_end, 4.0);
    nh_private.param("d_critic", param.d_critic, 0.5);

    nh_private.param("safe_dist", param.safe_dist, 1.0);
    nh_private.param("dist_th", param.dist_th, 0.6);
    nh_private.param("power_dist", param.power_dist, 3);

    nh_private.param("v_max", param.v_max, 2.0);
    nh_private.param("a_max", param.a_max, 4.0);
    nh_private.param("J", param.J, 10);
    nh_private.param("h_fov", param.h_fov, 90.0);
    nh_private.param("v_fov", param.v_fov, 73.8);
    return param;
}

inline MpPlanner::Param loadMpPlannerParam(ros::NodeHandle nh_private){
    MpPlanner::Param param;
    nh_private.param("L", param.L, 5.0);
    nh_private.param("h_ang_max", param.h_ang_max, 45.0);
    nh_private.param("v_ang", param.v_ang, 10.0);
    nh_private.param("v_des", param.v_des, 1.5);
    nh_private.param("k_th", param.k_th, 180.0);
    nh_private.param("h_res", param.h_res, 20);
    nh_private.param("v_res", param.v_res, 5);
    
    param.eval_param = loadMpEvalParam(nh_private);

    return param;
}

inline Eigen::Vector3d get_vector3d_from_ros(const ros::NodeHandle& nh_private, std::string param_name){
    double x, y, z;
    bool found = true;
    found = found && nh_private.getParam(param_name+"_1", x);
    found = found && nh_private.getParam(param_name+"_2", y);
    found = found && nh_private.getParam(param_name+"_3", z);
    if(!found){
        ROS_ERROR_STREAM(param_name<<" Not found");
        return Eigen::Vector3d::Zero();
    } 
    return Eigen::Vector3d(x, y, z);
}

}

#endif
