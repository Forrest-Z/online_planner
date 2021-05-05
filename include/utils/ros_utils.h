#ifndef ONLINE_ROS_UTILS_H_
#define ONLINE_ROS_UTILS_H_

#include <ros/ros.h>
#include <planner/planner_common.h>
#include <mapping/octomap_handler.h>

namespace online_planner{
inline OctomapHandler::Param loadOctomapHandlerParam(ros::NodeHandle nh_private){
    OctomapHandler::Param param;
    nh_private.param("oct_res", param.oct_res, 0.1);
    nh_private.param("max_range", param.max_range, 7.0);
    double x_min, y_min, z_min, x_max, y_max, z_max;
    nh_private.param("x_min", x_min, -1.0);
    nh_private.param("y_min", y_min, -5.0);
    nh_private.param("z_min", z_min, -0.2);
    nh_private.param("x_max", x_max,  20.0);
    nh_private.param("y_max", y_max,  10.0);
    nh_private.param("z_max", z_max,  5.0);
    param.bbxMin.x() = x_min;
    param.bbxMin.y() = y_min;
    param.bbxMin.z() = z_min;
    param.bbxMax.x() = x_max;
    param.bbxMax.y() = y_max;
    param.bbxMax.z() = z_max;
    return param;
}

}

#endif
