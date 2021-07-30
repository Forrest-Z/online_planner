#ifndef PLANNER_COMMON_H_
#define PLANNER_COMMON_H_

//This file provide common types and definition that might be useful for planners
#include <string>
#include <transform_utils/transform_utils.h>
#include <Eigen/Dense>

namespace online_planner{

#define DIM_PLANNING 3
#define MP_ORDER 5

//Easy setpoint for return
struct SetPoint{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d accel;
    double yaw;
};

struct pvaState{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d accel;
    double yaw;
};

struct SE3State{
    Eigen::Vector3d Position;
    Eigen::Matrix3d Rotation;
};

using SetPoint = struct SetPoint;
using pvaState = struct pvaState;

namespace local_planner_types{
    const std::string tMotionPrimitives = "motion_primitives";
} //namespace local_planner_types

} //online_planner

#endif
