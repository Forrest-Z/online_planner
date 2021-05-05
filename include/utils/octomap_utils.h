#ifndef PA_OCTOMAP_UTILS_H_
#define PA_OCTOMAP_UTILS_H_

#include <octomap/math/Pose6D.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <octomap/Pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Utility library for transforming to octomap types
namespace online_planner{

inline octomath::Vector3 toOctVec(Eigen::Vector3d v){
    octomath::Vector3 u;
    u.x() = (float)v.x(); u.y() = (float)v.y(); u.z() = (float)v.z();
    return u;
}

inline octomath::Vector3 toOctVec(geometry_msgs::Point v){
    octomath::Vector3 u;
    u.x() = (float)v.x; u.y() = (float)v.y; u.z() = (float)v.z;
    return u;
}

inline octomath::Quaternion toOctQuat(geometry_msgs::Quaternion q){
    octomath::Quaternion oq;
    oq.u() = q.w; oq.x() = q.x; oq.y() = q.y; oq.z() = q.z;
    return oq;
}

inline octomath::Pose6D geoPose2octPose(geometry_msgs::Pose p){
    octomath::Vector3 oct_vec=  toOctVec(p.position);
    octomath::Quaternion oct_quat = toOctQuat(p.orientation);
    octomath::Pose6D oct_pose(oct_vec, oct_quat);
    return oct_pose;
}

inline void pc2ToOctomap(const sensor_msgs::PointCloud2& cloud, octomap::Pointcloud& octomapCloud) {
    octomapCloud.reserve(cloud.data.size() / cloud.point_step);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"); 
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
         // Check if the point is invalid
         if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
           octomapCloud.push_back(*iter_x, *iter_y, *iter_z);
       }
}
}//namespace online_planner


#endif