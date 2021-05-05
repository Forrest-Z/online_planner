#ifndef ONLINE_OCTOMAP_HANDLER_H_
#define ONLINE_OCTOMAP_HANDLER_H_

#include <memory>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <utils/octomap_utils.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace online_planner{
class OctomapHandler{
public:
    struct Param{
        double oct_res;     //octomap resolution
        double max_range;   //maximum depth range;
        octomap::point3d bbxMin, bbxMax;
    };
    OctomapHandler(Param param);
    ~OctomapHandler(){}
    double getDistanceAtPosition(Eigen::Vector3d p) const;
    double getDistanceAtPosition(octomath::Vector3 p) const;
    double getSafeDistanceAtPosition(Eigen::Vector3d p, double coll_thr) const;
    void insertPointcloud(sensor_msgs::PointCloud2 pcd, geometry_msgs::Pose T_wc);
    double castRay(Eigen::Vector3d origin, Eigen::Vector3d direction, double range, bool ignore_unknown=false);
private:
    std::unique_ptr<octomap::OcTree> ot_;
    std::unique_ptr<DynamicEDTOctomap> edt_;

    //loaded from Param
    double oct_res;     //octomap resolution
    double max_range;   //maximum depth range
    octomap::point3d bbxMin, bbxMax;  
};
}//namespace online_planner

#endif