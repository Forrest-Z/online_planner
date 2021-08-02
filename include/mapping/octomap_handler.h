#ifndef ONLINE_OCTOMAP_HANDLER_H_
#define ONLINE_OCTOMAP_HANDLER_H_

#include <memory>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <utils/octomap_utils.h>
#include <pcl/conversions.h>
#include <depth2pc/depth_to_pc.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <octomap_msgs/Octomap.h>

namespace online_planner{
class OctomapHandler{
public:
    struct Param{
        double oct_res;     //octomap resolution
        double max_range;   //maximum depth range;
        octomap::point3d bbxMin, bbxMax;
        bool verbose;
        int undersample_rate;
        bool is_perspective;
    };
    OctomapHandler(Param param);
    ~OctomapHandler(){}
    double getDistanceAtPosition(Eigen::Vector3d p) const;
    double getDistanceAtPosition(octomath::Vector3 p) const;
    double getSafeDistanceAtPosition(Eigen::Vector3d p, double coll_thr) const;
    pcl::PointCloud<pcl::PointXYZI> trackChanges(); //track changes in sub_ot_
    void insertPointcloud(const sensor_msgs::ImageConstPtr img, octomath::Pose6D T_wc_oct);
    void insertUpdate(pcl::PointCloud<pcl::PointXYZI> changed_set);
    void getOctomapMsg(octomap_msgs::Octomap& oct_msg);
    double castRay(Eigen::Vector3d origin, Eigen::Vector3d direction, double range, bool ignore_unknown=false);
    void setDepthcamModel(sensor_msgs::CameraInfo cam_info){
        std::unique_lock<std::mutex> lock(sub_ot_mtx_);
        if(depthcam_model_set) return;
        depthcam_model.fromCameraInfo(cam_info);
        depthcam_model_set = true;
    }
    bool isCamModelSet(){
        std::unique_lock<std::mutex> lock(sub_ot_mtx_);
        return depthcam_model_set;
    }
protected:
    //under pub_ot_mtx_
    std::unique_ptr<octomap::OcTree> pub_ot_;  
    std::unique_ptr<DynamicEDTOctomap> edt_; // shared by pub_ot_mtx_

    //under sub_ot_mtx_
    std::unique_ptr<octomap::OcTree> sub_ot_;
    bool depthcam_model_set;
    image_geometry::PinholeCameraModel depthcam_model;
    
    //loaded from Param
    double oct_res;     //octomap resolution
    double max_range;   //maximum depth range
    int undersample_rate;
    octomap::point3d bbxMin, bbxMax;  
    bool verbose;
    bool is_perspective;

    mutable std::mutex pub_ot_mtx_, sub_ot_mtx_;
};
}//namespace online_planner

#endif