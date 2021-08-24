#pragma once

#include <wrapper/base_wrapper.h>
#include <traj_lib/MinJerkPolyTraj.h>
#include <mapping/octomap_handler.h>
#include <mapping/featuremap_handler.h>
#include <vins_vio_mod/KeyframeInfo.h>

//very simple wrapper which performs single line flight assuming no obstruction on flight path
namespace online_planner{
class MaptestWrapper: public BaseWrapper{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MaptestWrapper();
    ~MaptestWrapper(){}

protected:
    std::shared_ptr<OctomapHandler> ot_handle_;
    std::shared_ptr<FeatureMapHandler> fm_handle_;
    bool ot_verbose_, fm_verbose_;

    //ros publisher
    ros::Publisher octomap_pub, fm_pub;

    //ros subscriber
    ros::Subscriber depth_img_sub, depth_cam_info_sub, kf_info_sub;
    void depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg);
    void depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);
    void kfInfoCallback(const vins_vio_mod::KeyframeInfoConstPtr& kf_msg);

    //loaded from rosparam or loaded at construction
    transform_utils::SE3 T_bc;
    double t_plan_delay;
    double v_des;
    Eigen::MatrixX3d M;    
    std::pair<double, double> fov_rads;

    //planning related. shared under traj_path_mtx_;
    traj_lib::MinJerkPolyTraj trajectory;
    double last_valid_yaw;
    
    //timers
    ros::Timer global_plan_timer;
    ros::Timer local_plan_timer;
    ros::Timer visualize_timer;
    void airsimGlobalCallback(const ros::TimerEvent& e); //for airsim simulation
    void localPlannerCallback(const ros::TimerEvent& e); 
    void visualizeCallback(const ros::TimerEvent& e);

    virtual traj_lib::FlatState getInitState() override;
    void computePlan();
};
}