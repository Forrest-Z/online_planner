#ifndef PARHP_WRAPPER_H_
#define PARHP_WRAPPER_H_

#include <planner/local_planner/motion_primitives_planner.h>
#include <mapping/octomap_handler.h>
#include <wrapper/base_wrapper.h>
#include <depth2pc/depth_to_pc.h>
#include <memory>
#include <utils/timer.h>
#include <mapping/featuremap_handler.h>
#include <vins_vio_mod/KeyframeInfo.h>

namespace online_planner{
class PaRhpWrapper : public BaseWrapper{
public:
    PaRhpWrapper();
    ~PaRhpWrapper();
private:
    MpPlanner* mp_planner_;
    std::shared_ptr<OctomapHandler> ot_handle_;
    std::shared_ptr<FeatureMapHandler> fm_handle_;
    bool ot_verbose_, fm_verbose_;

    //ros publisher
    ros::Publisher octomap_pub;

    //ros subscriber
    ros::Subscriber depth_img_sub, depth_cam_info_sub, kf_info_sub;
    void depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg);
    void depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);
    void kfInfoCallback(const vins_vio_mod::KeyframeInfoConstPtr& kf_msg);

    //loaded from ros
    transform_utils::SE3 T_bc;    
    
    ros::Timer global_plan_timer;
    ros::Timer local_plan_timer;
    ros::Timer visualize_timer;

    void airsimGlobalCallback(const ros::TimerEvent& e); //for airsim simulation
    void mavrosGlobalCallback(const ros::TimerEvent& e); //for mavros real world flight. assume manual initialization
    void localPlannerCallback(const ros::TimerEvent& e); 
    void visualizeCallback(const ros::TimerEvent& e); 

    virtual traj_lib::FlatState getInitState() override;
    
    //timer
    Timer tictoc_localplanning;
    std::mutex t_mtx_;
};
}

#endif