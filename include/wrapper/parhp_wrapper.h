#ifndef MP_AGNOSTIC_WRAPPER_H_
#define MP_AGNOSTIC_WRAPPER_H_

#include <planner/local_planner/motion_primitives_planner.h>
#include <mapping/octomap_handler.h>
#include <wrapper/base_wrapper.h>
#include <depth2pc/depth_to_pc.h>
#include <memory>

namespace online_planner{
class PaRhpWrapper : public BaseWrapper{
public:
    PaRhpWrapper();
    ~PaRhpWrapper();
private:
    MpPlanner* mp_planner_;
    std::shared_ptr<OctomapHandler> ot_handle_;

    //ros publisher
    ros::Publisher octomap_pub;

    //ros subscriber
    ros::Subscriber depth_img_sub, depth_cam_info_sub;
    void depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg);
    void depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);

    //loaded from ros
    transform_utils::SE3 T_bc;    
    
    ros::Timer global_plan_timer;
    ros::Timer local_plan_timer;
    ros::Timer visualize_timer;

    void airsimGlobalCallback(const ros::TimerEvent& e); //for airsim simulation
    void mavrosGlobalCallback(const ros::TimerEvent& e); //for mavros real world flight. assume manual initialization
    void localPlannerCallback(const ros::TimerEvent& e); 
    void visualizeCallback(const ros::TimerEvent& e); 
};
}

#endif