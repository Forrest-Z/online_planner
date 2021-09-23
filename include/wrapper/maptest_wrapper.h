#pragma once

#include <wrapper/base_wrapper.h>
#include <traj_lib/MinJerkPolyTraj.h>
#include <mapping/octomap_handler.h>
#include <mapping/featuremap_handler.h>
#include <utils/featuremap_types.h>
#include <visualization_msgs/MarkerArray.h>

//very simple wrapper which performs single line flight assuming no obstruction on flight path
namespace online_planner{

struct visQueryResult{
    visQueryResult(){
        T_wc = transform_utils::from_Matrix4d(Eigen::Matrix4d::Identity());
        f_vec.clear();
    }
    visQueryResult(transform_utils::SE3 Twc, std::vector<FeatureInfo> vec):T_wc(Twc), f_vec(vec){}
    transform_utils::SE3 T_wc; //camera pose
    std::vector<FeatureInfo> f_vec;
};

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
    ros::Publisher octomap_pub, fm_pub, query_result_pub;

    //ros subscriber
    ros::Subscriber depth_img_sub, depth_cam_info_sub, kf_info_sub;
    void depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg);
    void depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);
    void KfInfoCallback(const vins_vio_mod::KeyframeInfoConstPtr& kf_msg);
    void generateQueryResultViz(visualization_msgs::MarkerArray& qr_msg, const visQueryResult& result);

    //loaded from rosparam or loaded at construction
    transform_utils::SE3 T_bc;
    double t_plan_delay;
    double v_des;
    double d_fcut;
    Eigen::MatrixX3d M;    
    std::pair<double, double> fov_rads;
    //visualization parameters
    double l_arrow, d_cam, s_markers;

    //planning related. shared under traj_path_mtx_;
    traj_lib::MinJerkPolyTraj trajectory;
    bool is_trajectory_set;
    double last_valid_yaw;
    
    //timers
    ros::Timer global_plan_timer;
    ros::Timer local_plan_timer;
    ros::Timer visualize_timer;
    void airsimGlobalCallback(const ros::TimerEvent& e); //for airsim simulation
    void localPlannerCallback(const ros::TimerEvent& e); 
    void visualizeCallback(const ros::TimerEvent& e);

    //functions those are allowed to gain state_mtx_, traj_path_mtx_
    virtual traj_lib::FlatState getInitState() override;
    void computePlan(ros::Time t_preplan, traj_lib::FlatState init_state);
    bool getTimeSpan(double& start, double& end);
    visQueryResult visibilityQueryTest(const double& t_start, const double& t_end, const double& ivl);
    bool getSetpoints(const double& t_start, const double& t_end, std::vector<mavros_msgs::PositionTarget>& sp_vec);

    Timer kf_callback_timer;
};
}