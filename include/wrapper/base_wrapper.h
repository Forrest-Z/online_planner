#ifndef WRAPPER_BASE_H_
#define WRAPPER_BASE_H_

#include <planner/global_planner/global_planner.h>
#include <planner/local_planner/local_planner.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <transform_utils/transform_utils.h>
#include <airsim_controller/PositionTargets.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <traj_lib/MavTrajBase.h>
#include <string>
#include <mutex>
#include <std_srvs/Trigger.h>

#define IG_P 7 // 1+2+4
#define IG_V 56 // 8+16+32
#define IG_A 448 // 64, 128, 256
#define IG_YAW 1024
#define IG_YAWR 2048

// Describes basic threading for wrapper nodes
namespace online_planner{

class BaseWrapper{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
enum class Status{
    ODOM_NOT_SET,
    ODOM_INIT,
    TAKEOFF_SEQUENCE, 
    INIT_HOVER,
    FLIGHT,
    REACHED_GOAL
};
    //virtual int run() = 0;
    BaseWrapper();
    ~BaseWrapper();
    //state getters. all gains state_mtx_
    traj_lib::FlatState getCurrFlatState();
    traj_lib::MavState getCurrState();
    bool transformStabilized();
protected:
    ros::NodeHandle nh_default_; // handles heavy message processing.
    ros::NodeHandle nh_custom_; // handles more lightweight, high priority message / services 
    ros::CallbackQueue custom_queue;
    std::shared_ptr<ros::AsyncSpinner> custom_spinner;

    //All ros related actors are defined under nh_custom_.
    //publisher
    ros::Publisher sp_pub; //setpoint/reference trajectory publisher
    mavros_msgs::PositionTarget pos_sp;
    ros::Publisher airsim_vel_pub; // if airsim, just for VIO initializing sequence
    airsim_ros_pkgs::VelCmd vel_cmd_;
   
    //airsim related single step function
    void airsimVioInitSingleIter();
    void airsimTakeoffSingleIter();
    void airsimHoverSingleIter();

    //subscriber. callback functions gain state_mtx_ lock
    ros::Subscriber stat_sub;
    void statCallback(const mavros_msgs::StateConstPtr& msg);
    ros::Subscriber odom_sub;
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    //client
    ros::ServiceClient arming_client, set_mode_client, control_stop_client;

    int n_bound, n_hover;
    int n_executed;
    int n_stable;
    
    //mission, frame related variables
    Eigen::Vector3d goal_u, goal_o; //goal position in user frame/odom frame
    std::string world_frame_name, odom_topic_name;
    mavros_msgs::State mav_stat;

    std::mutex state_mtx_; //following variables are controlled under state_mtx_
    nav_msgs::Odometry imu_odom; //IMU frame odometry
    traj_lib::MavState curr_state;
    traj_lib::FlatState curr_flat_state;
    ros::Time t_last_odom_input;

    std_srvs::Trigger trig;

    //status
    Status status_; //only updated from globalplanthread
    bool mavros_okay, transform_stabilized;

    //loaded from ros and not modified further
    transform_utils::SE3 T_bi; //body to IMU transform
    transform_utils::SE3 T_ub_init; // initial pose of body frame w.r.t user frame
    int type_mask; //type_mask to use in planning
    double dt_control; //setpoint publishing rate. limited to airsim processing speed
    double dt_local_planning; // local planning rate
    double dt_global_planning; // global planning rate
    int sp_per_plan; //how many setpoints are published per local trajectory plan?
    bool is_mavros; //true if using mavros. if false, select airsim_controller/PositionTargets
    bool is_odom_ned; //assume NWU for interface frame, but odometry can be in NED convention
    double takeoff_height, takeoff_speed; // 
    ros::Time reference_time;   //time
    // verbosities
    bool print_setstate, print_trajectory;

    //trajectory & path information
    std::mutex traj_path_mtx_;
    traj_lib::MavTrajBase* current_best_trajectory; //whole trajectory
    std::vector<Eigen::Vector4d> selected_path; //x, y, z path

    virtual traj_lib::FlatState getInitState() = 0;
    mavros_msgs::PositionTarget SetPointToPt(const traj_lib::SetPoint& state);
    double getSimTime(ros::Time t){return (t - reference_time).toSec();}

    //following functions gain state_mtx_
    void checkGoalReached();
};

inline transform_utils::SE3 loadTransformFromRos(std::string prefix, const ros::NodeHandle& nh){
    transform_utils::SE3 T;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    p.x() = nh.param(prefix+"_x", 0.0);
    p.y() = nh.param(prefix+"_y", 0.0);
    p.z() = nh.param(prefix+"_z", 0.0);
    q.x() = nh.param(prefix+"_qx", 0.0);
    q.y() = nh.param(prefix+"_qy", 0.0);
    q.z() = nh.param(prefix+"_qz", 0.0);
    q.w() = nh.param(prefix+"_qw", 1.0);
    T = transform_utils::utils::from_vec3_quat(p, q);
    return T;
}

}

#endif